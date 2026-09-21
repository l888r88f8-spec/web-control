"""Tests for the shared atomic writer and navigation persistence."""
import os
from pathlib import Path

import pytest

from web_control.atomic_io import atomic_write_text
from web_control.navigation_store import (
    NavigationStore,
    default_navigation_path,
    validate_navigation_record,
)


def test_atomic_write_text_replaces_target_with_written_sibling(tmp_path, monkeypatch):
    path = tmp_path / "config" / "web_control" / "paused_navigation.json"
    replacements = []
    real_replace = os.replace

    def spying_replace(source, destination):
        source_path = Path(source)
        assert source_path.read_text(encoding="utf-8") == '{"a": 1}\n'
        replacements.append((source_path, Path(destination)))
        real_replace(source, destination)

    monkeypatch.setattr(os, "replace", spying_replace)
    atomic_write_text(path, '{"a": 1}\n')

    assert len(replacements) == 1
    temporary, destination = replacements[0]
    assert temporary.parent == path.parent
    assert destination == path
    assert path.read_text(encoding="utf-8") == '{"a": 1}\n'
    assert [entry.name for entry in path.parent.iterdir()] == ["paused_navigation.json"]


def _record(**overrides):
    record = {
        "schema_version": 1,
        "phase": "paused",
        "x": 1.0,
        "y": 2.0,
        "z": 0.0,
        "yaw": 0.5,
        "frame": "map",
        "map_name": "site_a",
        "saved_at": 5.0,
    }
    record.update(overrides)
    return record


def test_navigation_store_round_trip(tmp_path):
    store = NavigationStore(tmp_path / "config" / "paused_navigation.json")

    assert store.load() is None
    store.save(_record())
    assert store.load() == _record()
    store.clear()
    assert store.load() is None
    store.clear()  # clearing an absent record is not an error


@pytest.mark.parametrize(
    "bad",
    [
        None,
        [],
        {
            "schema_version": 2,
            "phase": "paused",
            "x": 1,
            "y": 2,
            "z": 0,
            "yaw": 0,
            "frame": "map",
            "map_name": "site_a",
            "saved_at": 1,
        },
        _record(phase="flying"),
        _record(frame="   "),
        _record(map_name="../etc"),
        _record(x="not-a-number"),
        _record(y=float("inf")),
        _record(saved_at="later"),
    ],
)
def test_validate_navigation_record_rejects_bad_payloads(bad):
    with pytest.raises(ValueError):
        validate_navigation_record(bad)


def test_validate_navigation_record_normalises_numeric_strings():
    record = validate_navigation_record(
        _record(x="1.5", y=2, z="-0.5", yaw="0", saved_at=3)
    )

    assert record["x"] == 1.5
    assert record["y"] == 2.0
    assert record["z"] == -0.5
    assert record["yaw"] == 0.0
    assert record["saved_at"] == 3.0


def test_navigation_store_ignores_corrupt_file(tmp_path, caplog, capsys):
    path = tmp_path / "paused_navigation.json"
    path.write_text("{not json", encoding="utf-8")

    assert NavigationStore(path).load() is None

    log_output = caplog.text + capsys.readouterr().err
    assert "invalid paused navigation record" in log_output
    assert str(path) in log_output


def test_default_navigation_path_lives_under_runtime_home():
    assert default_navigation_path(Path("/home/robot")) == Path(
        "/home/robot/.config/web_control/paused_navigation.json"
    )
