"""Focused tests for locked SelectionStore read-modify-write persistence."""

from __future__ import annotations

import json
import os
import threading
from pathlib import Path

import pytest

from web_control.map_selection import (
    SelectionStore,
    default_joystick_preferences,
    saved_joystick_preferences,
    validate_joystick_preferences,
)


def test_save_rewrites_only_name_and_preserves_ui_and_unknown_keys(tmp_path):
    path = tmp_path / "selected_map.json"
    path.write_text(
        json.dumps(
            {
                "name": "site_a",
                "ui": {
                    "joystick": {"size": 200, "x_ratio": 0.2, "y_ratio": 0.6},
                    "theme": "dark",
                },
                "extra": {"keep": True},
            }
        )
        + "\n",
        encoding="utf-8",
    )

    SelectionStore(path).save("site_b")

    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload == {
        "name": "site_b",
        "ui": {
            "joystick": {"size": 200, "x_ratio": 0.2, "y_ratio": 0.6},
            "theme": "dark",
        },
        "extra": {"keep": True},
    }
    assert SelectionStore(path).load() == "site_b"


def test_save_keeps_legacy_minimal_shape(tmp_path):
    path = tmp_path / "selected_map.json"
    store = SelectionStore(path)
    store.save("site_a")

    store.save("site_b")

    assert path.read_text(encoding="utf-8") == '{"name": "site_b"}\n'
    assert store.load() == "site_b"


def test_load_joystick_preferences_returns_defaults_for_missing_or_invalid(tmp_path):
    path = tmp_path / "selected_map.json"
    store = SelectionStore(path)
    assert store.load_joystick_preferences() == default_joystick_preferences()

    for ui in (
        {"joystick": {"size": 100, "x_ratio": 2.0, "y_ratio": -1.0}},
        {"joystick": {"size": "big", "x_ratio": None}},
        {"joystick": []},
        "not-an-object",
    ):
        path.write_text(
            json.dumps({"name": "site_a", "ui": ui}) + "\n", encoding="utf-8"
        )
        assert store.load_joystick_preferences() == default_joystick_preferences()
        assert store.load() == "site_a"


def test_load_joystick_preferences_keeps_valid_fields_and_defaults_invalid_ones(tmp_path):
    path = tmp_path / "selected_map.json"
    path.write_text(
        json.dumps(
            {"name": "site_a", "ui": {"joystick": {"size": 240, "x_ratio": 0.5}}}
        ),
        encoding="utf-8",
    )

    assert SelectionStore(path).load_joystick_preferences() == {
        "size": 240,
        "x_ratio": 0.5,
        "y_ratio": 0.75,
    }


def test_save_joystick_preferences_preserves_name_and_unknown_nested_ui_keys(tmp_path):
    path = tmp_path / "selected_map.json"
    path.write_text(
        json.dumps({"name": "site_a", "ui": {"theme": "dark"}, "extra": 7}) + "\n",
        encoding="utf-8",
    )

    saved = SelectionStore(path).save_joystick_preferences(
        {"size": 200, "x_ratio": 0.3, "y_ratio": 0.4}
    )

    assert saved == {"size": 200, "x_ratio": 0.3, "y_ratio": 0.4}
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload == {
        "name": "site_a",
        "ui": {
            "theme": "dark",
            "joystick": {"size": 200, "x_ratio": 0.3, "y_ratio": 0.4},
        },
        "extra": 7,
    }
    assert SelectionStore(path).load() == "site_a"


def test_save_joystick_preferences_replaces_a_non_object_ui_value(tmp_path):
    path = tmp_path / "selected_map.json"
    path.write_text(
        json.dumps({"name": "site_a", "ui": "broken"}) + "\n", encoding="utf-8"
    )

    SelectionStore(path).save_joystick_preferences(
        {"size": 180, "x_ratio": 0.1, "y_ratio": 0.9}
    )

    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["ui"] == {
        "joystick": {"size": 180, "x_ratio": 0.1, "y_ratio": 0.9}
    }
    assert SelectionStore(path).load() == "site_a"


@pytest.mark.parametrize(
    "payload",
    [
        None,
        [],
        {},
        {"size": 100, "x_ratio": 0.5, "y_ratio": 0.5},
        {"size": 300, "x_ratio": 0.5, "y_ratio": 0.5},
        {"size": "180", "x_ratio": 0.5, "y_ratio": 0.5},
        {"size": True, "x_ratio": 0.5, "y_ratio": 0.5},
        {"size": float("nan"), "x_ratio": 0.5, "y_ratio": 0.5},
        {"size": 180, "x_ratio": -0.1, "y_ratio": 0.5},
        {"size": 180, "x_ratio": 1.1, "y_ratio": 0.5},
        {"size": 180, "x_ratio": 0.5},
        {"size": 180, "y_ratio": 0.5},
    ],
)
def test_validate_joystick_preferences_rejects_malformed_or_out_of_range(payload):
    with pytest.raises(ValueError):
        validate_joystick_preferences(payload)


def test_saved_joystick_preferences_falls_back_per_field():
    assert saved_joystick_preferences(None) == default_joystick_preferences()
    assert saved_joystick_preferences(
        {"size": 200, "x_ratio": float("inf"), "y_ratio": 0.25}
    ) == {"size": 200.0, "x_ratio": 0.05, "y_ratio": 0.25}


def test_joystick_write_replaces_file_atomically(tmp_path, monkeypatch):
    path = tmp_path / "web_control" / "selected_map.json"
    store = SelectionStore(path)
    store.save("site_a")
    store.save_joystick_preferences({"size": 200, "x_ratio": 0.2, "y_ratio": 0.6})

    replacements = []
    real_replace = os.replace

    def spying_replace(source, destination):
        source_path = Path(source)
        payload = json.loads(source_path.read_text(encoding="utf-8"))
        assert payload["name"] == "site_a"
        assert payload["ui"]["joystick"]["size"] == 220
        replacements.append((source_path, Path(destination)))
        real_replace(source, destination)

    monkeypatch.setattr(os, "replace", spying_replace)
    store.save_joystick_preferences({"size": 220, "x_ratio": 0.3, "y_ratio": 0.7})

    assert len(replacements) == 1
    temporary, destination = replacements[0]
    assert temporary.parent == path.parent
    assert destination == path
    assert store.load_joystick_preferences() == {
        "size": 220,
        "x_ratio": 0.3,
        "y_ratio": 0.7,
    }
    assert [entry.name for entry in path.parent.iterdir()] == ["selected_map.json"]


def test_concurrent_selection_and_joystick_writes_preserve_both(tmp_path):
    path = tmp_path / "selected_map.json"
    path.write_text(
        json.dumps({"name": "site_seed", "ui": {"theme": "dark"}, "extra": 1}) + "\n",
        encoding="utf-8",
    )
    store = SelectionStore(path)
    errors = []

    def write_name(index):
        try:
            store.save(f"site_{index}")
        except Exception as exc:  # pragma: no cover - surfaced by the assert
            errors.append(exc)

    def write_joystick(index):
        try:
            store.save_joystick_preferences(
                {"size": 140 + index, "x_ratio": 0.25, "y_ratio": 0.5}
            )
        except Exception as exc:  # pragma: no cover - surfaced by the assert
            errors.append(exc)

    threads = [
        threading.Thread(
            target=write_name if index % 2 == 0 else write_joystick, args=(index,)
        )
        for index in range(40)
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert errors == []
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["name"].startswith("site_")
    assert 140 <= payload["ui"]["joystick"]["size"] <= 260
    assert payload["ui"]["joystick"]["x_ratio"] == 0.25
    assert payload["ui"]["theme"] == "dark"
    assert payload["extra"] == 1
