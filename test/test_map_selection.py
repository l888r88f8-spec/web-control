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


def _two_stick(throttle=None, steering=None):
    def stick(overrides):
        return {
            "size": 200.0,
            "x": 20.0,
            "y": 320.0,
            **(overrides or {}),
        }

    return {"throttle": stick(throttle), "steering": stick(steering)}


def test_save_rewrites_only_name_and_preserves_ui_and_unknown_keys(tmp_path):
    path = tmp_path / "selected_map.json"
    path.write_text(
        json.dumps(
            {
                "name": "site_a",
                "ui": {
                    "joystick": _two_stick(),
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
            "joystick": _two_stick(),
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


def test_load_joystick_preferences_returns_two_stick_defaults_for_missing_or_invalid(
    tmp_path,
):
    path = tmp_path / "selected_map.json"
    store = SelectionStore(path)
    assert store.load_joystick_preferences() == default_joystick_preferences()

    for ui in (
        {"joystick": {"throttle": [], "steering": []}},
        {"joystick": {"throttle": {"size": "big", "x": None, "y": None}}},
        {"joystick": []},
        "not-an-object",
    ):
        path.write_text(
            json.dumps({"name": "site_a", "ui": ui}) + "\n", encoding="utf-8"
        )
        assert store.load_joystick_preferences() == default_joystick_preferences()
        assert store.load() == "site_a"


def test_load_joystick_preferences_keeps_valid_fields_and_defaults_invalid_ones(
    tmp_path,
):
    path = tmp_path / "selected_map.json"
    path.write_text(
        json.dumps(
            {
                "name": "site_a",
                "ui": {
                    "joystick": {
                        "throttle": {"size": 240, "x": 40, "y": -1},
                        "steering": {"size": "nope", "x": 10**400, "y": 260},
                    }
                },
            }
        ),
        encoding="utf-8",
    )

    assert SelectionStore(path).load_joystick_preferences() == {
        "throttle": {"size": 240, "x": 40, "y": None},
        "steering": {"size": 180, "x": None, "y": 260},
    }


def test_load_joystick_preferences_migrates_legacy_single_joystick(tmp_path):
    path = tmp_path / "selected_map.json"
    path.write_text(
        json.dumps(
            {"name": "site_a", "ui": {"joystick": {"size": 240, "x_ratio": 0.5}}}
        ),
        encoding="utf-8",
    )

    assert SelectionStore(path).load_joystick_preferences() == {
        "throttle": {"size": 240, "x": None, "y": None},
        "steering": {"size": 240, "x": None, "y": None},
    }
    assert SelectionStore(path).load() == "site_a"


def test_save_joystick_preferences_preserves_name_and_unknown_nested_ui_keys(
    tmp_path,
):
    path = tmp_path / "selected_map.json"
    path.write_text(
        json.dumps({"name": "site_a", "ui": {"theme": "dark"}, "extra": 7}) + "\n",
        encoding="utf-8",
    )

    saved = SelectionStore(path).save_joystick_preferences(_two_stick())

    assert saved == _two_stick()
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload == {
        "name": "site_a",
        "ui": {"theme": "dark", "joystick": _two_stick()},
        "extra": 7,
    }
    assert SelectionStore(path).load() == "site_a"


def test_save_joystick_preferences_replaces_a_non_object_ui_value(tmp_path):
    path = tmp_path / "selected_map.json"
    path.write_text(
        json.dumps({"name": "site_a", "ui": "broken"}) + "\n", encoding="utf-8"
    )

    SelectionStore(path).save_joystick_preferences(_two_stick())

    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["ui"] == {"joystick": _two_stick()}
    assert SelectionStore(path).load() == "site_a"


@pytest.mark.parametrize(
    "payload",
    [
        None,
        [],
        {},
        {"throttle": {"size": 180, "x": 0, "y": 0}},
        {
            "throttle": {"size": 100, "x": 0, "y": 0},
            "steering": {"size": 180, "x": 0, "y": 0},
        },
        {
            "throttle": {"size": 300, "x": 0, "y": 0},
            "steering": {"size": 180, "x": 0, "y": 0},
        },
        {
            "throttle": {"size": "180", "x": 0, "y": 0},
            "steering": {"size": 180, "x": 0, "y": 0},
        },
        {
            "throttle": {"size": True, "x": 0, "y": 0},
            "steering": {"size": 180, "x": 0, "y": 0},
        },
        {
            "throttle": {"size": float("nan"), "x": 0, "y": 0},
            "steering": {"size": 180, "x": 0, "y": 0},
        },
        {
            "throttle": {"size": 180, "x": -0.1, "y": 0},
            "steering": {"size": 180, "x": 0, "y": 0},
        },
        {
            "throttle": {"size": 180, "x": 0},
            "steering": {"size": 180, "x": 0, "y": 0},
        },
        {
            "throttle": {"size": 180, "y": 0},
            "steering": {"size": 180, "x": 0, "y": 0},
        },
        {"throttle": [], "steering": {"size": 180, "x": 0, "y": 0}},
    ],
)
def test_validate_joystick_preferences_rejects_malformed_or_out_of_range(payload):
    with pytest.raises(ValueError):
        validate_joystick_preferences(payload)


def test_saved_joystick_preferences_falls_back_per_field():
    assert saved_joystick_preferences(None) == default_joystick_preferences()
    assert saved_joystick_preferences(
        {
            "throttle": {"size": 200, "x": float("inf"), "y": 25},
            "steering": {"size": 150, "x": 10, "y": "bad"},
        }
    ) == {
        "throttle": {"size": 200.0, "x": None, "y": 25.0},
        "steering": {"size": 150.0, "x": 10.0, "y": None},
    }


@pytest.mark.parametrize(
    "role,key",
    [
        ("throttle", "size"),
        ("throttle", "x"),
        ("throttle", "y"),
        ("steering", "size"),
        ("steering", "x"),
        ("steering", "y"),
    ],
)
def test_validate_joystick_preferences_rejects_overflowing_integer(role, key):
    payload = _two_stick()
    payload[role][key] = 10**400

    with pytest.raises(ValueError):
        validate_joystick_preferences(payload)


def test_saved_joystick_preferences_falls_back_for_overflowing_integer():
    assert saved_joystick_preferences(
        {
            "throttle": {"size": 10**400, "x": 10**400, "y": 25},
            "steering": {"size": 10**400, "x": 10, "y": 10**400},
        }
    ) == {
        "throttle": {"size": 180.0, "x": None, "y": 25.0},
        "steering": {"size": 180.0, "x": 10.0, "y": None},
    }


def test_load_joystick_preferences_recovers_from_overflowing_saved_values(tmp_path):
    path = tmp_path / "selected_map.json"
    path.write_text(
        json.dumps(
            {
                "name": "site_a",
                "ui": {
                    "joystick": {
                        "throttle": {"size": 10**400, "x": 10**400, "y": 40},
                        "steering": {"size": 220, "x": 10**400, "y": 10**400},
                    }
                },
            }
        )
        + "\n",
        encoding="utf-8",
    )

    assert SelectionStore(path).load_joystick_preferences() == {
        "throttle": {"size": 180.0, "x": None, "y": 40.0},
        "steering": {"size": 220.0, "x": None, "y": None},
    }


def test_joystick_write_replaces_file_atomically(tmp_path, monkeypatch):
    path = tmp_path / "web_control" / "selected_map.json"
    store = SelectionStore(path)
    store.save("site_a")
    store.save_joystick_preferences(_two_stick())

    replacements = []
    real_replace = os.replace

    def spying_replace(source, destination):
        source_path = Path(source)
        payload = json.loads(source_path.read_text(encoding="utf-8"))
        assert payload["name"] == "site_a"
        assert payload["ui"]["joystick"]["throttle"]["size"] == 220
        replacements.append((source_path, Path(destination)))
        real_replace(source, destination)

    monkeypatch.setattr(os, "replace", spying_replace)
    store.save_joystick_preferences(_two_stick(throttle={"size": 220}))

    assert len(replacements) == 1
    temporary, destination = replacements[0]
    assert temporary.parent == path.parent
    assert destination == path
    assert store.load_joystick_preferences() == _two_stick(throttle={"size": 220})
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
                _two_stick(throttle={"size": 140 + index})
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
    assert 140 <= payload["ui"]["joystick"]["throttle"]["size"] <= 260
    assert payload["ui"]["joystick"]["throttle"]["x"] == 20
    assert payload["ui"]["theme"] == "dark"
    assert payload["extra"] == 1
