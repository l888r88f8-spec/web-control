"""Persistent map selection and joystick layout preferences.

The selection store owns the selected map directory name and the validated
joystick settings saved under ``ui.joystick`` in the same JSON file.  Every
write is a locked read-modify-write over the existing file with atomic
replacement, so a selection update and a joystick update cannot erase each
other and unknown top-level and nested keys are preserved.
"""

import json
import logging
import math
import threading
from pathlib import Path
from typing import Any

from web_control.atomic_io import atomic_write_text
from web_control.map_catalog import validate_map_name

SELECTION_FILENAME = "selected_map.json"
LOGGER = logging.getLogger(__name__)

JOYSTICK_DEFAULT_SIZE = 180.0
JOYSTICK_MIN_SIZE = 140.0
JOYSTICK_MAX_SIZE = 260.0
JOYSTICK_DEFAULT_X_RATIO = 0.05
JOYSTICK_DEFAULT_Y_RATIO = 0.75

# Serializes read-modify-write of the shared file across store instances and
# HTTP worker threads so the name update and the ui.joystick update cannot
# overwrite each other.
_STORE_LOCK = threading.RLock()


def default_selection_path(home: Path | None = None) -> Path:
    """Return the persisted selection path under ``home`` (default: runtime home)."""
    return _runtime_home(home) / ".config" / "web_control" / SELECTION_FILENAME


def _runtime_home(home: Path | None) -> Path:
    return Path(home) if home is not None else Path.home()


def _atomic_write_text(path: Path, text: str) -> None:
    """Write ``text`` to ``path`` by replacing a fully written sibling file."""
    atomic_write_text(path, text)


def default_joystick_preferences() -> dict[str, float]:
    """Return the fallback joystick layout when nothing valid is saved."""
    return {
        "size": JOYSTICK_DEFAULT_SIZE,
        "x_ratio": JOYSTICK_DEFAULT_X_RATIO,
        "y_ratio": JOYSTICK_DEFAULT_Y_RATIO,
    }


def _finite_number(payload: dict[str, Any], key: str) -> float:
    """Return ``payload[key]`` as a finite float, rejecting bools and strings."""
    if key not in payload:
        raise ValueError(f"missing {key}")
    value = payload[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{key} must be finite")
    return number


def validate_joystick_preferences(payload: Any) -> dict[str, float]:
    """Return a complete validated joystick setting for a preference write."""
    if not isinstance(payload, dict):
        raise ValueError("joystick preferences must be an object")
    size = _finite_number(payload, "size")
    if size < JOYSTICK_MIN_SIZE or size > JOYSTICK_MAX_SIZE:
        raise ValueError(
            f"size must be within [{JOYSTICK_MIN_SIZE:g}, {JOYSTICK_MAX_SIZE:g}]"
        )
    x_ratio = _finite_number(payload, "x_ratio")
    if x_ratio < 0.0 or x_ratio > 1.0:
        raise ValueError("x_ratio must be within [0, 1]")
    y_ratio = _finite_number(payload, "y_ratio")
    if y_ratio < 0.0 or y_ratio > 1.0:
        raise ValueError("y_ratio must be within [0, 1]")
    return {"size": size, "x_ratio": x_ratio, "y_ratio": y_ratio}


def _saved_number(value: Any, minimum: float, maximum: float, default: float) -> float:
    """Return a finite in-range saved value, else ``default``."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return default
    number = float(value)
    if not math.isfinite(number) or number < minimum or number > maximum:
        return default
    return number


def saved_joystick_preferences(payload: Any) -> dict[str, float]:
    """Return saved joystick settings, falling back per field when unusable.

    Invalid or partially missing saved UI data must never invalidate a valid
    selected map, so every field is repaired independently.
    """
    defaults = default_joystick_preferences()
    if not isinstance(payload, dict):
        return defaults
    return {
        "size": _saved_number(
            payload.get("size"),
            JOYSTICK_MIN_SIZE,
            JOYSTICK_MAX_SIZE,
            defaults["size"],
        ),
        "x_ratio": _saved_number(
            payload.get("x_ratio"), 0.0, 1.0, defaults["x_ratio"]
        ),
        "y_ratio": _saved_number(
            payload.get("y_ratio"), 0.0, 1.0, defaults["y_ratio"]
        ),
    }


class SelectionStore:
    """JSON-backed store for the selected map and joystick preferences."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)

    def _read_payload(self) -> dict[str, Any]:
        """Return the saved object, or ``{}`` when absent or unusable."""
        try:
            content = self.path.read_text(encoding="utf-8")
        except FileNotFoundError:
            return {}
        except (OSError, UnicodeDecodeError) as exc:
            LOGGER.warning("invalid saved map selection at %s: %s", self.path, exc)
            return {}
        try:
            payload = json.loads(content)
        except ValueError as exc:
            LOGGER.warning("invalid saved map selection at %s: %s", self.path, exc)
            return {}
        if not isinstance(payload, dict):
            LOGGER.warning(
                "invalid saved map selection at %s: expected an object", self.path
            )
            return {}
        return payload

    def load(self) -> str | None:
        """Return the stored map name, or ``None`` when absent or unusable."""
        try:
            return validate_map_name(self._read_payload().get("name"))
        except ValueError as exc:
            LOGGER.warning("invalid saved map selection at %s: %s", self.path, exc)
            return None

    def save(self, name: str) -> None:
        """Persist one map name, preserving every other saved key."""
        validate_map_name(name)
        with _STORE_LOCK:
            payload = self._read_payload()
            payload["name"] = name
            _atomic_write_text(self.path, json.dumps(payload) + "\n")

    def load_joystick_preferences(self) -> dict[str, float]:
        """Return validated joystick settings, falling back to defaults."""
        payload = self._read_payload()
        ui = payload.get("ui")
        joystick = ui.get("joystick") if isinstance(ui, dict) else None
        return saved_joystick_preferences(joystick)

    def save_joystick_preferences(self, preferences: Any) -> dict[str, float]:
        """Validate and store joystick settings without touching the map name."""
        validated = validate_joystick_preferences(preferences)
        with _STORE_LOCK:
            payload = self._read_payload()
            ui = payload.get("ui")
            if not isinstance(ui, dict):
                ui = {}
            ui["joystick"] = validated
            payload["ui"] = ui
            _atomic_write_text(self.path, json.dumps(payload) + "\n")
        return validated
