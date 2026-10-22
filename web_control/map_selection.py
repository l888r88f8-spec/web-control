"""Persistent map selection, loading mode, auto-relocalization, and UI preferences.

The selection store owns the selected map directory name, the next-start
loading mode, the next-start automatic-relocalization preference, the validated
joystick settings saved under ``ui.joystick``, and the shared topic-logging
choice saved under ``ui.topic_logging_enabled`` in the same JSON file.  Every
write is a locked read-modify-write over the existing file with atomic
replacement, so these updates cannot erase each other and unknown top-level and
nested keys are preserved.
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

# The saved loading preference.  ``auto`` is the historical behaviour: use a
# validated tiled map when one exists and otherwise the full GlobalMap.pcd.
# Manual ``tiled`` never falls back silently; ``global`` always uses the full
# GlobalMap.pcd even when a valid tiled map exists.
MAP_MODES = ("auto", "tiled", "global")
DEFAULT_MAP_MODE = "auto"

# Topic/IMU diagnostics logging is enabled for backward compatibility.  The
# choice is shared by every browser and stored beside the UI preferences.
DEFAULT_TOPIC_LOGGING_ENABLED = True

# Automatic relocalization is the historical launch behaviour for maps that
# contain a Scan Context database.  The preference is persisted separately from
# ``mode`` and only takes effect when the selected map actually has sc_db.bin.
DEFAULT_AUTO_RELOCALIZATION = True

JOYSTICK_DEFAULT_SIZE = 180.0
JOYSTICK_MIN_SIZE = 140.0
JOYSTICK_MAX_SIZE = 260.0
# The two independent mapping sticks.  ``x``/``y`` are stable offsets in map
# viewport pixels; ``None`` means "use the browser's role default" because the
# correct offset depends on the live viewport, not on the backend.
JOYSTICK_ROLES = ("throttle", "steering")

# Serializes read-modify-write of the shared file across store instances and
# HTTP worker threads so the name update and the ui.joystick update cannot
# overwrite each other.
_STORE_LOCK = threading.RLock()


def default_selection_path(home: Path | None = None) -> Path:
    """Return the persisted selection path under ``home`` (default: runtime home)."""
    return _runtime_home(home) / ".config" / "web_control" / SELECTION_FILENAME


def validate_map_mode(mode: Any) -> str:
    """Return a mode explicitly supplied by a client, rejecting every other type."""
    if not isinstance(mode, str) or mode not in MAP_MODES:
        raise ValueError("mode must be one of: auto, tiled, global")
    return mode


def saved_map_mode(mode: Any) -> str:
    """Return a usable persisted mode, defaulting unknown values to ``auto``."""
    try:
        return validate_map_mode(mode)
    except ValueError:
        return DEFAULT_MAP_MODE


def validate_topic_logging_enabled(payload: Any) -> bool:
    """Return a strictly supplied JSON boolean for the logging switch.

    Only ``{"enabled": true|false}`` is accepted: strings, numbers, null,
    missing keys, extra keys, and non-object bodies are rejected so a typo can
    never silently turn logging off.
    """
    if not isinstance(payload, dict):
        raise ValueError("topic logging payload must be an object")
    if set(payload) != {"enabled"}:
        raise ValueError("topic logging payload must contain only enabled")
    enabled = payload["enabled"]
    if type(enabled) is not bool:
        raise ValueError("enabled must be a boolean")
    return enabled


def saved_topic_logging_enabled(value: Any) -> bool:
    """Return a saved boolean, defaulting old/invalid files to enabled."""
    return value if type(value) is bool else DEFAULT_TOPIC_LOGGING_ENABLED


def validate_auto_relocalization(value: Any) -> bool:
    """Return a strictly supplied JSON boolean for auto relocalization.

    Strings, numbers (including ``0``/``1``), null, and missing values are
    rejected so a client typo can never silently change the launch contract.
    """
    if type(value) is not bool:
        raise ValueError("auto_relocalization must be a boolean")
    return value


def saved_auto_relocalization(value: Any) -> bool:
    """Return a saved boolean, defaulting old/invalid files to enabled."""
    return value if type(value) is bool else DEFAULT_AUTO_RELOCALIZATION


def _runtime_home(home: Path | None) -> Path:
    return Path(home) if home is not None else Path.home()


def _atomic_write_text(path: Path, text: str) -> None:
    """Write ``text`` to ``path`` by replacing a fully written sibling file."""
    atomic_write_text(path, text)


def default_stick_preferences() -> dict[str, Any]:
    """Return one stick's fallback size and role-default pixel offsets.

    A null ``x``/``y`` tells the browser to use its role default position
    (throttle in the lower-left, steering in the lower-right).  The correct
    pixel offset depends on the live map viewport, so it is not stored here.
    """
    return {"size": JOYSTICK_DEFAULT_SIZE, "x": None, "y": None}


def default_joystick_preferences() -> dict[str, Any]:
    """Return the fallback layout for both mapping sticks."""
    return {role: default_stick_preferences() for role in JOYSTICK_ROLES}


def _finite_number(payload: dict[str, Any], key: str) -> float:
    """Return ``payload[key]`` as a finite float, rejecting bools and strings."""
    if key not in payload:
        raise ValueError(f"missing {key}")
    value = payload[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a number")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{key} must be finite") from exc
    if not math.isfinite(number):
        raise ValueError(f"{key} must be finite")
    return number


def _stick_size(payload: dict[str, Any]) -> float:
    """Return a validated clampable stick diameter."""
    size = _finite_number(payload, "size")
    if size < JOYSTICK_MIN_SIZE or size > JOYSTICK_MAX_SIZE:
        raise ValueError(
            f"size must be within [{JOYSTICK_MIN_SIZE:g}, {JOYSTICK_MAX_SIZE:g}]"
        )
    return size


def _offset(payload: dict[str, Any], key: str) -> float:
    """Return a finite nonnegative viewport-pixel offset."""
    value = _finite_number(payload, key)
    if value < 0.0:
        raise ValueError(f"{key} must be nonnegative")
    return value


def validate_stick_preferences(payload: Any) -> dict[str, float]:
    """Return one complete validated stick setting for a preference write."""
    if not isinstance(payload, dict):
        raise ValueError("stick preferences must be an object")
    return {
        "size": _stick_size(payload),
        "x": _offset(payload, "x"),
        "y": _offset(payload, "y"),
    }


def validate_joystick_preferences(payload: Any) -> dict[str, dict[str, float]]:
    """Return a complete validated two-stick setting for a preference write."""
    if not isinstance(payload, dict):
        raise ValueError("joystick preferences must be an object")
    validated: dict[str, dict[str, float]] = {}
    for role in JOYSTICK_ROLES:
        if role not in payload:
            raise ValueError(f"missing {role}")
        validated[role] = validate_stick_preferences(payload[role])
    return validated


def _saved_number(value: Any, minimum: float, maximum: float, default: float) -> float:
    """Return a finite in-range saved value, else ``default``."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return default
    try:
        number = float(value)
    except OverflowError:
        return default
    if not math.isfinite(number) or number < minimum or number > maximum:
        return default
    return number


def _saved_offset(value: Any) -> float | None:
    """Return a finite nonnegative saved offset, else ``None`` (role default)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        number = float(value)
    except OverflowError:
        return None
    if not math.isfinite(number) or number < 0.0:
        return None
    return number


def _saved_stick_preferences(payload: Any) -> dict[str, Any]:
    """Return one saved stick setting, repairing each field independently."""
    defaults = default_stick_preferences()
    if not isinstance(payload, dict):
        return defaults
    return {
        "size": _saved_number(
            payload.get("size"),
            JOYSTICK_MIN_SIZE,
            JOYSTICK_MAX_SIZE,
            defaults["size"],
        ),
        "x": _saved_offset(payload.get("x")),
        "y": _saved_offset(payload.get("y")),
    }


def saved_joystick_preferences(payload: Any) -> dict[str, Any]:
    """Return saved two-stick settings, falling back per field when unusable.

    Invalid or partially missing saved UI data must never invalidate a valid
    selected map, so every field is repaired independently.  The legacy
    single-joystick form is migrated by reusing its diameter for both sticks
    and letting each stick fall back to its role default position.
    """
    defaults = default_joystick_preferences()
    if not isinstance(payload, dict):
        return defaults
    if "throttle" in payload or "steering" in payload:
        return {
            role: _saved_stick_preferences(payload.get(role))
            for role in JOYSTICK_ROLES
        }
    legacy_size = _saved_number(
        payload.get("size"),
        JOYSTICK_MIN_SIZE,
        JOYSTICK_MAX_SIZE,
        JOYSTICK_DEFAULT_SIZE,
    )
    return {
        role: {"size": legacy_size, "x": None, "y": None}
        for role in JOYSTICK_ROLES
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

    def load_mode(self) -> str:
        """Return the saved loading mode, sanitizing missing/invalid values to auto."""
        return saved_map_mode(self._read_payload().get("mode"))

    def load_auto_relocalization(self) -> bool:
        """Return the saved auto-relocalization preference, defaulting to true."""
        return saved_auto_relocalization(
            self._read_payload().get("auto_relocalization")
        )

    def save(
        self,
        name: str,
        mode: str | None = None,
        auto_relocalization: bool | None = None,
    ) -> None:
        """Persist one map name and optionally mode/auto-relocalization.

        ``mode=None`` keeps the old name-only contract: the saved mode (even an
        unknown one) is preserved so a name update never rewrites a preference
        it was not asked to change.  The same holds for the auto-relocalization
        preference.  Explicit values are validated before the locked
        read-modify-write.
        """
        validate_map_name(name)
        if mode is not None:
            validate_map_mode(mode)
        if auto_relocalization is not None:
            validate_auto_relocalization(auto_relocalization)
        with _STORE_LOCK:
            payload = self._read_payload()
            payload["name"] = name
            if mode is not None:
                payload["mode"] = mode
            if auto_relocalization is not None:
                payload["auto_relocalization"] = auto_relocalization
            _atomic_write_text(self.path, json.dumps(payload) + "\n")

    def save_auto_relocalization(self, enabled: Any) -> bool:
        """Atomically merge the auto-relocalization flag without touching name/mode."""
        validated = validate_auto_relocalization(enabled)
        with _STORE_LOCK:
            payload = self._read_payload()
            payload["auto_relocalization"] = validated
            _atomic_write_text(self.path, json.dumps(payload) + "\n")
        return validated

    def load_joystick_preferences(self) -> dict[str, Any]:
        """Return validated two-stick settings, falling back to defaults."""
        payload = self._read_payload()
        ui = payload.get("ui")
        joystick = ui.get("joystick") if isinstance(ui, dict) else None
        return saved_joystick_preferences(joystick)

    def save_joystick_preferences(self, preferences: Any) -> dict[str, Any]:
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

    def load_topic_logging_enabled(self) -> bool:
        """Return the saved topic-logging choice, defaulting to enabled."""
        payload = self._read_payload()
        ui = payload.get("ui")
        value = ui.get("topic_logging_enabled") if isinstance(ui, dict) else None
        return saved_topic_logging_enabled(value)

    def save_topic_logging_enabled(self, enabled: Any) -> bool:
        """Atomically merge the logging choice without touching other prefs."""
        if type(enabled) is not bool:
            raise ValueError("topic logging enabled state must be a boolean")
        with _STORE_LOCK:
            payload = self._read_payload()
            ui = payload.get("ui")
            if not isinstance(ui, dict):
                ui = {}
            ui["topic_logging_enabled"] = enabled
            payload["ui"] = ui
            _atomic_write_text(self.path, json.dumps(payload) + "\n")
        return enabled
