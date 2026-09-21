"""Persistent resumable navigation destination for pause and resume.

Only the schema written by this module is accepted back.  Invalid or unreadable
files are treated as absent so a corrupt record can never publish a stale pose.
"""

import json
import logging
import math
from pathlib import Path
from typing import Any

from web_control.atomic_io import atomic_write_text
from web_control.map_catalog import validate_map_name

NAVIGATION_FILENAME = "paused_navigation.json"
SCHEMA_VERSION = 1
PHASES = ("pausing", "paused", "resuming")
LOGGER = logging.getLogger(__name__)


def default_navigation_path(home: Path | None = None) -> Path:
    """Return the persisted navigation record path under ``home``."""
    base = Path(home) if home is not None else Path.home()
    return base / ".config" / "web_control" / NAVIGATION_FILENAME


def validate_navigation_record(payload: Any) -> dict[str, Any]:
    """Return a normalised record, rejecting anything a resume cannot trust."""
    if not isinstance(payload, dict):
        raise ValueError("navigation record must be an object")
    version = payload.get("schema_version")
    if isinstance(version, bool) or version != SCHEMA_VERSION:
        raise ValueError(f"unsupported schema_version {version!r}")
    phase = payload.get("phase")
    if phase not in PHASES:
        raise ValueError(f"unsupported phase {phase!r}")
    frame = payload.get("frame")
    if not isinstance(frame, str) or not frame.strip():
        raise ValueError("frame must be a non-empty string")
    map_name = validate_map_name(payload.get("map_name"))
    pose: dict[str, float] = {}
    for key in ("x", "y", "z", "yaw"):
        if key not in payload:
            raise ValueError(f"missing {key}")
        try:
            value = float(payload[key])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{key} must be numeric") from exc
        if not math.isfinite(value):
            raise ValueError(f"{key} must be finite")
        pose[key] = value
    saved_at = payload.get("saved_at")
    if (
        isinstance(saved_at, bool)
        or not isinstance(saved_at, (int, float))
        or not math.isfinite(float(saved_at))
    ):
        raise ValueError("saved_at must be a finite number")
    return {
        "schema_version": SCHEMA_VERSION,
        "phase": phase,
        "x": pose["x"],
        "y": pose["y"],
        "z": pose["z"],
        "yaw": pose["yaw"],
        "frame": frame,
        "map_name": map_name,
        "saved_at": float(saved_at),
    }


class NavigationStore:
    """JSON-backed store for one interrupted or paused navigation destination."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)

    def load(self) -> dict[str, Any] | None:
        """Return the stored record, or ``None`` when absent or unusable."""
        try:
            content = self.path.read_text(encoding="utf-8")
        except FileNotFoundError:
            return None
        except (OSError, UnicodeDecodeError) as exc:
            LOGGER.warning("invalid paused navigation record at %s: %s", self.path, exc)
            return None
        try:
            payload = json.loads(content)
        except ValueError as exc:
            LOGGER.warning("invalid paused navigation record at %s: %s", self.path, exc)
            return None
        try:
            return validate_navigation_record(payload)
        except ValueError as exc:
            LOGGER.warning("invalid paused navigation record at %s: %s", self.path, exc)
            return None

    def save(self, record: dict[str, Any]) -> None:
        """Persist one validated record, replacing any previous record."""
        validated = validate_navigation_record(record)
        atomic_write_text(self.path, json.dumps(validated) + "\n")

    def clear(self) -> None:
        """Remove the persisted record; a missing file is not an error."""
        try:
            self.path.unlink()
        except FileNotFoundError:
            return
