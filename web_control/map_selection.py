"""Persistent map selection for web-managed launches.

The selection store keeps only a map directory name; the catalog revalidates
that name against the map root on every use.
"""

import json
import logging
from pathlib import Path

from web_control.atomic_io import atomic_write_text
from web_control.map_catalog import validate_map_name

SELECTION_FILENAME = "selected_map.json"
LOGGER = logging.getLogger(__name__)


def default_selection_path(home: Path | None = None) -> Path:
    """Return the persisted selection path under ``home`` (default: runtime home)."""
    return _runtime_home(home) / ".config" / "web_control" / SELECTION_FILENAME


def _runtime_home(home: Path | None) -> Path:
    return Path(home) if home is not None else Path.home()


def _atomic_write_text(path: Path, text: str) -> None:
    """Write ``text`` to ``path`` by replacing a fully written sibling file."""
    atomic_write_text(path, text)


class SelectionStore:
    """JSON-backed store holding only the selected map directory name."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)

    def load(self) -> str | None:
        """Return the stored map name, or ``None`` when absent or unusable."""
        try:
            content = self.path.read_text(encoding="utf-8")
        except FileNotFoundError:
            return None
        except (OSError, UnicodeDecodeError) as exc:
            LOGGER.warning("invalid saved map selection at %s: %s", self.path, exc)
            return None
        try:
            payload = json.loads(content)
        except ValueError as exc:
            LOGGER.warning("invalid saved map selection at %s: %s", self.path, exc)
            return None
        if not isinstance(payload, dict):
            LOGGER.warning("invalid saved map selection at %s: expected an object", self.path)
            return None
        try:
            return validate_map_name(payload.get("name"))
        except ValueError as exc:
            LOGGER.warning("invalid saved map selection at %s: %s", self.path, exc)
            return None

    def save(self, name: str) -> None:
        """Persist one map directory name, replacing any previous selection."""
        validate_map_name(name)
        _atomic_write_text(self.path, json.dumps({"name": name}) + "\n")
