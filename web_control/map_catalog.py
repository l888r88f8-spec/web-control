"""Catalog of selectable map directories for web-managed launches.

Only direct child directories of one configured root are exposed. Every name,
directory, and artifact path is resolved against that root so a client-supplied
name can never point a launch at a file outside the map directory.
"""

import os
from pathlib import Path
from typing import Any

import yaml

GLOBAL_MAP_NAME = "GlobalMap.pcd"
GROUND_MAP_NAME = "GroundMap.pcd"
SCAN_CONTEXT_DB_NAME = "sc_db.bin"
TILED_DIRECTORY_NAME = "tiled"
TILED_MANIFEST_NAME = "manifest.yaml"
TILED_SCHEMA_VERSION = 1


def validate_map_name(name: Any) -> str:
    """Return ``name`` as a direct-child map name, rejecting anything path-like."""
    if not isinstance(name, str):
        raise ValueError("map name must be a string")
    if not name.strip():
        raise ValueError("map name must not be empty")
    if name in {".", ".."}:
        raise ValueError(f"map name must not be {name!r}")
    if "/" in name or "\\" in name or "\x00" in name or name != Path(name).name:
        raise ValueError(f"map name must be a single path component: {name!r}")
    return name


class MapCatalog:
    """Read-only view of the selectable maps directly below ``root``."""

    def __init__(self, root: Path) -> None:
        self.root = Path(root).expanduser().resolve()

    def list_maps(self) -> list[dict[str, Any]]:
        """Describe every selectable map in stable name order."""
        if not self.root.is_dir():
            return []
        try:
            children = sorted(self.root.iterdir(), key=lambda child: child.name)
        except OSError:
            return []
        entries = []
        for child in children:
            try:
                entries.append(self._describe(child.name))
            except ValueError:
                continue
        return entries

    def get(self, name: Any) -> dict[str, Any]:
        """Describe one map, rejecting invalid names and escaping paths."""
        validate_map_name(name)
        return self._describe(name)

    def _describe(self, name: str) -> dict[str, Any]:
        child = self.root / name
        resolved_child = self._resolve_within(child, self.root, f"map {name!r}")
        if not child.is_dir():
            raise ValueError(f"map {name!r} is not a directory")
        global_map = self._required_artifact(child, resolved_child, GLOBAL_MAP_NAME)
        ground_map = self._required_artifact(child, resolved_child, GROUND_MAP_NAME)
        tiled_path, warning = self._tiled_entry(child, resolved_child)
        scan_context = self._optional_artifact(
            child, resolved_child, SCAN_CONTEXT_DB_NAME
        )
        return {
            "name": name,
            "global_map_path": str(global_map),
            "ground_map_path": str(ground_map),
            "tiled_path": None if tiled_path is None else str(tiled_path),
            "has_scan_context": scan_context is not None,
            "warning": warning,
        }

    @staticmethod
    def _resolve_within(path: Path, root: Path, label: str) -> Path:
        """Resolve ``path`` and require the result to stay inside ``root``."""
        try:
            resolved = path.resolve()
        except (OSError, RuntimeError) as exc:
            raise ValueError(f"unable to resolve {label}: {exc}") from exc
        if resolved != root and root not in resolved.parents:
            raise ValueError(f"{label} escapes the map root")
        return resolved

    def _required_artifact(
        self, child: Path, resolved_child: Path, filename: str
    ) -> Path:
        resolved = self._resolve_within(child / filename, resolved_child, filename)
        if not resolved.is_file() or not os.access(resolved, os.R_OK):
            raise ValueError(
                f"{filename} is not a readable file for map {child.name!r}"
            )
        return resolved

    def _optional_artifact(
        self, child: Path, resolved_child: Path, filename: str
    ) -> Path | None:
        try:
            resolved = self._resolve_within(child / filename, resolved_child, filename)
        except ValueError:
            return None
        if resolved.is_file() and os.access(resolved, os.R_OK):
            return resolved
        return None

    def _tiled_entry(
        self, child: Path, resolved_child: Path
    ) -> tuple[Path | None, str | None]:
        """Return the usable ``tiled/`` directory, or a fallback warning."""
        tiled_dir = child / TILED_DIRECTORY_NAME
        if not tiled_dir.is_symlink() and not tiled_dir.exists():
            return None, None
        try:
            resolved_tiled = self._resolve_within(
                tiled_dir, resolved_child, TILED_DIRECTORY_NAME
            )
        except ValueError as exc:
            return None, f"tiled directory is unusable: {exc}"
        if not resolved_tiled.is_dir():
            return None, "tiled path is not a directory"
        try:
            manifest = self._resolve_within(
                resolved_tiled / TILED_MANIFEST_NAME,
                resolved_tiled,
                TILED_MANIFEST_NAME,
            )
        except ValueError as exc:
            return None, f"tiled manifest is unusable: {exc}"
        if not manifest.is_file() or not os.access(manifest, os.R_OK):
            return None, "tiled manifest is missing"
        try:
            document = yaml.safe_load(manifest.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
            return None, f"tiled manifest is unreadable: {exc}"
        try:
            self._validate_manifest(document, resolved_tiled)
        except ValueError as exc:
            return None, f"tiled manifest is invalid: {exc}"
        return resolved_tiled, None

    @staticmethod
    def _validate_manifest(document: Any, tiled_root: Path) -> None:
        """Require a schema version 1 manifest whose tile files stay in ``tiled/``."""
        if not isinstance(document, dict):
            raise ValueError("manifest must be a mapping")
        version = document.get("schema_version")
        if isinstance(version, bool) or version != TILED_SCHEMA_VERSION:
            raise ValueError(f"unsupported schema_version {version!r}")
        tiles = document.get("tiles")
        if not isinstance(tiles, dict):
            raise ValueError("manifest must declare tiles")
        references: list[Any] = []
        for kind in ("global", "ground"):
            entries = tiles.get(kind)
            if not isinstance(entries, list):
                raise ValueError(f"tiles.{kind} must be a list")
            references.extend(entries)
        overview = document.get("overview")
        if overview is not None:
            if not isinstance(overview, dict):
                raise ValueError("overview must be a mapping")
            references.append(overview)
        for reference in references:
            if not isinstance(reference, dict):
                raise ValueError("tile entry must be a mapping")
            relative = reference.get("path")
            if not isinstance(relative, str) or not relative:
                raise ValueError("tile entry is missing a path")
            tile = Path(relative)
            if tile.is_absolute():
                raise ValueError(f"tile path must be relative: {relative!r}")
            try:
                resolved = (tiled_root / tile).resolve()
            except (OSError, RuntimeError) as exc:
                raise ValueError(
                    f"unable to resolve tile path {relative!r}: {exc}"
                ) from exc
            if resolved != tiled_root and tiled_root not in resolved.parents:
                raise ValueError(
                    f"tile path escapes the tiled directory: {relative!r}"
                )
            if not resolved.is_file():
                raise ValueError(f"tile file is missing: {relative!r}")
