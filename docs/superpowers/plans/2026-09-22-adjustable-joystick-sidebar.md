# Adjustable Joystick and Collapsible Sidebar Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the operator resize and reposition the Mapping joystick, persist those settings with the selected map, and smoothly collapse the right information sidebar at the same DOM button while keeping the Three.js canvas and HUD correct.

**Architecture:** `web_control/map_selection.py` becomes a locked compatible JSON store whose `save(name)` only rewrites `name` and whose new `save_joystick_preferences()` only rewrites `ui.joystick`, preserving unknown keys. `WebControlNode` exposes `GET/POST /api/ui/preferences` that delegate to that store and reuse the existing HTTP error mapping. In the browser, one `// --- Mapping joystick` block keeps the existing drive lifecycle and gains a `teleop.layoutMode` guard, a second `// --- Joystick layout` block owns layout-mode pointer gestures, viewport clamping, persistence calls, and HUD avoidance, and a third `// --- Sidebar collapse` block owns the sidebar's in-memory collapsed state, `aria-expanded`/`aria-controls`, CSS class, and `ResizeObserver` renderer resize.

**Tech Stack:** Python 3.10 / pytest 6.2.5, ROS 2 Python (`rclpy`, `geometry_msgs`; not importable in the test sandbox), `http.server.ThreadingHTTPServer`, browser JavaScript ES module tested with `node -e`, HTML/CSS.

**Locked interface decisions (used consistently by every task):**

- `GET /api/ui/preferences` returns `{"ok": true, "joystick": {"size": 180, "x_ratio": 0.05, "y_ratio": 0.75}, "stamp": <float>}`.
- `POST /api/ui/preferences` takes the complete joystick object as the whole request body, `{"size": <number>, "x_ratio": <number>, "y_ratio": <number>}`, and returns `{"ok": true, "message": "ui preferences saved", "stamp": <float>, "joystick": <validated object>}`.
- Store method names are `load_joystick_preferences()` / `save_joystick_preferences(preferences)`; validators are `validate_joystick_preferences` (strict, for POST) and `saved_joystick_preferences` (lenient, for reads).
- Defaults: size `180`, min `140`, max `260`, `x_ratio` `0.05`, `y_ratio` `0.75`. `x_ratio`/`y_ratio` are the joystick container's normalized center inside `#view3d`.
- HUD tie-break is deterministic: compute the overlap area of the joystick container with the lower-left and lower-right HUD candidate rectangles and use the lower-right corner only when its overlap is strictly smaller (`rightOverlap < leftOverlap`).

**Reading line numbers:** every `(currently ...)` line number refers to the
repository state at the moment this plan was written, before any task ran.
Later tasks shift those lines. The quoted rule, function, or marker text is the
authoritative anchor; use the line numbers only to orient.

**Constraints honored by this plan:**

- Current branch is `main`; implement directly, no worktree is created here.
- TDD and frequent focused commits; every commit lists its files explicitly.
- **Do not stage, edit, or commit `config/web_control.yaml`.** The operator has an unrelated uncommitted edit there (`teleop_max_linear_speed: 1.5` → `1.0`). This plan adds no new ROS parameters, so that file is never a commit target.
- Do not run `colcon`; the user compiles. Verification only runs pytest, `node --check`, `py_compile`, and `git diff --check`.
- Reuse the existing atomic write helper and `validate_map_name`; do not add ROS message dependencies or change teleop speed mapping, `/cmd_vel` sequencing, map selection semantics, launch files, or `package.xml`/`setup.py`.
- The sidebar collapsed state is browser-memory only: no backend write, no `sessionStorage`, no `localStorage`.

---

## File Structure

- Modify: `web_control/map_selection.py` — locked read-modify-write `save`, joystick validate/normalize helpers, `load_joystick_preferences`, `save_joystick_preferences`.
- Create: `test/test_map_selection.py` — focused store unit tests (legacy compatibility, preservation, validation, atomicity, concurrency).
- Modify: `web_control/web_control_node.py` — `ui_preferences_payload`, `save_ui_preferences`, and the two GET/POST route branches.
- Modify: `test/test_core.py` — extend the `_selection_node` patch list, add preference route tests, adjust the joystick overlay assertion, and add the adjustable-joystick and sidebar Node harnesses.
- Modify: `static/index.html` — joystick settings/move/resize markup, `id="controlPanel"`, and the single sidebar toggle button.
- Modify: `static/app.js` — `teleop.layoutMode` guard, the `// --- Joystick layout` block, the `// --- Sidebar collapse` block, plus `setTab`/`attachEvents`/`start` wiring.
- Modify: `static/styles.css` — `--joystick-size` scaling, layout-mode handles/settings, HUD corner class, grid `--panel-width` transition, panel fade, sidebar toggle.
- Modify: `README.md` — user documentation for adjustable joystick layout, preferences API, and sidebar collapse.

---

### Task 1: Locked `SelectionStore` read-modify-write and joystick preference validation

**Files:**
- Modify: `web_control/map_selection.py`
- Create: `test/test_map_selection.py`

- [ ] **Step 1: Write the failing store tests**

Create `test/test_map_selection.py` with exactly:

```python
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
```

- [ ] **Step 2: Run the focused tests to verify RED**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_map_selection.py
```

Expected: collection fails with `ImportError: cannot import name 'default_joystick_preferences' from 'web_control.map_selection'`.

- [ ] **Step 3: Write the store implementation**

Replace the whole of `web_control/map_selection.py` with exactly:

```python
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
```

- [ ] **Step 4: Run the focused tests to verify GREEN**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_map_selection.py
```

Expected: `21 passed`.

- [ ] **Step 5: Run the existing selection regressions to prove legacy compatibility**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_core.py -k "selection_store or maps_handler or selection_default_path or relocalization_parameter_cache"
```

Expected: all existing selection tests pass, including
`test_selection_store_replaces_selection_file_atomically` (its legacy temp file
is still exactly `{"name": "site_b"}`).

- [ ] **Step 6: Commit**

```bash
git add web_control/map_selection.py test/test_map_selection.py
git commit -m "feat: persist joystick preferences with locked selection writes"
```

Do not add `config/web_control.yaml` to this or any later commit.

---

### Task 2: Preference GET/POST HTTP routes

**Files:**
- Modify: `web_control/web_control_node.py`
- Test: `test/test_core.py`

- [ ] **Step 1: Write the failing route tests**

In `test/test_core.py`, inside `_selection_node` (currently around lines 2795–2871), extend the method patch list so the new node methods are attached to the ROS-free stub. Replace:

```python
    for name in (
        "maps_payload",
        "select_map",
        "require_valid_selection",
        "managed_start_command",
        "start_managed_target",
    ):
```

with:

```python
    for name in (
        "maps_payload",
        "select_map",
        "require_valid_selection",
        "managed_start_command",
        "start_managed_target",
        "ui_preferences_payload",
        "save_ui_preferences",
    ):
```

Then add these tests immediately after
`test_maps_handler_select_allowed_while_only_mapping_runs` (currently ends at
line 3036, before `test_managed_start_command_uses_tiled_map_without_writing_cache`):

```python
def test_ui_preferences_get_returns_defaults_and_saved_values(tmp_path):
    node = _selection_node(tmp_path)

    status, body = _serve_and_request(node, "/api/ui/preferences")

    assert status == 200
    assert body["ok"] is True
    assert body["joystick"] == {"size": 180, "x_ratio": 0.05, "y_ratio": 0.75}

    node.map_selection.save_joystick_preferences(
        {"size": 230, "x_ratio": 0.2, "y_ratio": 0.6}
    )

    status, body = _serve_and_request(node, "/api/ui/preferences")

    assert status == 200
    assert body["joystick"] == {"size": 230, "x_ratio": 0.2, "y_ratio": 0.6}


def test_ui_preferences_post_persists_complete_object_and_preserves_selection(tmp_path):
    _make_map_dir(tmp_path / "map", "site_a")
    node = _selection_node(tmp_path, selection="site_a")
    node.map_selection.path.write_text(
        json.dumps({"name": "site_a", "ui": {"theme": "dark"}, "extra": 1}) + "\n",
        encoding="utf-8",
    )

    status, body = _serve_and_request(
        node,
        "/api/ui/preferences",
        {"size": 210, "x_ratio": 0.15, "y_ratio": 0.65},
        "POST",
    )

    assert status == 200
    assert body["ok"] is True
    assert body["joystick"] == {"size": 210, "x_ratio": 0.15, "y_ratio": 0.65}
    assert node.map_selection.load() == "site_a"
    payload = json.loads(node.map_selection.path.read_text(encoding="utf-8"))
    assert payload == {
        "name": "site_a",
        "ui": {
            "theme": "dark",
            "joystick": {"size": 210, "x_ratio": 0.15, "y_ratio": 0.65},
        },
        "extra": 1,
    }


@pytest.mark.parametrize(
    "payload",
    [
        [],
        {},
        {"size": 139, "x_ratio": 0.5, "y_ratio": 0.5},
        {"size": 261, "x_ratio": 0.5, "y_ratio": 0.5},
        {"size": "210", "x_ratio": 0.5, "y_ratio": 0.5},
        {"size": float("nan"), "x_ratio": 0.5, "y_ratio": 0.5},
        {"size": 210, "x_ratio": -0.01, "y_ratio": 0.5},
        {"size": 210, "x_ratio": 0.5, "y_ratio": 1.01},
        {"size": 210, "x_ratio": 0.5},
    ],
)
def test_ui_preferences_post_rejects_invalid_payload_without_changing_file(
    tmp_path, payload
):
    node = _selection_node(tmp_path)
    node.map_selection.save_joystick_preferences(
        {"size": 200, "x_ratio": 0.2, "y_ratio": 0.8}
    )
    before = node.map_selection.path.read_text(encoding="utf-8")

    status, body = _serve_and_request(node, "/api/ui/preferences", payload, "POST")

    assert status == 400
    assert body["ok"] is False
    assert node.map_selection.path.read_text(encoding="utf-8") == before
    assert node.map_selection.load_joystick_preferences() == {
        "size": 200,
        "x_ratio": 0.2,
        "y_ratio": 0.8,
    }
```

- [ ] **Step 2: Run the new tests to verify RED**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_core.py -k "ui_preferences"
```

Expected: `_selection_node` raises `AttributeError: type object 'WebControlNode' has no attribute 'ui_preferences_payload'`, so all 11 tests error/fail.

- [ ] **Step 3: Add the node methods**

In `web_control/web_control_node.py`, add these two methods to `WebControlNode`
immediately after `maps_payload` and before `select_map` (currently line 827):

```python
    def ui_preferences_payload(self) -> dict[str, Any]:
        """Return the validated joystick layout shared by every browser."""
        return {
            "ok": True,
            "joystick": self.map_selection.load_joystick_preferences(),
            "stamp": self.now_sec(),
        }

    def save_ui_preferences(self, payload: Any) -> dict[str, Any]:
        """Validate and atomically merge a complete joystick setting."""
        validated = self.map_selection.save_joystick_preferences(payload)
        response = success_response("ui preferences saved", self.now_sec())
        response["joystick"] = validated
        return response
```

A `ValueError` from `save_joystick_preferences` is raised unchanged so the
existing `except (json.JSONDecodeError, ValueError)` clause in `do_POST`
returns HTTP 400 and the store leaves the last valid file intact.

- [ ] **Step 4: Add the GET route**

In `do_GET`, add this branch immediately after the
`elif parsed.path == "/api/maps":` branch (currently line 1685):

```python
                    elif parsed.path == "/api/ui/preferences":
                        self.send_json(node.ui_preferences_payload())
```

- [ ] **Step 5: Add the POST route**

In `do_POST`, add this branch immediately after the
`elif self.path == "/api/maps/select":` branch (currently line 1735):

```python
                    elif self.path == "/api/ui/preferences":
                        self.send_json(node.save_ui_preferences(payload))
```

- [ ] **Step 6: Run the new tests to verify GREEN**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_core.py -k "ui_preferences"
```

Expected: `11 passed`.

- [ ] **Step 7: Commit**

```bash
git add web_control/web_control_node.py test/test_core.py
git commit -m "feat: add UI preference GET and POST routes"
```

Do not stage `config/web_control.yaml`.

---

### Task 3: Adjustable joystick layout mode, persistence, clamping, and HUD avoidance

**Files:**
- Modify: `static/index.html`
- Modify: `static/app.js`
- Modify: `static/styles.css`
- Test: `test/test_core.py`

- [ ] **Step 1: Update the existing overlay assertion and add the failing layout test and Node harness**

In `test/test_core.py`, in
`test_mapping_joystick_is_a_mapping_only_sibling_overlay` (currently lines
5205–5222), replace:

```python
    assert ".joystick {" in styles_css
    assert "position: absolute;" in styles_css
    assert ".stage.mapping-active .hud" in styles_css
```

with:

```python
    assert ".joystick {" in styles_css
    assert "position: absolute;" in styles_css
    assert "--joystick-size" in styles_css
    assert ".stage.hud-lower-right .hud" in styles_css
    assert 'id="joystickSettings"' in viewport
    assert 'id="joystickMoveHandle"' in viewport
    assert 'id="joystickResizeHandle"' in viewport
```

Then add this layout test and Node harness immediately after
`test_mapping_joystick_pointer_lifecycle_and_throttling_execute_under_node`
(currently ends at line 5539, the end of the file):

```python
def test_mapping_joystick_exposes_adjustable_layout_controls():
    index_html, app_js, styles_css = _frontend_sources()

    assert 'id="joystickMoveHandle"' in index_html
    assert 'id="joystickResizeHandle"' in index_html
    assert 'id="joystickSettings"' in index_html
    assert 'id="joystickDone"' in index_html
    assert 'id="joystickSettingsError"' in index_html
    assert 'aria-label="Joystick settings"' in index_html

    assert "--joystick-size" in styles_css
    assert ".joystick.layout-mode" in styles_css
    assert ".stage.hud-lower-right .hud" in styles_css
    assert "function applyJoystickLayout()" in app_js
    assert "function clampedJoystickSize(value)" in app_js
    assert "function updateHudPlacement()" in app_js
    assert "function enterJoystickLayout()" in app_js
    assert "function exitJoystickLayout()" in app_js
    assert 'getJson("/api/ui/preferences")' in app_js
    assert 'postJson("/api/ui/preferences"' in app_js
    assert "teleop.layoutMode" in app_js


_JOYSTICK_LAYOUT_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const startA = source.indexOf("// --- Mapping joystick");
const endA = source.indexOf("// --- End mapping joystick");
const startB = source.indexOf("// --- Joystick layout");
const endB = source.indexOf("// --- End joystick layout");
if (startA < 0 || endA < 0 || startB < 0 || endB < 0) {
  throw new Error("joystick layout block not found");
}
const block = source.slice(startA, endA) + "\n" + source.slice(startB, endB);

function makeElement(id) {
  const classes = new Set();
  return {
    id,
    hidden: false,
    textContent: "",
    offsetWidth: 0,
    offsetHeight: 0,
    captured: [],
    attributes: {},
    style: {
      left: "0px",
      top: "0px",
      values: {},
      setProperty(name, value) { this.values[name] = value; },
    },
    classList: {
      toggle(name, on) {
        if (on === undefined) {
          if (classes.has(name)) classes.delete(name);
          else classes.add(name);
        } else if (on) {
          classes.add(name);
        } else {
          classes.delete(name);
        }
      },
      add(name) { classes.add(name); },
      remove(name) { classes.delete(name); },
      contains(name) { return classes.has(name); },
    },
    setAttribute(name, value) { this.attributes[name] = value; },
    getAttribute(name) { return name in this.attributes ? this.attributes[name] : null; },
    setPointerCapture(pointerId) { this.captured.push(pointerId); },
    addEventListener() {},
    getBoundingClientRect() {
      const left = Number.parseFloat(this.style.left) || 0;
      const top = Number.parseFloat(this.style.top) || 0;
      const width = this.offsetWidth || 0;
      const height = this.offsetHeight || 0;
      return { left, top, right: left + width, bottom: top + height, width, height };
    },
  };
}

const elements = {
  view3d: makeElement("view3d"),
  mappingJoystick: makeElement("mappingJoystick"),
  joystickBase: makeElement("joystickBase"),
  joystickKnob: makeElement("joystickKnob"),
  joystickStatus: makeElement("joystickStatus"),
  joystickMoveHandle: makeElement("joystickMoveHandle"),
  joystickResizeHandle: makeElement("joystickResizeHandle"),
  joystickSettings: makeElement("joystickSettings"),
  joystickDone: makeElement("joystickDone"),
  joystickSettingsError: makeElement("joystickSettingsError"),
  hud: makeElement("hud"),
};
elements.view3d.clientWidth = 400;
elements.view3d.clientHeight = 300;
elements.view3d.getBoundingClientRect = () => ({ left: 0, top: 0, right: 400, bottom: 300, width: 400, height: 300 });
elements.mappingJoystick.offsetWidth = 100;
elements.mappingJoystick.offsetHeight = 140;
elements.joystickBase.offsetWidth = 100;
elements.joystickBase.offsetHeight = 100;
elements.hud.offsetWidth = 60;
elements.hud.offsetHeight = 20;

const stage = makeElement("stage");
stage.getBoundingClientRect = () => ({ left: 0, top: 0, right: 400, bottom: 300, width: 400, height: 300 });

global.state = { view: "mapping" };
global.document = {
  hidden: false,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) { return selector === ".stage" ? stage : null; },
  addEventListener() {},
};
global.window = {
  addEventListener() {},
  setInterval() { return 1; },
  clearInterval() {},
};
global.viewerSessionId = () => "tab-session";

const teleopPosts = [];
const uiPosts = [];
let failUiSave = false;
global.postJson = async (url, payload) => {
  if (url === "/api/ui/preferences") {
    if (failUiSave) throw new Error("network down");
    uiPosts.push(payload);
    return { ok: true, joystick: payload };
  }
  teleopPosts.push({ url, payload });
  const active = payload.throttle !== 0 || payload.steering !== 0;
  return { ok: true, teleop: { active } };
};
global.getJson = async () => ({
  ok: true,
  joystick: { size: 220, x_ratio: 0.25, y_ratio: 0.5 },
});
global.fetchWithTimeout = async () => ({ ok: true });

// Run the two blocks in an isolated function scope so their declarations do
// not leak into this harness and clash with the names destructured below.
const joystickApi = new Function(block + `
return {
  teleop, joystickLayout, refreshJoystickPreferences, joystickPointerDown,
  enterJoystickLayout, exitJoystickLayout, beginJoystickMove, moveJoystickGesture,
  beginJoystickResize, resizeJoystickGesture, finishJoystickGesture,
  applyJoystickLayout, JOYSTICK_VIEWPORT_MARGIN,
};`)();
const {
  teleop, joystickLayout, refreshJoystickPreferences, joystickPointerDown,
  enterJoystickLayout, exitJoystickLayout, beginJoystickMove, moveJoystickGesture,
  beginJoystickResize, resizeJoystickGesture, finishJoystickGesture,
  applyJoystickLayout, JOYSTICK_VIEWPORT_MARGIN,
} = joystickApi;

function expect(condition, message) { if (!condition) throw new Error(message); }

function makeEvent(pointerId, clientX, clientY, currentTarget) {
  return {
    pointerId,
    pointerType: "mouse",
    clientX,
    clientY,
    currentTarget: currentTarget || elements.joystickBase,
    preventDefault() {},
    stopPropagation() {},
  };
}

(async () => {
  await refreshJoystickPreferences();
  expect(joystickLayout.size === 220, "preferences size applied");
  expect(joystickLayout.xRatio === 0.25, "preferences x ratio applied");
  expect(joystickLayout.yRatio === 0.5, "preferences y ratio applied");
  expect(
    elements.mappingJoystick.style.values["--joystick-size"] === "220px",
    "size css variable applied",
  );

  await joystickPointerDown(makeEvent(1, 30, 30));
  expect(teleop.active === true, "drive gesture active before layout mode");
  const teleopBeforeLayout = teleopPosts.length;

  await enterJoystickLayout();
  expect(teleop.layoutMode === true, "layout mode disables drive");
  expect(joystickLayout.mode === true, "layout mode state set");
  expect(elements.mappingJoystick.classList.contains("layout-mode"), "layout class applied");
  expect(teleopPosts.length === teleopBeforeLayout + 1, "entering layout posts one stop");
  expect(teleopPosts[teleopPosts.length - 1].payload.throttle === 0, "layout stop is zero throttle");
  expect(teleopPosts[teleopPosts.length - 1].payload.steering === 0, "layout stop is zero steering");

  const teleopAfterLayout = teleopPosts.length;
  await joystickPointerDown(makeEvent(2, 30, 30));
  expect(teleopPosts.length === teleopAfterLayout, "drive gesture disabled in layout mode");
  expect(teleop.pointerId === null, "disabled gesture does not capture the pointer");

  const uiBeforeResize = uiPosts.length;
  beginJoystickResize(makeEvent(3, 0, 0, elements.joystickResizeHandle));
  resizeJoystickGesture(makeEvent(3, 500, 500, elements.joystickResizeHandle));
  expect(joystickLayout.size === 260, "resize clamps to max");
  resizeJoystickGesture(makeEvent(3, -500, -500, elements.joystickResizeHandle));
  expect(joystickLayout.size === 140, "resize clamps to min");
  expect(uiPosts.length === uiBeforeResize, "resize preview does not write");
  await finishJoystickGesture(makeEvent(3, -500, -500, elements.joystickResizeHandle));
  expect(uiPosts.length === uiBeforeResize + 1, "resize release writes once");
  expect(uiPosts[uiPosts.length - 1].size === 140, "resize write carries the clamped size");

  const uiBeforeMove = uiPosts.length;
  beginJoystickMove(makeEvent(4, 0, 0, elements.joystickMoveHandle));
  moveJoystickGesture(makeEvent(4, -9999, -9999, elements.joystickMoveHandle));
  expect(
    Number.parseFloat(elements.mappingJoystick.style.left) === JOYSTICK_VIEWPORT_MARGIN,
    "move clamps the left edge",
  );
  expect(
    Number.parseFloat(elements.mappingJoystick.style.top) === JOYSTICK_VIEWPORT_MARGIN,
    "move clamps the top edge",
  );
  moveJoystickGesture(makeEvent(4, 9999, 9999, elements.joystickMoveHandle));
  expect(
    Number.parseFloat(elements.mappingJoystick.style.left) <= 400 - 100 - JOYSTICK_VIEWPORT_MARGIN,
    "move clamps the right edge",
  );
  expect(
    Number.parseFloat(elements.mappingJoystick.style.top) <= 300 - 140 - JOYSTICK_VIEWPORT_MARGIN,
    "move clamps the bottom edge",
  );
  expect(uiPosts.length === uiBeforeMove, "move preview does not write");
  await finishJoystickGesture(makeEvent(4, 9999, 9999, elements.joystickMoveHandle));
  expect(uiPosts.length === uiBeforeMove + 1, "move release writes once");
  const savedPosition = uiPosts[uiPosts.length - 1];
  expect(savedPosition.x_ratio >= 0 && savedPosition.x_ratio <= 1, "move x ratio in range");
  expect(savedPosition.y_ratio >= 0 && savedPosition.y_ratio <= 1, "move y ratio in range");

  joystickLayout.xRatio = 0;
  joystickLayout.yRatio = 1;
  applyJoystickLayout();
  expect(stage.classList.contains("hud-lower-right"), "HUD avoids the lower-left joystick");
  joystickLayout.xRatio = 1;
  joystickLayout.yRatio = 1;
  applyJoystickLayout();
  expect(!stage.classList.contains("hud-lower-right"), "HUD returns to the lower-left");

  failUiSave = true;
  beginJoystickResize(makeEvent(5, 0, 0, elements.joystickResizeHandle));
  resizeJoystickGesture(makeEvent(5, 20, 20, elements.joystickResizeHandle));
  await finishJoystickGesture(makeEvent(5, 20, 20, elements.joystickResizeHandle));
  expect(joystickLayout.mode === true, "failed save keeps layout mode");
  expect(
    typeof joystickLayout.saveError === "string" && joystickLayout.saveError.length > 0,
    "failed save records an error",
  );
  expect(elements.joystickSettingsError.hidden === false, "failed save shows the error");
  failUiSave = false;
  beginJoystickResize(makeEvent(6, 0, 0, elements.joystickResizeHandle));
  resizeJoystickGesture(makeEvent(6, 0, 0, elements.joystickResizeHandle));
  await finishJoystickGesture(makeEvent(6, 0, 0, elements.joystickResizeHandle));
  expect(joystickLayout.saveError === null, "retry clears the error");
  expect(elements.joystickSettingsError.hidden === true, "retry hides the error");

  exitJoystickLayout();
  expect(teleop.layoutMode === false, "exit restores drive control");
  expect(joystickLayout.mode === false, "exit clears layout mode");
  expect(!elements.mappingJoystick.classList.contains("layout-mode"), "exit removes the layout class");

  console.log("joystick layout ok");
})().catch((error) => { console.error(error); process.exit(1); });
"""


def test_mapping_joystick_layout_mode_persists_and_clamps_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _JOYSTICK_LAYOUT_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "joystick layout ok" in completed.stdout
```

- [ ] **Step 2: Run the new tests to verify RED**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_core.py -k "mapping_joystick"
```

Expected: `test_mapping_joystick_is_a_mapping_only_sibling_overlay` fails with
`ValueError: substring not found` (no `--joystick-size`), and
`test_mapping_joystick_exposes_adjustable_layout_controls` fails similarly;
`test_mapping_joystick_layout_mode_persists_and_clamps_under_node` fails with
`Error: joystick layout block not found` on stderr. The existing lifecycle
harness still passes.

- [ ] **Step 3: Replace the joystick markup**

In `static/index.html`, replace the current `#mappingJoystick` block (lines
20–27) with:

```html
          <div id="mappingJoystick" class="joystick" hidden>
            <button id="joystickMoveHandle" class="joystick-move-handle" type="button" aria-label="Move joystick"></button>
            <div id="joystickBase" class="joystick-base">
              <span class="joystick-axis joystick-axis-x"></span>
              <span class="joystick-axis joystick-axis-y"></span>
              <div id="joystickKnob" class="joystick-knob"></div>
              <button id="joystickResizeHandle" class="joystick-resize-handle" type="button" aria-label="Resize joystick"></button>
            </div>
            <div class="joystick-settings-controls">
              <button id="joystickSettings" class="joystick-settings-button" type="button" aria-label="Joystick settings" aria-pressed="false">Settings</button>
              <button id="joystickDone" class="joystick-done-button" type="button">Done</button>
            </div>
            <p id="joystickSettingsError" class="joystick-settings-error" role="status" aria-live="polite" hidden></p>
            <p id="joystickStatus" class="joystick-status" role="status" aria-live="polite">idle</p>
          </div>
```

The move handle stays a sibling above `#joystickBase`; the resize handle is a
child of the base. Both are `<button>` elements so they are focusable and
labelled, and both stop pointer propagation so events never reach the map
canvas.

- [ ] **Step 4: Disable drive gestures in layout mode**

In `static/app.js`, inside the existing `teleop` object in the
`// --- Mapping joystick` block (currently lines 1120–1130), add a
`layoutMode` field after `sendTimer: null,`:

```javascript
  sendTimer: null,
  layoutMode: false,
};
```

Then add the guard as the first line of `joystickPointerDown` (currently line 1256):

```javascript
async function joystickPointerDown(event) {
  if (teleop.layoutMode) return;
  if (teleop.pointerId !== null) return;
```

The existing `// --- Mapping joystick` block is otherwise unchanged; the
existing Node lifecycle harness slices exactly this block and must keep passing.

- [ ] **Step 5: Add the joystick layout block**

In `static/app.js`, insert this block immediately after the
`// --- End mapping joystick` line (currently line 1329) and before
`function maybeClearGoalFromActionStatus()`:

```javascript
// --- Joystick layout
const JOYSTICK_MIN_SIZE = 140;
const JOYSTICK_MAX_SIZE = 260;
const JOYSTICK_DEFAULT_SIZE = 180;
const JOYSTICK_DEFAULT_X_RATIO = 0.05;
const JOYSTICK_DEFAULT_Y_RATIO = 0.75;
const JOYSTICK_VIEWPORT_MARGIN = 12;
const HUD_CORNER_MARGIN = 12;

const joystickLayout = {
  size: JOYSTICK_DEFAULT_SIZE,
  xRatio: JOYSTICK_DEFAULT_X_RATIO,
  yRatio: JOYSTICK_DEFAULT_Y_RATIO,
  mode: false,
  gesture: null,
  saveError: null,
};

function clampNumber(value, minimum, maximum) {
  return Math.max(minimum, Math.min(maximum, value));
}

function finiteNumberOr(value, fallback) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function clampedJoystickSize(value) {
  return clampNumber(value, JOYSTICK_MIN_SIZE, JOYSTICK_MAX_SIZE);
}

function joystickViewport() {
  return document.getElementById("view3d");
}

function joystickContainer() {
  return document.getElementById("mappingJoystick");
}

function applyJoystickSize() {
  const joystick = joystickContainer();
  if (!joystick) return;
  joystick.style.setProperty("--joystick-size", `${joystickLayout.size}px`);
}

function joystickBounds(width, height, viewportWidth, viewportHeight) {
  return {
    maxLeft: Math.max(
      JOYSTICK_VIEWPORT_MARGIN,
      viewportWidth - width - JOYSTICK_VIEWPORT_MARGIN,
    ),
    maxTop: Math.max(
      JOYSTICK_VIEWPORT_MARGIN,
      viewportHeight - height - JOYSTICK_VIEWPORT_MARGIN,
    ),
  };
}

function applyJoystickLayout() {
  const joystick = joystickContainer();
  const viewport = joystickViewport();
  applyJoystickSize();
  if (!joystick || !viewport) return;
  const viewportWidth = viewport.clientWidth || 0;
  const viewportHeight = viewport.clientHeight || 0;
  if (!viewportWidth || !viewportHeight) {
    updateHudPlacement();
    return;
  }
  const width = joystick.offsetWidth || 0;
  const height = joystick.offsetHeight || 0;
  const bounds = joystickBounds(width, height, viewportWidth, viewportHeight);
  const left = clampNumber(
    joystickLayout.xRatio * viewportWidth - width / 2,
    JOYSTICK_VIEWPORT_MARGIN,
    bounds.maxLeft,
  );
  const top = clampNumber(
    joystickLayout.yRatio * viewportHeight - height / 2,
    JOYSTICK_VIEWPORT_MARGIN,
    bounds.maxTop,
  );
  joystick.style.left = `${left}px`;
  joystick.style.top = `${top}px`;
  updateHudPlacement();
}

function joystickRatiosFromPosition(left, top, width, height, viewport) {
  const viewportWidth = Math.max(1, viewport.clientWidth || 1);
  const viewportHeight = Math.max(1, viewport.clientHeight || 1);
  return {
    xRatio: clampNumber((left + width / 2) / viewportWidth, 0, 1),
    yRatio: clampNumber((top + height / 2) / viewportHeight, 0, 1),
  };
}

function beginJoystickMove(event) {
  if (!joystickLayout.mode || joystickLayout.gesture) return;
  event.preventDefault();
  event.stopPropagation();
  const joystick = joystickContainer();
  const viewport = joystickViewport();
  if (!joystick || !viewport) return;
  const joystickRect = joystick.getBoundingClientRect();
  const viewportRect = viewport.getBoundingClientRect();
  joystickLayout.gesture = {
    kind: "move",
    pointerId: event.pointerId,
    startX: event.clientX,
    startY: event.clientY,
    left: joystickRect.left - viewportRect.left,
    top: joystickRect.top - viewportRect.top,
    width: joystick.offsetWidth || 0,
    height: joystick.offsetHeight || 0,
  };
  const handle = event.currentTarget;
  if (handle && typeof handle.setPointerCapture === "function") {
    handle.setPointerCapture(event.pointerId);
  }
}

function moveJoystickGesture(event) {
  const gesture = joystickLayout.gesture;
  if (!gesture || gesture.kind !== "move" || event.pointerId !== gesture.pointerId) return;
  event.preventDefault();
  event.stopPropagation();
  const joystick = joystickContainer();
  const viewport = joystickViewport();
  if (!joystick || !viewport) return;
  const viewportWidth = viewport.clientWidth || 0;
  const viewportHeight = viewport.clientHeight || 0;
  const bounds = joystickBounds(gesture.width, gesture.height, viewportWidth, viewportHeight);
  const left = clampNumber(
    gesture.left + (event.clientX - gesture.startX),
    JOYSTICK_VIEWPORT_MARGIN,
    bounds.maxLeft,
  );
  const top = clampNumber(
    gesture.top + (event.clientY - gesture.startY),
    JOYSTICK_VIEWPORT_MARGIN,
    bounds.maxTop,
  );
  joystick.style.left = `${left}px`;
  joystick.style.top = `${top}px`;
  const ratios = joystickRatiosFromPosition(
    left,
    top,
    gesture.width,
    gesture.height,
    viewport,
  );
  joystickLayout.xRatio = ratios.xRatio;
  joystickLayout.yRatio = ratios.yRatio;
  updateHudPlacement();
}

function beginJoystickResize(event) {
  if (!joystickLayout.mode || joystickLayout.gesture) return;
  event.preventDefault();
  event.stopPropagation();
  joystickLayout.gesture = {
    kind: "resize",
    pointerId: event.pointerId,
    startX: event.clientX,
    startY: event.clientY,
    startSize: joystickLayout.size,
  };
  const handle = event.currentTarget;
  if (handle && typeof handle.setPointerCapture === "function") {
    handle.setPointerCapture(event.pointerId);
  }
}

function resizeJoystickGesture(event) {
  const gesture = joystickLayout.gesture;
  if (!gesture || gesture.kind !== "resize" || event.pointerId !== gesture.pointerId) return;
  event.preventDefault();
  event.stopPropagation();
  const delta = event.clientX - gesture.startX + (event.clientY - gesture.startY);
  joystickLayout.size = clampedJoystickSize(gesture.startSize + delta);
  applyJoystickLayout();
}

async function finishJoystickGesture(event) {
  const gesture = joystickLayout.gesture;
  if (!gesture) return;
  if (event && event.pointerId !== undefined && event.pointerId !== gesture.pointerId) return;
  if (event) {
    event.preventDefault();
    event.stopPropagation();
  }
  joystickLayout.gesture = null;
  await persistJoystickPreferences();
}

function renderJoystickSettingsError() {
  const element = document.getElementById("joystickSettingsError");
  if (!element) return;
  if (joystickLayout.saveError) {
    element.hidden = false;
    element.textContent = `Save failed: ${joystickLayout.saveError}`;
  } else {
    element.hidden = true;
    element.textContent = "";
  }
}

function setJoystickHandleAttributes() {
  const settings = document.getElementById("joystickSettings");
  if (settings) settings.setAttribute("aria-pressed", String(joystickLayout.mode));
  const joystick = joystickContainer();
  if (joystick) joystick.classList.toggle("layout-mode", joystickLayout.mode);
}

async function enterJoystickLayout() {
  if (joystickLayout.mode) return;
  teleop.layoutMode = true;
  await stopTeleop("settings");
  joystickLayout.mode = true;
  joystickLayout.saveError = null;
  setJoystickHandleAttributes();
  renderJoystickSettingsError();
  applyJoystickLayout();
}

function exitJoystickLayout() {
  if (!joystickLayout.mode && !teleop.layoutMode) return;
  teleop.layoutMode = false;
  joystickLayout.mode = false;
  joystickLayout.gesture = null;
  setJoystickHandleAttributes();
  renderJoystickSettingsError();
  applyJoystickLayout();
}

async function persistJoystickPreferences() {
  try {
    const data = await postJson("/api/ui/preferences", {
      size: joystickLayout.size,
      x_ratio: joystickLayout.xRatio,
      y_ratio: joystickLayout.yRatio,
    });
    const saved = data && data.joystick ? data.joystick : null;
    if (saved) {
      joystickLayout.size = clampedJoystickSize(
        finiteNumberOr(saved.size, joystickLayout.size),
      );
      joystickLayout.xRatio = clampNumber(
        finiteNumberOr(saved.x_ratio, joystickLayout.xRatio),
        0,
        1,
      );
      joystickLayout.yRatio = clampNumber(
        finiteNumberOr(saved.y_ratio, joystickLayout.yRatio),
        0,
        1,
      );
    }
    joystickLayout.saveError = null;
    renderJoystickSettingsError();
    applyJoystickLayout();
    return true;
  } catch (error) {
    joystickLayout.saveError = error.message;
    renderJoystickSettingsError();
    applyJoystickLayout();
    return false;
  }
}

async function refreshJoystickPreferences() {
  try {
    const data = await getJson("/api/ui/preferences");
    const saved = data && data.joystick ? data.joystick : null;
    if (saved) {
      joystickLayout.size = clampedJoystickSize(
        finiteNumberOr(saved.size, JOYSTICK_DEFAULT_SIZE),
      );
      joystickLayout.xRatio = clampNumber(
        finiteNumberOr(saved.x_ratio, JOYSTICK_DEFAULT_X_RATIO),
        0,
        1,
      );
      joystickLayout.yRatio = clampNumber(
        finiteNumberOr(saved.y_ratio, JOYSTICK_DEFAULT_Y_RATIO),
        0,
        1,
      );
    }
  } catch (_error) {
    // Keep the defaults already in joystickLayout when the request fails.
  }
  applyJoystickLayout();
}

function overlapArea(first, second) {
  const width = Math.max(
    0,
    Math.min(first.right, second.right) - Math.max(first.left, second.left),
  );
  const height = Math.max(
    0,
    Math.min(first.bottom, second.bottom) - Math.max(first.top, second.top),
  );
  return width * height;
}

function updateHudPlacement() {
  const stage = document.querySelector(".stage");
  const hud = document.getElementById("hud");
  const joystick = joystickContainer();
  if (!stage || !hud) return;
  if (state.view !== "mapping" || !joystick || joystick.hidden) {
    stage.classList.remove("hud-lower-right");
    return;
  }
  const stageRect = stage.getBoundingClientRect();
  const joystickRect = joystick.getBoundingClientRect();
  const width = hud.offsetWidth || 0;
  const height = hud.offsetHeight || 0;
  const lowerLeft = {
    left: stageRect.left + HUD_CORNER_MARGIN,
    right: stageRect.left + HUD_CORNER_MARGIN + width,
    top: stageRect.bottom - HUD_CORNER_MARGIN - height,
    bottom: stageRect.bottom - HUD_CORNER_MARGIN,
  };
  const lowerRight = {
    left: stageRect.right - HUD_CORNER_MARGIN - width,
    right: stageRect.right - HUD_CORNER_MARGIN,
    top: stageRect.bottom - HUD_CORNER_MARGIN - height,
    bottom: stageRect.bottom - HUD_CORNER_MARGIN,
  };
  const leftOverlap = overlapArea(joystickRect, lowerLeft);
  const rightOverlap = overlapArea(joystickRect, lowerRight);
  stage.classList.toggle("hud-lower-right", rightOverlap < leftOverlap);
}
// --- End joystick layout
```

- [ ] **Step 6: Wire the layout block into `setTab`, `attachEvents`, and `start`**

In `setTab` (currently lines 1368–1400), immediately after
`setMappingJoystickVisible(tab === "mapping");`, add:

```javascript
  if (tab === "mapping") {
    applyJoystickLayout();
  } else {
    exitJoystickLayout();
    updateHudPlacement();
  }
```

In `attachEvents`, immediately after the existing
`window.addEventListener("blur", handleJoystickBlur);` line (currently line 1563),
add:

```javascript
  const joystickMoveHandle = document.getElementById("joystickMoveHandle");
  if (joystickMoveHandle) {
    joystickMoveHandle.addEventListener("pointerdown", beginJoystickMove);
    joystickMoveHandle.addEventListener("pointermove", moveJoystickGesture);
    joystickMoveHandle.addEventListener("pointerup", finishJoystickGesture);
    joystickMoveHandle.addEventListener("pointercancel", finishJoystickGesture);
    joystickMoveHandle.addEventListener("lostpointercapture", finishJoystickGesture);
  }
  const joystickResizeHandle = document.getElementById("joystickResizeHandle");
  if (joystickResizeHandle) {
    joystickResizeHandle.addEventListener("pointerdown", beginJoystickResize);
    joystickResizeHandle.addEventListener("pointermove", resizeJoystickGesture);
    joystickResizeHandle.addEventListener("pointerup", finishJoystickGesture);
    joystickResizeHandle.addEventListener("pointercancel", finishJoystickGesture);
    joystickResizeHandle.addEventListener("lostpointercapture", finishJoystickGesture);
  }
  const joystickSettings = document.getElementById("joystickSettings");
  if (joystickSettings) joystickSettings.addEventListener("click", enterJoystickLayout);
  const joystickDone = document.getElementById("joystickDone");
  if (joystickDone) joystickDone.addEventListener("click", exitJoystickLayout);
  window.addEventListener("resize", applyJoystickLayout);
```

In `start()` (currently lines 1650–1671), after `refreshMaps();` add:

```javascript
  refreshJoystickPreferences();
```

(Leave `setSidebarCollapsed(false)` for Task 4.)

- [ ] **Step 7: Replace the joystick and HUD styles**

In `static/styles.css`, replace the whole block from `.joystick {` (line 546)
through `.stage.mapping-active .hud { ... }` (ending line 627) with exactly:

```css
.joystick {
  --joystick-size: 180px;
  position: absolute;
  left: 0;
  top: 0;
  z-index: 6;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 6px;
  touch-action: none;
  user-select: none;
}

.joystick[hidden] {
  display: none;
}

.joystick-move-handle,
.joystick-resize-handle {
  display: none;
}

.joystick.layout-mode .joystick-move-handle,
.joystick.layout-mode .joystick-resize-handle {
  display: block;
}

.joystick-move-handle {
  width: calc(var(--joystick-size) * 0.26);
  height: calc(var(--joystick-size) * 0.1);
  padding: 0;
  border: 1px dashed var(--line);
  border-radius: 999px;
  background: var(--glass);
  cursor: grab;
  touch-action: none;
}

.joystick-base {
  position: relative;
  width: var(--joystick-size);
  height: var(--joystick-size);
  border-radius: 50%;
  border: 1px solid var(--line);
  background: var(--glass);
  box-shadow: var(--shadow-soft);
  backdrop-filter: blur(22px) saturate(1.35);
  -webkit-backdrop-filter: blur(22px) saturate(1.35);
  touch-action: none;
}

.joystick-axis {
  position: absolute;
  background: var(--line);
  pointer-events: none;
}

.joystick-axis-x {
  left: calc(var(--joystick-size) * 0.08);
  right: calc(var(--joystick-size) * 0.08);
  top: 50%;
  height: 1px;
}

.joystick-axis-y {
  top: calc(var(--joystick-size) * 0.08);
  bottom: calc(var(--joystick-size) * 0.08);
  left: 50%;
  width: 1px;
}

.joystick-knob {
  position: absolute;
  left: 50%;
  top: 50%;
  width: calc(var(--joystick-size) * 0.34);
  height: calc(var(--joystick-size) * 0.34);
  margin: calc(var(--joystick-size) * -0.17) 0 0 calc(var(--joystick-size) * -0.17);
  border-radius: 50%;
  border: 1px solid var(--liquid-glass-edge);
  background: var(--glass-strong);
  box-shadow: var(--shadow-soft);
  pointer-events: none;
  transition: transform 90ms linear;
}

.joystick-resize-handle {
  position: absolute;
  right: calc(var(--joystick-size) * -0.08);
  bottom: calc(var(--joystick-size) * -0.08);
  width: calc(var(--joystick-size) * 0.16);
  height: calc(var(--joystick-size) * 0.16);
  padding: 0;
  border: 1px solid var(--line);
  border-radius: 50%;
  background: var(--glass-strong);
  cursor: nwse-resize;
  touch-action: none;
}

.joystick-settings-controls {
  display: flex;
  align-items: center;
  gap: 6px;
}

.joystick-settings-button,
.joystick-done-button {
  min-height: calc(var(--joystick-size) * 0.15);
  padding: 0 calc(var(--joystick-size) * 0.06);
  border: 1px solid var(--line);
  border-radius: 999px;
  background: var(--glass);
  color: var(--text);
  font-size: calc(var(--joystick-size) * 0.075);
  cursor: pointer;
}

.joystick-done-button {
  display: none;
}

.joystick.layout-mode .joystick-settings-button {
  display: none;
}

.joystick.layout-mode .joystick-done-button {
  display: inline-flex;
}

.joystick-settings-error {
  margin: 0;
  color: var(--bad);
  font-size: calc(var(--joystick-size) * 0.067);
}

.joystick-settings-error[hidden] {
  display: none;
}

.joystick-status {
  margin: 0;
  color: var(--muted);
  font-size: calc(var(--joystick-size) * 0.067);
}

.joystick-status.active {
  color: var(--accent);
}

.joystick-status.error {
  color: var(--bad);
}

.stage.hud-lower-right .hud {
  left: auto;
  right: 12px;
}
```

Inside the existing `@media (max-width: 900px)` block, replace the Mapping
offset (currently lines 1274–1276):

```css
  .stage.mapping-active .hud {
    left: 148px;
  }
```

with:

```css
  .stage.hud-lower-right .hud {
    left: auto;
    right: 8px;
  }
```

- [ ] **Step 8: Run the new tests to verify GREEN**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_core.py -k "mapping_joystick"
```

Expected: `4 passed`, including the existing pointer lifecycle harness (prints
`mapping joystick ok`) and the new layout harness (prints
`joystick layout ok`).

- [ ] **Step 9: Confirm the JavaScript parses**

Run:

```bash
node --check static/app.js
```

Expected: exit code 0, no output.

- [ ] **Step 10: Commit**

```bash
git add static/index.html static/app.js static/styles.css test/test_core.py
git commit -m "feat: adjust and persist mapping joystick layout"
```

Do not stage `config/web_control.yaml`.

---

### Task 4: Collapsible sidebar with ResizeObserver and accessibility

**Files:**
- Modify: `static/index.html`
- Modify: `static/app.js`
- Modify: `static/styles.css`
- Test: `test/test_core.py`

- [ ] **Step 1: Write the failing sidebar test and Node harness**

Add this test and Node harness at the end of `test/test_core.py` (after the
Task 3 additions):

```python
def test_sidebar_toggle_is_single_accessible_button():
    index_html, app_js, styles_css = _frontend_sources()

    assert index_html.count('id="sidebarToggle"') == 1
    assert 'aria-controls="controlPanel"' in index_html
    assert 'id="controlPanel"' in index_html
    assert 'class="sidebar-toggle"' in index_html

    assert "grid-template-columns: minmax(0, 1fr) var(--panel-width, 360px);" in styles_css
    assert ".app.sidebar-collapsed" in styles_css
    assert "transition: grid-template-columns 260ms ease;" in styles_css
    assert ".app.sidebar-collapsed .panel" in styles_css
    assert ".app.system-mode .sidebar-toggle" in styles_css
    assert "var(--panel-width, 340px);" in styles_css

    assert "function setSidebarCollapsed(collapsed)" in app_js
    assert "function toggleSidebar()" in app_js
    assert "function observeMapStageResize()" in app_js
    assert "new ResizeObserver" in app_js
    assert 'document.getElementById("sidebarToggle").addEventListener' in app_js


_SIDEBAR_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const blockStart = source.indexOf("// --- Sidebar collapse");
const blockEnd = source.indexOf("// --- End sidebar collapse");
if (blockStart < 0 || blockEnd < 0) throw new Error("sidebar block not found");
const block = source.slice(blockStart, blockEnd);
if (block.includes("localStorage") || block.includes("sessionStorage")) {
  throw new Error("sidebar state must not be persisted");
}

function makeElement() {
  const classes = new Set();
  return {
    hidden: false,
    attributes: {},
    classList: {
      toggle(name, on) { if (on) classes.add(name); else classes.delete(name); },
      contains(name) { return classes.has(name); },
    },
    setAttribute(name, value) { this.attributes[name] = value; },
    getAttribute(name) { return name in this.attributes ? this.attributes[name] : null; },
  };
}

const appElement = makeElement();
const stageElement = makeElement();
const toggleElement = makeElement();
const elements = { sidebarToggle: toggleElement };
const observers = [];
global.document = {
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) {
    if (selector === ".app") return appElement;
    if (selector === ".stage") return stageElement;
    return null;
  },
};
global.ResizeObserver = class {
  constructor(callback) { this.callback = callback; this.target = null; observers.push(this); }
  observe(target) { this.target = target; }
};
let resizeCalls = 0;
let layoutCalls = 0;
global.resizeAll = () => { resizeCalls += 1; };
global.applyJoystickLayout = () => { layoutCalls += 1; };

const sidebarApi = new Function(block + `
return { sidebarState, setSidebarCollapsed, toggleSidebar, observeMapStageResize };`)();
const { sidebarState, setSidebarCollapsed, toggleSidebar, observeMapStageResize } =
  sidebarApi;

function expect(condition, message) { if (!condition) throw new Error(message); }

expect(sidebarState.collapsed === false, "sidebar starts expanded");
setSidebarCollapsed(false);
expect(toggleElement.getAttribute("aria-expanded") === "true", "expanded aria state");
expect(toggleElement.getAttribute("aria-label") === "Collapse sidebar", "expanded label");

toggleSidebar();
expect(sidebarState.collapsed === true, "toggle collapses");
expect(appElement.classList.contains("sidebar-collapsed"), "collapsed class applied");
expect(toggleElement.getAttribute("aria-expanded") === "false", "collapsed aria state");
expect(toggleElement.getAttribute("aria-label") === "Expand sidebar", "collapsed label");

toggleSidebar();
expect(sidebarState.collapsed === false, "toggle expands");
expect(!appElement.classList.contains("sidebar-collapsed"), "collapsed class removed");
expect(toggleElement.getAttribute("aria-expanded") === "true", "re-expanded aria state");

observeMapStageResize();
expect(observers.length === 1, "one ResizeObserver created");
expect(observers[0].target === stageElement, "map stage observed");
observers[0].callback();
expect(resizeCalls === 1, "renderer resized on stage change");
expect(layoutCalls === 1, "joystick layout recalculated on stage change");

console.log("sidebar collapse ok");
"""


def test_sidebar_collapse_state_toggles_without_persistence_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _SIDEBAR_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "sidebar collapse ok" in completed.stdout
```

- [ ] **Step 2: Run the sidebar tests to verify RED**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_core.py -k "sidebar"
```

Expected: `test_sidebar_toggle_is_single_accessible_button` fails with
`AssertionError` on `id="sidebarToggle"`, and
`test_sidebar_collapse_state_toggles_without_persistence_under_node` fails with
`Error: sidebar block not found` on stderr. The pre-existing
`test_frontend_exposes_mapping_tab_and_mapping_sidebar` and
`test_system_tab_hides_sidebar_and_expands_stage` still pass.

- [ ] **Step 3: Add the `controlPanel` id and the single toggle button**

In `static/index.html`, change the sidebar aside (currently line 65) from:

```html
      <aside class="panel">
```

to:

```html
      <aside id="controlPanel" class="panel">
```

Then, immediately before the `</section>` that closes `.stage` (currently line
63), add:

```html
        <button id="sidebarToggle" class="sidebar-toggle" type="button" aria-expanded="true" aria-controls="controlPanel" aria-label="Collapse sidebar">
          <span class="sidebar-toggle-arrow" aria-hidden="true">›</span>
        </button>
```

The button lives inside `.stage`, anchored to its right edge, so the same DOM
element travels with the moving boundary instead of being replaced.

- [ ] **Step 4: Add the sidebar collapse block**

In `static/app.js`, insert this block immediately after the
`// --- End joystick layout` line and before
`function maybeClearGoalFromActionStatus()`:

```javascript
// --- Sidebar collapse
const sidebarState = { collapsed: false };

function setSidebarCollapsed(collapsed) {
  sidebarState.collapsed = Boolean(collapsed);
  const app = document.querySelector(".app");
  if (app) app.classList.toggle("sidebar-collapsed", sidebarState.collapsed);
  const toggle = document.getElementById("sidebarToggle");
  if (toggle) {
    toggle.setAttribute("aria-expanded", String(!sidebarState.collapsed));
    toggle.setAttribute(
      "aria-label",
      sidebarState.collapsed ? "Expand sidebar" : "Collapse sidebar",
    );
  }
}

function toggleSidebar() {
  setSidebarCollapsed(!sidebarState.collapsed);
}

function observeMapStageResize() {
  if (typeof ResizeObserver !== "function") return;
  const stage = document.querySelector(".stage");
  if (!stage) return;
  const observer = new ResizeObserver(() => {
    resizeAll();
    if (typeof applyJoystickLayout === "function") applyJoystickLayout();
  });
  observer.observe(stage);
}
// --- End sidebar collapse
```

The block never touches `localStorage`, `sessionStorage`, or the network, so
the collapsed state is browser-memory only.

- [ ] **Step 5: Wire the toggle and observer**

In `attachEvents`, immediately after the existing
`document.getElementById("refreshMaps").addEventListener(...)` line (currently
line 1543), add:

```javascript
  document.getElementById("sidebarToggle").addEventListener("click", toggleSidebar);
  observeMapStageResize();
```

In `start()`, after `refreshJoystickPreferences();` (added in Task 3) add:

```javascript
  setSidebarCollapsed(false);
```

- [ ] **Step 6: Add the grid transition and toggle styles**

In `static/styles.css`, change the `.app` rule (currently lines 120–128) to:

```css
.app {
  position: relative;
  z-index: 1;
  display: grid;
  grid-template-columns: minmax(0, 1fr) var(--panel-width, 360px);
  min-width: 760px;
  width: 100%;
  height: 100%;
  transition: grid-template-columns 260ms ease;
}

.app.sidebar-collapsed {
  --panel-width: 0px;
}
```

In the `.panel` rule (currently lines 629–640), add `overflow-x: hidden;` and a
transition, so it reads:

```css
.panel {
  min-width: 0;
  height: 100%;
  overflow-y: auto;
  overflow-x: hidden;
  -webkit-overflow-scrolling: touch;
  border-left: 1px solid var(--line);
  background: var(--panel-bg);
  padding: 16px;
  backdrop-filter: blur(22px) saturate(1.35);
  -webkit-backdrop-filter: blur(22px) saturate(1.35);
  box-shadow: -18px 0 48px rgba(0, 0, 0, 0.3);
  transition: opacity 260ms ease, transform 260ms ease;
}
```

Immediately after the `.panel` rule, add:

```css
.app.sidebar-collapsed .panel {
  opacity: 0;
  transform: translateX(12px);
  pointer-events: none;
}

.sidebar-toggle {
  position: absolute;
  z-index: 7;
  top: 50%;
  right: 0;
  transform: translateY(-50%);
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 30px;
  height: 54px;
  padding: 0;
  border: 1px solid var(--line);
  border-radius: 999px 0 0 999px;
  background: var(--glass-strong);
  color: var(--text);
  cursor: pointer;
  box-shadow: var(--shadow-soft);
  backdrop-filter: blur(22px) saturate(1.35);
  -webkit-backdrop-filter: blur(22px) saturate(1.35);
}

.sidebar-toggle-arrow {
  display: inline-block;
  font-size: 18px;
  line-height: 1;
  transition: transform 260ms ease;
}

.app.sidebar-collapsed .sidebar-toggle-arrow {
  transform: rotate(180deg);
}

.app.system-mode .sidebar-toggle {
  display: none;
}
```

Finally, update both responsive `.app` rules so they animate the same variable.
In `@media (max-width: 900px)` (currently line 1214) and
`@media (max-width: 520px)` (currently line 1281), change
`grid-template-columns: minmax(420px, 1fr) 340px;` to:

```css
    grid-template-columns: minmax(420px, 1fr) var(--panel-width, 340px);
```

- [ ] **Step 7: Run the sidebar tests to verify GREEN**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_core.py -k "sidebar"
```

Expected: `4 passed`, with the Node harness printing `sidebar collapse ok`.

- [ ] **Step 8: Confirm the JavaScript parses and the system-tab regression still holds**

Run:

```bash
node --check static/app.js
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_core.py -k "system_tab"
```

Expected: `node --check` exits 0; `test_system_tab_hides_sidebar_and_expands_stage`
and `test_system_tab_uses_refined_dashboard_elements` pass.

- [ ] **Step 9: Commit**

```bash
git add static/index.html static/app.js static/styles.css test/test_core.py
git commit -m "feat: add collapsible information sidebar"
```

Do not stage `config/web_control.yaml`.

---

### Task 5: Documentation and final regression

**Files:**
- Modify: `README.md`
- Test: whole package

- [ ] **Step 1: Document the adjustable joystick and sidebar**

In `README.md`, after the `## 选择地图（Map 选择器）` section (currently ends at
line 26) and before `## 暂停与恢复导航（Pause / Resume / Clear）`, insert:

```markdown
## 可调摇杆布局（Mapping）

**Mapping** 页面左下角的虚拟摇杆带一个随摇杆移动的 **Settings** 按钮：

- 点击设置按钮会先请求一次 teleop 停止并结束发送循环，然后进入布局模式；布局模式下车速控制被禁用。**Done** 退出布局模式并恢复车速控制。
- 布局模式提供摇杆上方的移动手柄和右下角的缩放手柄；两者使用 Pointer Events 与 pointer capture，事件不会传递到 Three.js 画布。手柄可聚焦并带 `aria-label`。
- 拖动缩放手柄实时改变摇杆底座直径，范围 `140px`–`260px`，未保存过设置时默认 `180px`；旋钮、十字轴、设置控件与状态文字都通过 `--joystick-size` 派生并等比缩放。
- 拖动手柄或缩放手柄时只在浏览器内实时预览，指针释放或取消时各写入一次后端。
- 位置以视口归一化中心坐标 `x_ratio`、`y_ratio` 保存；浏览器在移动、恢复、窗口缩放、侧边栏动画和切换标签页时把摇杆、手柄与状态区夹在可见地图区域内。默认位置靠近左下角，不同分辨率下仍然可用。
- 摇杆设置保存在 `~/.config/web_control/selected_map.json` 的 `ui.joystick` 下，例如 `{"name": "hesai_sim2", "ui": {"joystick": {"size": 180, "x_ratio": 0.05, "y_ratio": 0.75}}}`。旧格式 `{"name": "..."}` 仍然可用并回退到默认值；地图选择只更新 `name`，摇杆设置只更新 `ui.joystick`，两者都不会覆盖未知键。
- 设置对该 web-control 服务是全局的：所有浏览器加载同一份布局，最后一次完成的调整生效。
- 后端接口：`GET /api/ui/preferences` 返回校验后的摇杆设置；`POST /api/ui/preferences` 接受完整摇杆设置对象 `{"size": ..., "x_ratio": ..., "y_ratio": ...}`，校验并原子合并后返回保存结果。非法、非有限或越界值返回 HTTP 400 且不修改文件；保存失败时摇杆界面显示错误并保持布局模式，便于继续调整后重试。
- HUD 默认停在左下角；当摇杆区域与左下角重叠时移到右下角，与右下角重叠时回到左下角。其他标签页沿用原有 HUD 位置。
- 布局模式激活时离开 Mapping 会退出布局模式，且不会在其他标签页重新启用 teleop。

## 折叠右侧信息栏

- 地图区域与右侧信息栏交界处（垂直居中）有一个切换按钮；折叠与展开使用同一个 DOM 按钮，只有箭头方向和 `aria-expanded` / `aria-label` 变化。
- 折叠时网格中的侧边栏轨道在约 `260ms` 内过渡到 `0px`，面板内容同时淡出并轻微右移；展开时反向播放。切换按钮跟随移动的边界，不会跳到新位置。
- 地图区域上的 `ResizeObserver` 在过渡期间调用现有的 Three.js `resizeAll()`，让画布宽高比与分辨率保持同步。
- 折叠状态只保存在当前页面内存中，不写入后端、`sessionStorage` 或 `localStorage`；每次页面加载都从展开状态开始。
- **System** 标签页继续使用现有的隐藏面板布局并隐藏该切换按钮；返回 **Navigation** 或 **Mapping** 时恢复当前页面内的折叠状态并显示同一个按钮。
```

- [ ] **Step 2: Run the focused suites for every changed area**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_map_selection.py test/test_core.py -k "map_selection or ui_preferences or mapping_joystick or sidebar or system_tab or maps_handler"
```

Expected: every selected test passes (the store tests, the 11 preference route
tests, the 4 joystick tests, the 4 sidebar tests, the system-tab tests, and the
map-selection handler tests).

- [ ] **Step 3: Run the full package regression**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test
```

Expected: exactly one failure, `test_web_control_yaml_declares_teleop_defaults`
with `assert 1.0 == 1.5`. That failure is the operator's unrelated, uncommitted
`config/web_control.yaml` edit and is out of scope: do **not** edit or stage
`config/web_control.yaml` to make it pass. Every other test passes, and
`test/test_map_selection.py` contributes 21 passing tests. The proxy variables
are unset because the HTTP endpoint tests open real loopback sockets.

- [ ] **Step 4: Run the syntax gates**

Run:

```bash
node --check static/app.js
python3 -m py_compile web_control/*.py
git diff --check
```

Expected: each command exits zero; `git diff --check` reports no whitespace
errors.

- [ ] **Step 5: Confirm the unrelated config change is untouched**

Run:

```bash
git status --short
git log --oneline -6
git log --name-only -5 --pretty=format:'%h %s'
```

Expected: `git status --short` still shows ` M config/web_control.yaml` as an
unstaged modification; none of the new commits lists
`config/web_control.yaml`; the working tree file still contains
`teleop_max_linear_speed: 1.0`.

- [ ] **Step 6: Commit the documentation**

```bash
git add README.md
git commit -m "docs: document adjustable joystick and sidebar"
```

Do not stage `config/web_control.yaml`.

---

## Final Verification Checklist

- [ ] `python3 -m pytest -q test/test_map_selection.py` → `21 passed`.
- [ ] `python3 -m pytest -q test/test_core.py -k "ui_preferences"` → `11 passed`.
- [ ] `python3 -m pytest -q test/test_core.py -k "mapping_joystick"` → `4 passed`.
- [ ] `python3 -m pytest -q test/test_core.py -k "sidebar"` → `4 passed`.
- [ ] `python3 -m pytest -q test` → only the pre-existing
  `test_web_control_yaml_declares_teleop_defaults` failure remains.
- [ ] `node --check static/app.js` and `python3 -m py_compile web_control/*.py`
  and `git diff --check` all exit zero.
- [ ] `config/web_control.yaml` is still an unstaged, uncommitted working-tree
  modification and appears in no commit.
- [ ] No `colcon` command was run.
