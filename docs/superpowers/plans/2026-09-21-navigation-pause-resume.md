# Web Navigation Pause and Resume Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let a web operator pause the current navigation goal without stopping the navigation launch, resume the saved destination from the robot's current pose, or clear it permanently, with the paused destination surviving browser refreshes and `web_control` restarts.

**Architecture:** A pure state machine (`NavigationControl`) owns the lifecycle phases and legal commands; a JSON store (`NavigationStore`) persists one paused destination atomically; the web node wires them to a zero-UUID `action_msgs/srv/CancelGoal` client and exposes pause/resume/clear endpoints plus a `navigation_control` block in `GET /api/state`. The browser renders only server-owned state and polls until a transition settles. Pause cancellations keep the destination; Clear cancellations discard it; a pending-intent flag lets the same Action `canceled` status resolve to `Paused` or `Idle`. An empty `GoalStatusArray` performs the one-time restart reconciliation so a persisted `pausing`/`resuming` record cannot stay stuck.

**Tech Stack:** Python 3.10, ROS 2 Humble Python (`rclpy`, `action_msgs`), `geometry_msgs`, stdlib `http.server`, vanilla JavaScript/HTML.

---

## File Structure

| File | Responsibility |
| --- | --- |
| `web_control/atomic_io.py` (create) | One `atomic_write_text` used by both persistence stores. |
| `web_control/navigation_store.py` (create) | `NavigationStore` schema validation plus atomic load/save/clear of `~/.config/web_control/paused_navigation.json`. |
| `web_control/navigation_control.py` (create) | ROS-free `NavigationControl` state machine, `select_action_status`, and navigation error types. |
| `web_control/map_selection.py` (modify) | Delegate its private atomic writer to `atomic_io.atomic_write_text`. |
| `web_control/web_control_node.py` (modify) | `cancel_goal_service` parameter, `CancelGoal` client plus the `_cancel_all_goals` cancellation primitive, command methods, HTTP routes, `/api/state.navigation_control`. |
| `config/web_control.yaml` (modify) | Declare `cancel_goal_service: "/p2p_move_base/_action/cancel_goal"`. |
| `static/index.html` (modify) | Target Pose primary/clear buttons plus the state status line. |
| `static/app.js` (modify) | `updateNavigationControls`, `postNavigationCommand`, state polling and button wiring. |
| `test/test_navigation_store.py` (create) | Atomic writer and navigation store tests. |
| `test/test_navigation_control.py` (create) | State machine, capability, restart reconciliation, and stale-status tests. |
| `test/test_core.py` (modify) | ROS-free node command tests, HTTP endpoint tests, frontend harness, README test. |
| `README.md` (modify) | Operator documentation for pause/resume/clear and the cancel service parameter. |

`package.xml` is **not** modified: `action_msgs` is already an `exec_depend`. `dddmr_sys_core` is **not** introduced.

Commit discipline: every commit runs `git add` with the exact files named in that task only. Never run `git add -A` or `git add .`.

## Fixed design decisions

1. Restart with an active goal but no saved target shows `Navigating`, allows Clear, and disables Pause.
2. Clear while `Pausing` switches the pending intent to clear and reuses the in-flight cancel; Clear while `Resuming` returns HTTP 409.
3. A cancel service timeout returns HTTP 504, keeps the persisted `pausing` phase, and the browser keeps polling.
4. `paused_goal` is part of `GET /api/state.navigation_control`.
5. A new goal removes the paused record only after the pose publish call returns successfully.
6. `select_action_status` prefers active statuses over newer completed ones, and a per-transition goal-stamp floor (recorded at new-goal publish and at Resume) discards every replaced-goal status at or below the floor. Only a newer `accepted`/`executing` confirms the new goal: a newer `canceling` does not confirm it, and a newer terminal (`canceled`/`succeeded`/`aborted`) that arrives before any confirmation means the new goal already ended, so a Resume returns to `Paused` with its record kept and a normal new goal becomes `Idle`.
7. The `action_msgs.srv.CancelGoal` import is isolated from the main ROS import so a missing service type degrades to HTTP 503 for pause/clear instead of setting `rclpy` to `None`; `_cancel_all_goals` reads `return_code` and returns `accepted`/`empty`/`rejected` rather than an ambiguous `ok`.
8. Stopping the navigation process settles the state machine through `on_navigation_stopped` and clears `published_goal`, so the browser never keeps showing an active goal after Stop.
9. An empty `GoalStatusArray` (no goals at all) reconciles a restarted record once through `note_no_active_goals`: `pausing`/`resuming` become `paused` with the destination kept, a record-less control becomes `idle`, and an already `paused` record is unchanged. This uses a dedicated method rather than a synthetic `goal_stamp=0` status so the transition floor is never bypassed.

---

### Task 1: Shared atomic write helper

**Files:**
- Create: `web_control/atomic_io.py`
- Modify: `web_control/map_selection.py`
- Test: `test/test_navigation_store.py`

- [ ] **Step 1: Write the failing test**

Create `test/test_navigation_store.py` with:

```python
"""Tests for the shared atomic writer and navigation persistence."""
import os
from pathlib import Path

from web_control.atomic_io import atomic_write_text


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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest -q test/test_navigation_store.py -k atomic_write`
Expected: FAIL with `ModuleNotFoundError: No module named 'web_control.atomic_io'`.

- [ ] **Step 3: Write minimal implementation**

Create `web_control/atomic_io.py`:

```python
"""Atomic file replacement helpers shared by the persistence stores."""

import os
import tempfile
from pathlib import Path


def atomic_write_text(path: Path, text: str) -> None:
    """Write ``text`` to ``path`` by replacing a fully written sibling file."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent, prefix=f".{path.name}.", suffix=".tmp"
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
```

Modify `web_control/map_selection.py`: replace the imports

```python
import json
import logging
import os
import tempfile
from pathlib import Path
```

with

```python
import json
import logging
from pathlib import Path

from web_control.atomic_io import atomic_write_text
```

and replace the whole private body

```python
def _atomic_write_text(path: Path, text: str) -> None:
    """Write ``text`` to ``path`` by replacing a fully written sibling file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent, prefix=f".{path.name}.", suffix=".tmp"
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
```

with

```python
def _atomic_write_text(path: Path, text: str) -> None:
    """Write ``text`` to ``path`` by replacing a fully written sibling file."""
    atomic_write_text(path, text)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest -q test/test_navigation_store.py -k atomic_write`
Expected: `1 passed`.

Run: `python3 -m pytest -q test/test_core.py -k "selection_store or selection_default or map_selection"`
Expected: `8 passed` (the existing selection-store atomicity test still passes through the shared helper).

- [ ] **Step 5: Commit**

```bash
git add web_control/atomic_io.py web_control/map_selection.py test/test_navigation_store.py
git commit -m "refactor: share atomic file replacement helper"
```

---

### Task 2: Persistent navigation destination store

**Files:**
- Create: `web_control/navigation_store.py`
- Test: `test/test_navigation_store.py`

- [ ] **Step 1: Write the failing tests**

Add these imports to the top of `test/test_navigation_store.py`:

```python
import pytest

from web_control.navigation_store import (
    NavigationStore,
    default_navigation_path,
    validate_navigation_record,
)
```

Append to `test/test_navigation_store.py`:

```python
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


def test_navigation_store_ignores_corrupt_file(tmp_path, caplog):
    path = tmp_path / "paused_navigation.json"
    path.write_text("{not json", encoding="utf-8")

    assert NavigationStore(path).load() is None
    assert "invalid paused navigation record" in caplog.text


def test_default_navigation_path_lives_under_runtime_home():
    assert default_navigation_path(Path("/home/robot")) == Path(
        "/home/robot/.config/web_control/paused_navigation.json"
    )
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest -q test/test_navigation_store.py -k "navigation_store or validate_navigation or default_navigation"`
Expected: FAIL with `ModuleNotFoundError: No module named 'web_control.navigation_store'`.

- [ ] **Step 3: Write minimal implementation**

Create `web_control/navigation_store.py`:

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest -q test/test_navigation_store.py`
Expected: `14 passed`.

- [ ] **Step 5: Commit**

```bash
git add web_control/navigation_store.py test/test_navigation_store.py
git commit -m "feat: persist a pausable navigation destination"
```

---

### Task 3: Navigation control state machine

**Files:**
- Create: `web_control/navigation_control.py`
- Test: `test/test_navigation_control.py`

- [ ] **Step 1: Write the failing tests**

Create `test/test_navigation_control.py`:

```python
"""Tests for the ROS-free navigation pause/resume state machine."""
import pytest

from web_control.navigation_control import (
    CLEARING,
    IDLE,
    NAVIGATING,
    PAUSED,
    PAUSING,
    RESUMING,
    NavigationConflictError,
    NavigationControl,
    select_action_status,
)
from web_control.navigation_store import NavigationStore


class _Stamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class _GoalInfo:
    def __init__(self, goal_stamp):
        self.stamp = _Stamp(goal_stamp // 1_000_000_000, goal_stamp % 1_000_000_000)


class _Status:
    def __init__(self, value, goal_stamp):
        self.status = value
        self.goal_info = _GoalInfo(goal_stamp)


def _record(phase="paused", map_name="site_a"):
    return {
        "schema_version": 1,
        "phase": phase,
        "x": 1.0,
        "y": 2.0,
        "z": 0.0,
        "yaw": 0.5,
        "frame": "map",
        "map_name": map_name,
        "saved_at": 5.0,
    }


def _control(tmp_path, record=None):
    store = NavigationStore(tmp_path / "config" / "paused_navigation.json")
    if record is not None:
        store.save(record)
    return NavigationControl(store), store


def _arm(control, stamp=5):
    # Publish first, then feed the status, mirroring the node: the transition
    # floor is recorded from the previously known goal stamp.
    control.record_goal_published(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}, "map", "site_a", 10.0
    )
    control.note_action_status("executing", stamp, 10.0)


def test_select_action_status_prefers_active_over_newer_completed():
    selected = select_action_status([_Status(4, 100), _Status(2, 200)])
    assert selected["name"] == "executing"

    selected = select_action_status([_Status(2, 300), _Status(4, 100)])
    assert selected["name"] == "executing"

    selected = select_action_status([_Status(4, 300), _Status(6, 200)])
    assert selected["name"] == "succeeded"

    assert select_action_status([]) is None


def test_select_action_status_empty_or_terminal_only():
    assert select_action_status([]) is None
    assert select_action_status(None) is None

    selected = select_action_status([_Status(5, 100), _Status(5, 200)])
    assert selected["name"] == "canceled"
    assert selected["goal_stamp"] == 200
    assert selected["values"] == [5, 5]



def test_navigating_pause_canceled_reaches_paused_then_resume(tmp_path):
    control, store = _control(tmp_path)
    _arm(control)
    assert control.snapshot("site_a")["state"] == NAVIGATING
    assert control.snapshot("site_a")["can_pause"] is True

    control.begin_pause(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}, "map", "site_a", 10.0
    )
    assert control.snapshot("site_a")["state"] == PAUSING
    assert store.load()["phase"] == "pausing"

    control.note_action_status("canceled", 5, 11.0)
    paused = control.snapshot("site_a")
    assert paused["state"] == PAUSED
    assert paused["can_resume"] is True
    assert store.load()["phase"] == "paused"

    record = control.begin_resume(12.0, "site_a")
    assert record["x"] == 1.0
    assert control.snapshot("site_a")["state"] == RESUMING
    assert store.load()["phase"] == "resuming"

    control.note_action_status("accepted", 9, 13.0)
    assert control.snapshot("site_a")["state"] == NAVIGATING
    assert store.load() is None


def test_pausing_clear_reuses_cancel_and_discards_destination(tmp_path):
    control, store = _control(tmp_path)
    _arm(control)
    control.begin_pause(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}, "map", "site_a", 10.0
    )

    assert control.begin_clear(10.0) is False
    assert control.snapshot("site_a")["state"] == CLEARING

    control.note_action_status("canceled", 5, 11.0)
    assert control.snapshot("site_a")["state"] == IDLE
    assert store.load() is None


def test_clear_paused_goal_needs_no_cancel(tmp_path):
    control, store = _control(tmp_path, _record())

    assert control.begin_clear(10.0) is False
    assert control.snapshot("site_a")["state"] == IDLE
    assert store.load() is None


def test_resume_rejects_when_map_does_not_match(tmp_path):
    control, store = _control(tmp_path, _record(map_name="site_a"))

    with pytest.raises(NavigationConflictError, match="different map"):
        control.begin_resume(10.0, "site_b")

    assert control.snapshot("site_b")["state"] == PAUSED
    assert store.load()["phase"] == "paused"


def test_resume_rejects_while_resuming(tmp_path):
    control, store = _control(tmp_path, _record())
    control.begin_resume(10.0, "site_a")

    with pytest.raises(NavigationConflictError, match="no paused navigation goal"):
        control.begin_resume(10.0, "site_a")


def test_restart_with_active_goal_and_no_record_allows_clear_only(tmp_path):
    control, store = _control(tmp_path)
    control.note_action_status("executing", 7, 9.0)

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == NAVIGATING
    assert snapshot["can_pause"] is False
    assert snapshot["can_clear"] is True
    assert snapshot["paused_goal"] is None


def test_restart_pausing_record_with_active_goal_retries_cancel(tmp_path):
    control, store = _control(tmp_path, _record(phase="pausing"))

    action = control.note_action_status("executing", 7, 9.0)

    assert action == "retry_cancel"
    assert control.snapshot("site_a")["state"] == PAUSING
    assert store.load()["phase"] == "pausing"


def test_restart_pausing_record_without_active_goal_becomes_paused(tmp_path):
    control, store = _control(tmp_path, _record(phase="pausing"))

    control.note_action_status("canceled", 7, 9.0)

    assert control.snapshot("site_a")["state"] == PAUSED
    assert store.load()["phase"] == "paused"


def test_restart_resuming_record_with_active_goal_is_adopted(tmp_path):
    control, store = _control(tmp_path, _record(phase="resuming"))

    assert control.note_action_status("accepted", 7, 9.0) is None
    assert control.snapshot("site_a")["state"] == NAVIGATING
    assert store.load() is None


def test_restart_resuming_record_without_active_goal_returns_to_paused(tmp_path):
    control, store = _control(tmp_path, _record(phase="resuming"))

    control.note_action_status("aborted", 7, 9.0)

    assert control.snapshot("site_a")["state"] == PAUSED
    assert store.load()["phase"] == "paused"


def test_no_active_goals_reconciles_pausing_record_to_paused(tmp_path):
    control, store = _control(tmp_path, _record(phase="pausing"))

    control.note_no_active_goals(9.0)

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == PAUSED
    assert snapshot["can_resume"] is True
    assert store.load()["phase"] == "paused"


def test_no_active_goals_reconciles_resuming_record_to_paused(tmp_path):
    control, store = _control(tmp_path, _record(phase="resuming"))

    control.note_no_active_goals(9.0)

    assert control.snapshot("site_a")["state"] == PAUSED
    assert control.snapshot("site_a")["can_resume"] is True
    assert store.load()["phase"] == "paused"


def test_no_active_goals_without_record_is_idle(tmp_path):
    control, store = _control(tmp_path)

    control.note_no_active_goals(9.0)

    assert control.snapshot("site_a")["state"] == IDLE
    assert store.load() is None


def test_no_active_goals_keeps_already_paused_record(tmp_path):
    control, store = _control(tmp_path, _record())

    control.note_no_active_goals(9.0)

    assert control.snapshot("site_a")["state"] == PAUSED
    assert store.load()["phase"] == "paused"


def test_no_active_goals_reconciles_only_once(tmp_path):
    control, store = _control(tmp_path, _record(phase="pausing"))

    control.note_no_active_goals(9.0)
    assert control.snapshot("site_a")["state"] == PAUSED

    control.begin_resume(10.0, "site_a")
    assert control.snapshot("site_a")["state"] == RESUMING

    # A later empty array must not disturb an established transition.
    control.note_no_active_goals(11.0)
    assert control.snapshot("site_a")["state"] == RESUMING
    assert store.load()["phase"] == "resuming"


def test_stale_status_is_ignored_with_goal_stamp_floor(tmp_path):
    control, store = _control(tmp_path)
    control.note_action_status("executing", 100, 9.0)

    control.note_action_status("succeeded", 50, 10.0)

    assert control.snapshot("site_a")["state"] == NAVIGATING


def test_navigation_stopped_settles_each_phase(tmp_path):
    # Navigating -> Idle with no saved destination.
    control, store = _control(tmp_path)
    _arm(control)
    control.on_navigation_stopped(11.0)
    assert control.snapshot("site_a")["state"] == IDLE
    assert store.load() is None

    # Pausing -> Paused and the destination is kept.
    control, store = _control(tmp_path)
    _arm(control)
    control.begin_pause(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}, "map", "site_a", 10.0
    )
    control.on_navigation_stopped(12.0)
    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == PAUSED
    assert snapshot["can_resume"] is True
    assert store.load()["phase"] == "paused"

    # Resuming -> Paused and the record is kept.
    control, store = _control(tmp_path, _record(phase="resuming"))
    control.on_navigation_stopped(13.0)
    assert control.snapshot("site_a")["state"] == PAUSED
    assert store.load()["phase"] == "paused"

    # Clearing -> Idle and the record is discarded.
    control, store = _control(tmp_path)
    _arm(control)
    control.begin_pause(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}, "map", "site_a", 10.0
    )
    assert control.begin_clear(10.0) is False
    assert control.snapshot("site_a")["state"] == CLEARING
    control.on_navigation_stopped(14.0)
    assert control.snapshot("site_a")["state"] == IDLE
    assert store.load() is None

    # Paused already has the right state.
    control, store = _control(tmp_path, _record())
    control.on_navigation_stopped(15.0)
    assert control.snapshot("site_a")["state"] == PAUSED
    assert store.load()["phase"] == "paused"


def test_resume_ignores_replayed_paused_terminal_status(tmp_path):
    control, store = _control(tmp_path)
    _arm(control)
    control.begin_pause(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}, "map", "site_a", 10.0
    )
    control.note_action_status("canceled", 5, 11.0)
    assert control.snapshot("site_a")["state"] == PAUSED

    control.begin_resume(12.0, "site_a")
    # The paused goal's retained terminal statuses must not end the resume.
    control.note_action_status("canceled", 5, 13.0)
    assert control.snapshot("site_a")["state"] == RESUMING
    assert store.load()["phase"] == "resuming"
    control.note_action_status("succeeded", 5, 14.0)
    assert control.snapshot("site_a")["state"] == RESUMING
    assert store.load()["phase"] == "resuming"

    control.note_action_status("accepted", 9, 15.0)
    assert control.snapshot("site_a")["state"] == NAVIGATING
    assert store.load() is None


def test_new_goal_ignores_stale_succeeded_status(tmp_path):
    control, store = _control(tmp_path)
    _arm(control)

    control.record_goal_published(
        {"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0}, "map", "site_a", 11.0
    )
    # The replaced goal's retained terminal status must not clear the new goal.
    control.note_action_status("succeeded", 5, 12.0)

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == NAVIGATING
    assert snapshot["has_goal"] is True
    assert store.load() is None

    control.note_action_status("executing", 9, 13.0)
    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == NAVIGATING
    assert snapshot["can_pause"] is True


def test_newer_canceling_does_not_confirm_resume(tmp_path):
    control, store = _control(tmp_path, _record())
    control.begin_resume(12.0, "site_a")
    assert control.snapshot("site_a")["state"] == RESUMING

    # The new goal being canceled is not an accepted/executing confirmation,
    # so it must not clear the resuming record or mark Resume successful.
    control.note_action_status("canceling", 9, 13.0)

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == RESUMING
    assert snapshot["can_pause"] is False
    assert store.load()["phase"] == "resuming"

    # The final terminal still arrives and returns the goal to Paused.
    control.note_action_status("canceled", 9, 14.0)
    assert control.snapshot("site_a")["state"] == PAUSED
    assert store.load()["phase"] == "paused"


def test_newer_canceled_before_active_returns_resume_to_paused(tmp_path):
    control, store = _control(tmp_path, _record())
    control.begin_resume(12.0, "site_a")

    control.note_action_status("canceled", 9, 13.0)

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == PAUSED
    assert snapshot["can_resume"] is True
    assert store.load()["phase"] == "paused"


def test_newer_terminal_before_active_sends_new_goal_to_idle(tmp_path):
    control, store = _control(tmp_path)
    control.record_goal_published(
        {"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0}, "map", "site_a", 11.0
    )

    # The new goal terminated before any accepted/executing confirmation.
    control.note_action_status("succeeded", 9, 12.0)

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == IDLE
    assert snapshot["has_goal"] is False
    assert store.load() is None


def test_map_change_invalidates_paused_record(tmp_path):
    control, store = _control(tmp_path, _record(map_name="site_a"))

    control.note_map_selected("site_b", 10.0)

    assert store.load() is None
    assert control.snapshot("site_b")["state"] == IDLE
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest -q test/test_navigation_control.py`
Expected: FAIL with `ModuleNotFoundError: No module named 'web_control.navigation_control'`.

- [ ] **Step 3: Write minimal implementation**

Create `web_control/navigation_control.py`:

```python
"""Server-owned state machine for pausing, resuming, and clearing a goal.

The control object performs no ROS I/O: the web node feeds it selected Action
statuses and map selections and asks it which commands are currently legal.
Destination persistence is delegated to a :class:`NavigationStore`.
"""

from __future__ import annotations

import threading
from typing import Any

from web_control.navigation_store import SCHEMA_VERSION, NavigationStore

STATUS_NAMES = {
    0: "unknown",
    1: "accepted",
    2: "executing",
    3: "canceling",
    4: "succeeded",
    5: "canceled",
    6: "aborted",
}
ACTIVE_STATUS_VALUES = (1, 2, 3)
ACTIVE_STATUS_NAMES = ("accepted", "executing", "canceling")

IDLE = "idle"
NAVIGATING = "navigating"
PAUSING = "pausing"
PAUSED = "paused"
RESUMING = "resuming"
CLEARING = "clearing"
PENDING_PAUSE = "pause"
PENDING_CLEAR = "clear"


class NavigationConflictError(RuntimeError):
    """Raised when a navigation command is illegal in the current state."""


class NavigationUnavailableError(RuntimeError):
    """Raised when the cancel goal service cannot accept a request."""


class NavigationTimeoutError(RuntimeError):
    """Raised when the cancel goal service does not answer in time."""


def select_action_status(status_list: Any) -> dict[str, Any] | None:
    """Return the active or newest GoalStatus as a plain dictionary.

    ``GoalStatusArray`` can retain recently completed goals, so the last array
    element is not reliable.  Active goals win; otherwise the newest goal
    timestamp wins.  ``goal_stamp`` lets the caller discard older updates.
    """
    entries = []
    for index, status in enumerate(status_list or []):
        value = int(getattr(status, "status", 0))
        goal_info = getattr(status, "goal_info", None)
        stamp = getattr(goal_info, "stamp", None)
        goal_stamp = 0
        if stamp is not None:
            goal_stamp = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(
                getattr(stamp, "nanosec", 0)
            )
        entries.append({"value": value, "goal_stamp": goal_stamp, "index": index})
    if not entries:
        return None
    active = [entry for entry in entries if entry["value"] in ACTIVE_STATUS_VALUES]
    pool = active or entries
    selected = max(pool, key=lambda entry: (entry["goal_stamp"], entry["index"]))
    return {
        "value": selected["value"],
        "name": STATUS_NAMES.get(selected["value"], "unknown"),
        "values": [entry["value"] for entry in entries],
        "goal_stamp": selected["goal_stamp"],
    }


class NavigationControl:
    """Pause/resume/clear state shared by the HTTP worker and ROS callbacks."""

    def __init__(self, store: NavigationStore) -> None:
        self.store = store
        self._lock = threading.RLock()
        self._record = store.load()
        self._phase = IDLE
        self._pending: str | None = None
        self._current_goal: dict[str, Any] | None = None
        self._active = False
        self._reconciled = False
        self._last_status_stamp = -1
        # A per-transition floor: statuses not newer than this goal stamp belong
        # to the goal being replaced and must not end the new transition.
        self._transition_floor = -1
        self._awaiting_active = False
        if self._record is not None:
            self._phase = self._record["phase"]
            if self._phase == PAUSING:
                self._pending = PENDING_PAUSE

    def _clear_record_locked(self) -> None:
        self._record = None
        self.store.clear()

    def _set_record_phase_locked(self, phase: str, now: float) -> None:
        if self._record is None:
            return
        record = dict(self._record)
        record["phase"] = phase
        record["saved_at"] = now
        self.store.save(record)
        self._record = record

    def _has_goal_locked(self) -> bool:
        return (
            self._active or self._current_goal is not None or self._record is not None
        )

    def needs_cancel(self) -> bool:
        with self._lock:
            return self._pending is not None and self._phase in (PAUSING, CLEARING)

    # -- Action status feed -------------------------------------------------

    def note_action_status(
        self, status_name: str, goal_stamp: int, now: float
    ) -> str | None:
        """Apply a selected Action status; stale transition updates are ignored.

        Returns ``"retry_cancel"`` once at startup when an interrupted Pause
        must be retried against the still-active goal.

        After a fresh goal publish or a resume, ``_transition_floor`` holds the
        goal stamp of the replaced goal.  While the new goal is awaiting an
        active status, every update with ``goal_stamp <= floor`` (the replaced
        goal's retained ``canceled``/``succeeded``/``canceling``/``aborted``) is
        discarded; only a newer ``accepted`` or ``executing`` confirms the new
        goal.  A newer ``canceling`` is the new goal already being canceled, so
        it does not confirm the transition and the final terminal is still
        awaited.  A newer terminal that arrives before any confirmation means
        the new goal already ended: a Resume returns to ``paused`` with its
        record kept, and a normal new goal becomes ``idle``.
        """
        with self._lock:
            if goal_stamp < self._last_status_stamp:
                return None
            if self._awaiting_active:
                if goal_stamp <= self._transition_floor:
                    return None
                if status_name in ("accepted", "executing"):
                    self._awaiting_active = False
                    self._transition_floor = -1
                elif status_name in ("canceled", "succeeded", "aborted"):
                    self._awaiting_active = False
                    self._transition_floor = -1
                    self._last_status_stamp = max(self._last_status_stamp, goal_stamp)
                    self._settle_new_goal_terminated_locked(now)
                    return None
                else:
                    # A newer ``canceling``/unknown is not a confirmation: keep
                    # waiting for the terminal status without clearing a
                    # resuming record or marking the resume successful.
                    self._last_status_stamp = max(self._last_status_stamp, goal_stamp)
                    return None
            self._last_status_stamp = max(self._last_status_stamp, goal_stamp)
            action = None
            if not self._reconciled:
                self._reconciled = True
                action = self._reconcile_locked(status_name, now)
            self._apply_status_locked(status_name, now)
            return action

    def _reconcile_locked(self, status_name: str, now: float) -> str | None:
        active = status_name in ACTIVE_STATUS_NAMES
        if self._record is None:
            self._phase = NAVIGATING if active else IDLE
            return None
        phase = self._record["phase"]
        if phase == PAUSING:
            if active:
                self._phase = PAUSING
                self._pending = PENDING_PAUSE
                return "retry_cancel"
            self._set_record_phase_locked(PAUSED, now)
            self._phase = PAUSED
            self._pending = None
            return None
        if phase == RESUMING:
            if active:
                self._clear_record_locked()
                self._phase = NAVIGATING
            else:
                self._set_record_phase_locked(PAUSED, now)
                self._phase = PAUSED
            self._pending = None
            return None
        self._phase = PAUSED
        self._pending = None
        return None

    def note_no_active_goals(self, now: float) -> None:
        """Reconcile the first status-less update, e.g. an empty array.

        At startup the Action server can publish an empty ``GoalStatusArray``
        (or none at all) while a persisted ``pausing``/``resuming`` record
        exists.  Settling that one-time reconciliation here keeps a restarted
        transition from staying stuck forever without inventing a synthetic
        ``goal_stamp=0`` status that would bypass the transition floor.
        ``pausing``/``resuming`` become ``paused`` with the destination kept, a
        record-less control becomes ``idle``, an already ``paused`` record is
        unchanged, and any later empty array is a no-op so an in-flight
        transition is never disturbed.
        """
        with self._lock:
            if self._reconciled:
                return
            self._reconciled = True
            self._reconcile_no_active_locked(now)

    def _reconcile_no_active_locked(self, now: float) -> None:
        if self._record is None:
            # Do not clobber a goal published in this session before the first
            # status arrived; the empty array only means restart-with-no-goal
            # when nothing is pending.
            if self._current_goal is None and not self._awaiting_active:
                self._phase = IDLE
            return
        if self._record["phase"] in (PAUSING, RESUMING):
            self._set_record_phase_locked(PAUSED, now)
        self._phase = PAUSED
        self._pending = None

    def _settle_new_goal_terminated_locked(self, now: float) -> None:
        """Settle a new goal that ended before an active status confirmed it.

        A resuming goal returns to ``paused`` with its record kept so the
        operator can try again; a normal new goal has no destination to keep
        and becomes ``idle``.
        """
        self._active = False
        self._current_goal = None
        if self._phase == RESUMING and self._record is not None:
            self._set_record_phase_locked(PAUSED, now)
            self._phase = PAUSED
        else:
            self._phase = IDLE
        self._pending = None

    def _apply_status_locked(self, status_name: str, now: float) -> None:
        if status_name in ACTIVE_STATUS_NAMES:
            self._active = True
            if self._phase in (RESUMING, CLEARING, PAUSED, IDLE):
                self._clear_record_locked()
                self._phase = NAVIGATING
                self._pending = None
            return
        self._active = False
        if status_name == "canceled":
            if self._pending == PENDING_PAUSE:
                self._set_record_phase_locked(PAUSED, now)
                self._phase = PAUSED
                self._current_goal = None
                self._pending = None
            elif self._pending == PENDING_CLEAR:
                self._clear_record_locked()
                self._current_goal = None
                self._phase = IDLE
                self._pending = None
            return
        if status_name in ("succeeded", "aborted"):
            if self._record is not None:
                if self._phase in (NAVIGATING, CLEARING, RESUMING, PAUSING):
                    self._current_goal = None
                    self._phase = PAUSED
                    self._pending = None
                return
            self._current_goal = None
            if self._phase in (NAVIGATING, CLEARING, RESUMING, PAUSING):
                self._phase = IDLE
                self._pending = None

    # -- Commands -----------------------------------------------------------

    def record_goal_published(
        self,
        pose: dict[str, Any],
        frame: str,
        map_name: str | None,
        now: float,
    ) -> None:
        """Adopt a newly published goal; the old paused record is discarded."""
        with self._lock:
            self._clear_record_locked()
            self._current_goal = {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "z": float(pose.get("z", 0.0)),
                "yaw": float(pose["yaw"]),
                "frame": frame,
                "map_name": map_name,
            }
            self._phase = NAVIGATING
            self._pending = None
            self._active = False
            self._transition_floor = self._last_status_stamp
            self._awaiting_active = True

    def begin_pause(
        self,
        pose: dict[str, Any],
        frame: str,
        map_name: str,
        now: float,
    ) -> dict[str, Any]:
        """Persist the destination as ``pausing``; the caller sends cancel."""
        with self._lock:
            if (
                self._phase != NAVIGATING
                or not self._active
                or self._current_goal is None
                or self._pending is not None
            ):
                raise NavigationConflictError("no active navigation goal to pause")
            record = {
                "schema_version": SCHEMA_VERSION,
                "phase": PAUSING,
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "z": float(pose.get("z", 0.0)),
                "yaw": float(pose["yaw"]),
                "frame": frame,
                "map_name": map_name,
                "saved_at": now,
            }
            self.store.save(record)
            self._record = record
            self._phase = PAUSING
            self._pending = PENDING_PAUSE
            return dict(record)

    def begin_clear(self, now: float) -> bool:
        """Enter clearing; returns True when the caller must cancel the goal."""
        with self._lock:
            if self._phase == PAUSING:
                self._pending = PENDING_CLEAR
                self._phase = CLEARING
                return False
            if self._phase == CLEARING:
                raise NavigationConflictError("clear is already in progress")
            if self._phase == RESUMING or self._pending is not None:
                raise NavigationConflictError(
                    "navigation command in progress; try again"
                )
            if self._phase == PAUSED and self._record is not None:
                self._clear_record_locked()
                self._current_goal = None
                self._active = False
                self._phase = IDLE
                return False
            if self._phase == NAVIGATING:
                self._current_goal = None
                if self._active:
                    self._phase = CLEARING
                    self._pending = PENDING_CLEAR
                    return True
                self._active = False
                self._phase = IDLE
                return False
            raise NavigationConflictError("no navigation goal to clear")

    def begin_resume(self, now: float, current_map: str | None) -> dict[str, Any]:
        """Mark the saved destination ``resuming`` and return it to publish."""
        with self._lock:
            if (
                self._phase != PAUSED
                or self._record is None
                or self._pending is not None
            ):
                raise NavigationConflictError("no paused navigation goal to resume")
            if current_map is None or self._record["map_name"] != current_map:
                raise NavigationConflictError(
                    "paused goal belongs to a different map"
                )
            self._set_record_phase_locked(RESUMING, now)
            self._phase = RESUMING
            self._current_goal = {
                "x": self._record["x"],
                "y": self._record["y"],
                "z": self._record["z"],
                "yaw": self._record["yaw"],
                "frame": self._record["frame"],
                "map_name": self._record["map_name"],
            }
            self._active = False
            self._transition_floor = self._last_status_stamp
            self._awaiting_active = True
            return dict(self._record)

    def note_cancel_failed(self, now: float) -> None:
        """An explicit rejection or unavailable service ends the transition."""
        with self._lock:
            if self._pending == PENDING_PAUSE:
                self._clear_record_locked()
            self._pending = None
            self._phase = NAVIGATING if self._active else IDLE

    def note_cancel_empty(self, now: float) -> None:
        """A clear whose cancel matched no goal is already complete."""
        with self._lock:
            if self._pending == PENDING_CLEAR:
                self._clear_record_locked()
                self._current_goal = None
                self._active = False
                self._phase = IDLE
                self._pending = None

    def note_resume_publish_failed(self, now: float) -> None:
        """A definite publish failure restores the resumable paused state."""
        with self._lock:
            if self._phase != RESUMING or self._record is None:
                return
            self._set_record_phase_locked(PAUSED, now)
            self._phase = PAUSED
            self._pending = None

    def note_map_selected(self, map_name: str | None, now: float) -> None:
        """Drop a paused destination that belongs to a different map."""
        with self._lock:
            if self._record is not None and self._record["map_name"] != map_name:
                self._clear_record_locked()
                if self._phase in (PAUSED, PAUSING):
                    self._phase = NAVIGATING if self._active else IDLE
                    self._pending = None

    def on_navigation_stopped(self, now: float) -> None:
        """Settle the control state when the navigation process stops.

        A process stop is not an operator command: an interrupted Pause or
        Resume keeps its saved destination as ``paused`` so it survives and can
        still be resumed, while Navigating and Clearing settle to ``Idle``.
        """
        with self._lock:
            self._awaiting_active = False
            self._transition_floor = -1
            if self._phase == NAVIGATING:
                self._current_goal = None
                self._active = False
                self._pending = None
                self._phase = IDLE
            elif self._phase == PAUSING:
                self._set_record_phase_locked(PAUSED, now)
                self._current_goal = None
                self._active = False
                self._pending = None
                self._phase = PAUSED
            elif self._phase == RESUMING:
                self._set_record_phase_locked(PAUSED, now)
                self._current_goal = None
                self._active = False
                self._pending = None
                self._phase = PAUSED
            elif self._phase == CLEARING:
                self._clear_record_locked()
                self._current_goal = None
                self._active = False
                self._pending = None
                self._phase = IDLE
            # PAUSED and IDLE already have the right state.

    # -- Read model ---------------------------------------------------------

    def snapshot(self, current_map: str | None) -> dict[str, Any]:
        """Return the server-owned navigation control state for the browser."""
        with self._lock:
            return {
                "state": self._phase,
                "has_goal": self._has_goal_locked(),
                "can_pause": (
                    self._phase == NAVIGATING
                    and self._active
                    and self._current_goal is not None
                    and self._pending is None
                ),
                "can_resume": (
                    self._phase == PAUSED
                    and self._record is not None
                    and current_map is not None
                    and self._record["map_name"] == current_map
                    and self._pending is None
                ),
                "can_clear": (
                    self._has_goal_locked()
                    and self._phase in (NAVIGATING, PAUSED, PAUSING)
                ),
                "paused_goal": (
                    dict(self._record) if self._record is not None else None
                ),
            }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest -q test/test_navigation_control.py`
Expected: all selected tests pass (no `FAILED` or `ERROR` entries). Recount the
file's test functions rather than hard-coding a number; the transition floor,
restart reconciliation, process-stop, and map-change tests all live in this one
module.

- [ ] **Step 5: Commit**

```bash
git add web_control/navigation_control.py test/test_navigation_control.py
git commit -m "feat: add navigation pause/resume state machine"
```

---

### Task 4: ROS wiring, `CancelGoal` cancellation primitive, and `/api/state` capability block

**Files:**
- Modify: `web_control/web_control_node.py`
- Modify: `config/web_control.yaml`
- Test: `test/test_core.py`

This task owns the whole cancellation primitive: the isolated `CancelGoal`
import, the client, the `CANCEL_GOAL_TIMEOUT_SEC` bound, and `_cancel_all_goals`.
Task 5's pause/clear commands only call `_cancel_all_goals`; they must not
redefine it.

- [ ] **Step 1: Write the failing tests**

Append to the end of `test/test_core.py`:

```python
class _FakeCancelResponse:
    def __init__(self, goals_canceling, return_code=0):
        self.goals_canceling = goals_canceling
        self.return_code = return_code


class _FakeCancelGoalRequest:
    class _GoalId:
        def __init__(self):
            self.uuid = []

    class _Stamp:
        def __init__(self):
            self.sec = 0
            self.nanosec = 0

    class _GoalInfo:
        def __init__(self):
            self.goal_id = _FakeCancelGoalRequest._GoalId()
            self.stamp = _FakeCancelGoalRequest._Stamp()

    def __init__(self):
        self.goal_info = _FakeCancelGoalRequest._GoalInfo()


class _FakeCancelGoal:
    Request = _FakeCancelGoalRequest


class _FakeCancelClient:
    def __init__(self, ready=True, future=None, results=(1,), return_code=0):
        self.ready = ready
        self.future = future if future is not None else _FakeFuture()
        self.results = list(results)
        self.return_code = return_code
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        self.future.response = _FakeCancelResponse(
            [object() for _ in self.results], return_code=self.return_code
        )
        return self.future


class _FakeHeader:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""


class _FakePosition:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _FakeOrientation:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class _FakePose:
    def __init__(self):
        self.position = _FakePosition()
        self.orientation = _FakeOrientation()


class _FakePoseStamped:
    def __init__(self):
        self.header = _FakeHeader()
        self.pose = _FakePose()


class _FakePublisher:
    def __init__(self, published):
        self.published = published

    def publish(self, msg):
        self.published.append(msg)


def _paused_record(phase="paused", map_name="site_a"):
    return {
        "schema_version": 1,
        "phase": phase,
        "x": 1.5,
        "y": -2.5,
        "z": 0.25,
        "yaw": 0.75,
        "frame": "map",
        "map_name": map_name,
        "saved_at": 5.0,
    }


def _navigation_node(
    tmp_path,
    record=None,
    selection="site_a",
    client=None,
    ready=True,
    results=(1,),
    methods=("state_payload",),
):
    from web_control.map_selection import SelectionStore
    from web_control.navigation_control import NavigationControl
    from web_control.navigation_store import NavigationStore

    web_control_node.CancelGoal = _FakeCancelGoal
    web_control_node.PoseStamped = _FakePoseStamped
    node = type("_NavigationNodeStub", (), {})()
    node.lock = threading.RLock()
    node.map_frame = "map"
    node.now_sec = lambda: 10.0
    node.ros_stamp = lambda: None
    node.get_logger = lambda: _LoggerStub()
    node.map_selection = SelectionStore(tmp_path / "config" / "selected_map.json")
    if selection is not None:
        node.map_selection.save(selection)
    node.navigation_store = NavigationStore(
        tmp_path / "config" / "paused_navigation.json"
    )
    if record is not None:
        node.navigation_store.save(record)
    node.navigation_control = NavigationControl(node.navigation_store)
    node.published_goal = None
    node.cancel_goal_client = (
        client if client is not None else _FakeCancelClient(ready=ready, results=results)
    )
    if client is None:
        node.cancel_goal_client.future.done_flag = True
    node.goal_publications = []
    node.goal_pub = _FakePublisher(node.goal_publications)
    node.action_status = SourceCache("action_status", 2.0)
    node.odom = SourceCache("odom", 2.0)
    node.layers = {
        name: SourceCache(name, 2.0, static_source=True)
        for name in ("local_map", "mapcloud", "mapground")
    }
    node.paths = {}
    node.viewers = ViewerRegistry(timeout_sec=10.0)
    for name in methods:
        setattr(
            node,
            name,
            MethodType(getattr(web_control_node.WebControlNode, name), node),
        )
    return node


def _arm_active_goal(node, pose=None):
    pose = pose or {"x": 1.5, "y": -2.5, "z": 0.25, "yaw": 0.75}
    # Publish first, then feed the status, mirroring the real node ordering so
    # the transition floor is taken from the previous goal stamp.
    node.navigation_control.record_goal_published(pose, "map", "site_a", 10.0)
    node.navigation_control.note_action_status("executing", 5, 10.0)
    node.published_goal = dict(pose)


def test_node_declares_cancel_goal_service_and_client():
    package_root = Path(__file__).resolve().parents[1]
    source = (package_root / "web_control" / "web_control_node.py").read_text(
        encoding="utf-8"
    )

    assert "from action_msgs.srv import CancelGoal" in source
    assert "CancelGoal = None" in source
    assert '"cancel_goal_service", "/p2p_move_base/_action/cancel_goal"' in source
    assert 'CancelGoal, str(self.get_parameter("cancel_goal_service").value)' in source

    config = (package_root / "config" / "web_control.yaml").read_text(encoding="utf-8")
    assert 'cancel_goal_service: "/p2p_move_base/_action/cancel_goal"' in config


def test_cancel_goal_import_is_isolated_from_main_ros_import():
    package_root = Path(__file__).resolve().parents[1]
    source = (package_root / "web_control" / "web_control_node.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)

    main_ros_try = None
    cancel_goal_try = None
    for statement in tree.body:
        if not isinstance(statement, ast.Try):
            continue
        statements = ast.unparse(statement)
        if "import rclpy" in statements:
            main_ros_try = statement
        if "from action_msgs.srv import CancelGoal" in statements:
            cancel_goal_try = statement

    assert main_ros_try is not None, "main ROS import try block not found"
    assert cancel_goal_try is not None, "isolated CancelGoal import try not found"
    assert cancel_goal_try is not main_ros_try, (
        "CancelGoal must not share the rclpy import try, or a missing service "
        "type would set rclpy to None and disable the whole node"
    )
    assert "from action_msgs.srv import CancelGoal" not in ast.unparse(main_ros_try)
    assert len(cancel_goal_try.handlers) == 1
    assert "CancelGoal = None" in ast.unparse(cancel_goal_try.handlers[0])


def test_action_status_callback_uses_selected_status():
    source = (
        Path(__file__).resolve().parents[1] / "web_control" / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "select_action_status(msg.status_list)" in source
    assert "self.navigation_control.note_action_status(" in source
    assert "self.navigation_control.note_no_active_goals(" in source


def test_state_payload_exposes_navigation_control_snapshot(tmp_path):
    node = _navigation_node(tmp_path, record=_paused_record())

    body = node.state_payload()

    assert body["navigation_control"]["state"] == "paused"
    assert body["navigation_control"]["can_resume"] is True
    assert body["navigation_control"]["paused_goal"]["x"] == 1.5
    assert body["navigation_control"]["paused_goal"]["map_name"] == "site_a"


class _EmptyGoalStatusArray:
    def __init__(self):
        self.status_list = []


def test_action_status_callback_empty_list_reconciles_pausing(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(phase="pausing"),
        methods=("action_status_callback", "state_payload"),
    )

    node.action_status_callback(_EmptyGoalStatusArray())

    assert node.action_status.data == {"status": "idle", "status_values": []}
    assert node.action_status.count == 0
    snapshot = node.state_payload()["navigation_control"]
    assert snapshot["state"] == "paused"
    assert snapshot["can_resume"] is True
    assert node.navigation_store.load()["phase"] == "paused"


def test_action_status_callback_empty_list_and_no_record_is_idle(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=("action_status_callback", "state_payload"),
    )

    node.action_status_callback(_EmptyGoalStatusArray())

    assert node.action_status.data == {"status": "idle", "status_values": []}
    assert node.action_status.count == 0
    snapshot = node.state_payload()["navigation_control"]
    assert snapshot["state"] == "idle"
    assert snapshot["paused_goal"] is None


def test_cancel_all_goals_sends_zero_uuid_and_zero_stamp(tmp_path):
    client = _FakeCancelClient(return_code=0, results=(1,))
    node = _navigation_node(tmp_path, client=client, methods=("_cancel_all_goals",))
    client.future.done_flag = True

    assert node._cancel_all_goals() == "accepted"
    request = client.requests[0]
    assert request.goal_info.goal_id.uuid == [0] * 16
    assert request.goal_info.stamp.sec == 0
    assert request.goal_info.stamp.nanosec == 0


def test_cancel_all_goals_reports_rejected_return_code(tmp_path):
    client = _FakeCancelClient(return_code=2, results=(1,))
    node = _navigation_node(tmp_path, client=client, methods=("_cancel_all_goals",))
    client.future.done_flag = True

    assert node._cancel_all_goals() == "rejected"


def test_cancel_all_goals_distinguishes_accepted_and_empty(tmp_path):
    accepted_client = _FakeCancelClient(return_code=0, results=(1,))
    accepted_node = _navigation_node(
        tmp_path, client=accepted_client, methods=("_cancel_all_goals",)
    )
    accepted_client.future.done_flag = True

    empty_client = _FakeCancelClient(return_code=0, results=())
    empty_node = _navigation_node(
        tmp_path, client=empty_client, methods=("_cancel_all_goals",)
    )
    empty_client.future.done_flag = True

    assert accepted_node._cancel_all_goals() == "accepted"
    assert empty_node._cancel_all_goals() == "empty"


def test_cancel_all_goals_maps_future_error_to_unavailable(tmp_path):
    future = _FakeFuture(error=RuntimeError("service crashed"))
    future.done_flag = True
    client = _FakeCancelClient(future=future)
    node = _navigation_node(tmp_path, client=client, methods=("_cancel_all_goals",))

    with pytest.raises(web_control_node.NavigationUnavailableError):
        node._cancel_all_goals()


def test_cancel_all_goals_times_out(tmp_path, monkeypatch):
    # The wait is bounded, so a service that never answers must fail the request
    # thread instead of blocking the HTTP worker forever.
    future = _FakeFuture()
    client = _FakeCancelClient(future=future)
    node = _navigation_node(tmp_path, client=client, methods=("_cancel_all_goals",))
    monkeypatch.setattr(web_control_node, "CANCEL_GOAL_TIMEOUT_SEC", 0.0)

    with pytest.raises(web_control_node.NavigationTimeoutError):
        node._cancel_all_goals()

    assert future.cancelled is True

```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest -q test/test_core.py -k "cancel_goal_service or cancel_goal_import or selected_status or navigation_control_snapshot or empty_list or cancel_all_goals"`
Expected: every selected test fails. The source strings, the isolated
`CancelGoal` import, `_cancel_all_goals`, and the `navigation_control` snapshot
are all missing, and the empty-list callback tests see the old early-`return` so
`action_status.data` stays `None` and the persisted `pausing` record is never
reconciled. Recount the selected tests rather than hard-coding a failure count.

- [ ] **Step 3: Write minimal implementation**

In `web_control/web_control_node.py`, leave the existing main ROS import `try`
block unchanged except that it must **not** import the cancel service. Add a
separate, independent import immediately after the main `try/except
ModuleNotFoundError` block (before the `lio_sam_hesai` import):

```python
try:
    from action_msgs.srv import CancelGoal
except ModuleNotFoundError:
    CancelGoal = None
```

The cancel service lives in its own `try` so a system that ships
`action_msgs.msg.GoalStatusArray` and `rclpy` but not `action_msgs.srv.CancelGoal`
still imports the node with a working `rclpy`; only pause/clear are rejected with
HTTP 503. Folding `CancelGoal` into the main ROS import would make the whole
`except` run, set `rclpy = None`, and disable the entire web node. This split is
enforced by `test_cancel_goal_import_is_isolated_from_main_ros_import`.

After the line `from web_control.map_selection import SelectionStore, default_selection_path`, add:

```python
from web_control.navigation_control import (
    CLEARING,
    NavigationConflictError,
    NavigationControl,
    NavigationTimeoutError,
    NavigationUnavailableError,
    PAUSING,
    RESUMING,
    select_action_status,
)
from web_control.navigation_store import NavigationStore, default_navigation_path
```

Add a timeout constant next to the existing stop timeouts:

```python
CANCEL_GOAL_TIMEOUT_SEC = 2.0
```

In `WebControlNode.__init__`, after the `action_status_topic` declaration, add:

```python
        self.declare_parameter(
            "cancel_goal_service", "/p2p_move_base/_action/cancel_goal"
        )
```

After `self.map_selection = SelectionStore(default_selection_path())`, add:

```python
        self.navigation_store = NavigationStore(default_navigation_path())
        self.navigation_control = NavigationControl(self.navigation_store)
        self.published_goal: dict[str, Any] | None = None
```

Immediately before `self.initialpose_pub = self.create_publisher(`, add:

```python
        self.cancel_goal_client: Any | None = None
        if CancelGoal is not None:
            self.cancel_goal_client = self.create_client(
                CancelGoal, str(self.get_parameter("cancel_goal_service").value)
            )
        else:
            self.get_logger().warning(
                "action_msgs CancelGoal srv is unavailable; pause and clear "
                "requests will be rejected"
            )
```

Replace the whole `action_status_callback` method with:

```python
    def action_status_callback(self, msg: GoalStatusArray) -> None:
        selected = select_action_status(msg.status_list)
        now = self.now_sec()
        if selected is None:
            with self.lock:
                self.action_status.update(
                    {"status": "idle", "status_values": []},
                    stamp=now,
                    frame_id="",
                    count=0,
                )
            self.navigation_control.note_no_active_goals(now)
            return
        with self.lock:
            self.action_status.update(
                {"status": selected["name"], "status_values": selected["values"]},
                stamp=now,
                frame_id="",
                count=len(selected["values"]),
            )
        action = self.navigation_control.note_action_status(
            selected["name"], selected["goal_stamp"], now
        )
        if action == "retry_cancel":
            threading.Thread(
                target=self._retry_startup_cancel,
                name="web_control_retry_cancel",
                daemon=True,
            ).start()

    def _cancel_all_goals(self) -> str:
        """Cancel every server goal with a zero UUID.

        Returns ``"accepted"`` when the service accepts the request and reports
        goals canceling, ``"empty"`` when it accepts but matches no goal, and
        ``"rejected"`` for any non-zero ``return_code``.  A raised service
        future is mapped to :class:`NavigationUnavailableError`.
        """
        client = self.cancel_goal_client
        if client is None:
            raise NavigationUnavailableError("cancel goal service unavailable")
        try:
            ready = bool(client.service_is_ready())
        except Exception:
            ready = False
        if not ready:
            raise NavigationUnavailableError("cancel goal service unavailable")
        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        future = client.call_async(request)
        deadline = time.monotonic() + CANCEL_GOAL_TIMEOUT_SEC
        while not future.done():
            if time.monotonic() >= deadline:
                future.cancel()
                raise NavigationTimeoutError("cancel goal request timed out")
            time.sleep(0.01)
        try:
            response = future.result()
        except Exception as exc:
            raise NavigationUnavailableError(
                f"cancel goal service call failed: {exc}"
            ) from exc
        if response is None:
            raise NavigationUnavailableError("cancel goal service returned no response")
        if int(getattr(response, "return_code", 0)) != 0:
            return "rejected"
        results = list(getattr(response, "goals_canceling", []) or [])
        return "accepted" if results else "empty"

    def _retry_startup_cancel(self) -> None:
        try:
            result = self._cancel_all_goals()
        except (NavigationUnavailableError, NavigationTimeoutError):
            return
        if result != "accepted":
            self.navigation_control.note_cancel_failed(self.now_sec())
```

`_cancel_all_goals` is committed here, together with the client and timeout
constant, so `_retry_startup_cancel` never references an undefined method inside
this commit. The `time` and `Any` imports already exist in the module. Task 5
calls `_cancel_all_goals` from `pause_navigation`/`clear_navigation` but must not
define it again.

In `state_payload`, replace

```python
                "viewers": self.viewers.count(now),
                "map_frame": self.map_frame,
```

with

```python
                "viewers": self.viewers.count(now),
                "map_frame": self.map_frame,
                "navigation_control": self.navigation_control.snapshot(
                    self.map_selection.load()
                ),
```

In `config/web_control.yaml`, after the `action_status_topic` line, add:

```yaml
    cancel_goal_service: "/p2p_move_base/_action/cancel_goal"  # Cancel goal service
```

Do not touch `package.xml`; `action_msgs` is already declared.

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest -q test/test_core.py -k "cancel_goal_service or cancel_goal_import or selected_status or navigation_control_snapshot or empty_list or cancel_all_goals"`
Expected: every selected test passes with no `FAILED`/`ERROR` entries. Recount
the test functions inserted by this task instead of hard-coding a pass count.

- [ ] **Step 5: Commit**

```bash
git add web_control/web_control_node.py config/web_control.yaml test/test_core.py
git commit -m "feat: expose navigation control state and cancel goal client primitive"
```

---

### Task 5: Pause, resume, and clear command methods

**Files:**
- Modify: `web_control/web_control_node.py`
- Test: `test/test_core.py`

- [ ] **Step 1: Write the failing tests**

Append to the end of `test/test_core.py`:

```python
def test_pause_navigation_persists_pausing_then_canceled_reaches_paused(tmp_path):
    # The zero-UUID/zero-stamp request shape is asserted by Task 4's
    # ``test_cancel_all_goals_sends_zero_uuid_and_zero_stamp``; this test owns the
    # pause behavior around the accepted cancel.
    node = _navigation_node(
        tmp_path,
        methods=("_cancel_all_goals", "pause_navigation"),
    )
    _arm_active_goal(node)

    response = node.pause_navigation()

    assert response["navigation_control"]["state"] == "pausing"
    assert response["navigation_control"]["paused_goal"]["phase"] == "pausing"
    assert len(node.cancel_goal_client.requests) == 1

    node.navigation_control.note_action_status("canceled", 5, 11.0)
    assert node.navigation_control.snapshot("site_a")["state"] == "paused"
    assert node.navigation_store.load()["phase"] == "paused"


def test_pause_navigation_timeout_keeps_pausing_record(tmp_path, monkeypatch):
    future = _FakeFuture()
    client = _FakeCancelClient(ready=True, future=future)
    node = _navigation_node(
        tmp_path,
        client=client,
        methods=("_cancel_all_goals", "pause_navigation"),
    )
    _arm_active_goal(node)
    monkeypatch.setattr(web_control_node, "CANCEL_GOAL_TIMEOUT_SEC", 0.0)

    with pytest.raises(web_control_node.NavigationTimeoutError):
        node.pause_navigation()

    assert node.navigation_control.snapshot("site_a")["state"] == "pausing"
    assert node.navigation_store.load()["phase"] == "pausing"


def test_pause_navigation_unavailable_removes_record(tmp_path):
    node = _navigation_node(
        tmp_path,
        ready=False,
        methods=("_cancel_all_goals", "pause_navigation"),
    )
    _arm_active_goal(node)

    with pytest.raises(web_control_node.NavigationUnavailableError):
        node.pause_navigation()

    assert node.navigation_store.load() is None
    assert node.navigation_control.snapshot("site_a")["state"] == "navigating"


def test_pause_navigation_empty_cancel_removes_record(tmp_path):
    node = _navigation_node(
        tmp_path,
        results=(),
        methods=("_cancel_all_goals", "pause_navigation"),
    )
    _arm_active_goal(node)

    with pytest.raises(web_control_node.NavigationConflictError):
        node.pause_navigation()

    assert node.navigation_store.load() is None
    assert node.navigation_control.snapshot("site_a")["state"] == "navigating"


def test_pause_navigation_rejected_cancel_restores_navigating(tmp_path):
    client = _FakeCancelClient(return_code=1, results=(1,))
    node = _navigation_node(
        tmp_path,
        client=client,
        methods=("_cancel_all_goals", "pause_navigation"),
    )
    client.future.done_flag = True
    _arm_active_goal(node)

    with pytest.raises(web_control_node.NavigationConflictError):
        node.pause_navigation()

    assert node.navigation_store.load() is None
    assert node.navigation_control.snapshot("site_a")["state"] == "navigating"


def test_clear_paused_goal_skips_cancel_service(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=("state_payload", "_cancel_all_goals", "clear_navigation"),
    )

    response = node.clear_navigation()

    assert response["navigation_control"]["state"] == "idle"
    assert node.cancel_goal_client.requests == []
    assert node.navigation_store.load() is None


def test_resume_republishes_saved_pose_and_enters_resuming(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=("state_payload", "_publish_goal_pose", "resume_navigation"),
    )

    response = node.resume_navigation()

    assert response["navigation_control"]["state"] == "resuming"
    message = node.goal_publications[-1]
    assert message.header.frame_id == "map"
    assert message.pose.position.x == 1.5
    assert message.pose.position.y == -2.5
    assert message.pose.position.z == 0.25
    assert node.navigation_store.load()["phase"] == "resuming"

    node.navigation_control.note_action_status("accepted", 9, 30.0)
    assert node.navigation_control.snapshot("site_a")["state"] == "navigating"
    assert node.navigation_store.load() is None


def test_clear_while_pausing_reuses_cancel(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=(
            "_publish_goal_pose",
            "_cancel_all_goals",
            "pause_navigation",
            "clear_navigation",
        ),
    )
    _arm_active_goal(node)
    node.pause_navigation()

    response = node.clear_navigation()

    assert response["navigation_control"]["state"] == "clearing"
    assert len(node.cancel_goal_client.requests) == 1

    node.navigation_control.note_action_status("canceled", 5, 12.0)
    assert node.navigation_control.snapshot("site_a")["state"] == "idle"


def test_clear_navigation_runtime_state_settles_control_and_goal(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=(
            "_publish_goal_pose",
            "_cancel_all_goals",
            "pause_navigation",
            "clear_navigation_runtime_state",
        ),
    )
    _arm_active_goal(node)
    node.pause_navigation()
    assert node.navigation_control.snapshot("site_a")["state"] == "pausing"

    node.clear_navigation_runtime_state()

    # Stopping navigation must not leave a phantom active goal in the browser.
    assert node.published_goal is None
    snapshot = node.navigation_control.snapshot("site_a")
    assert snapshot["state"] == "paused"
    assert snapshot["can_resume"] is True
    assert node.navigation_store.load()["phase"] == "paused"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest -q test/test_core.py -k "pause_navigation or clear_paused_goal or resume_republishes or clear_while_pausing or navigation_runtime_state"`
Expected: every selected test fails. `_cancel_all_goals` already exists from
Task 4, but this task's command methods (`_publish_goal_pose`,
`pause_navigation`, `resume_navigation`, `clear_navigation`) do not exist yet
(`AttributeError: type object 'WebControlNode' has no attribute
'pause_navigation'`), so each behavior test fails at its first command call;
`clear_navigation_runtime_state` does not yet settle the control state or clear
`published_goal`. Recount the selected tests rather than hard-coding a failure
count.

- [ ] **Step 3: Write minimal implementation**

In `web_control/web_control_node.py`, insert these methods immediately before `def publish_goal`:

```python
    def _publish_goal_pose(
        self, pose: dict[str, Any], frame: str | None = None
    ) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.ros_stamp()
        msg.header.frame_id = frame or self.map_frame
        msg.pose.position.x = float(pose["x"])
        msg.pose.position.y = float(pose["y"])
        msg.pose.position.z = float(pose.get("z", 0.0))
        q = quaternion_from_yaw(float(pose["yaw"]))
        msg.pose.orientation.x = q["x"]
        msg.pose.orientation.y = q["y"]
        msg.pose.orientation.z = q["z"]
        msg.pose.orientation.w = q["w"]
        self.goal_pub.publish(msg)

    def pause_navigation(self) -> dict[str, Any]:
        map_name = self.map_selection.load()
        if map_name is None:
            raise NavigationConflictError("no map selected; cannot pause navigation")
        with self.lock:
            goal = dict(self.published_goal) if self.published_goal else None
        if goal is None:
            raise NavigationConflictError("no active navigation goal to pause")
        self.navigation_control.begin_pause(
            goal, self.map_frame, map_name, self.now_sec()
        )
        try:
            result = self._cancel_all_goals()
        except NavigationTimeoutError:
            raise
        except NavigationUnavailableError:
            self.navigation_control.note_cancel_failed(self.now_sec())
            raise
        if result != "accepted":
            # ``empty`` and ``rejected`` both mean nothing is canceling, so the
            # pause never started; restore the previous navigating state.
            self.navigation_control.note_cancel_failed(self.now_sec())
            raise NavigationConflictError("no active navigation goal to cancel")
        response = success_response("pause requested", self.now_sec())
        response["navigation_control"] = self.navigation_control.snapshot(map_name)
        return response

    def resume_navigation(self) -> dict[str, Any]:
        map_name = self.map_selection.load()
        record = self.navigation_control.begin_resume(self.now_sec(), map_name)
        try:
            self._publish_goal_pose(record, record["frame"])
        except Exception as exc:
            self.navigation_control.note_resume_publish_failed(self.now_sec())
            raise NavigationUnavailableError(f"resume publish failed: {exc}") from exc
        with self.lock:
            self.published_goal = {
                "x": record["x"],
                "y": record["y"],
                "z": record["z"],
                "yaw": record["yaw"],
            }
        response = success_response("resume published", self.now_sec())
        response["navigation_control"] = self.navigation_control.snapshot(map_name)
        return response

    def clear_navigation(self) -> dict[str, Any]:
        map_name = self.map_selection.load()
        needs_cancel = self.navigation_control.begin_clear(self.now_sec())
        if needs_cancel:
            try:
                result = self._cancel_all_goals()
            except NavigationTimeoutError:
                raise
            except NavigationUnavailableError:
                self.navigation_control.note_cancel_failed(self.now_sec())
                raise
            if result == "empty":
                self.navigation_control.note_cancel_empty(self.now_sec())
            elif result == "rejected":
                # The service refused; the goal may still be running, so keep
                # the previous state instead of pretending the clear finished.
                self.navigation_control.note_cancel_failed(self.now_sec())
                raise NavigationUnavailableError("cancel goal request rejected")
        with self.lock:
            self.published_goal = None
        response = success_response("clear requested", self.now_sec())
        response["navigation_control"] = self.navigation_control.snapshot(map_name)
        return response
```

The `PoseStamped` import already exists in the module. `_cancel_all_goals` is
defined and committed in Task 4; these methods only call it, so this task adds no
cancel-client code and no `time` loop of its own.

Then replace the existing `clear_navigation_runtime_state` body with:

```python
    def clear_navigation_runtime_state(self) -> None:
        with self.lock:
            self.action_status.clear()
            for cache in self.paths.values():
                cache.clear()
            # mapcloud/mapground are published by the navigation process; drop
            # their last cached clouds so the viewer stops showing them after
            # navigation stops. local_map is localization-owned, leave it.
            for name in ("mapcloud", "mapground"):
                if name in self.layers:
                    self.layers[name].clear()
            # Stop is not an operator command: settle the pause/resume state so
            # the browser stops advertising an active goal, while an
            # interrupted Pause or Resume keeps its saved destination paused.
            self.published_goal = None
            self.navigation_control.on_navigation_stopped(self.now_sec())
```

`handle_managed_exit("navigation")` already calls this method, so the navigation
stop branch is covered without a second edit. Clearing `published_goal` here is
what prevents the Target Pose buttons from still showing an active goal after
Stop.

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest -q test/test_core.py -k "pause_navigation or clear_paused_goal or resume_republishes or clear_while_pausing or navigation_runtime_state"`
Expected: every selected test passes with no `FAILED`/`ERROR` entries. Recount
the test functions inserted by this task instead of hard-coding a pass count.
This task's pause tests cover the accepted/empty/rejected/unavailable/timeout
outcomes; the cancel-primitive internals live in Task 4.

- [ ] **Step 5: Commit**

```bash
git add web_control/web_control_node.py test/test_core.py
git commit -m "feat: add pause, resume, and clear command handling"
```

---

### Task 6: Replacement goal semantics and map-change invalidation

**Files:**
- Modify: `web_control/web_control_node.py`
- Test: `test/test_core.py`

- [ ] **Step 1: Write the failing tests**

In `test/test_core.py`, inside `_selection_node`, immediately before the `for name in (` binding loop, add:

```python
    from web_control.navigation_control import NavigationControl
    from web_control.navigation_store import NavigationStore

    node.navigation_store = NavigationStore(
        tmp_path / "config" / "paused_navigation.json"
    )
    node.navigation_control = NavigationControl(node.navigation_store)
```

Then append these tests to the end of `test/test_core.py`:

```python
def test_publish_goal_rejected_while_pausing(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=(
            "_publish_goal_pose",
            "_cancel_all_goals",
            "pause_navigation",
            "publish_goal",
        ),
    )
    _arm_active_goal(node)
    node.pause_navigation()

    with pytest.raises(web_control_node.NavigationConflictError):
        node.publish_goal({"x": 3.0, "y": 4.0, "yaw": 0.0})

    assert node.goal_publications == []
    assert node.navigation_control.snapshot("site_a")["state"] == "pausing"


def test_new_goal_discards_paused_record_after_publish(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=("_publish_goal_pose", "publish_goal"),
    )

    response = node.publish_goal({"x": 5.0, "y": 6.0, "yaw": 1.0})

    assert response["navigation_control"]["state"] == "navigating"
    assert node.navigation_store.load() is None
    assert len(node.goal_publications) == 1
    assert node.goal_publications[0].pose.position.x == 5.0


def test_select_map_clears_paused_goal_from_previous_map(tmp_path):
    root = tmp_path / "map"
    _make_map_dir(root, "site_a")
    _make_map_dir(root, "site_b")
    node = _selection_node(tmp_path, selection="site_a")
    node.navigation_store.save(_paused_record(map_name="site_a"))

    status, body = _serve_and_request(
        node, "/api/maps/select", {"name": "site_b"}, "POST"
    )

    assert status == 200
    assert body["selection"]["name"] == "site_b"
    assert node.navigation_store.load() is None
    assert node.navigation_control.snapshot("site_b")["state"] == "idle"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest -q test/test_core.py -k "publish_goal_rejected or new_goal_discards or select_map_clears_paused"`
Expected: `3 failed` (the pausing rejection is not raised, the paused record survives a new goal, and `select_map` leaves the old record).

- [ ] **Step 3: Write minimal implementation**

In `web_control/web_control_node.py`, replace the existing `publish_goal` method with:

```python
    def publish_goal(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_name = self.map_selection.load()
        snapshot = self.navigation_control.snapshot(map_name)
        if snapshot["state"] in (PAUSING, RESUMING, CLEARING):
            raise NavigationConflictError(
                "navigation is busy; wait for the current command to finish"
            )
        pose = validate_pose_payload(payload)
        self._publish_goal_pose(pose)
        now = self.now_sec()
        self.navigation_control.record_goal_published(
            pose, self.map_frame, map_name, now
        )
        with self.lock:
            self.published_goal = {
                "x": pose["x"],
                "y": pose["y"],
                "z": pose["z"],
                "yaw": pose["yaw"],
            }
        response = success_response("goal published", now)
        response["navigation_control"] = self.navigation_control.snapshot(map_name)
        return response
```

In `select_map`, replace

```python
            self.map_selection.save(name)
            selection = {
```

with

```python
            self.map_selection.save(name)
            self.navigation_control.note_map_selected(name, self.now_sec())
            selection = {
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest -q test/test_core.py -k "publish_goal_rejected or new_goal_discards or select_map_clears_paused"`
Expected: `3 passed`.

- [ ] **Step 5: Commit**

```bash
git add web_control/web_control_node.py test/test_core.py
git commit -m "feat: replace a paused goal only after a new publish succeeds"
```

---

### Task 7: HTTP endpoints and error mapping

**Files:**
- Modify: `web_control/web_control_node.py`
- Test: `test/test_core.py`

- [ ] **Step 1: Write the failing tests**

Append to the end of `test/test_core.py`:

```python
def test_state_endpoint_reports_paused_navigation_controls(tmp_path):
    node = _navigation_node(tmp_path, record=_paused_record())

    status, body = _serve_and_request(node, "/api/state")

    assert status == 200
    assert body["navigation_control"]["state"] == "paused"
    assert body["navigation_control"]["can_resume"] is True
    assert body["navigation_control"]["can_clear"] is True
    assert body["navigation_control"]["can_pause"] is False


def test_pause_endpoint_returns_409_when_no_active_goal(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=("_cancel_all_goals", "pause_navigation"),
    )

    status, body = _serve_and_request(node, "/api/navigation/pause", {}, "POST")

    assert status == 409
    assert body["ok"] is False
    assert node.cancel_goal_client.requests == []


def test_clear_endpoint_returns_200_for_paused_goal(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=("_cancel_all_goals", "clear_navigation"),
    )

    status, body = _serve_and_request(node, "/api/navigation/clear", {}, "POST")

    assert status == 200
    assert body["navigation_control"]["state"] == "idle"
    assert body["navigation_control"]["paused_goal"] is None


def test_pause_endpoint_returns_504_on_timeout(tmp_path, monkeypatch):
    node = _navigation_node(
        tmp_path,
        client=_FakeCancelClient(ready=True, future=_FakeFuture()),
        methods=("_cancel_all_goals", "pause_navigation"),
    )
    _arm_active_goal(node)
    monkeypatch.setattr(web_control_node, "CANCEL_GOAL_TIMEOUT_SEC", 0.0)

    status, body = _serve_and_request(node, "/api/navigation/pause", {}, "POST")

    assert status == 504
    assert body["ok"] is False
    assert node.navigation_control.snapshot("site_a")["state"] == "pausing"
    assert node.navigation_store.load()["phase"] == "pausing"


def test_pause_endpoint_returns_503_when_service_missing(tmp_path):
    node = _navigation_node(
        tmp_path,
        ready=False,
        methods=("_cancel_all_goals", "pause_navigation"),
    )
    _arm_active_goal(node)

    status, body = _serve_and_request(node, "/api/navigation/pause", {}, "POST")

    assert status == 503
    assert body["ok"] is False


def test_goal_endpoint_rejected_while_pausing(tmp_path):
    node = _navigation_node(
        tmp_path,
        methods=(
            "_publish_goal_pose",
            "_cancel_all_goals",
            "pause_navigation",
            "publish_goal",
        ),
    )
    _arm_active_goal(node)
    node.pause_navigation()

    status, body = _serve_and_request(
        node, "/api/goal", {"x": 3.0, "y": 4.0, "yaw": 0.0}, "POST"
    )

    assert status == 409
    assert body["ok"] is False


def test_resume_endpoint_republishes_paused_goal(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=("state_payload", "_publish_goal_pose", "resume_navigation"),
    )

    status, body = _serve_and_request(node, "/api/navigation/resume", {}, "POST")

    assert status == 200
    assert body["ok"] is True
    assert body["navigation_control"]["state"] == "resuming"
    assert node.goal_publications[-1].pose.position.x == 1.5
    assert node.navigation_store.load()["phase"] == "resuming"


def test_clear_endpoint_returns_409_while_resuming(tmp_path):
    node = _navigation_node(
        tmp_path,
        record=_paused_record(),
        methods=(
            "state_payload",
            "_publish_goal_pose",
            "resume_navigation",
            "clear_navigation",
        ),
    )
    node.resume_navigation()

    status, body = _serve_and_request(node, "/api/navigation/clear", {}, "POST")

    assert status == 409
    assert body["ok"] is False
    assert node.navigation_control.snapshot("site_a")["state"] == "resuming"
    assert node.navigation_store.load()["phase"] == "resuming"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest -q test/test_core.py -k "state_endpoint_reports or pause_endpoint or clear_endpoint or goal_endpoint_rejected or resume_endpoint"`
Expected: all selected tests fail: `/api/navigation/pause`, `/api/navigation/resume`, and `/api/navigation/clear` return HTTP 404 `{"ok": false, "message": "unknown endpoint"}`.

- [ ] **Step 3: Write minimal implementation**

In `WebControlNode.make_handler`'s `do_POST`, after the `/api/goal` branch, add:

```python
                    elif self.path == "/api/navigation/pause":
                        self.send_json(node.pause_navigation())
                    elif self.path == "/api/navigation/resume":
                        self.send_json(node.resume_navigation())
                    elif self.path == "/api/navigation/clear":
                        self.send_json(node.clear_navigation())
```

In the same `do_POST` `except` chain, immediately before `except SaveMapBusyError as exc:`, add:

```python
                except NavigationTimeoutError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=504)
                except NavigationUnavailableError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=503)
                except NavigationConflictError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=409)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest -q test/test_core.py -k "state_endpoint_reports or pause_endpoint or clear_endpoint or goal_endpoint_rejected or resume_endpoint"`
Expected: `8 passed` (all three endpoints covered: pause 409/504/503, resume
200, and clear 200-paused/409-resuming).

- [ ] **Step 5: Commit**

```bash
git add web_control/web_control_node.py test/test_core.py
git commit -m "feat: serve navigation pause, resume, and clear endpoints"
```

---

### Task 8: Frontend pause/resume controls

**Files:**
- Modify: `static/index.html`
- Modify: `static/app.js`
- Test: `test/test_core.py`

- [ ] **Step 1: Write the failing tests**

Append to the end of `test/test_core.py`:

```python
_NAVIGATION_PANEL_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const start = source.indexOf("// --- Navigation pause/resume controls");
const end = source.indexOf("// --- End navigation pause/resume controls");
if (start < 0 || end < 0) throw new Error("navigation control block not found");
const block = source.slice(start, end);
const publishStart = source.indexOf("function updatePublishPoseButton()");
if (publishStart < 0) throw new Error("updatePublishPoseButton not found");
const publishEnd = source.indexOf("\n}", publishStart);
if (publishEnd < 0) throw new Error("updatePublishPoseButton end not found");
const publishBlock = source.slice(publishStart, publishEnd + 2);

function makeElement() {
  return { textContent: "", disabled: false };
}
const elements = {
  navigationPrimary: makeElement(),
  navigationClear: makeElement(),
  navigationControlState: makeElement(),
  publishResult: makeElement(),
  publishPose: makeElement(),
};
global.document = {
  getElementById(id) { return elements[id] || null; },
};

let state = { mode: "goal", navigationControl: null, lastGoal: null };
let clearedScenes = 0;
let poseUpdates = 0;
let markerUpdates = 0;
let polled = 0;
let captured = null;

function clearNavigationSceneState() { clearedScenes += 1; state.lastGoal = null; }
function updateTargetPoseDisplay() { poseUpdates += 1; }
function updateTargetMarkers() { markerUpdates += 1; }
function pollState() { polled += 1; }
global.postJson = async (url, payload) => {
  captured = { url, payload };
  return {
    ok: true,
    message: "pause requested",
    stamp: 100.0,
    navigation_control: {
      state: "pausing", has_goal: true, can_pause: false,
      can_resume: false, can_clear: true, paused_goal: null,
    },
  };
};

eval(publishBlock + "\n" + block);

function expect(condition, message) { if (!condition) throw new Error(message); }

updateNavigationControls({ state: "navigating", has_goal: true, can_pause: true, can_resume: false, can_clear: true, paused_goal: null });
expect(elements.navigationPrimary.textContent === "Pause Navigation", "navigating primary label");
expect(elements.navigationPrimary.disabled === false, "navigating primary enabled");
expect(elements.navigationClear.disabled === false, "navigating clear enabled");
expect(elements.navigationControlState.textContent === "Navigating", "navigating state label");
expect(elements.publishPose.disabled === false, "navigating publish enabled");

updateNavigationControls({ state: "pausing", has_goal: true, can_pause: false, can_resume: false, can_clear: true, paused_goal: null });
expect(elements.navigationPrimary.textContent === "Pausing...", "pausing label");
expect(elements.navigationPrimary.disabled === true, "pausing disabled");
expect(elements.publishPose.disabled === true, "pausing publish disabled");

updateNavigationControls({ state: "paused", has_goal: true, can_pause: false, can_resume: true, can_clear: true, paused_goal: { x: 1, y: 2, z: 0, yaw: 0.5 } });
expect(elements.navigationPrimary.textContent === "Resume Navigation", "paused label");
expect(elements.navigationPrimary.disabled === false, "paused primary enabled");
expect(elements.publishPose.disabled === false, "paused publish enabled");
expect(state.lastGoal && state.lastGoal.x === 1, "paused destination not shown");
expect(poseUpdates > 0 && markerUpdates > 0, "paused destination not rendered");

updateNavigationControls({ state: "resuming", has_goal: true, can_pause: false, can_resume: false, can_clear: false, paused_goal: null });
expect(elements.navigationPrimary.textContent === "Resuming...", "resuming label");
expect(elements.navigationPrimary.disabled === true, "resuming disabled");
expect(elements.publishPose.disabled === true, "resuming publish disabled");

const before = clearedScenes;
updateNavigationControls({ state: "clearing", has_goal: true, can_pause: false, can_resume: false, can_clear: false, paused_goal: null });
expect(elements.navigationClear.textContent === "Clearing...", "clearing label");
expect(elements.navigationClear.disabled === true, "clearing disabled");
expect(elements.publishPose.disabled === true, "clearing publish disabled");

updateNavigationControls({ state: "idle", has_goal: false, can_pause: false, can_resume: false, can_clear: false, paused_goal: null });
expect(clearedScenes > before, "terminal cleanup not run");
expect(elements.navigationPrimary.textContent === "Pause Navigation", "idle primary label");
expect(elements.navigationPrimary.disabled === true, "idle primary disabled");
expect(elements.navigationClear.disabled === true, "idle clear disabled");
expect(elements.publishPose.disabled === false, "idle publish enabled");

(async () => {
  await postNavigationCommand("/api/navigation/pause", "Pause failed");
  expect(captured && captured.url === "/api/navigation/pause", "command url wrong");
  expect(elements.navigationControlState.textContent === "Pausing", "command response not applied");
  expect(elements.publishPose.disabled === true, "command response did not disable publish");
  expect(polled > 0, "poll after command missing");
  console.log("navigation control behavior ok");
})().catch((error) => { console.error(error); process.exit(1); });
"""


def test_navigation_page_has_pause_resume_controls():
    index_html = (
        Path(__file__).resolve().parents[1] / "static" / "index.html"
    ).read_text(encoding="utf-8")

    assert 'id="navigationPrimary"' in index_html
    assert 'id="navigationClear"' in index_html
    assert 'id="navigationControlState"' in index_html


def test_frontend_navigation_controls_behavior_executes_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _NAVIGATION_PANEL_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "navigation control behavior ok" in completed.stdout
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest -q test/test_core.py -k "pause_resume_controls or navigation_controls_behavior"`
Expected: `2 failed` (`id="navigationPrimary"` is absent and the harness reports `navigation control block not found`).

- [ ] **Step 3: Write minimal implementation**

In `static/index.html`, replace the Target Pose `<section>` body

```html
        <section>
          <h2>Target Pose</h2>
          <dl class="metrics">
            <dt>x</dt><dd id="goalX">--</dd>
            <dt>y</dt><dd id="goalY">--</dd>
            <dt>z</dt><dd id="goalZ">--</dd>
            <dt>yaw</dt><dd id="goalYaw">--</dd>
          </dl>
        </section>
```

with

```html
        <section>
          <h2>Target Pose</h2>
          <dl class="metrics">
            <dt>x</dt><dd id="goalX">--</dd>
            <dt>y</dt><dd id="goalY">--</dd>
            <dt>z</dt><dd id="goalZ">--</dd>
            <dt>yaw</dt><dd id="goalYaw">--</dd>
          </dl>
          <div class="actions">
            <button id="navigationPrimary" type="button" disabled>Pause Navigation</button>
            <button id="navigationClear" type="button" disabled>Clear Goal</button>
          </div>
          <p id="navigationControlState" class="result" role="status" aria-live="polite">Idle</p>
        </section>
```

In `static/app.js`, add `navigationControl: null,` to the `state` object (after `navigationAction: null,`).

Immediately after the definition of `updateTargetPoseDisplay`, insert this delimited block:

```javascript
// --- Navigation pause/resume controls
function updateNavigationControls(control) {
  const previous = state.navigationControl;
  state.navigationControl = control || null;
  const primary = document.getElementById("navigationPrimary");
  const clear = document.getElementById("navigationClear");
  const status = document.getElementById("navigationControlState");
  const value = control || {
    state: "idle",
    has_goal: false,
    can_pause: false,
    can_resume: false,
    can_clear: false,
    paused_goal: null,
  };
  const labels = {
    idle: "Idle",
    navigating: "Navigating",
    pausing: "Pausing",
    paused: "Paused",
    resuming: "Resuming",
    clearing: "Clearing",
  };
  status.textContent = labels[value.state] || "Idle";
  if (value.state === "pausing") {
    primary.textContent = "Pausing...";
    primary.disabled = true;
  } else if (value.state === "resuming") {
    primary.textContent = "Resuming...";
    primary.disabled = true;
  } else if (value.state === "paused") {
    primary.textContent = "Resume Navigation";
    primary.disabled = !value.can_resume;
  } else {
    primary.textContent = "Pause Navigation";
    primary.disabled = !value.can_pause;
  }
  clear.textContent = value.state === "clearing" ? "Clearing..." : "Clear Goal";
  clear.disabled = !value.can_clear;
  if (value.paused_goal) {
    state.lastGoal = { ...value.paused_goal };
    updateTargetPoseDisplay();
    updateTargetMarkers();
  }
  if (!value.has_goal && previous && previous.has_goal && !value.paused_goal) {
    clearNavigationSceneState();
  }
  updatePublishPoseButton();
}

async function postNavigationCommand(path, failureLabel) {
  const result = document.getElementById("publishResult");
  try {
    const response = await postJson(path, {});
    if (response.navigation_control) {
      updateNavigationControls(response.navigation_control);
    }
    result.textContent = `${response.message} @ ${new Date(response.stamp * 1000).toLocaleTimeString()}`;
    pollState();
  } catch (error) {
    // The server state stays authoritative; the periodic poll keeps rendering
    // the transition even after a timeout or conflict.
    result.textContent = `${failureLabel}: ${error.message}`;
    pollState();
  }
}
// --- End navigation pause/resume controls
```

In `updateStatus`, add `updateNavigationControls(data.navigation_control);` immediately after the `maybeClearGoalFromActionStatus();` call.

Replace the existing `updatePublishPoseButton` in `static/app.js` with a
transition-aware version so a Goal-mode publish cannot race a server transition.
The server still returns HTTP 409 as the authoritative fallback if a stale tab
somehow posts during a transition.

```javascript
function updatePublishPoseButton() {
  const button = document.getElementById("publishPose");
  const control = state.navigationControl;
  const transitioning = Boolean(control) && ["pausing", "resuming", "clearing"].includes(control.state);
  button.disabled = state.mode === "view" || (state.mode === "goal" && transitioning);
  button.textContent = "Publish Pose";
}
```

Because `updateNavigationControls` calls `updatePublishPoseButton` at the end of
every state update, the Publish Pose button is disabled in Goal mode for the
whole `pausing`/`resuming`/`clearing` window and re-enabled as soon as the
snapshot settles.

In `attachEvents`, immediately before `updatePublishPoseButton();`, add:

```javascript
  document.getElementById("navigationPrimary").addEventListener("click", () => {
    const control = state.navigationControl;
    if (control && control.state === "paused") {
      postNavigationCommand("/api/navigation/resume", "Resume failed");
    } else {
      postNavigationCommand("/api/navigation/pause", "Pause failed");
    }
  });
  document.getElementById("navigationClear").addEventListener("click", () => postNavigationCommand("/api/navigation/clear", "Clear failed"));
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `node --check static/app.js`
Expected: no output, exit 0.

Run: `python3 -m pytest -q test/test_core.py -k "pause_resume_controls or navigation_controls_behavior"`
Expected: `2 passed`.

- [ ] **Step 5: Commit**

```bash
git add static/index.html static/app.js test/test_core.py
git commit -m "feat: add navigation pause, resume, and clear controls"
```

---

### Task 9: Operator documentation and full verification

**Files:**
- Modify: `README.md`
- Test: `test/test_core.py`

- [ ] **Step 1: Write the failing test**

Append to the end of `test/test_core.py`:

```python
def test_readme_documents_navigation_pause_and_cancel_service():
    readme = (Path(__file__).resolve().parents[1] / "README.md").read_text(
        encoding="utf-8"
    )

    assert "Pause Navigation" in readme
    assert "Resume Navigation" in readme
    assert "paused_navigation.json" in readme
    assert "cancel_goal_service" in readme
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest -q test/test_core.py -k readme_documents`
Expected: FAIL with `AssertionError` on the first missing string.

- [ ] **Step 3: Write minimal implementation**

Append this section to `README.md`:

```markdown
## 暂停与恢复导航（Pause / Resume / Clear）

**Navigation** 页面的 **Target Pose** 区域提供导航目标控制：

- **Pause Navigation**：在当前目标 accepted/executing 时可用。网页节点先把目标保存为 `pausing`，再通过 `cancel_goal_service`（默认 `/p2p_move_base/_action/cancel_goal`，使用零 UUID）取消当前 Action 目标；收到 Action 的 `canceled` 后进入 **Paused**，不会停止 navigation launch。
- **Resume Navigation**：在 **Paused** 且当前所选地图与保存地图一致时可用。网页节点用保存的 x/y/z/yaw/frame 重新发布目标，由导航从机器人当前位姿重新规划；新目标被 accepted/executing 后才删除保存记录。
- **Clear Goal**：取消当前目标或删除已暂停的目标。删除已暂停目标不会调用 Action 服务；取消进行中（**Pausing**）时点击 Clear 会复用同一次取消请求。**Resuming** 期间 Clear 返回 HTTP 409。
- 状态显示为 `Idle`、`Navigating`、`Pausing`、`Paused`、`Resuming` 或 `Clearing`；过渡状态期间会禁用可能重复发布目标的按钮。
- 暂停目标持久化在 `~/.config/web_control/paused_navigation.json`，浏览器刷新或网页节点重启后仍可 Resume。重启时若检测到 active 但无保存目标，显示 **Navigating**，允许 Clear，Pause 禁用。
- 网页节点重启后若 Action 状态数组为空（当前没有任何目标），持久化的 **Pausing**/**Resuming** 会一次性归一为 **Paused**，保存目标保留、可继续 Resume；已处于 **Paused** 的记录保持不变。
- 选择另一张地图会清除上一张地图的暂停目标；取消服务超时返回 HTTP 504，状态保持 **Pausing**，前端继续轮询直到 Action 状态确定。
- `GET /api/state` 的 `navigation_control` 字段是按钮文案与可用状态的唯一权威来源。
```

- [ ] **Step 4: Run the full verification sweep**

Run: `env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY python3 -m pytest -q test`
Expected: exit code 0; the summary reports no `FAILED` or `ERROR` entries.

Run: `node --check static/app.js`
Expected: no output, exit 0.

Run: `python3 -m py_compile web_control/*.py`
Expected: no output, exit 0.

Run: `git diff --check`
Expected: no output, exit 0.

Do not run `colcon build`; the user compiles the ROS workspace separately.

- [ ] **Step 5: Commit**

```bash
git add README.md test/test_core.py
git commit -m "docs: document navigation pause and resume"
```

---

## Full verification

After all nine tasks, run the complete sweep exactly:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY python3 -m pytest -q test
node --check static/app.js
python3 -m py_compile web_control/*.py
git diff --check
```

Rationale: the HTTP tests bind `127.0.0.1` and call it through `urllib`; inherited proxy variables redirect those loopback requests and make them fail, so they are unset for the suite. `git diff --check` catches trailing whitespace and conflict markers. `colcon build` is **not** run in this plan; the user compiles the workspace.

Expected: every command exits 0, pytest reports no failures, and `git status` shows a clean tree with the nine commits above.

## Design clarifications resolved

The read-only repository analysis produced twelve risks (R1-R12). Each is resolved in a specific task. Six follow-up clarifications (C1-C6) from plan review are resolved below as well:

- **R1 — `GoalStatusArray` retains completed goals.** `select_action_status` prefers `accepted`/`executing`/`canceling` and otherwise takes the newest `goal_info.stamp`, so a stale array tail cannot select the wrong goal. Task 3 (`test_select_action_status_prefers_active_over_newer_completed`), wired in Task 4. The retained-status transition gap is closed by the per-transition floor in C4, and the empty-array restart gap by C6.
- **R2 — `web_control` is not an Action client and owns no goal handle.** Cancellation uses `action_msgs/srv/CancelGoal` with a zero UUID and zero timestamp, so no `dddmr_sys_core` import and no navigation-package change are needed. Task 4 owns the client and the `_cancel_all_goals` primitive (asserted in `test_cancel_all_goals_sends_zero_uuid_and_zero_stamp`, `test_cancel_all_goals_reports_rejected_return_code`, `test_cancel_all_goals_distinguishes_accepted_and_empty`, `test_cancel_all_goals_maps_future_error_to_unavailable`, `test_cancel_all_goals_times_out`), `package.xml` untouched.
- **R3 — Restart during `Pausing` could lose the destination or leave the goal running.** Reconciling a `pausing` record with an active status returns `"retry_cancel"` and the node retries in a daemon thread; without an active goal (including an empty status array, C6) the record becomes `paused`. Task 3 (`test_restart_pausing_record_with_active_goal_retries_cancel`, `test_restart_pausing_record_without_active_goal_becomes_paused`, `test_no_active_goals_reconciles_pausing_record_to_paused`) and Task 4 (`_retry_startup_cancel`, `test_action_status_callback_empty_list_reconciles_pausing`).
- **R4 — Restart during `Resuming` could publish the goal twice.** A `resuming` record with an active status removes the record and settles on `Navigating`; without an active goal (including an empty status array, C6) it returns to `paused`. Task 3 (`test_restart_resuming_record_with_active_goal_is_adopted`, `test_restart_resuming_record_without_active_goal_returns_to_paused`, `test_no_active_goals_reconciles_resuming_record_to_paused`).
- **R5 — Pause and Clear both end in an Action `canceled` status.** A pending-intent flag (`pause` vs `clear`) decides whether `canceled` yields `Paused` or `Idle`; switching intent during `Pausing` reuses the same cancel. Task 3 (`test_navigating_pause_canceled_reaches_paused_then_resume`, `test_pausing_clear_reuses_cancel_and_discards_destination`) and Task 5 (`test_clear_while_pausing_reuses_cancel`).
- **R6 — A cancel timeout must not discard a possibly-pausable goal.** Timeout raises `NavigationTimeoutError`, the `pausing` record and phase are left untouched, the handler returns 504, and the browser keeps polling. Task 4 (`test_cancel_all_goals_times_out`) and Task 5 (`test_pause_navigation_timeout_keeps_pausing_record`), Task 7 (`test_pause_endpoint_returns_504_on_timeout`).
- **R7 — A paused goal must survive refresh and restart and be rendered.** `NavigationStore` persists it atomically and `snapshot()["paused_goal"]` is part of `/api/state.navigation_control`. Task 2 (store round trip) and Tasks 4/7 (`test_state_payload_exposes_navigation_control_snapshot`, `test_state_endpoint_reports_paused_navigation_controls`).
- **R8 — Coordinates from another map are invalid.** `select_map` calls `note_map_selected`, which deletes a record whose `map_name` differs, and `can_resume` requires the current map to match. Task 3 (`test_map_change_invalidates_paused_record`, `test_resume_rejects_when_map_does_not_match`) and Task 6 (`test_select_map_clears_paused_goal_from_previous_map`).
- **R9 — A new goal must beat a paused goal without losing the record on failure.** `publish_goal` rejects while `Pausing`/`Resuming`/`Clearing` and calls `record_goal_published` only after `_publish_goal_pose` returns, so the record is discarded only on a successful publish. Task 6 (`test_publish_goal_rejected_while_pausing`, `test_new_goal_discards_paused_record_after_publish`).
- **R10 — Duplicate Pause/Resume must not send duplicate ROS commands.** The state machine raises `NavigationConflictError` for illegal or in-flight commands and the handler maps it to HTTP 409 before any ROS call. Task 3 (`test_resume_rejects_while_resuming`), Task 6, Task 7 (`test_pause_endpoint_returns_409_when_no_active_goal`).
- **R11 — An active goal after restart has no saved destination.** Reconciliation with no record and an active status shows `Navigating` with `can_pause=False` and `can_clear=True`. Task 3 (`test_restart_with_active_goal_and_no_record_allows_clear_only`).
- **R12 — Clear semantics differ per phase.** Clear while `Pausing` reuses the in-flight cancel and discards the destination; Clear while `Resuming` returns 409; Clear of a `paused` record does not call the Action service. Task 3 (`test_clear_paused_goal_needs_no_cancel`), Task 5 (`test_clear_paused_goal_skips_cancel_service`), Task 7 (`test_clear_endpoint_returns_200_for_paused_goal`, `test_clear_endpoint_returns_409_while_resuming`).

- **C1 — A missing `action_msgs.srv.CancelGoal` must not disable the whole node.** The cancel-service import lives in its own `try/except ModuleNotFoundError` that only sets `CancelGoal = None`; the main ROS `try` never mentions it, so `rclpy`, `GoalStatusArray`, and the rest of the node keep working when only the service type is unavailable (pause/clear then return HTTP 503). Task 4 (`test_cancel_goal_import_is_isolated_from_main_ros_import`).
- **C2 — `CancelGoal` responses are not all success.** `_cancel_all_goals` returns `"accepted"` only for `return_code == 0` with a non-empty `goals_canceling`, `"empty"` for `return_code == 0` with none, and `"rejected"` for any non-zero code; a raised `future.result()` maps to `NavigationUnavailableError`. Pause treats anything but `"accepted"` as `note_cancel_failed` + HTTP 409; Clear treats `"empty"` as completion and `"rejected"` as 503. The vague `"ok"` string is gone. Task 4 (the `_cancel_all_goals` primitive with `test_cancel_all_goals_sends_zero_uuid_and_zero_stamp`, `test_cancel_all_goals_reports_rejected_return_code`, `test_cancel_all_goals_distinguishes_accepted_and_empty`, `test_cancel_all_goals_maps_future_error_to_unavailable`, `test_cancel_all_goals_times_out`) and Task 5 (`test_pause_navigation_rejected_cancel_restores_navigating`).
- **C3 — Stopping navigation must settle server state, not leave a phantom goal.** `clear_navigation_runtime_state` now clears `published_goal` and calls `navigation_control.on_navigation_stopped(now)`: Navigating→Idle, Pausing→Paused (destination kept), Resuming→Paused (record kept), Clearing→Idle (record discarded), Paused unchanged. `handle_managed_exit("navigation")` inherits this, so the buttons stop advertising an active goal after Stop. Task 3 (`test_navigation_stopped_settles_each_phase`), Task 5 (wiring + `test_clear_navigation_runtime_state_settles_control_and_goal`).
- **C4 — Retained Action statuses could end the wrong transition.** `record_goal_published` and `begin_resume` record a dedicated per-transition goal-stamp floor. While the new goal is awaiting an active status, every update with `goal_stamp <= floor` (the replaced goal's `canceled`/`succeeded`/`canceling`/`aborted`) is discarded. Only a newer `accepted`/`executing` confirms the new goal. A newer `canceling` is the new goal already being canceled: it does not clear a resuming record and does not mark Resume successful, so the final terminal is still awaited. A newer terminal (`canceled`/`succeeded`/`aborted`) that arrives before any confirmation means the new goal already ended: a Resume returns to `Paused` with its record kept, and a normal new goal becomes `Idle`. This replaces relying on the global `last_status_stamp` alone. Task 3 (`test_resume_ignores_replayed_paused_terminal_status`, `test_new_goal_ignores_stale_succeeded_status`, `test_newer_canceling_does_not_confirm_resume`, `test_newer_canceled_before_active_returns_resume_to_paused`, `test_newer_terminal_before_active_sends_new_goal_to_idle`).
- **C5 — The browser must not publish a goal during a server transition.** `updatePublishPoseButton` disables Publish Pose while `navigation_control.state` is `pausing`/`resuming`/`clearing` in Goal mode, and `updateNavigationControls` re-runs it after each snapshot. The server's HTTP 409 remains the authoritative fallback. Task 8 (Node harness publish-pose assertions) and Task 7 (`test_goal_endpoint_rejected_while_pausing`).
- **C6 — An empty `GoalStatusArray` must not leave a restart stuck.** At startup the Action server can report no goals at all; the old callback returned immediately when the selection was `None`, so a persisted `pausing`/`resuming` phase never reconciled and stayed stuck forever. `note_no_active_goals` performs the one-time reconciliation directly (`pausing`/`resuming`→`paused` with the destination kept, no record→`idle`, already `paused` unchanged) instead of a synthetic `goal_stamp=0` status that would bypass the transition floor; later empty arrays are no-ops, and `action_status` is updated to `idle` with `count=0`. Task 3 (`test_select_action_status_empty_or_terminal_only`, `test_no_active_goals_reconciles_pausing_record_to_paused`, `test_no_active_goals_reconciles_resuming_record_to_paused`, `test_no_active_goals_without_record_is_idle`, `test_no_active_goals_keeps_already_paused_record`, `test_no_active_goals_reconciles_only_once`) and Task 4 (`test_action_status_callback_empty_list_reconciles_pausing`, `test_action_status_callback_empty_list_and_no_record_is_idle`).
