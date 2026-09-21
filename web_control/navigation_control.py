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
