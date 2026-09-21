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
        # Set only when a Clear's first CancelGoal returned ``empty`` while the
        # goal was still awaiting an active status; consumed by the first
        # confirming accepted/executing so at most one retry is ever sent.
        self._retry_cancel_on_active = False
        # True when a Clear began while the goal's first active status had not
        # arrived yet, so an ``empty`` CancelGoal result may be a registration
        # race that justifies exactly one retry even after confirmation.
        self._clear_unconfirmed = False
        # True once a Pause cancel was accepted and is still tracked by the
        # in-flight Action transition.  A record restored from disk is never
        # tracked: after a node restart there is no in-process cancel to reuse.
        self._pause_cancel_tracked = False
        # State-owned retry lease.  At most one background cancel retry may be
        # outstanding; while a lease exists operator goal publishes/resumes are
        # rejected so a delayed retry can never cancel a newer goal.
        self._retry_serial = 0
        self._retry_lease: int | None = None
        self._retry_kind: str | None = None
        # Two-phase goal publish reservation: the transition floor is armed
        # before the ROS publish and the previous paused record is only removed
        # once the publish is committed.
        self._publish_serial = 0
        self._publish_reservation: dict[str, Any] | None = None
        self._transition_ended = False
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

    def _invalidate_retry_locked(self) -> None:
        """Drop any outstanding retry lease so a delayed retry revalidates dead."""
        self._retry_serial += 1
        self._retry_lease = None
        self._retry_kind = None

    def _arm_retry_locked(self, kind: str | None) -> int | None:
        """Acquire the single retry lease; returns its generation or ``None``."""
        if self._retry_lease is not None or kind is None:
            return None
        self._retry_serial += 1
        self._retry_lease = self._retry_serial
        self._retry_kind = kind
        return self._retry_lease

    def retry_token(self) -> int | None:
        with self._lock:
            return self._retry_lease

    def retry_kind(self) -> str | None:
        with self._lock:
            return self._retry_kind

    def retry_lease_active(self) -> bool:
        with self._lock:
            return self._retry_lease is not None

    def validate_retry(self, token: int | None, kind: str | None) -> bool:
        """Revalidate a retry lease generation and its pending intent."""
        with self._lock:
            if token is None or self._retry_lease != token:
                return False
            if kind is None or self._retry_kind != kind:
                return False
            return self._pending == kind

    def release_retry(self, token: int | None) -> None:
        with self._lock:
            if token is not None and self._retry_lease != token:
                return
            self._retry_lease = None
            self._retry_kind = None

    def needs_cancel(self) -> bool:
        with self._lock:
            return self._pending is not None and self._phase in (PAUSING, CLEARING)

    def has_goal(self) -> bool:
        with self._lock:
            return self._has_goal_locked()

    def pending_kind(self) -> str | None:
        with self._lock:
            return self._pending

    # -- Action status feed -------------------------------------------------

    def note_action_status(
        self, status_name: str, goal_stamp: int, now: float
    ) -> str | None:
        """Apply a selected Action status; stale transition updates are ignored.

        Returns ``"retry_cancel"`` once when a cancellation must be retried: at
        startup for an interrupted Pause against the still-active goal, or when
        a Clear whose first CancelGoal returned ``empty`` (it raced goal
        registration) sees the goal finally confirmed by a newer
        ``accepted``/``executing`` status.  A Clear whose first cancel was
        already accepted keeps clearing without sending a duplicate cancel.

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
                    retry_cancel = self._retry_cancel_on_active
                    self._retry_cancel_on_active = False
                    if self._phase == CLEARING:
                        # Keep clearing until a terminal status settles it.  A
                        # retry is only due if the first CancelGoal matched no
                        # goal (it raced registration); an already accepted
                        # cancel is in flight and must not be duplicated.
                        self._active = True
                        self._last_status_stamp = max(
                            self._last_status_stamp, goal_stamp
                        )
                        if retry_cancel:
                            self._clear_unconfirmed = False
                            token = self._arm_retry_locked(PENDING_CLEAR)
                            return "retry_cancel" if token is not None else None
                        return None
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
            if action == "retry_cancel":
                # The retry lease must exist before the status callback spawns a
                # thread, so a delayed thread has a generation to revalidate.
                kind = self._pending
                token = self._arm_retry_locked(kind)
                if token is None:
                    return None
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
        operator can try again; a new goal, including one whose clear raced
        registration, has no destination to keep and becomes ``idle``.
        """
        self._active = False
        self._retry_cancel_on_active = False
        self._clear_unconfirmed = False
        self._pause_cancel_tracked = False
        self._transition_ended = True
        self._current_goal = None
        if self._phase == RESUMING and self._record is not None:
            self._set_record_phase_locked(PAUSED, now)
            self._phase = PAUSED
        else:
            self._clear_record_locked()
            self._phase = IDLE
        self._pending = None

    def _apply_status_locked(self, status_name: str, now: float) -> None:
        if status_name in ACTIVE_STATUS_NAMES:
            self._active = True
            if self._phase == CLEARING:
                # A clear transition survives accepted/executing/canceling
                # until a terminal status settles it.
                return
            if self._phase in (RESUMING, PAUSED, IDLE):
                self._clear_record_locked()
                self._phase = NAVIGATING
                self._pending = None
            return
        self._active = False
        self._retry_cancel_on_active = False
        self._clear_unconfirmed = False
        if status_name == "canceled":
            if self._pending == PENDING_PAUSE:
                self._pause_cancel_tracked = False
                self._set_record_phase_locked(PAUSED, now)
                self._phase = PAUSED
                self._current_goal = None
                self._pending = None
            elif self._pending == PENDING_CLEAR:
                self._pause_cancel_tracked = False
                self._clear_record_locked()
                self._current_goal = None
                self._phase = IDLE
                self._pending = None
            return
        if status_name in ("succeeded", "aborted"):
            if self._phase == CLEARING:
                # Clear always discards the destination; a terminal that ends
                # the goal before "canceled" settles it just the same.
                self._pause_cancel_tracked = False
                self._clear_record_locked()
                self._current_goal = None
                self._active = False
                self._phase = IDLE
                self._pending = None
                return
            if self._record is not None:
                if self._phase in (NAVIGATING, RESUMING, PAUSING):
                    self._pause_cancel_tracked = False
                    self._current_goal = None
                    self._phase = PAUSED
                    self._pending = None
                return
            self._current_goal = None
            if self._phase in (NAVIGATING, RESUMING, PAUSING):
                self._phase = IDLE
                self._pending = None

    # -- Commands -----------------------------------------------------------

    @staticmethod
    def _goal_from_pose(
        pose: dict[str, Any], frame: str | None, map_name: str | None
    ) -> dict[str, Any]:
        return {
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "z": float(pose.get("z", 0.0)),
            "yaw": float(pose["yaw"]),
            "frame": frame,
            "map_name": map_name,
        }

    def prepare_goal_publish(
        self,
        pose: dict[str, Any],
        frame: str,
        map_name: str | None,
        now: float,
    ) -> int:
        """Arm the new-goal transition floor before the ROS publish happens.

        The previous paused record is preserved until :meth:`commit_goal_publish`
        succeeds so a failed publish can be rolled back to the exact previous
        state.
        """
        with self._lock:
            if self._retry_lease is not None:
                raise NavigationConflictError(
                    "navigation command in progress; try again"
                )
            if self._publish_reservation is not None:
                raise NavigationConflictError("goal publish already in progress")
            self._publish_serial += 1
            token = self._publish_serial
            self._publish_reservation = {
                "token": token,
                "phase": self._phase,
                "record": (
                    dict(self._record) if self._record is not None else None
                ),
                "current_goal": (
                    dict(self._current_goal)
                    if self._current_goal is not None
                    else None
                ),
                "active": self._active,
                "awaiting_active": self._awaiting_active,
                "transition_floor": self._transition_floor,
                "last_status_stamp": self._last_status_stamp,
                "pending": self._pending,
                "reconciled": self._reconciled,
                "retry_cancel_on_active": self._retry_cancel_on_active,
                "clear_unconfirmed": self._clear_unconfirmed,
                "pause_cancel_tracked": self._pause_cancel_tracked,
                "transition_ended": self._transition_ended,
                "goal": self._goal_from_pose(pose, frame, map_name),
            }
            # Arm the floor before the ROS publish: a status that confirms the
            # new goal while the publish is in flight is then compared against
            # the replaced goal's stamp instead of having its own stamp adopted.
            self._transition_floor = self._last_status_stamp
            self._awaiting_active = True
            self._active = False
            self._pending = None
            self._retry_cancel_on_active = False
            self._clear_unconfirmed = False
            self._pause_cancel_tracked = False
            self._transition_ended = False
            # An operator publish supersedes any startup reconciliation.
            self._reconciled = True
            return token

    def commit_goal_publish(self, token: int) -> None:
        """Record a successful ROS publish and discard the replaced record."""
        with self._lock:
            reservation = self._publish_reservation
            if reservation is None or reservation["token"] != token:
                return
            self._publish_reservation = None
            self._clear_record_locked()
            if self._transition_ended:
                # The new goal already ended before commit completed: do not
                # resurrect a phantom goal.
                self._current_goal = None
                self._active = False
                self._awaiting_active = False
                self._transition_floor = -1
                self._phase = IDLE
                self._pending = None
                return
            self._current_goal = dict(reservation["goal"])
            self._phase = NAVIGATING
            self._pending = None
            self._reconciled = True
            if self._awaiting_active:
                # Keep the replaced goal's stamp armed (not any status merely
                # observed during the publish) so the new goal still confirms.
                self._transition_floor = reservation["last_status_stamp"]
                self._active = False
            else:
                # A status confirmed the new goal during the publish; keep that
                # confirmation instead of re-arming the floor over it.
                self._transition_floor = -1

    def rollback_goal_publish(self, token: int) -> None:
        """Restore the exact pre-publish state after a failed ROS publish."""
        with self._lock:
            reservation = self._publish_reservation
            if reservation is None or reservation["token"] != token:
                return
            self._publish_reservation = None
            self._phase = reservation["phase"]
            self._current_goal = reservation["current_goal"]
            self._active = reservation["active"]
            self._awaiting_active = reservation["awaiting_active"]
            self._transition_floor = reservation["transition_floor"]
            self._last_status_stamp = reservation["last_status_stamp"]
            self._pending = reservation["pending"]
            self._reconciled = reservation["reconciled"]
            self._retry_cancel_on_active = reservation["retry_cancel_on_active"]
            self._clear_unconfirmed = reservation["clear_unconfirmed"]
            self._pause_cancel_tracked = reservation["pause_cancel_tracked"]
            self._transition_ended = reservation["transition_ended"]
            record = reservation["record"]
            if record is None:
                self._clear_record_locked()
            else:
                self.store.save(record)
                self._record = dict(record)

    def record_goal_published(
        self,
        pose: dict[str, Any],
        frame: str,
        map_name: str | None,
        now: float,
    ) -> None:
        """Adopt a newly published goal; the old paused record is discarded."""
        token = self.prepare_goal_publish(pose, frame, map_name, now)
        self.commit_goal_publish(token)

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
            # The node is about to send the cancel for this in-session pause, so
            # a later Clear may reuse it.  A record restored from disk never
            # takes this path.
            self._pause_cancel_tracked = True
            return dict(record)

    def begin_clear(self, now: float) -> bool:
        """Enter clearing; returns True when the caller must cancel the goal."""
        with self._lock:
            if self._publish_reservation is not None:
                raise NavigationConflictError(
                    "navigation command in progress; try again"
                )
            if self._phase == PAUSING:
                self._pending = PENDING_CLEAR
                self._phase = CLEARING
                self._retry_cancel_on_active = False
                self._clear_unconfirmed = False
                # An explicit operator command supersedes startup reconcile.
                self._reconciled = True
                if self._pause_cancel_tracked:
                    # The in-session Pause cancel is already in flight; reuse it.
                    return False
                # Restored pausing, or a timeout left the cancel untracked: the
                # scheduled retry (if any) cannot complete this clear, so drop
                # it and request a fresh cancel.
                self._invalidate_retry_locked()
                return True
            if self._phase == CLEARING:
                raise NavigationConflictError("clear is already in progress")
            if self._phase == RESUMING or self._pending is not None:
                raise NavigationConflictError(
                    "navigation command in progress; try again"
                )
            if self._phase == PAUSED and self._record is not None:
                self._invalidate_retry_locked()
                self._clear_record_locked()
                self._current_goal = None
                self._active = False
                self._phase = IDLE
                self._reconciled = True
                return False
            if self._phase == NAVIGATING:
                if self._active or self._awaiting_active:
                    # Also cancel a goal that was published but whose first
                    # active status has not arrived yet; keep the goal until
                    # the cancellation really settles.  Remember that the clear
                    # began before confirmation so an ``empty`` result after the
                    # goal is confirmed still gets exactly one retry.
                    self._invalidate_retry_locked()
                    self._clear_unconfirmed = self._awaiting_active
                    self._phase = CLEARING
                    self._pending = PENDING_CLEAR
                    self._retry_cancel_on_active = False
                    self._reconciled = True
                    return True
                self._invalidate_retry_locked()
                self._current_goal = None
                self._active = False
                self._phase = IDLE
                self._reconciled = True
                return False
            raise NavigationConflictError("no navigation goal to clear")

    def begin_resume(
        self,
        now: float,
        current_map: str | None,
        current_frame: str | None = None,
    ) -> dict[str, Any]:
        """Mark the saved destination ``resuming`` and return it to publish."""
        with self._lock:
            if self._retry_lease is not None:
                raise NavigationConflictError(
                    "navigation command in progress; try again"
                )
            if self._publish_reservation is not None:
                raise NavigationConflictError(
                    "navigation command in progress; try again"
                )
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
            if current_frame is not None and self._record["frame"] != current_frame:
                raise NavigationConflictError(
                    "paused goal belongs to a different frame"
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
            self._retry_cancel_on_active = False
            self._clear_unconfirmed = False
            # An explicit operator command supersedes startup reconciliation so
            # an empty first status array cannot revert the resume.
            self._reconciled = True
            return dict(self._record)

    def note_cancel_accepted(self, token: int | None = None) -> None:
        """Mark a CancelGoal request as accepted (no retry is due)."""
        with self._lock:
            if token is not None and self._retry_lease != token:
                return
            if self._pending is None:
                return
            if self._pending == PENDING_PAUSE:
                self._pause_cancel_tracked = True
            self._retry_cancel_on_active = False
            self._clear_unconfirmed = False

    def note_cancel_timeout(self, now: float) -> int | None:
        """A cancel timed out; keep the state and arm one bounded retry."""
        lease = self.note_cancel_timeout_lease(now)
        return lease[0] if lease is not None else None

    def note_cancel_timeout_lease(self, now: float) -> tuple[int, str] | None:
        """Arm the bounded timeout retry and return its ``(generation, kind)``.

        The kind is read from the newly armed lease under the same lock, so a
        caller that started a Pause while a concurrent Clear changed the pending
        intent retries Clear instead of its stale assumption.
        """
        with self._lock:
            if self._pending not in (PENDING_PAUSE, PENDING_CLEAR):
                return None
            # The request gave up on the future, so there is no tracked
            # completion to reuse: a later Clear must send a new cancel.
            self._pause_cancel_tracked = False
            token = self._arm_retry_locked(self._pending)
            if token is None:
                return None
            return token, self._retry_kind

    def note_cancel_failed(self, now: float, token: int | None = None) -> None:
        """An explicit rejection or unavailable service ends the transition.

        Either intent discards a persisted destination: Pause was abandoned and
        Clear means the operator no longer wants the goal.  When the Action may
        still be running, the goal is kept so Clear can be retried.  A late
        callback with no matching pending intent/generation is a no-op so it can
        never manufacture an unmanageable Idle state beside a paused record.
        """
        with self._lock:
            if token is not None:
                if self._retry_lease != token or self._pending is None:
                    return
            elif self._pending is None:
                return
            self._invalidate_retry_locked()
            self._retry_cancel_on_active = False
            self._clear_unconfirmed = False
            self._pause_cancel_tracked = False
            if self._pending in (PENDING_PAUSE, PENDING_CLEAR):
                self._clear_record_locked()
            self._pending = None
            if (
                self._active
                or self._awaiting_active
                or self._current_goal is not None
            ):
                self._phase = NAVIGATING
            else:
                self._phase = IDLE

    def note_cancel_empty(self, now: float, token: int | None = None) -> str | None:
        """Handle a clear whose cancel matched no goal.

        When the goal was published but has not been confirmed active yet, the
        empty result may simply have raced registration.  Stay in ``clearing``
        and retain the goal so the retry triggered by the confirming status can
        cancel it.  When the clear began before confirmation and the goal is now
        confirmed, return ``"retry_cancel"`` to arm exactly one retry; otherwise
        the clear is already complete.  A late empty result with no matching
        pending intent is a no-op.
        """
        with self._lock:
            if self._pending != PENDING_CLEAR:
                return None
            if token is not None and self._retry_lease != token:
                return None
            if self._awaiting_active:
                # Only this registration race justifies one retry once the goal
                # is confirmed active.
                self._retry_cancel_on_active = True
                return None
            self._retry_cancel_on_active = False
            if self._clear_unconfirmed:
                # The clear began before the goal confirmed registration, so
                # this empty result is the race: retry once now.
                self._clear_unconfirmed = False
                armed = self._arm_retry_locked(PENDING_CLEAR)
                return "retry_cancel" if armed is not None else None
            self._clear_record_locked()
            self._current_goal = None
            self._active = False
            self._phase = IDLE
            self._pending = None
            self._pause_cancel_tracked = False
            return None

    def note_resume_publish_failed(self, now: float) -> None:
        """A definite publish failure restores the resumable paused state.

        Every transition field is reset so a stale terminal status cannot treat
        the failed resume as an ended new goal and delete the paused record.
        """
        with self._lock:
            if self._phase != RESUMING or self._record is None:
                return
            self._awaiting_active = False
            self._transition_floor = -1
            self._current_goal = None
            self._retry_cancel_on_active = False
            self._clear_unconfirmed = False
            self._pause_cancel_tracked = False
            self._invalidate_retry_locked()
            self._set_record_phase_locked(PAUSED, now)
            self._phase = PAUSED
            self._pending = None

    def note_map_selected(self, map_name: str | None, now: float) -> None:
        """Drop a paused destination that belongs to a different map."""
        with self._lock:
            if self._record is not None and self._record["map_name"] != map_name:
                self._invalidate_retry_locked()
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
            self._reconciled = False
            self._awaiting_active = False
            self._transition_floor = -1
            self._retry_cancel_on_active = False
            self._clear_unconfirmed = False
            self._pause_cancel_tracked = False
            self._transition_ended = False
            self._publish_reservation = None
            self._invalidate_retry_locked()
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
                    and self._retry_lease is None
                    and self._publish_reservation is None
                ),
                "can_clear": (
                    self._has_goal_locked()
                    and self._phase in (NAVIGATING, PAUSED, PAUSING)
                    and self._publish_reservation is None
                ),
                "paused_goal": (
                    dict(self._record) if self._record is not None else None
                ),
            }
