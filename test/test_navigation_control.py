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
