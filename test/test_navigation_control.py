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


def test_pausing_clear_survives_canceling_until_canceled(tmp_path):
    control, store = _control(tmp_path)
    _arm(control)
    control.begin_pause(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}, "map", "site_a", 10.0
    )
    assert control.begin_clear(10.0) is False

    # An ordinary "canceling" status is the reused cancel already in flight; it
    # must not destroy the clear transition before the terminal arrives.
    control.note_action_status("canceling", 5, 11.0)
    assert control.snapshot("site_a")["state"] == CLEARING

    control.note_action_status("canceled", 5, 12.0)
    assert control.snapshot("site_a")["state"] == IDLE
    assert store.load() is None


def test_pausing_clear_terminal_success_discards_record(tmp_path):
    control, store = _control(tmp_path)
    _arm(control)
    control.begin_pause(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}, "map", "site_a", 10.0
    )
    assert control.begin_clear(10.0) is False

    # The goal completed before the reused cancel settled.  Clear still wins:
    # the destination is discarded and the state settles to Idle.
    control.note_action_status("succeeded", 5, 11.0)

    assert control.snapshot("site_a")["state"] == IDLE
    assert store.load() is None


@pytest.mark.parametrize("active_status", ["accepted", "executing", "canceling"])
def test_active_clear_survives_ordinary_active_status(tmp_path, active_status):
    control, store = _control(tmp_path)
    _arm(control)
    assert control.begin_clear(10.0) is True

    control.note_action_status(active_status, 7, 11.0)
    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == CLEARING
    assert snapshot["has_goal"] is True

    control.note_action_status("canceled", 7, 12.0)
    assert control.snapshot("site_a")["state"] == IDLE
    assert store.load() is None


def test_pausing_clear_cancel_failure_discards_paused_record(tmp_path):
    control, store = _control(tmp_path)
    _arm(control)
    control.begin_pause(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}, "map", "site_a", 10.0
    )
    assert control.begin_clear(10.0) is False
    assert store.load()["phase"] == "pausing"

    # The operator chose Clear: the paused record must go even though the
    # cancellation failed, and the still-running goal must stay clearable.
    control.note_cancel_failed(11.0)

    assert store.load() is None
    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == NAVIGATING
    assert snapshot["has_goal"] is True
    assert snapshot["can_clear"] is True
    assert snapshot["can_pause"] is True


def test_begin_clear_before_active_requests_cancel_and_keeps_goal(tmp_path):
    control, store = _control(tmp_path)
    control.record_goal_published(
        {"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0}, "map", "site_a", 10.0
    )

    assert control.begin_clear(10.0) is True

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == CLEARING
    assert snapshot["has_goal"] is True


def test_clear_before_active_empty_cancel_retries_after_registration(tmp_path):
    control, store = _control(tmp_path)
    control.record_goal_published(
        {"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0}, "map", "site_a", 10.0
    )
    assert control.begin_clear(10.0) is True

    # The first cancel raced goal registration and matched no goal: the clear
    # must stay pending rather than claim success.
    control.note_cancel_empty(10.0)
    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == CLEARING
    assert snapshot["has_goal"] is True

    # A newer accepted confirms registration and asks for exactly one retry.
    assert control.note_action_status("accepted", 7, 11.0) == "retry_cancel"
    assert control.snapshot("site_a")["state"] == CLEARING
    assert control.note_action_status("executing", 7, 12.0) is None

    # The retried cancel is accepted and the terminal settles the clear.
    control.note_action_status("canceling", 7, 13.0)
    assert control.snapshot("site_a")["state"] == CLEARING
    control.note_action_status("canceled", 7, 14.0)
    assert control.snapshot("site_a")["state"] == IDLE
    assert store.load() is None


@pytest.mark.parametrize("active_status", ["accepted", "executing"])
def test_clear_before_active_accepted_cancel_does_not_retry(tmp_path, active_status):
    control, store = _control(tmp_path)
    control.record_goal_published(
        {"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0}, "map", "site_a", 10.0
    )
    assert control.begin_clear(10.0) is True

    # The initial CancelGoal was accepted (not empty), so the confirming
    # accepted/executing status must keep clearing without a duplicate cancel.
    assert control.note_action_status(active_status, 7, 11.0) is None
    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == CLEARING
    assert snapshot["has_goal"] is True

    # The original cancel's terminal still settles the clear normally.
    control.note_action_status("canceling", 7, 12.0)
    assert control.snapshot("site_a")["state"] == CLEARING
    control.note_action_status("canceled", 7, 13.0)
    assert control.snapshot("site_a")["state"] == IDLE
    assert store.load() is None


def test_clear_retry_flag_does_not_leak_into_later_goal(tmp_path):
    control, store = _control(tmp_path)
    control.record_goal_published(
        {"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0}, "map", "site_a", 10.0
    )
    assert control.begin_clear(10.0) is True
    control.note_cancel_empty(10.0)
    assert control.note_action_status("accepted", 7, 11.0) == "retry_cancel"

    # The retried cancel did not confirm; the failed clear settles Navigating.
    control.note_cancel_failed(12.0)
    assert control.snapshot("site_a")["state"] == NAVIGATING

    # A later goal whose initial cancel is accepted must not reuse the stale
    # retry intent from the failed clear.
    control.record_goal_published(
        {"x": 5.0, "y": 6.0, "z": 0.0, "yaw": 0.0}, "map", "site_a", 13.0
    )
    assert control.begin_clear(13.0) is True
    assert control.note_action_status("accepted", 9, 14.0) is None
    assert control.snapshot("site_a")["state"] == CLEARING


def test_clear_retry_failure_restores_navigating_with_goal(tmp_path):
    control, store = _control(tmp_path)
    control.record_goal_published(
        {"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0}, "map", "site_a", 10.0
    )
    assert control.begin_clear(10.0) is True
    control.note_cancel_empty(10.0)
    assert control.note_action_status("accepted", 7, 11.0) == "retry_cancel"

    # The retry matched no goal after the goal was confirmed active: report a
    # truthful Navigating state and keep the goal so Clear can be retried.
    control.note_cancel_failed(12.0)

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == NAVIGATING
    assert snapshot["has_goal"] is True
    assert snapshot["can_clear"] is True
    assert snapshot["can_pause"] is True


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


def test_resume_rejects_when_frame_does_not_match(tmp_path):
    control, store = _control(tmp_path, _record())

    with pytest.raises(NavigationConflictError, match="different frame"):
        control.begin_resume(10.0, "site_a", "odom")

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == PAUSED
    assert snapshot["can_resume"] is True
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


def test_navigation_stopped_rearms_startup_reconciliation(tmp_path):
    control, store = _control(tmp_path, _record(phase="pausing"))
    # The first runtime reconciles the interrupted pause exactly once.
    control.note_no_active_goals(9.0)
    assert control.snapshot("site_a")["state"] == PAUSED
    assert store.load()["phase"] == "paused"

    control.on_navigation_stopped(10.0)

    # A navigation process restart must reconcile the next runtime again
    # instead of treating the first status as already processed.
    assert control._reconciled is False
    control.note_no_active_goals(11.0)
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
