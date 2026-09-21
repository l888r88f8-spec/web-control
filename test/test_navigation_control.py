"""Tests for the ROS-free navigation pause/resume state machine."""
import pytest

from web_control.navigation_control import (
    CLEARING,
    IDLE,
    NAVIGATING,
    PAUSED,
    PAUSING,
    PENDING_CLEAR,
    PENDING_PAUSE,
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


# --- F1: clear published before the goal is confirmed active -----------------


def test_clear_accepted_before_empty_triggers_exactly_one_retry(tmp_path):
    control, store = _control(tmp_path)
    control.record_goal_published(
        {"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0}, "map", "site_a", 10.0
    )
    assert control.begin_clear(10.0) is True

    # The accepted status is processed while the HTTP thread is still blocked
    # inside CancelGoal; it confirms registration before the empty result lands.
    assert control.note_action_status("accepted", 7, 11.0) is None
    assert control.snapshot("site_a")["state"] == CLEARING

    # The first cancel matched no goal, so the clear must stay pending and arm
    # exactly one retry.
    assert control.note_cancel_empty(11.5) == "retry_cancel"
    assert control.snapshot("site_a")["state"] == CLEARING
    assert control.snapshot("site_a")["has_goal"] is True
    token = control.retry_token()
    assert token is not None
    assert control.validate_retry(token, PENDING_CLEAR) is True

    # A later executing status must not arm a second retry.
    assert control.note_action_status("executing", 7, 12.0) is None
    assert control.retry_token() == token

    control.note_action_status("canceled", 7, 13.0)
    assert control.snapshot("site_a")["state"] == IDLE
    assert store.load() is None


def test_clear_before_active_accepted_result_never_retries(tmp_path):
    control, store = _control(tmp_path)
    control.record_goal_published(
        {"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0}, "map", "site_a", 10.0
    )
    assert control.begin_clear(10.0) is True

    # The initial cancel matched the goal: no registration race, so a confirming
    # status must never schedule a retry.
    control.note_cancel_accepted()
    assert control.note_action_status("accepted", 7, 11.0) is None
    assert control.retry_token() is None
    assert control.retry_lease_active() is False

    control.note_action_status("canceled", 7, 12.0)
    assert control.snapshot("site_a")["state"] == IDLE
    assert store.load() is None


def test_clear_empty_before_confirmation_retries_on_confirmation(tmp_path):
    control, store = _control(tmp_path)
    control.record_goal_published(
        {"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0}, "map", "site_a", 10.0
    )
    assert control.begin_clear(10.0) is True

    # Empty result lands before the goal is confirmed.
    assert control.note_cancel_empty(10.5) is None
    assert control.retry_token() is None

    # Confirmation now arms exactly one retry.
    assert control.note_action_status("accepted", 7, 11.0) == "retry_cancel"
    token = control.retry_token()
    assert token is not None
    assert control.validate_retry(token, PENDING_CLEAR) is True
    assert control.note_action_status("executing", 7, 12.0) is None
    assert control.retry_token() == token


# --- F2: retry lease blocks operator publish/resume --------------------------


def test_publish_and_resume_blocked_while_retry_lease_outstanding(tmp_path):
    control, store = _control(tmp_path, _record(phase="pausing"))
    assert control.note_action_status("executing", 7, 9.0) == "retry_cancel"
    token = control.retry_token()
    assert token is not None

    with pytest.raises(NavigationConflictError):
        control.begin_resume(10.0, "site_a")

    with pytest.raises(NavigationConflictError):
        control.prepare_goal_publish(
            {"x": 3.0, "y": 4.0, "z": 0.0, "yaw": 0.0}, "map", "site_a", 10.0
        )

    # The old goal terminates while the retry is delayed; pending intent is gone
    # so revalidation must fail and release the lease.
    control.note_action_status("canceled", 7, 11.0)
    assert control.snapshot("site_a")["state"] == PAUSED
    assert control.validate_retry(token, PENDING_PAUSE) is False
    control.release_retry(token)
    assert control.retry_lease_active() is False
    assert control.begin_resume(12.0, "site_a")["x"] == 1.0


# --- F3: late failure/empty callbacks are idempotent no-ops ------------------


def test_late_cancel_failure_after_terminal_is_noop(tmp_path):
    control, store = _control(tmp_path)
    _arm(control)
    control.begin_pause(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}, "map", "site_a", 10.0
    )
    control.note_action_status("canceled", 5, 11.0)
    assert control.snapshot("site_a")["state"] == PAUSED

    # A callback that arrives after the transition already settled must not
    # create an unmanageable Idle state with a persisted paused record.
    control.note_cancel_failed(12.0)

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == PAUSED
    assert snapshot["can_resume"] is True
    assert store.load()["phase"] == "paused"


def test_late_cancel_empty_after_terminal_is_noop(tmp_path):
    control, store = _control(tmp_path)
    _arm(control)
    control.begin_pause(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}, "map", "site_a", 10.0
    )
    control.note_action_status("canceled", 5, 11.0)
    assert control.snapshot("site_a")["state"] == PAUSED

    assert control.note_cancel_empty(12.0) is None

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == PAUSED
    assert snapshot["can_resume"] is True
    assert store.load()["phase"] == "paused"


def test_late_cancel_failure_with_unknown_token_is_noop(tmp_path):
    control, store = _control(tmp_path, _record())
    control.note_cancel_failed(9.0, token=99)

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == PAUSED
    assert snapshot["can_resume"] is True
    assert store.load()["phase"] == "paused"


# --- F4: timeout recovery through the retry lease ----------------------------


def test_cancel_timeout_arms_retry_and_clear_requests_fresh_cancel(tmp_path):
    control, store = _control(tmp_path)
    _arm(control)
    control.begin_pause(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}, "map", "site_a", 10.0
    )

    token = control.note_cancel_timeout(10.5)
    assert token is not None
    assert control.validate_retry(token, PENDING_PAUSE) is True
    # The timeout contract keeps the pausing record for polling.
    assert control.snapshot("site_a")["state"] == PAUSING
    assert store.load()["phase"] == "pausing"

    # Clear during the timed-out pause cannot reuse an untracked cancel: it
    # invalidates the delayed retry and asks for a fresh cancel.
    assert control.begin_clear(11.0) is True
    assert control.snapshot("site_a")["state"] == CLEARING
    assert control.validate_retry(token, PENDING_PAUSE) is False


def test_cancel_timeout_retry_failure_settles_navigating_with_goal(tmp_path):
    control, store = _control(tmp_path)
    _arm(control)
    control.begin_clear(10.0)
    token = control.note_cancel_timeout(10.5)
    assert token is not None

    # The bounded retry was rejected/empty while the goal is still active.
    control.note_cancel_failed(11.0, token=token)
    control.release_retry(token)

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == NAVIGATING
    assert snapshot["has_goal"] is True
    assert snapshot["can_clear"] is True
    assert control.retry_lease_active() is False


# --- F5: operator commands supersede startup reconciliation ------------------


def test_restored_pausing_clear_before_first_status_supersedes_reconcile(tmp_path):
    control, store = _control(tmp_path, _record(phase="pausing"))

    # Restored pausing has no in-process cancel, so Clear must send a fresh one.
    assert control.begin_clear(10.0) is True
    assert control.snapshot("site_a")["state"] == CLEARING

    # Startup reconciliation must not overwrite the explicit command.
    control.note_no_active_goals(10.5)
    assert control.snapshot("site_a")["state"] == CLEARING
    assert control.note_action_status("accepted", 7, 11.0) is None
    assert control.snapshot("site_a")["state"] == CLEARING

    control.note_action_status("canceled", 7, 12.0)
    assert control.snapshot("site_a")["state"] == IDLE
    assert store.load() is None


def test_restored_paused_resume_before_empty_status_is_not_reverted(tmp_path):
    control, store = _control(tmp_path, _record())

    control.begin_resume(10.0, "site_a")
    assert control.snapshot("site_a")["state"] == RESUMING

    # The first status-less update must not roll the resume back to paused and
    # invite a duplicate publish.
    control.note_no_active_goals(10.5)
    assert control.snapshot("site_a")["state"] == RESUMING
    assert store.load()["phase"] == "resuming"


# --- F6: resume publish failure resets every transition field ----------------


def test_resume_publish_failure_resets_transition_fields(tmp_path):
    control, store = _control(tmp_path)
    _arm(control)
    control.begin_pause(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}, "map", "site_a", 10.0
    )
    control.note_action_status("canceled", 5, 11.0)
    assert control.snapshot("site_a")["state"] == PAUSED

    control.begin_resume(12.0, "site_a")
    assert control.snapshot("site_a")["state"] == RESUMING

    control.note_resume_publish_failed(12.5)

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == PAUSED
    assert snapshot["can_resume"] is True
    assert store.load()["phase"] == "paused"

    # A stale terminal arriving after the failed publish must not delete the
    # now-paused record.
    control.note_action_status("aborted", 9, 13.0)
    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == PAUSED
    assert snapshot["can_resume"] is True
    assert store.load()["phase"] == "paused"


# --- F7: two-phase goal publish reservation ----------------------------------


def test_goal_publish_reservation_arms_floor_and_commits(tmp_path):
    control, store = _control(tmp_path, _record())
    token = control.prepare_goal_publish(
        {"x": 5.0, "y": 6.0, "z": 0.0, "yaw": 1.0}, "map", "site_a", 10.0
    )
    # The old paused record survives until the publish is committed.
    assert store.load()["phase"] == "paused"

    # A status that arrives inside the ROS publish must be treated as the new
    # goal's confirmation, not filtered by a post-publish floor.
    assert control.note_action_status("accepted", 9, 10.1) is None

    control.commit_goal_publish(token)
    assert store.load() is None
    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == NAVIGATING
    assert snapshot["can_pause"] is True

    control.note_action_status("succeeded", 9, 11.0)
    assert control.snapshot("site_a")["state"] == IDLE


def test_goal_publish_reservation_rolls_back_to_paused(tmp_path):
    control, store = _control(tmp_path, _record())
    token = control.prepare_goal_publish(
        {"x": 5.0, "y": 6.0, "z": 0.0, "yaw": 1.0}, "map", "site_a", 10.0
    )
    control.rollback_goal_publish(token)

    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == PAUSED
    assert snapshot["can_resume"] is True
    assert snapshot["paused_goal"]["x"] == 1.0
    assert store.load()["phase"] == "paused"
    assert control.retry_lease_active() is False

    # A stale terminal for the aborted publish must not disturb the record.
    control.note_action_status("aborted", 9, 11.0)
    snapshot = control.snapshot("site_a")
    assert snapshot["state"] == PAUSED
    assert store.load()["phase"] == "paused"
