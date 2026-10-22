"""Focused regression tests for topic/motion diagnostics and JSONL persistence."""

from __future__ import annotations

import builtins
import json
import os
import shutil
import subprocess
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import MethodType, SimpleNamespace

import pytest

from web_control.diagnostics import (
    TOPIC_LOG_KEYS,
    DiagnosticsLogWriter,
    DiagnosticsService,
    MotionTracker,
    TopicRateMonitor,
)
from web_control.log_store import ManagedLogStore
from web_control.map_selection import (
    SelectionStore,
    validate_topic_logging_enabled,
)
from web_control import web_control_node


def _odom_msg(speed, stamp, frame="odom1", child="base_link1", yaw=0.0):
    # Real nav_msgs/Odometry nests velocity as twist.twist.linear.x
    # (TwistWithCovariance.twist is a geometry_msgs/Twist).
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=int(stamp), nanosec=int((stamp % 1) * 1e9)),
            frame_id=frame,
        ),
        child_frame_id=child,
        twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=speed))),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=__import__("math").sin(yaw / 2.0),
                    w=__import__("math").cos(yaw / 2.0),
                )
            )
        ),
    )


def test_rate_uses_received_monotonic_window_and_bounded_counts():
    monitor = TopicRateMonitor("imu", "IMU", "/imu", "sensor_msgs/msg/Imu")
    for index in range(10):
        monitor.record(index * 0.1)
    snapshot = monitor.snapshot(0.95)
    assert snapshot["hz"] == pytest.approx(10.0)
    assert snapshot["received_count"] == 10
    assert snapshot["status"] == "active"
    assert snapshot["last_age_sec"] == pytest.approx(0.05)

    # Old arrivals leave the rate window but the bounded lifetime counter stays.
    snapshot = monitor.snapshot(8.0)
    assert snapshot["window_count"] == 0
    assert snapshot["hz"] == 0.0
    assert snapshot["received_count"] == 10
    assert snapshot["status"] == "stale"


def test_publisher_and_waiting_unavailable_states_are_explicit():
    monitor = TopicRateMonitor("lidar", "Lidar", "/lidar_points", "PointCloud2")
    assert monitor.snapshot(0.0)["status"] == "waiting"
    monitor.set_publisher_count(0, 0.0)
    assert monitor.snapshot(1.0)["status"] == "no_publisher"
    monitor.set_publisher_count(1, 1.0)
    assert monitor.snapshot(1.1)["status"] == "waiting"
    disabled = TopicRateMonitor(
        "imu", "IMU", "/imu", "Imu", enabled=False
    )
    assert disabled.snapshot(0.0)["status"] == "unavailable"


def test_steady_forward_and_reverse_wheel_distance_is_symmetric():
    for speed in (1.0, -1.0):
        tracker = MotionTracker(debounce_sec=0.3)
        for index in range(10):
            tracker.update(
                speed, 0.0, index * 0.1, "odom1", "base_link1", index * 0.1
            )
        snapshot = tracker.snapshot(0.95)
        assert snapshot["state"] == "moving"
        assert snapshot["direction"] == ("reverse" if speed < 0 else "forward")
        assert snapshot["total_distance_m"] == pytest.approx(0.9)
        assert snapshot["segment_distance_m"] == pytest.approx(0.9)
        assert snapshot["segment_duration_sec"] == pytest.approx(0.9)
        assert snapshot["validity"]["valid"] is True


def test_reversing_interval_integrates_absolute_motion_not_signed_average():
    tracker = MotionTracker(debounce_sec=0.0)
    tracker.update(1.0, 0.0, 0.0, "odom1", "base_link1", 0.0)
    tracker.update(-1.0, 0.0, 0.1, "odom1", "base_link1", 0.1)
    snapshot = tracker.snapshot(0.1)
    # A +1 -> -1 crossing over 0.1 s still travels 0.05 m (the |v| integral
    # with linear variation); the signed trapezoid would cancel it to zero.
    assert snapshot["total_distance_m"] == pytest.approx(0.05)
    assert snapshot["segment_distance_m"] == pytest.approx(0.05)


def test_segment_duration_accepts_zero_start_stamp_and_zero_stop_stamp():
    tracker = MotionTracker(debounce_sec=0.0)
    tracker.update(1.0, 0.0, 0.0, "odom1", "base_link1", 0.0)
    tracker.update(0.0, 0.0, 0.4, "odom1", "base_link1", 0.4)
    event = tracker.take_events()[-1]
    assert event["event"] == "motion_stop"
    assert event["segment_duration_sec"] == pytest.approx(0.4)
    assert tracker.snapshot(0.4)["segment_duration_sec"] == pytest.approx(0.4)


def test_stationary_noise_is_not_integrated_or_called_moving():
    tracker = MotionTracker(debounce_sec=0.2)
    for index in range(20):
        tracker.update(0.01, 0.0, index * 0.1, "odom1", "base_link1", index * 0.1)
    snapshot = tracker.snapshot(1.9)
    assert snapshot["state"] == "stationary"
    assert snapshot["total_distance_m"] == 0.0
    assert snapshot["validity"]["valid"] is True


def test_debounce_delays_motion_start_and_stop_events():
    tracker = MotionTracker(debounce_sec=0.3)
    tracker.update(1.0, 0.0, 0.0, "odom1", "base_link1", 0.0)
    tracker.update(1.0, 0.0, 0.1, "odom1", "base_link1", 0.1)
    assert tracker.snapshot(0.1)["state"] == "unknown"
    tracker.update(1.0, 0.0, 0.3, "odom1", "base_link1", 0.3)
    assert tracker.snapshot(0.3)["state"] == "moving"
    events = tracker.take_events()
    assert events[-1]["event"] == "motion_start"

    tracker.update(0.0, 0.0, 0.4, "odom1", "base_link1", 0.4)
    tracker.update(0.0, 0.0, 0.7, "odom1", "base_link1", 0.7)
    assert tracker.snapshot(0.7)["state"] == "stationary"
    assert tracker.take_events()[-1]["event"] == "motion_stop"


def test_gap_and_timestamp_reset_never_bridge_distance():
    gap = MotionTracker(debounce_sec=0.1, max_gap_sec=0.5)
    for index in range(3):
        gap.update(1.0, 0.0, index * 0.1, "odom1", "base_link1", index * 0.1)
    before = gap.snapshot(0.2)["total_distance_m"]
    gap.update(1.0, 0.0, 0.9, "odom1", "base_link1", 0.9)
    gap.update(1.0, 0.0, 1.0, "odom1", "base_link1", 1.0)
    after = gap.snapshot(1.0)
    assert before == pytest.approx(0.2)
    assert after["total_distance_m"] == pytest.approx(before + 0.1)
    assert after["validity"]["incomplete"] is True
    assert any(
        event.get("reason") == "timestamp_gap" for event in gap.take_events()
    )

    reset = MotionTracker(debounce_sec=0.1)
    for stamp in (0.0, 0.1, 0.2):
        reset.update(1.0, 0.0, stamp, "odom1", "base_link1", stamp)
    before = reset.snapshot(0.2)["total_distance_m"]
    reset.update(1.0, 0.0, 0.1, "odom1", "base_link1", 0.3)
    assert reset.snapshot(0.3)["total_distance_m"] == pytest.approx(before)
    assert any(
        event.get("reason") == "timestamp_reset" for event in reset.take_events()
    )


def test_frame_change_invalid_values_and_staleness_pause_without_fake_distance():
    tracker = MotionTracker(debounce_sec=0.1, stale_timeout_sec=2.0)
    for stamp in (0.0, 0.1, 0.2):
        tracker.update(1.0, 0.0, stamp, "odom1", "base_link1", stamp)
    before = tracker.snapshot(0.2)["total_distance_m"]
    tracker.update(1.0, 0.0, 0.3, "other_odom", "base_link1", 0.3)
    after_frame = tracker.snapshot(0.3)
    assert after_frame["total_distance_m"] == pytest.approx(before)
    assert after_frame["validity"]["incomplete"] is True

    tracker.update(float("nan"), float("inf"), 0.4, "other_odom", "base_link1", 0.4)
    tracker.update(0.01, 0.0, 0.1, "other_odom", "base_link1", 0.5)
    assert tracker.snapshot(0.5)["total_distance_m"] == pytest.approx(before)
    stale = tracker.snapshot(3.0)
    assert stale["state"] == "unknown"
    assert stale["validity"]["paused"] is True
    assert stale["validity"]["incomplete"] is True
    assert stale["total_distance_m"] == pytest.approx(before)
    assert tracker.snapshot(3.1)["state"] == "unknown"


def test_chassis_lifecycle_resets_baseline_and_retains_known_total():
    tracker = MotionTracker(debounce_sec=0.1)
    for stamp in (0.0, 0.1, 0.2):
        tracker.update(1.0, 0.0, stamp, "odom1", "base_link1", stamp)
    total = tracker.snapshot(0.2)["total_distance_m"]
    tracker.note_source_lifecycle("stop", 0.3, wall_now=100.3)
    stopped = tracker.snapshot(0.3)
    assert stopped["state"] == "unknown"
    assert stopped["validity"]["paused"] is True
    assert stopped["total_distance_m"] == pytest.approx(total)
    assert any(
        event.get("reason") == "chassis_stop" for event in tracker.take_events()
    )
    # A fresh sample after restart is only a new baseline; no lost-time bridge.
    tracker.update(1.0, 0.0, 0.4, "odom1", "base_link1", 0.4)
    tracker.update(1.0, 0.0, 0.5, "odom1", "base_link1", 0.5)
    assert tracker.snapshot(0.5)["total_distance_m"] == pytest.approx(total + 0.1)


def test_never_received_odom_total_is_null_not_zero():
    tracker = MotionTracker()
    snapshot = tracker.snapshot(0.0)
    assert snapshot["total_distance_m"] is None
    assert snapshot["validity"]["available"] is False
    assert snapshot["state"] == "unknown"


def test_service_status_transitions_and_imu_observation_are_auxiliary():
    monitors = {
        "imu": TopicRateMonitor("imu", "IMU", "/imu", "sensor_msgs/msg/Imu"),
        "odom": TopicRateMonitor("odom", "Odom", "/odom1", "nav_msgs/msg/Odometry"),
    }
    service = DiagnosticsService(monitors=monitors, motion=MotionTracker())
    imu_msg = SimpleNamespace(
        linear_acceleration=SimpleNamespace(x=1.0, y=2.0, z=9.8),
        angular_velocity=SimpleNamespace(x=0.1, y=0.2, z=0.3),
    )
    service.record_imu(imu_msg, 1.0)
    service.update_publisher_counts({"imu": 1, "odom": 1}, 1.0)
    payload, events = service.tick(1.1, {"imu": 1, "odom": 1})
    assert payload["imu_observation"]["motion_proof"] is False
    assert payload["imu_observation"]["acceleration_mps2"]["z"] == pytest.approx(9.8)
    assert any(event["kind"] == "topic_status" for event in events)


def test_service_record_odom_reads_real_nav_msgs_twist_with_covariance_shape():
    monitors = {
        "odom": TopicRateMonitor("odom", "Odom", "/odom1", "nav_msgs/msg/Odometry")
    }
    service = DiagnosticsService(monitors, MotionTracker(debounce_sec=0.0))
    for index in range(4):
        service.record_odom(_odom_msg(1.0, index * 0.1), index * 0.1)
    payload, events = service.tick(0.3)
    motion = payload["motion"]
    assert motion["state"] == "moving"
    assert motion["speed_mps"] == pytest.approx(1.0)
    assert motion["total_distance_m"] == pytest.approx(0.3)
    assert motion["validity"]["invalid_samples"] == 0
    assert any(event["event"] == "motion_start" for event in events)


def test_real_nav_msgs_odometry_via_service_in_fresh_ros_process():
    ros_setups = sorted(Path("/opt/ros").glob("*/setup.bash"))
    if not ros_setups:
        pytest.skip("no sourced ROS 2 installation is available")
    root = Path(__file__).resolve().parents[1]
    script = r'''
import json
import math
import sys

sys.path.insert(0, sys.argv[1])
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry
from web_control.diagnostics import DiagnosticsService, MotionTracker, TopicRateMonitor

monitors = {
    "odom": TopicRateMonitor("odom", "Odom", "/odom1", "nav_msgs/msg/Odometry")
}
service = DiagnosticsService(monitors, MotionTracker(debounce_sec=0.3))
for index in range(120):
    msg = Odometry()
    msg.header.stamp.sec = index // 50
    msg.header.stamp.nanosec = (index % 50) * 20_000_000
    msg.header.frame_id = "odom1"
    msg.child_frame_id = "base_link1"
    msg.twist = TwistWithCovariance()
    if index < 40:
        msg.twist.twist.linear.x = 1.0
    elif index < 80:
        msg.twist.twist.linear.x = -1.0
    else:
        msg.twist.twist.linear.x = 0.0
    service.record_odom(msg, index * 0.02)
payload, events = service.tick(119 * 0.02)
motion = payload["motion"]
print(json.dumps({
    "hz": payload["topics"][0]["hz"],
    "received": payload["topics"][0]["received_count"],
    "state": motion["state"],
    "total": motion["total_distance_m"],
    "invalid": motion["validity"]["invalid_samples"],
    "event_kinds": [event["event"] for event in events],
}))
'''
    command = (
        f"source {ros_setups[0]} && exec python3 -c "
        + __import__("shlex").quote(script)
        + " "
        + __import__("shlex").quote(str(root))
    )
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    env.pop("PYTHONHOME", None)
    result = subprocess.run(
        ["bash", "-c", command],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
        env=env,
    )
    payload = json.loads(result.stdout.strip().splitlines()[-1])
    assert payload["received"] == 120
    assert payload["hz"] == pytest.approx(50.0, abs=0.1)
    assert payload["invalid"] == 0
    assert payload["state"] == "stationary"
    assert payload["total"] == pytest.approx(1.58)
    assert "motion_start" in payload["event_kinds"]
    assert "motion_stop" in payload["event_kinds"]


def _diag_store(tmp_path):
    return ManagedLogStore(tmp_path / "logs")


def test_jsonl_writer_records_structured_entries(tmp_path):
    store = _diag_store(tmp_path)
    writer = DiagnosticsLogWriter(store)
    assert writer.write({"kind": "snapshot", "motion": {"state": "moving"}}, now=time.time())
    session = writer._session
    writer.close()
    text = session.path.read_text(encoding="utf-8")
    payloads = [
        json.loads(line)
        for line in text.splitlines()
        if line.startswith("{")
    ]
    assert payloads[0]["kind"] == "log_start"
    assert payloads[-1]["kind"] == "snapshot"
    assert payloads[-1]["motion"]["state"] == "moving"
    assert writer.status()["error"] is None


def test_header_write_failure_is_not_reported_as_saved_and_recovers(tmp_path, monkeypatch):
    real_open = builtins.open
    state = {"fail": True, "handles": []}

    class FailingHandle:
        def __init__(self, handle):
            self._handle = handle
            self.closed = False
            state["handles"].append(self)

        def write(self, text):
            if state["fail"]:
                raise OSError("No space left on device")
            return self._handle.write(text)

        def flush(self):
            return self._handle.flush()

        def close(self):
            self.closed = True
            return self._handle.close()

        def __getattr__(self, name):
            return getattr(self._handle, name)

    def failing_open(*args, **kwargs):
        return FailingHandle(real_open(*args, **kwargs))

    monkeypatch.setattr("web_control.log_store.open", failing_open, raising=False)
    store = _diag_store(tmp_path)
    writer = DiagnosticsLogWriter(store)
    assert writer.error is not None
    assert "No space left on device" in writer.error
    assert writer.write({"kind": "snapshot"}) is False
    assert writer.error is not None
    assert writer.status()["active_session"] is None
    sessions = store.list_sessions("diagnostics")
    assert sessions
    assert all(not row["active"] for row in sessions)
    assert all(row["size"] == 0 for row in sessions)
    assert state["handles"] and all(handle.closed for handle in state["handles"])

    state["fail"] = False
    assert writer.write({"kind": "snapshot", "recovered": True}) is True
    assert writer.error is None
    session_id = writer.status()["active_session"]
    assert session_id
    assert any(row["session"] == session_id for row in store.list_sessions("diagnostics"))
    writer.close()
    assert writer.status()["active_session"] is None
    assert all(handle.closed for handle in state["handles"])


def test_writer_rotates_on_local_day_boundary(tmp_path):
    store = _diag_store(tmp_path)
    writer = DiagnosticsLogWriter(store)
    first = writer._session
    writer.write({"kind": "snapshot"}, now=time.time())
    writer.write({"kind": "snapshot"}, now=time.time() + 86400)
    assert writer.rotations >= 1
    assert writer._session is not first
    writer.close()


def test_writer_rotates_at_size_and_prunes_only_completed_diagnostics(tmp_path, monkeypatch):
    store = _diag_store(tmp_path)
    writer = DiagnosticsLogWriter(store, max_bytes=1024)
    first = writer._session
    writer.write({"kind": "snapshot", "blob": "x" * 2000})
    assert writer.write({"kind": "snapshot", "blob": "y" * 2000})
    assert writer.rotations >= 1
    writer.close()
    sessions = store.list_sessions("diagnostics")
    assert len(sessions) >= 2
    assert first.path.exists()

    # A separate completed old diagnostics log is deleted; a hardware log with
    # the same age is not touched.
    old = store.start_session("diagnostics", ["diagnostics", "jsonl"])
    old.write_line("{}")
    store.end_session(old)
    hardware = store.start_session("chassis", ["ros2", "launch", "hunter"])
    hardware.write_line("hardware")
    store.end_session(hardware)
    old_epoch = time.time() - 9 * 86400
    os.utime(old.path, (old_epoch, old_epoch))
    os.utime(hardware.path, (old_epoch, old_epoch))
    writer2 = DiagnosticsLogWriter(store, retain_days=7, retain_bytes=100 * 1024 * 1024)
    writer2.prune(time.time())
    active_session = writer2.status()["active_session"]
    assert any(
        row["session"] == active_session
        for row in store.list_sessions("diagnostics")
    )
    writer2.close()
    assert not old.path.exists()
    assert hardware.path.exists()


def test_writer_retains_only_latest_completed_bytes_and_surfaces_failure(tmp_path):
    store = _diag_store(tmp_path)
    writer = DiagnosticsLogWriter(store, retain_bytes=2048)
    # Complete two large files, then let a later prune enforce the byte cap.
    for _ in range(2):
        session = store.start_session("diagnostics", ["diagnostics", "jsonl"])
        session.write_line("z" * 3000)
        store.end_session(session)
    writer.prune(time.time())
    writer.close()
    total = sum(
        row["size"]
        for row in store.list_sessions("diagnostics")
        if not row["active"]
    )
    assert total <= 2048

    class BrokenSession:
        bytes_written = 0

        def write_line(self, _line):
            from web_control.log_store import LogStorageError

            raise LogStorageError("disk full")

        def close(self):
            pass

    writer._session = BrokenSession()
    assert writer.write({"kind": "snapshot"}) is False
    assert "disk full" in writer.error


class _LoggerStub:
    def info(self, *_a, **_k):
        pass

    def warning(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass

    def debug(self, *_a, **_k):
        pass


def _serve(node):
    handler = web_control_node.WebControlNode.make_handler(node)
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def test_http_diagnostics_and_saved_logs_category(tmp_path):
    store = _diag_store(tmp_path)
    writer = DiagnosticsLogWriter(store)
    monitors = {
        "odom": TopicRateMonitor("odom", "Odom", "/odom1", "nav_msgs/msg/Odometry")
    }
    node = SimpleNamespace()
    node.diagnostics_service = DiagnosticsService(monitors, MotionTracker())
    node.diagnostics_log = writer
    node.diagnostics_enabled = True
    node.diagnostics_sample_period = 1.0
    node.diagnostics_snapshot_period = 5.0
    node.log_store = store
    node.managed_logs = {}
    node.static_dir = Path(__file__).resolve().parents[1] / "static"
    node.now_sec = lambda: 123.0
    node.get_logger = lambda: _LoggerStub()
    for name in (
        "diagnostics_payload",
        "_require_log_target",
        "saved_logs_payload",
        "saved_log_tail",
        "saved_log_download_path",
    ):
        setattr(node, name, MethodType(getattr(web_control_node.WebControlNode, name), node))
    writer.write({"kind": "snapshot", "motion": {"state": "stationary"}})
    session_id = writer.status()["active_session"]
    server, _thread = _serve(node)
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        with urllib.request.urlopen(f"{base}/api/diagnostics", timeout=10) as response:
            payload = json.loads(response.read().decode("utf-8"))
        assert payload["ok"] is True
        assert payload["motion"]["total_distance_m"] is None
        assert payload["logging"]["error"] is None

        with urllib.request.urlopen(
            f"{base}/api/logs/sessions?target=diagnostics", timeout=10
        ) as response:
            sessions = json.loads(response.read().decode("utf-8"))
        assert sessions["ok"] is True
        assert sessions["sessions"][0]["session"] == session_id

        with urllib.request.urlopen(
            f"{base}/api/logs/tail?target=diagnostics&session={session_id}", timeout=10
        ) as response:
            tail = response.read().decode("utf-8")
        assert '"kind":"snapshot"' in tail
        with urllib.request.urlopen(
            f"{base}/api/logs/download?target=diagnostics&session={session_id}",
            timeout=10,
        ) as response:
            downloaded = response.read().decode("utf-8")
        assert '"kind":"snapshot"' in downloaded
    finally:
        server.shutdown()
        server.server_close()
        writer.close()


def test_shutdown_flushes_motion_after_last_snapshot_and_final_total(tmp_path, monkeypatch):
    store = _diag_store(tmp_path)
    writer = DiagnosticsLogWriter(store)
    monitors = {
        "odom": TopicRateMonitor("odom", "Odom", "/odom1", "nav_msgs/msg/Odometry")
    }
    service = DiagnosticsService(monitors, MotionTracker(debounce_sec=0.0))
    node = SimpleNamespace(diagnostics_service=service, diagnostics_log=writer)
    for name in ("_write_diagnostics", "shutdown_diagnostics", "note_chassis_diagnostics_lifecycle"):
        setattr(node, name, MethodType(getattr(web_control_node.WebControlNode, name), node))
    node.get_logger = lambda: _LoggerStub()

    service.record_odom(_odom_msg(0.0, 0.0), 0.0)
    payload, _events = service.tick(0.0)
    writer.write({"kind": "snapshot", **payload})
    # Motion begins and ends after the last periodic snapshot.
    service.record_odom(_odom_msg(1.0, 0.1), 0.1)
    service.record_odom(_odom_msg(1.0, 0.2), 0.2)
    node.note_chassis_diagnostics_lifecycle("stop")
    monkeypatch.setattr(web_control_node.time, "monotonic", lambda: 0.2)
    session_id = writer.status()["active_session"]
    node.shutdown_diagnostics()

    path = store.resolve_session_file("diagnostics", session_id)
    records = [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.startswith("{")
    ]
    shutdown = [record for record in records if record.get("kind") == "shutdown"][-1]
    assert shutdown["motion"]["total_distance_m"] == pytest.approx(0.15)
    assert shutdown["total_distance_m"] == pytest.approx(0.15)
    assert shutdown["speed_mps"] == pytest.approx(1.0)
    assert shutdown["segment_distance_m"] == pytest.approx(0.15)
    assert shutdown["validity"]["reason"] == "chassis_stop"
    assert shutdown["imu_observation"] is None
    assert any(record.get("event") == "motion_start" for record in records)
    assert any(record.get("event") == "motion_interruption" for record in records)
    assert shutdown["logging"]["error"] is None


def test_frontend_panel_chips_global_poll_and_log_category():
    root = Path(__file__).resolve().parents[1]
    html = (root / "static" / "index.html").read_text(encoding="utf-8")
    js = (root / "static" / "app.js").read_text(encoding="utf-8")
    assert html.index('id="diagnosticsTitle"') < html.index('id="tfTreeTitle"')
    assert 'data-diagnostics-chip="imu"' in html
    assert html.count('data-diagnostics-chip="imu"') == 2
    assert 'data-diagnostics-chip="lidar"' in html
    assert '<option value="diagnostics">话题与运动日志</option>' in html
    # Topic/IMU log writing is controlled from both map pages, not System.
    assert html.count('data-topic-logging-toggle') == 2
    nav_start = html.index('id="navigationTopicLoggingToggle"')
    assert nav_start < html.index('id="mappingTopicLoggingToggle"')
    for marker in ("navigationTopicLoggingToggle", "mappingTopicLoggingToggle"):
        tag_start = html.index(f'id="{marker}"')
        tag = html[tag_start:html.index(">", tag_start)]
        assert 'class="start-log topic-logging-toggle"' in tag
        assert 'aria-pressed="false"' in tag
        assert "disabled" in tag
    assert html.count('data-topic-logging-state') == 2
    assert 'id="navigationTopicLoggingHint"' not in html
    assert 'id="mappingTopicLoggingHint"' not in html
    assert 'class="topic-logging-hint"' not in html
    for marker in ("navigationTopicLoggingToggle", "mappingTopicLoggingToggle"):
        tag_start = html.index(f'id="{marker}"')
        tag = html[tag_start:html.index(">", tag_start)]
        assert "aria-describedby" not in tag
    assert 'id="topicLoggingToggle"' not in html
    assert 'class="diagnostics-logging"' not in html
    assert 'querySelectorAll("[data-topic-logging-toggle]")' in js
    assert "function attachTopicLoggingEvents()" in js
    assert 'getJson("/api/diagnostics")' in js
    assert "setInterval(pollDiagnostics, 1000)" in js
    assert "function updateDiagnostics(data)" in js
    assert "textContent" in js
    assert "轮速估计" in js


def test_diagnostics_frontend_runtime_stale_and_error_rendering(tmp_path):
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    app = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    start = app.index("// --- Topic and wheel-motion diagnostics")
    end = app.index("async function pollDiagnostics()", start)
    block = app[start:end]
    harness = r'''
const elements = new Map();
function el(id) {
  if (!elements.has(id)) elements.set(id, {
    id, textContent: "", className: "", dataset: {}, children: [],
    appendChild(child) { this.children.push(child); },
    replaceChildren(...items) { this.children = items; },
  });
  return elements.get(id);
}
const chips = ["imu", "imu", "lidar", "lidar"].map((key) => ({
  dataset: { diagnosticsChip: key }, textContent: "",
}));
const document = {
  getElementById: el,
  createElement(tag) { return { tag, textContent: "", className: "", scope: "", colSpan: 0, dataset: {}, children: [], appendChild(child) { this.children.push(child); } }; },
  querySelectorAll(selector) { return selector === "[data-diagnostics-chip]" ? chips : []; },
};
const data = {
  ok: true, logging: {}, topics: [
    {key: "imu", topic: "/imu", message_type: "sensor_msgs/msg/Imu", publisher_count: 1, hz: 9.9, last_age_sec: 3.2, status: "stale"},
    {key: "lidar", topic: "/lidar_points", message_type: "sensor_msgs/msg/PointCloud2", publisher_count: 0, hz: 0, last_age_sec: null, status: "no_publisher"},
  ],
  imu_observation: {
    status: "stale", last_age_sec: 3.2,
    acceleration_mps2: {x: 1, y: 2, z: 9.8}, angular_velocity_rps: {x: 0.1, y: 0.2, z: 0.3},
  },
  motion: {state: "unknown", source_topic: "/odom1", speed_mps: 1.25, total_distance_m: 12.5, segment_distance_m: 0.5, segment_duration_sec: 0.8, validity: {available: true, valid: false, paused: true, reason: "odom_stale"}},
};
updateDiagnostics(data);
const output = {
  chips: chips.map((chip) => chip.textContent),
  chipStates: chips.map((chip) => chip.dataset.state),
  status: el("diagnosticsStatus").textContent,
  motion: el("diagnosticsMotionState").textContent,
  speed: el("diagnosticsSpeed").textContent,
  distance: el("diagnosticsDistance").textContent,
  segment: el("diagnosticsSegment").textContent,
  validity: el("diagnosticsValidity").textContent,
  imu: el("diagnosticsImu").textContent,
  rows: el("diagnosticsTopics").children.length,
  hz: el("diagnosticsTopics").children[0].children[3].textContent,
  stale: el("diagnosticsTopics").children[0].children[5].textContent,
};
const absentOdom = {
  ok: true, logging: {}, topics: [],
  motion: {state: "unknown", source_topic: "/odom1", speed_mps: null, total_distance_m: null, segment_distance_m: 0, segment_duration_sec: 0, validity: {available: false, valid: false, paused: false}},
};
updateDiagnostics(absentOdom);
output.absentSegment = el("diagnosticsSegment").textContent;
output.absentDistance = el("diagnosticsDistance").textContent;
output.absentValidity = el("diagnosticsValidity").textContent;
updateDiagnostics(null);
output.errorMotion = el("diagnosticsMotionState").textContent;
output.errorImu = el("diagnosticsImu").textContent;
output.errorRows = el("diagnosticsTopics").children.length;
console.log(JSON.stringify(output));
'''
    result = subprocess.run(
        [node, "--input-type=module", "-e", block + harness],
        check=True,
        capture_output=True,
        text=True,
        timeout=20,
    )
    payload = json.loads(result.stdout)
    assert payload["chips"] == ["IMU 数据过期", "IMU 数据过期", "Lidar 无发布者", "Lidar 无发布者"]
    assert payload["chipStates"] == ["stale", "stale", "no_publisher", "no_publisher"]
    assert payload["status"] == "2 个话题 · 0 个正常"
    assert "后台" not in payload["status"]
    assert payload["motion"] == "未知"
    assert payload["speed"] == "--（当前不可用）"
    assert payload["distance"].startswith("12.50 m")
    assert payload["segment"] == "0.50 m / 0.8 s"
    assert "已暂停" in payload["validity"]
    assert "累计里程保留" in payload["validity"]
    assert "最近一次" in payload["imu"] and "已过期" in payload["imu"]
    assert payload["rows"] == 2
    assert payload["hz"] == "--"
    assert payload["stale"] == "数据过期"
    assert payload["absentSegment"] == "--"
    assert payload["absentDistance"].startswith("--")
    assert "不可用" in payload["absentValidity"]
    assert payload["errorMotion"] == "未知"
    assert "不可用" in payload["errorImu"]
    assert payload["errorRows"] == 1


def _http_json(url, payload=None, method="GET", raw=None):
    data = raw.encode("utf-8") if raw is not None else None
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        request.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def test_topic_logging_preference_defaults_on_persists_and_preserves_other_prefs(tmp_path):
    path = tmp_path / "selected_map.json"
    path.write_text(
        json.dumps(
            {
                "name": "warehouse",
                "mode": "tiled",
                "ui": {
                    "joystick": {
                        "throttle": {"size": 180, "x": 10, "y": 20},
                        "steering": {"size": 190, "x": 30, "y": 40},
                    },
                    "unknown_ui": {"keep": [1, 2]},
                },
                "unknown_top": {"keep": True},
            }
        ),
        encoding="utf-8",
    )
    store = SelectionStore(path)
    assert store.load_topic_logging_enabled() is True

    assert store.save_topic_logging_enabled(False) is False
    saved = json.loads(path.read_text(encoding="utf-8"))
    assert saved["ui"]["topic_logging_enabled"] is False
    assert saved["name"] == "warehouse"
    assert saved["mode"] == "tiled"
    assert saved["ui"]["joystick"]["throttle"]["x"] == 10
    assert saved["ui"]["unknown_ui"] == {"keep": [1, 2]}
    assert saved["unknown_top"] == {"keep": True}
    # A new store/node reads the saved choice rather than the default.
    assert SelectionStore(path).load_topic_logging_enabled() is False

    path.write_text(
        json.dumps({"ui": {"topic_logging_enabled": "off"}}), encoding="utf-8"
    )
    assert SelectionStore(path).load_topic_logging_enabled() is True


def test_topic_logging_strict_bool_validation_rejects_every_bad_shape():
    for bad in (
        True,
        False,
        None,
        "false",
        0,
        1,
        [],
        {},
        {"enabled": True, "extra": 1},
        {"topic_logging_enabled": False},
        {"enabled": "false"},
        {"enabled": 0},
        {"enabled": None},
    ):
        with pytest.raises(ValueError):
            validate_topic_logging_enabled(bad)
    assert validate_topic_logging_enabled({"enabled": True}) is True
    assert validate_topic_logging_enabled({"enabled": False}) is False


def test_disabled_topic_logging_filters_snapshot_startup_status_and_enrichment(tmp_path):
    store = _diag_store(tmp_path)
    writer = DiagnosticsLogWriter(store)
    node = SimpleNamespace(
        diagnostics_log=writer,
        topic_logging_enabled=False,
        _diagnostics_lock=threading.RLock(),
        _diagnostics_log_error=None,
        get_logger=lambda: _LoggerStub(),
    )
    node._write_diagnostics = MethodType(
        web_control_node.WebControlNode._write_diagnostics, node
    )
    node._write_diagnostics(
        {
            "kind": "topic_status",
            "event": "status_change",
            "topic": "/imu",
            "status": "active",
            "topic_hz": 10.0,
            "topic_age_sec": 0.05,
            "publisher_count": 1,
            "topic_status": "active",
        },
        time.monotonic(),
    )
    node._write_diagnostics(
        {
            "kind": "startup",
            "source": "diagnostics",
            "enabled": True,
            "topics": [{"topic": "/imu", "hz": 10.0}],
            "imu_observation": {"acceleration_mps2": {"z": 9.8}},
            "motion": {"state": "stationary", "total_distance_m": 0.0},
        },
        time.monotonic(),
    )
    node._write_diagnostics(
        {
            "kind": "motion",
            "event": "motion_start",
            "source": "/odom1",
            "topic": "/odom1",
            "speed_mps": 1.0,
            "total_distance_m": 1.5,
            "segment_distance_m": 0.5,
            "segment_duration_sec": 0.5,
            "validity": {"valid": True},
            "topic_hz": 50.0,
            "topic_age_sec": 0.01,
            "publisher_count": 1,
            "topic_status": "active",
            "status": "active",
            "motion": {"state": "moving", "total_distance_m": 1.5},
        },
        time.monotonic(),
    )
    node._write_diagnostics(
        {
            "kind": "snapshot",
            "source": "diagnostics",
            "topics": [{"topic": "/imu", "hz": 10.0}],
            "imu_observation": {"acceleration_mps2": {"z": 9.8}},
            "motion": {"state": "moving", "speed_mps": 1.0},
        },
        time.monotonic(),
    )
    session = writer._session
    writer.close()
    records = [
        json.loads(line)
        for line in session.path.read_text(encoding="utf-8").splitlines()
        if line.startswith("{")
    ]
    assert all(record.get("kind") != "topic_status" for record in records)
    assert all(TOPIC_LOG_KEYS.isdisjoint(record) for record in records)
    motion = [record for record in records if record.get("event") == "motion_start"]
    assert len(motion) == 1
    assert motion[0]["source"] == "/odom1"
    assert motion[0]["speed_mps"] == pytest.approx(1.0)
    assert motion[0]["total_distance_m"] == pytest.approx(1.5)
    assert motion[0]["segment_distance_m"] == pytest.approx(0.5)
    assert motion[0]["segment_duration_sec"] == pytest.approx(0.5)
    assert motion[0]["validity"] == {"valid": True}
    snapshot = [record for record in records if record.get("kind") == "snapshot"]
    assert snapshot and snapshot[0]["motion"]["state"] == "moving"


def test_disabled_topic_logging_shutdown_filters_topics_imu_but_flushes_motion(tmp_path):
    store = _diag_store(tmp_path)
    writer = DiagnosticsLogWriter(store)
    service = DiagnosticsService(
        {
            "imu": TopicRateMonitor("imu", "IMU", "/imu", "sensor_msgs/msg/Imu"),
            "odom": TopicRateMonitor("odom", "Odom", "/odom1", "nav_msgs/msg/Odometry"),
        },
        MotionTracker(debounce_sec=0.0),
    )
    for index in range(4):
        service.record_odom(_odom_msg(1.0, index * 0.1), index * 0.1)
    service.record_imu(
        SimpleNamespace(
            linear_acceleration=SimpleNamespace(x=1.0, y=2.0, z=9.8),
            angular_velocity=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        ),
        0.3,
    )
    node = SimpleNamespace(
        diagnostics_service=service,
        diagnostics_log=writer,
        topic_logging_enabled=False,
        _diagnostics_lock=threading.RLock(),
        _diagnostics_log_error=None,
        get_logger=lambda: _LoggerStub(),
    )
    for name in ("_write_diagnostics", "shutdown_diagnostics"):
        setattr(node, name, MethodType(getattr(web_control_node.WebControlNode, name), node))
    session_id = writer.status()["active_session"]
    node.shutdown_diagnostics()
    records = [
        json.loads(line)
        for line in store.resolve_session_file("diagnostics", session_id)
        .read_text(encoding="utf-8")
        .splitlines()
        if line.startswith("{")
    ]
    shutdown = [record for record in records if record.get("kind") == "shutdown"][-1]
    assert all(TOPIC_LOG_KEYS.isdisjoint(record) for record in records)
    assert all(record.get("kind") != "topic_status" for record in records)
    assert shutdown["motion"]["total_distance_m"] == pytest.approx(0.3)
    assert shutdown["total_distance_m"] == pytest.approx(0.3)
    assert shutdown["speed_mps"] == pytest.approx(1.0)
    assert shutdown["validity"]["available"] is True
    assert any(record.get("event") == "motion_start" for record in records)


def test_live_diagnostics_frequency_and_imu_stay_visible_when_logging_off(
    tmp_path, monkeypatch
):
    store = _diag_store(tmp_path)
    writer = DiagnosticsLogWriter(store)
    service = DiagnosticsService(
        {
            "imu": TopicRateMonitor("imu", "IMU", "/imu", "sensor_msgs/msg/Imu"),
            "odom": TopicRateMonitor("odom", "Odom", "/odom1", "nav_msgs/msg/Odometry"),
        },
        MotionTracker(debounce_sec=0.0),
    )
    for index in range(3):
        service.record_odom(_odom_msg(1.0, index * 0.1), index * 0.1)
    service.record_imu(
        SimpleNamespace(
            linear_acceleration=SimpleNamespace(x=0.0, y=0.0, z=9.8),
            angular_velocity=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        ),
        0.2,
    )
    service.update_publisher_counts({"imu": 1, "odom": 1}, 0.2)
    node = SimpleNamespace(
        diagnostics_service=service,
        diagnostics_log=writer,
        topic_logging_enabled=False,
        _diagnostics_lock=threading.RLock(),
        diagnostics_enabled=True,
        diagnostics_sample_period=1.0,
        diagnostics_snapshot_period=5.0,
        log_store=store,
        now_sec=lambda: 123.0,
    )
    node.diagnostics_payload = MethodType(
        web_control_node.WebControlNode.diagnostics_payload, node
    )
    monkeypatch.setattr(web_control_node.time, "monotonic", lambda: 0.2)
    payload = node.diagnostics_payload()
    writer.close()
    assert payload["topic_logging_enabled"] is False
    odom = next(row for row in payload["topics"] if row["key"] == "odom")
    assert odom["hz"] == pytest.approx(10.0)
    assert odom["received_count"] == 3
    assert odom["status"] == "active"
    assert payload["imu_observation"] is not None


def test_topic_logging_toggle_audit_persists_and_io_failure_keeps_active_flag(
    tmp_path, monkeypatch
):
    prefs_path = tmp_path / "selected_map.json"
    prefs = SelectionStore(prefs_path)
    store = _diag_store(tmp_path)
    writer = DiagnosticsLogWriter(store)
    node = SimpleNamespace(
        map_selection=prefs,
        diagnostics_log=writer,
        topic_logging_enabled=True,
        _diagnostics_lock=threading.RLock(),
        _diagnostics_log_error=None,
        now_sec=lambda: 123.0,
        get_logger=lambda: _LoggerStub(),
    )
    for name in ("_write_diagnostics", "set_topic_logging"):
        setattr(node, name, MethodType(getattr(web_control_node.WebControlNode, name), node))
    response = node.set_topic_logging({"enabled": False})
    assert response["ok"] is True
    assert response["topic_logging_enabled"] is False
    assert node.topic_logging_enabled is False
    assert prefs.load_topic_logging_enabled() is False
    # Idempotent repeat is safe and does not create another audit row.
    session = writer._session
    before = session.path.read_text(encoding="utf-8").count('"kind":"logging_toggle"')
    response = node.set_topic_logging({"enabled": False})
    assert response["topic_logging_enabled"] is False
    during = session.path.read_text(encoding="utf-8").count('"kind":"logging_toggle"')
    assert during == before == 1

    real_save = prefs.save_topic_logging_enabled

    def fail_save(_enabled):
        raise OSError("disk full")

    monkeypatch.setattr(prefs, "save_topic_logging_enabled", fail_save)
    with pytest.raises(web_control_node.TopicLoggingPersistenceError):
        node.set_topic_logging({"enabled": True})
    assert node.topic_logging_enabled is False
    assert prefs.load_topic_logging_enabled() is False
    monkeypatch.setattr(prefs, "save_topic_logging_enabled", real_save)

    # The preference save can succeed while the audit write fails; the flag
    # stays applied and the existing logging.error channel reports the write.
    def fail_write(_entry, now=None):
        writer.error = "disk full"
        return False

    monkeypatch.setattr(writer, "write", fail_write)
    response = node.set_topic_logging({"enabled": True})
    assert response["ok"] is True
    assert response["topic_logging_enabled"] is True
    assert response["logging"]["error"] == "disk full"
    assert node.topic_logging_enabled is True
    assert prefs.load_topic_logging_enabled() is True
    writer.close()


def test_topic_logging_disable_waits_for_inflight_write_and_blocks_followups(tmp_path):
    class BlockingWriter:
        def __init__(self):
            self.entries = []
            self.error = None
            self.session_id = "fake-session"
            self.block_next = True
            self.entered = threading.Event()
            self.release = threading.Event()

        def write(self, entry, now=None):
            self.entries.append(dict(entry))
            if entry.get("kind") == "snapshot" and self.block_next:
                self.block_next = False
                self.entered.set()
                assert self.release.wait(5.0)
            return True

        def status(self):
            return {"error": self.error, "active_session": self.session_id}

    prefs = SelectionStore(tmp_path / "selected_map.json")
    writer = BlockingWriter()
    node = SimpleNamespace(
        map_selection=prefs,
        diagnostics_log=writer,
        topic_logging_enabled=True,
        _diagnostics_lock=threading.RLock(),
        _diagnostics_log_error=None,
        now_sec=lambda: 123.0,
        get_logger=lambda: _LoggerStub(),
    )
    node._write_diagnostics = MethodType(
        web_control_node.WebControlNode._write_diagnostics, node
    )
    node.set_topic_logging = MethodType(
        web_control_node.WebControlNode.set_topic_logging, node
    )
    snapshot = {
        "kind": "snapshot",
        "topics": [{"topic": "/imu", "hz": 10.0}],
        "imu_observation": {"z": 9.8},
        "motion": {"state": "moving", "total_distance_m": 2.0},
    }
    writer_thread = threading.Thread(
        target=node._write_diagnostics, args=(snapshot, time.monotonic())
    )
    writer_thread.start()
    assert writer.entered.wait(2.0)
    toggle_done = threading.Event()

    def toggle():
        node.set_topic_logging({"enabled": False})
        toggle_done.set()

    toggle_thread = threading.Thread(target=toggle)
    toggle_thread.start()
    time.sleep(0.1)
    assert not toggle_done.is_set()
    assert node.topic_logging_enabled is True
    writer.release.set()
    writer_thread.join(2.0)
    toggle_thread.join(2.0)
    assert toggle_done.is_set()
    assert node.topic_logging_enabled is False
    # The in-flight write finished before success; all later writes are gated.
    assert "topics" in writer.entries[0]
    node._write_diagnostics(snapshot, time.monotonic())
    assert TOPIC_LOG_KEYS.isdisjoint(writer.entries[-1])
    assert writer.entries[-1]["motion"]["total_distance_m"] == pytest.approx(2.0)
    audit = [entry for entry in writer.entries if entry.get("kind") == "logging_toggle"]
    assert len(audit) == 1
    assert audit[0]["topic_logging_enabled"] is False


def test_topic_logging_http_strict_post_persistence_audit_and_get(tmp_path, monkeypatch):
    store = _diag_store(tmp_path)
    writer = DiagnosticsLogWriter(store)
    prefs = SelectionStore(tmp_path / "selected_map.json")
    service = DiagnosticsService(
        {
            "odom": TopicRateMonitor(
                "odom", "Odom", "/odom1", "nav_msgs/msg/Odometry"
            )
        },
        MotionTracker(),
    )
    node = SimpleNamespace(
        diagnostics_service=service,
        diagnostics_log=writer,
        topic_logging_enabled=True,
        _diagnostics_lock=threading.RLock(),
        _diagnostics_log_error=None,
        diagnostics_enabled=True,
        diagnostics_sample_period=1.0,
        diagnostics_snapshot_period=5.0,
        log_store=store,
        map_selection=prefs,
        static_dir=Path(__file__).resolve().parents[1] / "static",
        managed_logs={},
        now_sec=lambda: 123.0,
        get_logger=lambda: _LoggerStub(),
    )
    for name in ("_write_diagnostics", "set_topic_logging", "diagnostics_payload"):
        setattr(node, name, MethodType(getattr(web_control_node.WebControlNode, name), node))
    server, _thread = _serve(node)
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        status, body = _http_json(f"{base}/api/diagnostics")
        assert status == 200 and body["topic_logging_enabled"] is True

        status, body = _http_json(
            f"{base}/api/diagnostics/logging", {"enabled": False}, "POST"
        )
        assert status == 200, body
        assert body["topic_logging_enabled"] is False
        assert node.topic_logging_enabled is False
        assert prefs.load_topic_logging_enabled() is False

        for bad in (
            {"enabled": "false"},
            {"enabled": 0},
            {"enabled": None},
            {},
            {"enabled": True, "unknown": 1},
            [],
        ):
            status, _body = _http_json(
                f"{base}/api/diagnostics/logging", bad, "POST"
            )
            assert status == 400
            assert node.topic_logging_enabled is False
        status, _body = _http_json(
            f"{base}/api/diagnostics/logging", raw="{bad", method="POST"
        )
        assert status == 400
        assert node.topic_logging_enabled is False

        def fail_save(_enabled):
            raise OSError("disk full")

        monkeypatch.setattr(prefs, "save_topic_logging_enabled", fail_save)
        status, body = _http_json(
            f"{base}/api/diagnostics/logging", {"enabled": True}, "POST"
        )
        assert status == 503, body
        assert node.topic_logging_enabled is False
        assert "disk full" in body["message"]

        status, body = _http_json(f"{base}/api/diagnostics")
        assert status == 200 and body["topic_logging_enabled"] is False
        session_id = writer.status()["active_session"]
        records = [
            json.loads(line)
            for line in store.resolve_session_file("diagnostics", session_id)
            .read_text(encoding="utf-8")
            .splitlines()
            if line.startswith("{")
        ]
        audit = [record for record in records if record.get("kind") == "logging_toggle"]
        assert len(audit) == 1
        assert audit[0]["topic_logging_enabled"] is False
        assert isinstance(audit[0]["at"], (int, float))
    finally:
        server.shutdown()
        server.server_close()
        writer.close()


def test_topic_logging_page_buttons_pending_rollback_error_and_stale_poll():
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed")
    app = (Path(__file__).resolve().parents[1] / "static" / "app.js").read_text(
        encoding="utf-8"
    )
    start = app.index("// --- Topic and wheel-motion diagnostics")
    end = app.index("// --- System TF tree", start)
    block = app[start:end]
    attach_start = app.index("function attachTopicLoggingEvents()")
    attach_end = app.index("function attachEvents()", attach_start)
    attach_block = app[attach_start:attach_end]
    harness = r'''
const elements = new Map();
function makeNode(id) {
  const listeners = new Map();
  const classes = new Set();
  const attributes = {};
  const node = {
    id, textContent: "", dataset: {}, children: [], disabled: false, checked: false,
    hidden: false, colSpan: 0, scope: "",
    appendChild(child) { this.children.push(child); },
    replaceChildren(...items) { this.children = items; },
    addEventListener(type, handler) {
      if (!listeners.has(type)) listeners.set(type, []);
      listeners.get(type).push(handler);
    },
    dispatchEvent(event) {
      return (listeners.get(event.type) || []).map((handler) => handler(event));
    },
    click(target = this) {
      return this.dispatchEvent({ type: "click", target, currentTarget: this });
    },
    setAttribute(name, value) { attributes[name] = String(value); },
    getAttribute(name) { return attributes[name]; },
    classList: {
      add(...names) { names.forEach((name) => classes.add(name)); },
      remove(...names) { names.forEach((name) => classes.delete(name)); },
      toggle(name, force) {
        const on = force === undefined ? !classes.has(name) : Boolean(force);
        if (on) classes.add(name); else classes.delete(name);
        return on;
      },
      contains(name) { return classes.has(name); },
    },
  };
  Object.defineProperty(node, "className", {
    get() { return Array.from(classes).join(" "); },
    set(value) {
      classes.clear();
      String(value).split(/\s+/).filter(Boolean).forEach((name) => classes.add(name));
    },
  });
  return node;
}
const loggingButtons = [
  makeNode("navigationTopicLoggingToggle"),
  makeNode("mappingTopicLoggingToggle"),
];
loggingButtons.forEach((button) => {
  button.dataset.topicLoggingToggle = "";
  button.classList.add("start-log", "topic-logging-toggle");
  elements.set(button.id, button);
});
const loggingStates = [
  makeNode("navigationTopicLoggingState"),
  makeNode("mappingTopicLoggingState"),
];
loggingStates.forEach((state) => {
  state.dataset.topicLoggingState = "";
  elements.set(state.id, state);
});
function el(id) {
  if (!elements.has(id)) elements.set(id, makeNode(id));
  return elements.get(id);
}
const document = {
  getElementById: el,
  createElement(tag) { const child = makeNode(""); child.tag = tag; return child; },
  querySelectorAll(selector) {
    if (selector === "[data-topic-logging-toggle]") return loggingButtons;
    if (selector === "[data-topic-logging-state]") return loggingStates;
    return [];
  },
};
let postJsonImpl = async () => { throw new Error("unexpected POST"); };
let getJsonImpl = async () => { throw new Error("unexpected GET"); };
const postCalls = [];
async function postJson(url, payload) {
  postCalls.push({ url, payload });
  return postJsonImpl(url, payload);
}
async function getJson(url) { return getJsonImpl(url); }
const diagnosticsData = (enabled, extra = {}) => ({
  ok: true, topic_logging_enabled: enabled, logging: {}, topics: [], motion: null, ...extra,
});
function snapshot() {
  return {
    labels: loggingButtons.map((button) => button.textContent),
    disabled: loggingButtons.map((button) => button.disabled),
    pressed: loggingButtons.map((button) => button.getAttribute("aria-pressed")),
    busy: loggingButtons.map((button) => button.getAttribute("aria-busy")),
    start: loggingButtons.map((button) => button.classList.contains("start-log")),
    stop: loggingButtons.map((button) => button.classList.contains("stop-log")),
    states: loggingStates.map((state) => state.textContent),
    stateHidden: loggingStates.map((state) => state.hidden),
    stateClasses: loggingStates.map((state) => state.className),
  };
}
const out = {};
renderTopicLogging();
out.initial = snapshot();
updateDiagnostics(diagnosticsData(true));
out.enabled = snapshot();

attachTopicLoggingEvents();
let resolvePost;
postJsonImpl = () => new Promise((resolve) => { resolvePost = resolve; });
const pending = loggingButtons[0].click()[0];
out.pending = snapshot();
out.pendingPayload = postCalls[postCalls.length - 1].payload;
// A poll racing the pending POST must not change either visible button.
updateDiagnostics(diagnosticsData(true));
out.pendingPoll = snapshot();
const callsWhilePending = postCalls.length;
loggingButtons[1].click({ tagName: "SPAN" });
out.pendingSecondClickCalls = postCalls.length - callsWhilePending;
resolvePost({ ok: true, topic_logging_enabled: false, logging: {} });
await pending;
out.afterSuccess = snapshot();

postJsonImpl = async () => { throw new Error("disk full"); };
getJsonImpl = async () => diagnosticsData(true);
await loggingButtons[1].click()[0];
out.failure = snapshot();
out.failurePayload = postCalls[postCalls.length - 1].payload;
// Polls must not erase the actionable failure message.
updateDiagnostics(diagnosticsData(true));
out.errorPersistsAfterPoll = snapshot().states;

// POST fails and the resync GET also fails: no false claim of a backend reread.
postJsonImpl = async () => { throw new Error("network down"); };
getJsonImpl = async () => { throw new Error("network down"); };
await loggingButtons[0].click()[0];
out.outage = snapshot();
updateDiagnostics(diagnosticsData(true));
out.outageErrorPersistsAfterPoll = snapshot().states;

postJsonImpl = async () => ({ ok: true, topic_logging_enabled: false, logging: {} });
await loggingButtons[0].click({ tagName: "SPAN" })[0];
out.recovered = snapshot();

// A GET that started before a successful POST is ignored when it settles.
updateDiagnostics(diagnosticsData(true));
let resolveGet;
getJsonImpl = () => new Promise((resolve) => { resolveGet = resolve; });
const oldPoll = pollDiagnostics();
await Promise.resolve();
postJsonImpl = async () => ({ ok: true, topic_logging_enabled: false, logging: {} });
await loggingButtons[1].click({ tagName: "SPAN" })[0];
resolveGet(diagnosticsData(true));
await oldPoll;
out.stalePoll = snapshot();

// An old backend without the boolean is handled without throwing or flipping.
postJsonImpl = async () => { throw new Error("unknown endpoint"); };
getJsonImpl = async () => ({ ok: true, logging: {}, topics: [], motion: null });
await loggingButtons[0].click()[0];
out.legacy = snapshot();
console.log(JSON.stringify(out));
'''
    result = subprocess.run(
        [node, "--input-type=module", "-e", block + attach_block + harness],
        check=True,
        capture_output=True,
        text=True,
        timeout=20,
    )
    payload = json.loads(result.stdout)

    assert payload["initial"]["disabled"] == [True, True]
    assert payload["initial"]["labels"] == ["开始记录话题", "开始记录话题"]
    assert payload["initial"]["pressed"] == ["false", "false"]
    assert payload["initial"]["start"] == [True, True]
    assert payload["initial"]["states"] == ["", ""]
    assert payload["initial"]["stateHidden"] == [True, True]

    assert payload["enabled"]["labels"] == ["停止记录话题", "停止记录话题"]
    assert payload["enabled"]["disabled"] == [False, False]
    assert payload["enabled"]["pressed"] == ["true", "true"]
    assert payload["enabled"]["start"] == [False, False]
    assert payload["enabled"]["stop"] == [True, True]
    assert payload["enabled"]["states"] == ["", ""]
    assert payload["enabled"]["stateHidden"] == [True, True]

    assert payload["pendingPayload"] == {"enabled": False}
    assert payload["pending"]["labels"] == ["停止记录话题", "停止记录话题"]
    assert payload["pending"]["disabled"] == [True, True]
    assert payload["pending"]["pressed"] == ["true", "true"]
    assert payload["pending"]["busy"] == ["true", "true"]
    assert payload["pending"]["states"] == ["正在保存...", "正在保存..."]
    assert payload["pending"]["stateHidden"] == [False, False]
    assert payload["pendingPoll"]["disabled"] == [True, True]
    assert payload["pendingPoll"]["pressed"] == ["true", "true"]
    assert payload["pendingSecondClickCalls"] == 0

    assert payload["afterSuccess"]["labels"] == ["开始记录话题", "开始记录话题"]
    assert payload["afterSuccess"]["disabled"] == [False, False]
    assert payload["afterSuccess"]["pressed"] == ["false", "false"]
    assert payload["afterSuccess"]["start"] == [True, True]
    assert payload["afterSuccess"]["stop"] == [False, False]
    assert payload["afterSuccess"]["states"] == ["", ""]
    assert payload["afterSuccess"]["stateHidden"] == [True, True]

    assert payload["failurePayload"] == {"enabled": True}
    assert payload["failure"]["labels"] == ["停止记录话题", "停止记录话题"]
    assert payload["failure"]["pressed"] == ["true", "true"]
    assert "保存话题日志开关失败" in payload["failure"]["states"][0]
    assert "已重新读取后台状态" in payload["failure"]["states"][0]
    assert payload["failure"]["stateHidden"] == [False, False]
    assert "is-error" in payload["failure"]["stateClasses"][0]
    assert "is-error" in payload["failure"]["stateClasses"][1]
    assert all("失败" in state for state in payload["errorPersistsAfterPoll"])

    assert payload["outage"]["labels"] == ["停止记录话题", "停止记录话题"]
    assert payload["outage"]["pressed"] == ["true", "true"]
    assert "未能确认后台状态，暂显示上次状态，请检查连接后重试" in payload["outage"]["states"][0]
    assert "已重新读取后台状态" not in payload["outage"]["states"][0]
    assert payload["outage"]["stateHidden"] == [False, False]
    assert all("失败" in state for state in payload["outageErrorPersistsAfterPoll"])

    assert payload["recovered"]["labels"] == ["开始记录话题", "开始记录话题"]
    assert payload["recovered"]["pressed"] == ["false", "false"]
    assert payload["recovered"]["states"] == ["", ""]
    assert payload["recovered"]["stateHidden"] == [True, True]

    assert payload["stalePoll"]["labels"] == ["开始记录话题", "开始记录话题"]
    assert payload["stalePoll"]["pressed"] == ["false", "false"]
    assert payload["stalePoll"]["states"] == ["", ""]
    assert payload["stalePoll"]["stateHidden"] == [True, True]

    assert payload["legacy"]["labels"] == ["开始记录话题", "开始记录话题"]
    assert payload["legacy"]["pressed"] == ["false", "false"]
    assert "保存话题日志开关失败" in payload["legacy"]["states"][0]
    assert "未能确认后台状态，暂显示上次状态，请检查连接后重试" in payload["legacy"]["states"][0]
    assert "已重新读取后台状态" not in payload["legacy"]["states"][0]
    assert payload["legacy"]["stateHidden"] == [False, False]

    # Every rendered state is identical on both pages and has unique elements.
    for key in ("initial", "enabled", "pending", "pendingPoll", "afterSuccess",
                "failure", "outage", "recovered", "stalePoll", "legacy"):
        assert payload[key]["labels"][0] == payload[key]["labels"][1]
        assert payload[key]["states"][0] == payload[key]["states"][1]
        assert payload[key]["pressed"][0] == payload[key]["pressed"][1]
