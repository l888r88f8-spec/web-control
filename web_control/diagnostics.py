"""ROS-free topic-rate, wheel-odometry motion, and diagnostics logging core.

The module deliberately never imports rclpy so the monitoring policy can be
unit-tested with plain clocks and message-shaped objects.  Distance is derived
only from measured chassis wheel odometry; IMU observations are auxiliary and
never prove motion.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Callable, Iterable, Optional

from web_control.log_store import LogStorageError, ManagedLogStore

DIAGNOSTICS_TARGET = "diagnostics"
DEFAULT_WINDOW_SEC = 5.0
DEFAULT_STALE_TIMEOUT_SEC = 2.0
DEFAULT_TOPIC_SPECS: tuple[dict[str, str], ...] = (
    {
        "key": "imu",
        "label": "IMU",
        "topic": "/imu",
        "message_type": "sensor_msgs/msg/Imu",
    },
    {
        "key": "lidar",
        "label": "Raw LiDAR",
        "topic": "/lidar_points",
        "message_type": "sensor_msgs/msg/PointCloud2",
    },
    {
        "key": "odom",
        "label": "Chassis Odom",
        "topic": "/odom1",
        "message_type": "nav_msgs/msg/Odometry",
    },
)


# Every persisted topic-metric/IMU key.  OFF removes these at the write
# boundary, including the enrichment copied onto motion transition events.
TOPIC_LOG_KEYS = frozenset(
    {
        "topics",
        "imu_observation",
        "topic_hz",
        "topic_age_sec",
        "publisher_count",
        "publisher_checked_age_sec",
        "topic_status",
        "message_type",
        "hz",
        "last_age_sec",
        # Motion events copy the topic status as a bare ``status`` key.
        "status",
        "window_sec",
        "window_count",
        "received_count",
        "last_received_stamp",
    }
)


def filter_topic_logging_entry(
    entry: dict[str, Any], enabled: bool
) -> Optional[dict[str, Any]]:
    """Return the entry to persist, or ``None`` when it is topic-only.

    This is the single formatter at the JSONL write boundary.  Disabling topic
    logging drops whole topic-status rows, topic snapshots, IMU observations,
    and topic metadata copied onto motion events, while preserving motion
    state, speed, source, distance, duration, validity, and audit fields.
    """
    if enabled:
        return entry
    if entry.get("kind") == "topic_status":
        return None
    return {key: value for key, value in entry.items() if key not in TOPIC_LOG_KEYS}


def _finite(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _iso_time(stamp: float) -> str:
    return datetime.fromtimestamp(stamp, tz=timezone.utc).isoformat(timespec="milliseconds")


def _msg_stamp(msg: Any) -> Optional[float]:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return None
    sec = _finite(getattr(stamp, "sec", None))
    nanosec = _finite(getattr(stamp, "nanosec", None))
    if sec is None or nanosec is None:
        return None
    return sec + nanosec * 1e-9


def _quaternion_yaw(msg: Any) -> Optional[float]:
    pose = getattr(getattr(msg, "pose", None), "pose", None)
    orientation = getattr(pose, "orientation", None)
    if orientation is None:
        return None
    x = _finite(getattr(orientation, "x", None))
    y = _finite(getattr(orientation, "y", None))
    z = _finite(getattr(orientation, "z", None))
    w = _finite(getattr(orientation, "w", None))
    if x is None or y is None or z is None or w is None:
        return None
    norm = x * x + y * y + z * z + w * w
    if norm <= 0.0:
        return None
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _linear_speed(msg: Any) -> Optional[float]:
    # nav_msgs/msg/Odometry.twist is a geometry_msgs/TwistWithCovariance,
    # whose actual Twist lives at msg.twist.twist (linear.x is nested twice).
    covariance = getattr(msg, "twist", None)
    twist = getattr(covariance, "twist", None)
    if twist is None:
        return None
    linear = getattr(twist, "linear", None)
    return _finite(getattr(linear, "x", None))


def _frame_pair(msg: Any) -> tuple[str, str]:
    header = getattr(msg, "header", None)
    return (
        str(getattr(header, "frame_id", "") or ""),
        str(getattr(msg, "child_frame_id", "") or ""),
    )


class TopicRateMonitor:
    """Bounded sliding-window receiver for one configured topic."""

    def __init__(
        self,
        key: str,
        label: str,
        topic: str,
        message_type: str,
        *,
        window_sec: float = DEFAULT_WINDOW_SEC,
        stale_timeout_sec: float = DEFAULT_STALE_TIMEOUT_SEC,
        enabled: bool = True,
        available: bool = True,
    ) -> None:
        self.key = str(key)
        self.label = str(label)
        self.topic = str(topic)
        self.message_type = str(message_type)
        self.window_sec = max(0.1, float(window_sec))
        self.stale_timeout_sec = max(0.1, float(stale_timeout_sec))
        self.enabled = bool(enabled)
        self.available = bool(available)
        self._received: deque[float] = deque(maxlen=8192)
        self._total = 0
        self._last_received: Optional[float] = None
        self._publisher_count: Optional[int] = None
        self._publisher_checked: Optional[float] = None
        self._last_error: Optional[str] = None
        self._lock = threading.RLock()

    def record(self, now_mono: float) -> None:
        now = _finite(now_mono)
        if now is None:
            return
        with self._lock:
            if self._last_received is not None and now < self._last_received:
                # A caller only uses one monotonic source; keep the counter
                # ordered if a test or clock reset hands us a smaller value.
                now = self._last_received
            self._received.append(now)
            self._last_received = now
            self._total = min(self._total + 1, 2**63 - 1)
            self._last_error = None

    def note_error(self, message: str) -> None:
        with self._lock:
            self._last_error = str(message)

    def set_publisher_count(self, count: Any, now_mono: Optional[float] = None) -> None:
        try:
            value = max(0, int(count))
        except (TypeError, ValueError):
            value = None
        with self._lock:
            self._publisher_count = value
            if now_mono is not None:
                self._publisher_checked = _finite(now_mono)

    def _trim(self, now: float) -> None:
        cutoff = now - self.window_sec
        while self._received and self._received[0] < cutoff:
            self._received.popleft()

    def snapshot(self, now_mono: float) -> dict[str, Any]:
        now = _finite(now_mono) or 0.0
        with self._lock:
            self._trim(now)
            age = None if self._last_received is None else max(0.0, now - self._last_received)
            if len(self._received) >= 2:
                span = self._received[-1] - self._received[0]
                hz = (len(self._received) - 1) / span if span > 0.0 else 0.0
            else:
                hz = 0.0
            status = self._status_locked(age)
            return {
                "key": self.key,
                "label": self.label,
                "topic": self.topic,
                "message_type": self.message_type,
                "enabled": self.enabled,
                "available": self.available,
                "status": status,
                "publisher_count": self._publisher_count,
                "publisher_checked_age_sec": (
                    None
                    if self._publisher_checked is None
                    else max(0.0, now - self._publisher_checked)
                ),
                "hz": round(float(hz), 3) if math.isfinite(hz) else 0.0,
                "window_sec": self.window_sec,
                "received_count": self._total,
                "window_count": len(self._received),
                "last_age_sec": None if age is None else round(age, 3),
                "last_received_stamp": self._last_received,
                "error": self._last_error,
            }

    def _status_locked(self, age: Optional[float]) -> str:
        if not self.enabled or not self.available:
            return "unavailable"
        if self._publisher_count == 0:
            return "no_publisher"
        if self._total <= 0:
            return "waiting"
        if age is not None and age > self.stale_timeout_sec:
            return "stale"
        return "active"

    def status(self, now_mono: float) -> str:
        with self._lock:
            age = None if self._last_received is None else max(0.0, now_mono - self._last_received)
            return self._status_locked(age)


class MotionTracker:
    """Wheel-odometry distance with hysteresis, gaps, and explicit validity.

    ``total_distance_m`` starts at ``None`` and only becomes a number after a
    valid odometry sample.  An interruption never invents distance: the gap is
    not bridged, the integration baseline is dropped, and the known total is
    retained with ``paused``/``incomplete`` flags.
    """

    def __init__(
        self,
        *,
        source_topic: str = "/odom1",
        start_threshold_mps: float = 0.05,
        stop_threshold_mps: float = 0.02,
        debounce_sec: float = 0.3,
        max_gap_sec: float = 0.5,
        max_speed_mps: float = 3.0,
        stale_timeout_sec: float = DEFAULT_STALE_TIMEOUT_SEC,
    ) -> None:
        self.source_topic = str(source_topic)
        self.start_threshold = max(0.0, float(start_threshold_mps))
        self.stop_threshold = max(0.0, float(stop_threshold_mps))
        if self.stop_threshold > self.start_threshold:
            self.stop_threshold = self.start_threshold
        self.debounce_sec = max(0.0, float(debounce_sec))
        self.max_gap_sec = max(0.001, float(max_gap_sec))
        self.max_speed_mps = max(0.001, float(max_speed_mps))
        self.stale_timeout_sec = max(0.1, float(stale_timeout_sec))
        self._lock = threading.RLock()
        self._state = "unknown"
        self._total: Optional[float] = None
        self._segment_distance = 0.0
        self._segment_start: Optional[float] = None
        self._last_segment_distance = 0.0
        self._last_segment_duration = 0.0
        self._last_stamp: Optional[float] = None
        self._last_speed: Optional[float] = None
        self._last_arrival: Optional[float] = None
        self._last_wall: Optional[float] = None
        self._last_yaw: Optional[float] = None
        self._last_speed_signed: Optional[float] = None
        self._last_sample_stamp: Optional[float] = None
        self._frame: Optional[tuple[str, str]] = None
        self._incomplete = False
        self._paused = False
        self._reason = "no_odom"
        self._candidate_since: Optional[float] = None
        self._stale_interrupted = False
        self._invalid_samples = 0
        self._events: deque[dict[str, Any]] = deque(maxlen=512)

    # -- ingress -----------------------------------------------------------
    def update(
        self,
        speed: Any,
        yaw: Any,
        stamp: Any,
        frame_id: Any,
        child_frame_id: Any,
        now_mono: float,
        wall_now: Optional[float] = None,
    ) -> None:
        now = _finite(now_mono)
        if now is None:
            return
        wall = _finite(wall_now)
        if wall is None:
            wall = time.time()
        numeric_speed = _finite(speed)
        numeric_stamp = _finite(stamp)
        frames = (str(frame_id or ""), str(child_frame_id or ""))
        with self._lock:
            if numeric_speed is None or numeric_stamp is None:
                self._invalid_samples += 1
                return
            stale_before = (
                self._last_arrival is not None
                and now - self._last_arrival > self.stale_timeout_sec
            )
            self._last_arrival = now
            if stale_before and self._state == "moving":
                self._interrupt("odom_stale", wall, numeric_stamp)
            if self._frame is not None and frames != self._frame:
                self._interrupt("frame_changed", wall, numeric_stamp)
            self._frame = frames
            if abs(numeric_speed) > self.max_speed_mps:
                self._interrupt("implausible_speed", wall, numeric_stamp)
                self._invalid_samples += 1
                return
            if self._last_stamp is not None and numeric_stamp <= self._last_stamp:
                self._interrupt("timestamp_reset", wall, numeric_stamp)
            if self._last_stamp is not None and numeric_stamp - self._last_stamp > self.max_gap_sec:
                self._interrupt("timestamp_gap", wall, numeric_stamp)
            dt = None
            if self._last_stamp is not None:
                dt = numeric_stamp - self._last_stamp
                if dt <= 0.0 or dt > self.max_gap_sec:
                    dt = None
            if self._total is None:
                self._total = 0.0
                self._incomplete = False
                self._reason = None
            moving_engaged = self._state == "moving" or abs(numeric_speed) >= self.start_threshold
            if dt is not None and self._last_speed is not None and moving_engaged:
                previous_speed = self._last_speed
                if previous_speed * numeric_speed >= 0.0:
                    # Same direction (including a zero endpoint): trapezoid of
                    # absolute speed.
                    delta = dt * (abs(previous_speed) + abs(numeric_speed)) * 0.5
                else:
                    # Reversal interval.  With linear speed variation the path
                    # is the integral of |v|, not the signed average (which
                    # would cancel equal forward/reverse motion to zero).
                    denominator = abs(previous_speed) + abs(numeric_speed)
                    delta = (
                        dt
                        * (previous_speed * previous_speed + numeric_speed * numeric_speed)
                        / (2.0 * denominator)
                    )
                if math.isfinite(delta) and delta >= 0.0:
                    self._total += delta
                    if self._segment_start is None:
                        self._segment_start = numeric_stamp - dt
                    self._segment_distance += delta
            self._last_stamp = numeric_stamp
            self._last_speed = numeric_speed
            self._last_speed_signed = numeric_speed
            self._last_sample_stamp = numeric_stamp
            self._last_wall = wall
            self._last_yaw = _finite(yaw)
            self._paused = False
            self._stale_interrupted = False
            self._update_motion_state(numeric_speed, numeric_stamp, wall)

    def _update_motion_state(self, speed: float, stamp: float, wall: float) -> None:
        magnitude = abs(speed)
        if self._state == "moving":
            if magnitude <= self.stop_threshold:
                if self._candidate_since is None:
                    self._candidate_since = stamp
                if stamp - self._candidate_since >= self.debounce_sec - 1e-9:
                    segment_start = (
                        self._segment_start
                        if self._segment_start is not None
                        else stamp
                    )
                    segment_duration = max(0.0, stamp - segment_start)
                    self._last_segment_distance = self._segment_distance
                    self._last_segment_duration = segment_duration
                    self._emit(
                        "motion_stop",
                        wall,
                        stamp,
                        state="stationary",
                        segment_distance=self._segment_distance,
                        segment_duration=segment_duration,
                    )
                    self._state = "stationary"
                    self._segment_distance = 0.0
                    self._segment_start = None
                    self._candidate_since = None
                    self._reset_baseline()
            else:
                self._candidate_since = None
            return
        if magnitude >= self.start_threshold:
            if self._candidate_since is None:
                self._candidate_since = stamp
            if stamp - self._candidate_since >= self.debounce_sec - 1e-9:
                self._state = "moving"
                if self._segment_start is None:
                    self._segment_start = stamp
                self._emit(
                    "motion_start",
                    wall,
                    stamp,
                    state="moving",
                    segment_distance=self._segment_distance,
                    segment_duration=max(0.0, stamp - self._segment_start),
                )
                self._candidate_since = None
            return
        self._candidate_since = None
        if magnitude <= self.stop_threshold:
            self._state = "stationary"

    def _reset_baseline(self) -> None:
        self._last_stamp = None
        self._last_speed = None

    def _interrupt(self, reason: str, wall: float, stamp: float) -> None:
        if self._state == "moving":
            last_sample = (
                self._last_sample_stamp
                if self._last_sample_stamp is not None
                else stamp
            )
            segment_start = (
                self._segment_start if self._segment_start is not None else stamp
            )
            segment_duration = max(0.0, last_sample - segment_start)
            self._last_segment_distance = self._segment_distance
            self._last_segment_duration = segment_duration
            self._emit(
                "motion_interruption",
                wall,
                stamp,
                state="unknown",
                reason=reason,
                segment_distance=self._segment_distance,
                segment_duration=segment_duration,
            )
        self._state = "unknown"
        self._paused = True
        self._incomplete = True
        self._reason = reason
        self._segment_distance = 0.0
        self._segment_start = None
        self._candidate_since = None
        self._reset_baseline()

    def _emit(
        self,
        event: str,
        wall: float,
        stamp: float,
        *,
        state: str,
        reason: Optional[str] = None,
        segment_distance: float,
        segment_duration: float,
    ) -> None:
        self._events.append(
            {
                "kind": "motion",
                "event": event,
                "at": wall,
                "at_iso": _iso_time(wall),
                "source": self.source_topic,
                "topic": self.source_topic,
                "sample_stamp": stamp,
                "state": state,
                "motion_state": state,
                "reason": reason,
                "speed_mps": self._last_speed_signed,
                "direction": self._direction_locked(),
                "yaw_rad": self._last_yaw,
                "segment_distance_m": round(float(segment_distance), 6),
                "segment_duration_sec": round(float(segment_duration), 3),
                "total_distance_m": self._total,
                "distance_source": "wheel_odometry",
                "distance_validity": self._validity_locked(),
            }
        )

    def _direction_locked(self) -> Optional[str]:
        speed = self._last_speed_signed
        if speed is None or abs(speed) <= self.stop_threshold:
            return None
        return "reverse" if speed < 0 else "forward"

    def _validity_locked(self) -> dict[str, Any]:
        return {
            "available": self._total is not None,
            "valid": self._total is not None and not self._paused,
            "paused": self._paused,
            "incomplete": self._incomplete,
            "reason": self._reason,
            "invalid_samples": self._invalid_samples,
        }

    def snapshot(self, now_mono: float) -> dict[str, Any]:
        now = _finite(now_mono) or 0.0
        with self._lock:
            age = None if self._last_arrival is None else max(0.0, now - self._last_arrival)
            stale = age is not None and age > self.stale_timeout_sec
            if stale and self._state == "moving":
                stale_wall = self._last_wall if self._last_wall is not None else time.time()
                stale_stamp = (
                    self._last_sample_stamp
                    if self._last_sample_stamp is not None
                    else 0.0
                )
                self._interrupt("odom_stale", stale_wall, stale_stamp)
            if stale:
                paused = True
                state = "unknown"
                reason = "odom_stale"
            else:
                paused = self._paused
                state = self._state
                reason = self._reason
            current_segment = self._segment_distance if state == "moving" else self._last_segment_distance
            if state == "moving" and self._segment_start is not None:
                last_sample = (
                    self._last_sample_stamp
                    if self._last_sample_stamp is not None
                    else self._segment_start
                )
                current_duration = max(0.0, last_sample - self._segment_start)
            else:
                current_duration = self._last_segment_duration
            if stale:
                self._incomplete = True
                if self._reason is None:
                    self._reason = "odom_stale"
            validity = self._validity_locked()
            if stale:
                validity["valid"] = False
                validity["paused"] = True
                validity["reason"] = "odom_stale"
            direction = self._direction_locked()
            return {
                "source": "wheel_odometry",
                "source_topic": self.source_topic,
                "topic": self.source_topic,
                "state": state,
                "motion": state,
                "speed_mps": self._last_speed_signed,
                "yaw_rad": self._last_yaw,
                "direction": direction,
                "segment_distance_m": round(float(current_segment), 6),
                "segment_duration_sec": round(float(current_duration), 3),
                "total_distance_m": None if self._total is None else round(float(self._total), 6),
                "distance_estimate": "wheel_estimated",
                "distance_label": "wheel estimate (current web session)",
                "validity": validity,
                "received": self._last_arrival is not None,
                "last_age_sec": None if age is None else round(age, 3),
                "stale": stale,
                "paused": paused,
                "incomplete": self._incomplete,
                "reason": reason,
                "last_sample_stamp": self._last_sample_stamp,
            }

    def note_source_lifecycle(
        self, action: str, now_mono: float, wall_now: Optional[float] = None
    ) -> None:
        """Reset integration at a managed chassis start/stop boundary.

        The cumulative session total is retained; only the current integration
        baseline/segment is dropped so no distance can be bridged across a
        restart.  A lifecycle event is always recorded, even when idle.
        """
        action = str(action or "unknown")
        now = _finite(now_mono) or time.monotonic()
        wall = _finite(wall_now)
        if wall is None:
            wall = time.time()
        with self._lock:
            reason = f"chassis_{action}"
            sample_stamp = (
                self._last_sample_stamp
                if self._last_sample_stamp is not None
                else 0.0
            )
            if self._state == "moving":
                self._interrupt(reason, wall, sample_stamp)
            else:
                self._emit(
                    "motion_lifecycle",
                    wall,
                    sample_stamp,
                    state="unknown",
                    reason=reason,
                    segment_distance=self._segment_distance,
                    segment_duration=self._last_segment_duration,
                )
                self._state = "unknown"
                self._paused = True
                if action == "stop" and self._total is not None:
                    self._incomplete = True
                self._reason = reason
                self._segment_distance = 0.0
                self._segment_start = None
                self._candidate_since = None
                self._reset_baseline()
            self._last_arrival = None

    def take_events(self) -> list[dict[str, Any]]:
        with self._lock:
            events = list(self._events)
            self._events.clear()
            return events


@dataclass
class DiagnosticsService:
    """Coordinates topic windows, motion inference, and transition events."""

    monitors: dict[str, TopicRateMonitor]
    motion: MotionTracker
    imu_observation: Optional[dict[str, Any]] = None
    _previous_status: dict[str, str] = field(default_factory=dict)
    _events: deque[dict[str, Any]] = field(default_factory=deque)
    _lock: threading.RLock = field(default_factory=threading.RLock)

    def __post_init__(self) -> None:
        for monitor in self.monitors.values():
            self._previous_status[monitor.key] = "unavailable"

    def record(self, key: str, now_mono: float) -> None:
        monitor = self.monitors.get(key)
        if monitor is not None:
            monitor.record(now_mono)

    def record_imu(self, msg: Any, now_mono: float) -> None:
        monitor = self.monitors.get("imu")
        if monitor is None:
            return
        monitor.record(now_mono)
        linear = getattr(msg, "linear_acceleration", None)
        angular = getattr(msg, "angular_velocity", None)
        observation = {
            "at": time.time(),
            "at_iso": _iso_time(time.time()),
            "acceleration_mps2": {
                axis: _finite(getattr(linear, axis, None))
                for axis in ("x", "y", "z")
            },
            "angular_velocity_rps": {
                axis: _finite(getattr(angular, axis, None))
                for axis in ("x", "y", "z")
            },
            "motion_proof": False,
            "note": "auxiliary observation; IMU alone is not motion proof",
        }
        with self._lock:
            self.imu_observation = observation

    def record_odom(self, msg: Any, now_mono: float) -> None:
        self.record("odom", now_mono)
        frame_id, child_frame_id = _frame_pair(msg)
        self.motion.update(
            _linear_speed(msg),
            _quaternion_yaw(msg),
            _msg_stamp(msg),
            frame_id,
            child_frame_id,
            now_mono,
        )

    def update_publisher_counts(self, counts: dict[str, Any], now_mono: float) -> None:
        for key, count in counts.items():
            monitor = self.monitors.get(key)
            if monitor is not None:
                monitor.set_publisher_count(count, now_mono)

    def snapshot(self, now_mono: float) -> dict[str, Any]:
        with self._lock:
            topics = [
                monitor.snapshot(now_mono) for monitor in self.monitors.values()
            ]
            observation = self.imu_observation
            if observation is not None and "imu" in self.monitors:
                imu = self.monitors["imu"].snapshot(now_mono)
                observation = {
                    **observation,
                    "status": imu["status"],
                    "last_age_sec": imu["last_age_sec"],
                }
        return {
            "ok": True,
            "source_mode": "wheel_odometry_primary_imu_auxiliary",
            "topics": topics,
            "imu_observation": observation,
            "motion": self.motion.snapshot(now_mono),
            "stamp": time.time(),
        }

    def tick(
        self, now_mono: float, publisher_counts: Optional[dict[str, Any]] = None
    ) -> tuple[dict[str, Any], list[dict[str, Any]]]:
        if publisher_counts:
            self.update_publisher_counts(publisher_counts, now_mono)
        payload = self.snapshot(now_mono)
        events: list[dict[str, Any]] = []
        with self._lock:
            for row in payload["topics"]:
                key = row["key"]
                previous = self._previous_status.get(key)
                if previous != row["status"]:
                    self._previous_status[key] = row["status"]
                    events.append(
                        {
                            "kind": "topic_status",
                            "event": "status_change",
                            "at": time.time(),
                            "at_iso": _iso_time(time.time()),
                            "source": row["topic"],
                            "topic": row["topic"],
                            "previous_status": previous,
                            "status": row["status"],
                            "publisher_count": row["publisher_count"],
                            "hz": row["hz"],
                            "last_age_sec": row["last_age_sec"],
                            "message_type": row["message_type"],
                        }
                    )
            events.extend(self.motion.take_events())
        return payload, events


class DiagnosticsLogWriter:
    """JSONL diagnostics writer reusing :class:`ManagedLogStore`.

    Files live in ``<managed_log_dir>/diagnostics`` and rotate at local day
    boundaries or the configured size.  Pruning only considers completed files
    inside that diagnostics target; hardware process logs are never scanned.
    """

    def __init__(
        self,
        store: ManagedLogStore,
        *,
        session_id: Optional[str] = None,
        max_bytes: int = 10 * 1024 * 1024,
        retain_days: int = 7,
        retain_bytes: int = 100 * 1024 * 1024,
        now: Optional[float] = None,
    ) -> None:
        self.store = store
        self.session_id = session_id or uuid.uuid4().hex
        self.max_bytes = max(1024, int(max_bytes))
        self.retain_days = max(1, int(retain_days))
        self.retain_bytes = max(1024, int(retain_bytes))
        self._lock = threading.RLock()
        self._session: Any = None
        self._day: Optional[str] = None
        self.error: Optional[str] = None
        self.last_entry_at: Optional[float] = None
        self.rotations = 0
        self._last_prune = 0.0
        try:
            self._open(now)
            self.prune(now)
        except LogStorageError as exc:
            self.error = str(exc)

    @staticmethod
    def _session_failure(session: Any) -> Optional[str]:
        """Return why a returned session cannot be written, if it cannot."""
        if session is None:
            return "log session unavailable"
        error = getattr(session, "write_error", None)
        if error:
            return str(error)
        if getattr(session, "closed", False):
            return "log session closed before write"
        return None

    def _open(self, now: Optional[float]) -> None:
        current = time.time() if now is None else float(now)
        self._day = time.strftime("%Y-%m-%d", time.localtime(current))
        session = None
        try:
            session = self.store.start_session(
                DIAGNOSTICS_TARGET, ["diagnostics", "jsonl", f"node={self.session_id}"]
            )
            failure = self._session_failure(session)
            if failure:
                raise LogStorageError(failure)
            # The managed header is text; every following line is JSON.
            session.write_line(
                json.dumps(
                    {
                        "kind": "log_start",
                        "session_id": self.session_id,
                        "at": current,
                        "at_iso": _iso_time(current),
                    },
                    separators=(",", ":"),
                )
            )
            failure = self._session_failure(session)
            if failure:
                raise LogStorageError(failure)
        except Exception:
            if session is not None:
                try:
                    self.store.end_session(session)
                except Exception:
                    pass
            raise
        self._session = session

    def _rotate(self, now: float) -> None:
        previous = self._session
        if previous is not None:
            self.store.end_session(previous)
        self._session = None
        self._open(now)
        self.rotations += 1
        self.prune(now)

    def write(self, entry: dict[str, Any], now: Optional[float] = None) -> bool:
        current = time.time() if now is None else float(now)
        record = dict(entry)
        record.setdefault("at", current)
        record.setdefault("at_iso", _iso_time(current))
        record.setdefault("session_id", self.session_id)
        record.setdefault("source", "diagnostics")
        with self._lock:
            session = self._session
            try:
                if session is None:
                    self._open(current)
                    session = self._session
                failure = self._session_failure(session)
                if failure:
                    raise LogStorageError(failure)
                day = time.strftime("%Y-%m-%d", time.localtime(current))
                if day != self._day or session.bytes_written >= self.max_bytes:
                    self._rotate(current)
                    session = self._session
                failure = self._session_failure(session)
                if failure:
                    raise LogStorageError(failure)
                session.write_line(json.dumps(record, separators=(",", ":"), allow_nan=False))
                failure = self._session_failure(session)
                if failure:
                    raise LogStorageError(failure)
                self.last_entry_at = current
                self.error = None
                return True
            except (LogStorageError, ValueError, TypeError) as exc:
                self.error = str(exc)
                try:
                    if self._session is not None:
                        self.store.end_session(self._session)
                except Exception:
                    pass
                self._session = None
                return False

    def prune(self, now: Optional[float] = None) -> list[str]:
        """Delete only completed diagnostics logs outside retention limits."""
        current = time.time() if now is None else float(now)
        cutoff = current - self.retain_days * 86400.0
        removed: list[str] = []
        with self._lock:
            try:
                rows = self.store.list_sessions(DIAGNOSTICS_TARGET)
            except (LogStorageError, ValueError, OSError):
                return removed
            active = {
                row["session"] for row in rows if row.get("active")
            }
            completed = [row for row in rows if not row.get("active")]
            # Age first, oldest first.
            for row in sorted(completed, key=lambda item: item.get("started_at", 0.0)):
                if row.get("started_at", 0.0) >= cutoff:
                    continue
                try:
                    self._delete_session(row["session"], active)
                except (OSError, ValueError, LogStorageError):
                    continue
                removed.append(row["session"])
            remaining = [
                row for row in completed if row["session"] not in removed
            ]
            total = sum(int(row.get("size", 0)) for row in remaining)
            for row in sorted(remaining, key=lambda item: item.get("started_at", 0.0)):
                if total <= self.retain_bytes:
                    break
                try:
                    self._delete_session(row["session"], active)
                except (OSError, ValueError, LogStorageError):
                    continue
                removed.append(row["session"])
                total -= int(row.get("size", 0))
        return removed

    def _delete_session(self, session_id: str, active: set[str]) -> None:
        if session_id in active:
            return
        path = self.store.resolve_session_file(DIAGNOSTICS_TARGET, session_id)
        target_dir = (self.store.root / DIAGNOSTICS_TARGET).resolve(strict=False)
        if path.parent.resolve(strict=False) != target_dir:
            return
        os.unlink(path)

    def close(self) -> None:
        with self._lock:
            if self._session is not None:
                self.store.end_session(self._session)
                self._session = None

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "session_id": self.session_id,
                "active_session": (
                    None if self._session is None else self._session.session_id
                ),
                "active_bytes": (
                    0 if self._session is None else int(self._session.bytes_written)
                ),
                "error": self.error,
                "last_entry_at": self.last_entry_at,
                "rotations": self.rotations,
                "max_bytes": self.max_bytes,
                "retain_days": self.retain_days,
                "retain_bytes": self.retain_bytes,
            }
