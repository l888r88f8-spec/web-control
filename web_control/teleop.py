"""ROS-free teleoperation state, axis mapping, and stop watchdog.

The controller owns input validation, the Ackermann mapping, the latest
accepted command, and the one-shot stop transitions.  It performs no ROS I/O
and reads time through an injected clock, so its behavior is unit-testable
without ROS.  ``WebControlNode`` adapts the returned command into a
``geometry_msgs/msg/Twist``.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable


MAX_CLIENT_ID_LENGTH = 128


def _finite_number(value: Any, name: str) -> float:
    """Return ``value`` as a finite float, rejecting booleans and strings."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _client_id(value: Any) -> str:
    """Return a trimmed nonempty bounded per-tab client identifier."""
    if not isinstance(value, str):
        raise ValueError("client_id must be a nonempty bounded string")
    client_id = value.strip()
    if not client_id or len(client_id) > MAX_CLIENT_ID_LENGTH:
        raise ValueError("client_id must be a nonempty bounded string")
    return client_id


def _sequence(value: Any) -> int:
    """Return ``value`` as a positive integer, rejecting bools and floats."""
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError("seq must be a positive integer")
    return value


def validate_teleop_parameters(
    *,
    max_linear_speed: float,
    wheelbase: float,
    max_steer_angle: float,
    steering_exponent: float,
    deadzone: float,
    command_timeout: float,
    publish_rate: float,
) -> None:
    """Reject nonpositive gains and an out-of-range dead zone at startup."""
    positive = {
        "max_linear_speed": max_linear_speed,
        "wheelbase": wheelbase,
        "max_steer_angle": max_steer_angle,
        "steering_exponent": steering_exponent,
        "command_timeout": command_timeout,
        "publish_rate": publish_rate,
    }
    for name, value in positive.items():
        if _finite_number(value, name) <= 0.0:
            raise ValueError(f"{name} must be positive")
    if not (0.0 < _finite_number(max_steer_angle, "max_steer_angle") < math.pi / 2.0):
        raise ValueError("max_steer_angle must be within (0, pi/2)")
    zone = _finite_number(deadzone, "deadzone")
    if zone < 0.0 or zone >= 0.5:
        raise ValueError("deadzone must be within [0, 0.5)")


class TeleopController:
    """Thread-safe latest-command and watchdog state for ``/api/teleop``."""

    def __init__(
        self,
        *,
        max_linear_speed: float,
        wheelbase: float,
        max_steer_angle: float,
        steering_exponent: float,
        deadzone: float,
        command_timeout: float,
        publish_rate: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        validate_teleop_parameters(
            max_linear_speed=max_linear_speed,
            wheelbase=wheelbase,
            max_steer_angle=max_steer_angle,
            steering_exponent=steering_exponent,
            deadzone=deadzone,
            command_timeout=command_timeout,
            publish_rate=publish_rate,
        )
        self.max_linear_speed = float(max_linear_speed)
        self.wheelbase = float(wheelbase)
        self.max_steer_angle = float(max_steer_angle)
        self.steering_exponent = float(steering_exponent)
        self.deadzone = float(deadzone)
        self.command_timeout = float(command_timeout)
        self.publish_rate = float(publish_rate)
        self.clock = clock
        self._lock = threading.RLock()
        self._active: dict[str, Any] | None = None
        self._last_command_at: float | None = None
        self._zero_pending = False
        self._closed = False
        self._last_seq: dict[str, int] = {}

    def map_axes(self, throttle: Any, steering: Any) -> dict[str, Any]:
        """Clamp, dead-zone, and Ackermann-map normalized axes to a command."""
        throttle_value = _finite_number(throttle, "throttle")
        steering_value = _finite_number(steering, "steering")
        throttle_value = max(-1.0, min(1.0, throttle_value))
        steering_value = max(-1.0, min(1.0, steering_value))
        if abs(throttle_value) < self.deadzone:
            throttle_value = 0.0
        if abs(steering_value) < self.deadzone:
            steering_value = 0.0
        speed = throttle_value * self.max_linear_speed
        steering_input = math.copysign(
            abs(steering_value) ** self.steering_exponent, steering_value
        )
        steering_angle = steering_input * self.max_steer_angle
        angular_velocity = speed / self.wheelbase * math.tan(steering_angle)
        return {
            "active": speed != 0.0,
            "throttle": throttle_value,
            "steering": steering_value,
            "steering_angle": steering_angle,
            "linear_x": speed,
            "angular_z": angular_velocity,
        }

    def apply_command(
        self, payload: Any, now: float | None = None
    ) -> dict[str, Any] | None:
        """Validate and store one command; invalid input leaves state untouched.

        Returns the accepted command, or ``None`` when the payload's per-client
        ``seq`` is not newer than the last accepted sequence.  A stale command
        is ignored entirely so it can never re-arm the watchdog after a newer
        stop.  Calls after :meth:`stop` are rejected because shutdown is
        terminal.
        """
        if not isinstance(payload, dict):
            raise ValueError("payload must be an object")
        if "throttle" not in payload:
            raise ValueError("missing throttle")
        if "steering" not in payload:
            raise ValueError("missing steering")
        client_id = _client_id(payload.get("client_id"))
        seq = _sequence(payload.get("seq"))
        command = self.map_axes(payload["throttle"], payload["steering"])
        timestamp = self.clock() if now is None else now
        with self._lock:
            if self._closed:
                raise RuntimeError("teleop controller is closed")
            last_seq = self._last_seq.get(client_id)
            if last_seq is not None and seq <= last_seq:
                return None
            self._last_seq[client_id] = seq
            if command["active"]:
                self._active = command
                self._last_command_at = timestamp
                self._zero_pending = False
            else:
                self._active = None
                self._last_command_at = None
                self._zero_pending = True
            return dict(command)

    def poll_publish(self, now: float | None = None) -> dict[str, Any] | None:
        """Return the command to publish, or ``None`` to stay silent.

        A fresh active command is returned on every call.  Once the active
        command times out, or a zero command was accepted, exactly one zero is
        returned and later calls return ``None`` until a new command arrives.
        """
        timestamp = self.clock() if now is None else now
        with self._lock:
            if self._closed:
                return None
            if self._active is not None:
                last = self._last_command_at
                if last is not None and timestamp - last <= self.command_timeout:
                    return dict(self._active)
                self._active = None
                self._last_command_at = None
                self._zero_pending = True
            if self._zero_pending:
                self._zero_pending = False
                return self._zero_command()
            return None

    def stop(self) -> dict[str, Any] | None:
        """Close the controller and return one zero, then ``None`` forever.

        The first call atomically marks the controller closed and clears all
        watchdog state, so a later or in-flight ``apply_command`` can never
        re-arm motion during shutdown.
        """
        with self._lock:
            if self._closed:
                return None
            self._closed = True
            self._active = None
            self._last_command_at = None
            self._zero_pending = False
            self._last_seq.clear()
            return self._zero_command()

    @staticmethod
    def _zero_command() -> dict[str, Any]:
        return {
            "active": False,
            "throttle": 0.0,
            "steering": 0.0,
            "steering_angle": 0.0,
            "linear_x": 0.0,
            "angular_z": 0.0,
        }
