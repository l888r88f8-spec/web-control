# Mapping Joystick Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a spring-return, Mapping-only virtual joystick that maps drag axes to Ackermann commands and publishes `geometry_msgs/msg/Twist` on `/cmd_vel` at 20 Hz while held, with exactly one zero on release, timeout, or shutdown.

**Architecture:** A new ROS-free `web_control/teleop.py` owns validation, the Ackermann mapping, latest-command state, the monotonic watchdog, and the one-shot zero transitions. `WebControlNode` owns the declared parameters, the `Twist` publisher and 20 Hz timer, and adapts the controller output; `POST /api/teleop` only parses JSON, calls `node.apply_teleop_command(payload)`, and returns its result. The browser adds a `static/app.js` joystick block that owns geometry, pointer lifecycle, ~10 Hz send throttling, visibility, and status, and never builds ROS messages.

**Tech Stack:** Python 3.10 / pytest 6.2.5, ROS 2 Python (`rclpy`, `geometry_msgs`; not importable in the test sandbox), browser JavaScript ES module tested with `node -e`, HTML/CSS.

**Constraints honored by this plan:**
- Current branch is `main`; implement directly.
- Tests are focused and meaningful: one new pure unit-test module plus essential node/HTTP integration tests plus one Node behavior harness and one layout assertion. No redundant source-string sweeps, no optional broad testing.
- The package already `<exec_depend>`s `geometry_msgs`; do **not** edit `package.xml` or `setup.py`.
- No ROS build (`colcon`) is run. The user compiles.
- No process-state gating in the joystick; no automatic Navigation stop or block.
- 20 Hz active publish, exactly one zero on release/timeout/shutdown, then silence so an idle joystick never fights Navigation.
- `ThreadingHTTPServer` callbacks and the ROS timer share only the locked `TeleopController`.

---

## File Structure

- Create: `web_control/teleop.py` — `validate_teleop_parameters` and `TeleopController`; no ROS imports, injected clock.
- Create: `test/test_teleop.py` — pure controller unit tests.
- Modify: `web_control/web_control_node.py` — `Twist` import, teleop parameter declarations/reads, publisher + 20 Hz timer, `apply_teleop_command`, `publish_teleop_twist`, `teleop_timer_callback`, `cleanup_teleop`, `POST /api/teleop`, `destroy_node` cleanup.
- Modify: `config/web_control.yaml` — eight `teleop_*` defaults.
- Modify: `test/test_core.py` — teleop node/HTTP integration tests, teleop YAML test, joystick layout test and Node harness.
- Modify: `static/index.html` — `#mappingJoystick` sibling overlay inside `#view3d`.
- Modify: `static/app.js` — joystick block, `setTab` wiring, `attachEvents` wiring.
- Modify: `static/styles.css` — joystick styling and Mapping-only HUD shift.

---

### Task 1: ROS-free teleop controller, axis mapping, and deadman

**Files:**
- Create: `web_control/teleop.py`
- Create: `test/test_teleop.py`

- [ ] **Step 1: Write the failing unit tests**

Create `test/test_teleop.py` with exactly:

```python
"""Unit tests for the ROS-free teleoperation controller."""

from __future__ import annotations

import math

import pytest

from web_control.teleop import TeleopController, validate_teleop_parameters


DEFAULTS = {
    "max_linear_speed": 1.5,
    "wheelbase": 0.65,
    "max_steer_angle": 0.461,
    "steering_exponent": 0.5,
    "deadzone": 0.08,
    "command_timeout": 0.4,
    "publish_rate": 20.0,
}


class FakeClock:
    def __init__(self, now: float = 0.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


def make_controller(clock=None, **overrides) -> TeleopController:
    parameters = dict(DEFAULTS)
    parameters.update(overrides)
    return TeleopController(clock=clock or FakeClock(), **parameters)


def test_map_axes_clamps_normalizes_and_applies_deadzone():
    controller = make_controller()

    clamped = controller.map_axes(2.0, -3.0)
    assert clamped["throttle"] == 1.0
    assert clamped["steering"] == -1.0

    dead = controller.map_axes(0.079, -0.079)
    assert dead["throttle"] == 0.0
    assert dead["steering"] == 0.0

    edge = controller.map_axes(0.08, 0.08)
    assert edge["throttle"] == 0.08
    assert edge["steering"] == 0.08


def test_map_axes_applies_ackermann_mapping_and_signs():
    controller = make_controller()

    expected_angle = 0.25 ** 0.5 * 0.461
    forward_left = controller.map_axes(0.6, 0.25)
    assert forward_left["linear_x"] == pytest.approx(0.9)
    assert forward_left["steering_angle"] == pytest.approx(expected_angle)
    assert forward_left["angular_z"] == pytest.approx(
        0.9 / 0.65 * math.tan(expected_angle)
    )
    assert forward_left["active"] is True

    reverse_right = controller.map_axes(-0.6, -0.25)
    assert reverse_right["linear_x"] == pytest.approx(-0.9)
    assert reverse_right["steering_angle"] == pytest.approx(-expected_angle)
    assert reverse_right["angular_z"] == pytest.approx(
        -0.9 / 0.65 * math.tan(-expected_angle)
    )


def test_pure_horizontal_axes_are_an_inactive_zero_command():
    controller = make_controller()

    command = controller.map_axes(0.0, 0.5)

    assert command["linear_x"] == 0.0
    assert command["angular_z"] == 0.0
    assert command["active"] is False


def test_apply_command_rejects_invalid_payloads():
    controller = make_controller()

    for payload in (
        None,
        [],
        {},
        {"throttle": 0.5},
        {"steering": 0.5},
        {"throttle": "0.5", "steering": 0.0},
        {"throttle": True, "steering": 0.0},
        {"throttle": float("nan"), "steering": 0.0},
        {"throttle": 0.0, "steering": float("inf")},
    ):
        with pytest.raises(ValueError):
            controller.apply_command(payload)


def test_invalid_command_does_not_replace_the_active_command():
    controller = make_controller()
    controller.apply_command({"throttle": 0.5, "steering": 0.0}, 1.0)

    with pytest.raises(ValueError):
        controller.apply_command({"throttle": float("nan"), "steering": 0.0}, 1.05)

    command = controller.poll_publish(1.1)
    assert command is not None
    assert command["linear_x"] == pytest.approx(0.75)


def test_idle_controller_stays_silent():
    controller = make_controller()

    assert controller.poll_publish(0.0) is None


def test_timeout_boundary_publishes_active_then_exactly_one_zero():
    controller = make_controller()
    controller.apply_command({"throttle": 1.0, "steering": 0.0}, 2.0)

    at_boundary = controller.poll_publish(2.4)
    assert at_boundary is not None
    assert at_boundary["linear_x"] == pytest.approx(1.5)

    zero = controller.poll_publish(2.400001)
    assert zero is not None
    assert zero["active"] is False
    assert zero["linear_x"] == 0.0
    assert zero["angular_z"] == 0.0

    assert controller.poll_publish(2.5) is None


def test_release_publishes_one_zero_and_reactivates_on_new_command():
    controller = make_controller()
    controller.apply_command({"throttle": 0.8, "steering": 0.0}, 0.0)

    release = controller.apply_command({"throttle": 0.0, "steering": 0.0}, 0.1)
    assert release["active"] is False

    zero = controller.poll_publish(0.1)
    assert zero is not None
    assert zero["linear_x"] == 0.0
    assert controller.poll_publish(0.2) is None

    controller.apply_command({"throttle": 0.4, "steering": 0.0}, 0.3)
    reactivated = controller.poll_publish(0.3)
    assert reactivated is not None
    assert reactivated["linear_x"] == pytest.approx(0.6)


def test_stop_returns_one_zero_and_clears_state():
    controller = make_controller()
    controller.apply_command({"throttle": 0.5, "steering": 0.2}, 0.0)

    zero = controller.stop()

    assert zero["active"] is False
    assert zero["linear_x"] == 0.0
    assert zero["angular_z"] == 0.0
    assert controller.poll_publish(0.0) is None


def test_validate_teleop_parameters_rejects_invalid_startup_values():
    validate_teleop_parameters(**DEFAULTS)

    for name in (
        "max_linear_speed",
        "wheelbase",
        "max_steer_angle",
        "steering_exponent",
        "command_timeout",
        "publish_rate",
    ):
        for value in (0.0, -1.0, float("nan")):
            with pytest.raises(ValueError):
                validate_teleop_parameters(**{**DEFAULTS, name: value})

    for value in (0.5, -0.1, float("inf")):
        with pytest.raises(ValueError):
            validate_teleop_parameters(**{**DEFAULTS, "deadzone": value})

    validate_teleop_parameters(**{**DEFAULTS, "deadzone": 0.0})

    with pytest.raises(ValueError):
        TeleopController(**{**DEFAULTS, "wheelbase": 0.0})
```

- [ ] **Step 2: Run the focused tests to verify RED**

Run:

```bash
python3 -m pytest -q test/test_teleop.py
```

Expected: collection fails with `ModuleNotFoundError: No module named 'web_control.teleop'`.

- [ ] **Step 3: Write the minimal implementation**

Create `web_control/teleop.py` with exactly:

```python
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


def _finite_number(value: Any, name: str) -> float:
    """Return ``value`` as a finite float, rejecting booleans and strings."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


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

    def apply_command(self, payload: Any, now: float | None = None) -> dict[str, Any]:
        """Validate and store one command; invalid input leaves state untouched."""
        if not isinstance(payload, dict):
            raise ValueError("payload must be an object")
        if "throttle" not in payload:
            raise ValueError("missing throttle")
        if "steering" not in payload:
            raise ValueError("missing steering")
        command = self.map_axes(payload["throttle"], payload["steering"])
        timestamp = self.clock() if now is None else now
        with self._lock:
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

    def stop(self) -> dict[str, Any]:
        """Clear the watchdog and return the one zero command for shutdown."""
        with self._lock:
            self._active = None
            self._last_command_at = None
            self._zero_pending = False
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
```

- [ ] **Step 4: Run the focused tests to verify GREEN**

Run:

```bash
python3 -m pytest -q test/test_teleop.py
```

Expected: all tests pass (`10 passed`; no failures).

- [ ] **Step 5: Commit**

```bash
git add web_control/teleop.py test/test_teleop.py
git commit -m "feat: add teleop controller and axis mapping"
```

---

### Task 2: WebControlNode publisher, timer, HTTP endpoint, and YAML parameters

**Files:**
- Modify: `web_control/web_control_node.py`
- Modify: `config/web_control.yaml`
- Test: `test/test_core.py`

- [ ] **Step 1: Write the failing node/HTTP integration tests**

In `test/test_core.py`, add this import beside the existing `web_control` imports (after the `from web_control.navigation_control import PENDING_CLEAR` line):

```python
from web_control.teleop import TeleopController
```

Then add these helpers and tests at the end of `test/test_core.py`:

```python
class _MutableClock:
    def __init__(self, now: float) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


class _FakeVector3:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _FakeTwist:
    def __init__(self) -> None:
        self.linear = _FakeVector3()
        self.angular = _FakeVector3()


def _teleop_node(clock=None, published=None, **overrides):
    parameters = {
        "max_linear_speed": 1.5,
        "wheelbase": 0.65,
        "max_steer_angle": 0.461,
        "steering_exponent": 0.5,
        "deadzone": 0.08,
        "command_timeout": 0.4,
        "publish_rate": 20.0,
    }
    parameters.update(overrides)
    node = type("_TeleopNodeStub", (), {})()
    node.now_sec = lambda: 10.0
    node.get_logger = lambda: _LoggerStub()
    node.teleop = TeleopController(clock=clock or (lambda: 0.0), **parameters)
    node.cmd_vel_publications = published if published is not None else []
    node.cmd_vel_pub = _FakePublisher(node.cmd_vel_publications)
    for name in (
        "apply_teleop_command",
        "publish_teleop_twist",
        "teleop_timer_callback",
        "cleanup_teleop",
    ):
        setattr(
            node,
            name,
            MethodType(getattr(web_control_node.WebControlNode, name), node),
        )
    return node


def test_teleop_handler_accepts_command_and_returns_mapped_twist():
    node = _teleop_node()

    status, body = _serve_and_request(
        node, "/api/teleop", {"throttle": 0.5, "steering": 0.25}, "POST"
    )

    assert status == 200
    assert body["ok"] is True
    assert body["teleop"]["throttle"] == 0.5
    assert body["teleop"]["steering"] == 0.25
    assert body["teleop"]["linear_x"] == pytest.approx(0.75)
    assert body["teleop"]["active"] is True
    assert node.cmd_vel_publications == []


def test_teleop_handler_rejects_invalid_payloads_without_replacing_command():
    node = _teleop_node()
    node.teleop.apply_command({"throttle": 0.5, "steering": 0.0}, 0.0)

    for payload in (
        [],
        {},
        {"throttle": "fast", "steering": 0.0},
        {"throttle": 0.5},
        {"throttle": float("nan"), "steering": 0.0},
    ):
        status, body = _serve_and_request(node, "/api/teleop", payload, "POST")
        assert status == 400, payload
        assert body["ok"] is False

    command = node.teleop.poll_publish(0.0)
    assert command is not None
    assert command["linear_x"] == pytest.approx(0.75)


def test_teleop_timer_publishes_active_command_then_exactly_one_zero(monkeypatch):
    monkeypatch.setattr(web_control_node, "Twist", _FakeTwist)
    clock = _MutableClock(0.0)
    node = _teleop_node(clock=clock)

    node.teleop_timer_callback()
    assert node.cmd_vel_publications == []

    node.teleop.apply_command({"throttle": 1.0, "steering": 0.0}, 0.0)
    node.teleop_timer_callback()
    assert len(node.cmd_vel_publications) == 1
    assert node.cmd_vel_publications[0].linear.x == pytest.approx(1.5)
    assert node.cmd_vel_publications[0].angular.z == pytest.approx(0.0)

    clock.now = 0.4
    node.teleop_timer_callback()
    assert len(node.cmd_vel_publications) == 2
    assert node.cmd_vel_publications[1].linear.x == 0.0

    clock.now = 0.5
    node.teleop_timer_callback()
    assert len(node.cmd_vel_publications) == 2


def test_teleop_cleanup_publishes_exactly_one_zero_on_shutdown(monkeypatch):
    monkeypatch.setattr(web_control_node, "Twist", _FakeTwist)
    node = _teleop_node()
    node.teleop.apply_command({"throttle": 0.5, "steering": 0.0}, 0.0)

    node.cleanup_teleop()

    assert len(node.cmd_vel_publications) == 1
    assert node.cmd_vel_publications[0].linear.x == 0.0
    assert node.cmd_vel_publications[0].angular.z == 0.0
    assert node.teleop.poll_publish(0.0) is None


def test_web_control_yaml_declares_teleop_defaults():
    package_root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load(
        (package_root / "config" / "web_control.yaml").read_text(encoding="utf-8")
    )
    parameters = config["web_control"]["ros__parameters"]

    assert parameters["teleop_cmd_vel_topic"] == "/cmd_vel"
    assert parameters["teleop_max_linear_speed"] == 1.5
    assert parameters["teleop_wheelbase"] == 0.65
    assert parameters["teleop_max_steer_angle"] == 0.461
    assert parameters["teleop_steering_exponent"] == 0.5
    assert parameters["teleop_deadzone"] == 0.08
    assert parameters["teleop_command_timeout"] == 0.4
    assert parameters["teleop_publish_rate"] == 20.0
```

- [ ] **Step 2: Run the new tests to verify RED**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_core.py -k "teleop"
```

Expected: the four node/HTTP tests fail with `AttributeError` because
`WebControlNode.apply_teleop_command` (and the other three methods) do not
exist, and `test_web_control_yaml_declares_teleop_defaults` fails with
`KeyError: 'teleop_cmd_vel_topic'`. The `/api/teleop` POST would also return
404 for the handler test.

- [ ] **Step 3: Declare and read the teleop parameters**

In `web_control/web_control_node.py`, extend the ROS import:

```python
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
```

and add `Twist = None` to the `except ModuleNotFoundError:` fallback block
(place it beside `PoseStamped = None`).

Add the controller import beside the other `web_control` imports:

```python
from web_control.teleop import TeleopController
```

Inside `WebControlNode.__init__`, after
`self.declare_parameter("viewer_timeout_sec", 10.0)`, add:

```python
        self.declare_parameter("teleop_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("teleop_max_linear_speed", 1.5)
        self.declare_parameter("teleop_wheelbase", 0.65)
        self.declare_parameter("teleop_max_steer_angle", 0.461)
        self.declare_parameter("teleop_steering_exponent", 0.5)
        self.declare_parameter("teleop_deadzone", 0.08)
        self.declare_parameter("teleop_command_timeout", 0.4)
        self.declare_parameter("teleop_publish_rate", 20.0)
```

After `self.viewer_timeout = float(self.get_parameter("viewer_timeout_sec").value)`,
add:

```python
        self.teleop = TeleopController(
            max_linear_speed=float(
                self.get_parameter("teleop_max_linear_speed").value
            ),
            wheelbase=float(self.get_parameter("teleop_wheelbase").value),
            max_steer_angle=float(
                self.get_parameter("teleop_max_steer_angle").value
            ),
            steering_exponent=float(
                self.get_parameter("teleop_steering_exponent").value
            ),
            deadzone=float(self.get_parameter("teleop_deadzone").value),
            command_timeout=float(
                self.get_parameter("teleop_command_timeout").value
            ),
            publish_rate=float(self.get_parameter("teleop_publish_rate").value),
        )
```

`TeleopController.__init__` calls `validate_teleop_parameters`, so a
nonpositive speed, wheelbase, steering angle, exponent, timeout, or publish
rate, or a dead zone outside `[0, 0.5)`, raises `ValueError` during node
startup.

- [ ] **Step 4: Add the Twist publisher and 20 Hz timer**

In `WebControlNode.__init__`, immediately after the `self.goal_pub = ...`
publisher block, add:

```python
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("teleop_cmd_vel_topic").value),
            10,
        )
        self.teleop_timer = self.create_timer(
            1.0 / self.teleop.publish_rate, self.teleop_timer_callback
        )
```

- [ ] **Step 5: Add the node methods and HTTP route**

Add these methods after `_publish_goal_pose` in `WebControlNode`:

```python
    def apply_teleop_command(self, payload: Any) -> dict[str, Any]:
        """Validate and store one teleop command; the timer publishes it."""
        command = self.teleop.apply_command(payload)
        response = success_response("teleop command accepted", self.now_sec())
        response["teleop"] = command
        return response

    def publish_teleop_twist(self, command: dict[str, Any]) -> None:
        msg = Twist()
        msg.linear.x = float(command["linear_x"])
        msg.angular.z = float(command["angular_z"])
        self.cmd_vel_pub.publish(msg)

    def teleop_timer_callback(self) -> None:
        command = self.teleop.poll_publish()
        if command is None:
            return
        self.publish_teleop_twist(command)

    def cleanup_teleop(self) -> None:
        """Publish exactly one zero Twist before the node shuts down."""
        self.publish_teleop_twist(self.teleop.stop())
```

In `do_POST`, add this branch after the `/api/viewer` branch:

```python
                    elif self.path == "/api/teleop":
                        self.send_json(node.apply_teleop_command(payload))
```

A `ValueError` from `apply_command` (malformed payload, missing axis,
nonnumeric axis, nonfinite axis) is already caught by the existing
`except (json.JSONDecodeError, ValueError)` clause and returned as HTTP 400
without touching the stored command.

In `destroy_node`, add the teleop cleanup as the first call so the zero is
published before `super().destroy_node()` destroys the publisher:

```python
    def destroy_node(self) -> bool:
        self.cleanup_teleop()
        self.cleanup_save_map()
        self.stop_managed_processes()
        self.stop_http_server()
        return super().destroy_node()
```

Do not add any Navigation detection, blocking, pausing, or stopping.

- [ ] **Step 6: Add the YAML defaults**

In `config/web_control.yaml`, after the `viewer_timeout_sec` line, add:

```yaml
    teleop_cmd_vel_topic: "/cmd_vel"           # Hunter velocity command topic
    teleop_max_linear_speed: 1.5              # Max forward/reverse speed (m/s)
    teleop_wheelbase: 0.65                    # Hunter wheelbase (m)
    teleop_max_steer_angle: 0.461             # Max central steering angle (rad)
    teleop_steering_exponent: 0.5             # Steering curve exponent
    teleop_deadzone: 0.08                     # Normalized center dead zone
    teleop_command_timeout: 0.4               # Watchdog timeout (s)
    teleop_publish_rate: 20.0                 # Active command publish rate (Hz)
```

Do not edit `package.xml` or `setup.py`; `geometry_msgs` is already a declared
dependency.

- [ ] **Step 7: Run the new tests to verify GREEN**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_core.py -k "teleop"
```

Expected: all five new tests pass.

- [ ] **Step 8: Commit**

```bash
git add web_control/web_control_node.py config/web_control.yaml test/test_core.py
git commit -m "feat: publish teleop cmd_vel from web node"
```

---

### Task 3: Mapping-only lower-left joystick UI

**Files:**
- Modify: `static/index.html`
- Modify: `static/app.js`
- Modify: `static/styles.css`
- Test: `test/test_core.py`

- [ ] **Step 1: Write the failing layout and Node behavior tests**

Add this layout test and Node harness to the end of `test/test_core.py`:

```python
def test_mapping_joystick_is_a_mapping_only_sibling_overlay():
    index_html, app_js, styles_css = _frontend_sources()

    view_start = index_html.index('id="view3d"')
    view_end = index_html.index('id="viewSystem"', view_start)
    viewport = index_html[view_start:view_end]

    assert 'id="mapCanvas3d"' in viewport
    assert 'id="mappingJoystick" class="joystick" hidden' in viewport
    assert viewport.index('id="mapCanvas3d"') < viewport.index('id="mappingJoystick"')
    assert 'id="joystickBase"' in viewport
    assert 'id="joystickKnob"' in viewport
    assert 'id="joystickStatus"' in viewport

    assert 'setMappingJoystickVisible(tab === "mapping")' in app_js
    assert ".joystick {" in styles_css
    assert "position: absolute;" in styles_css
    assert ".stage.mapping-active .hud" in styles_css
```

```python
_MAPPING_JOYSTICK_NODE_HARNESS = r"""
const fs = require("fs");
const source = fs.readFileSync(process.env.APP_JS, "utf8");
const blockStart = source.indexOf("// --- Mapping joystick");
const blockEnd = source.indexOf("// --- End mapping joystick");
if (blockStart < 0 || blockEnd < 0) throw new Error("mapping joystick block not found");
const block = source.slice(blockStart, blockEnd);

function makeElement() {
  return {
    hidden: false,
    textContent: "",
    className: "",
    style: {},
    captured: [],
    listeners: {},
    classList: {
      values: new Set(),
      toggle(name, on) {
        if (on) this.values.add(name);
        else this.values.delete(name);
      },
      contains(name) { return this.values.has(name); },
    },
    setPointerCapture(id) { this.captured.push(id); },
    addEventListener(name, handler) { this.listeners[name] = handler; },
    getBoundingClientRect() { return { left: 0, top: 0, width: 100, height: 100 }; },
  };
}

const elements = {
  mappingJoystick: makeElement(),
  joystickKnob: makeElement(),
  joystickStatus: makeElement(),
};
const stage = makeElement();
global.document = {
  hidden: false,
  getElementById(id) { return elements[id] || null; },
  querySelector(selector) { return selector === ".stage" ? stage : null; },
  addEventListener() {},
};
global.window = { addEventListener() {} };

const beacons = [];
try {
  Object.defineProperty(globalThis, "navigator", {
    value: { sendBeacon(url, body) { beacons.push({ url, body }); return true; } },
    configurable: true,
  });
} catch (error) {
  globalThis.navigator.sendBeacon = (url, body) => {
    beacons.push({ url, body });
    return true;
  };
}

let fakeNow = 0;
Date.now = () => fakeNow;

const posts = [];
global.postJson = async (url, payload) => {
  posts.push({ url, payload });
  const active = payload.throttle !== 0 || payload.steering !== 0;
  return { ok: true, teleop: { active } };
};
global.fetchWithTimeout = async () => ({ ok: true });

eval(block);

function expect(condition, message) { if (!condition) throw new Error(message); }

const baseElement = makeElement();
let propagationStops = 0;
function makeEvent(pointerId, clientX, clientY) {
  return {
    pointerId,
    pointerType: "mouse",
    clientX,
    clientY,
    currentTarget: baseElement,
    preventDefault() {},
    stopPropagation() { propagationStops += 1; },
  };
}

const radius = 50;
const upLeft = joystickAxes(-40, -40, radius);
expect(upLeft.throttle > 0 && upLeft.steering > 0, "upper-left is forward and left");
const upRight = joystickAxes(40, -40, radius);
expect(upRight.throttle > 0 && upRight.steering < 0, "upper-right is forward and right");
const downLeft = joystickAxes(-40, 40, radius);
expect(downLeft.throttle < 0 && downLeft.steering > 0, "lower-left is reverse and left");
const downRight = joystickAxes(40, 40, radius);
expect(downRight.throttle < 0 && downRight.steering < 0, "lower-right is reverse and right");
const horizontal = joystickAxes(-40, 0, radius);
expect(horizontal.throttle === 0 && horizontal.steering > 0, "horizontal drag has zero throttle");
const centered = joystickAxes(2, -2, radius);
expect(centered.throttle === 0 && centered.steering === 0, "dead zone centers the axes");
const clamped = joystickAxes(-100, -100, radius);
expect(clamped.throttle === 1 && clamped.steering === 1, "offsets clamp to the unit circle");

(async () => {
  fakeNow = 0;
  await joystickPointerDown(makeEvent(1, 30, 30));
  expect(posts.length === 1, "pointerdown sends the first command");
  expect(posts[0].url === "/api/teleop", "pointerdown posts to /api/teleop");
  expect(posts[0].payload.throttle > 0 && posts[0].payload.steering > 0, "pointerdown posts the upper-left axes");
  expect(baseElement.captured.length === 1, "pointerdown captures the active pointer");
  expect(propagationStops >= 1, "pointerdown stops propagation to the map canvas");
  expect(elements.joystickKnob.style.transform === "translate(-20px, -20px)", "knob follows the pointer");

  fakeNow = 50;
  await joystickPointerMove(makeEvent(1, 20, 20));
  expect(posts.length === 1, "pointermove is throttled within 100 ms");

  fakeNow = 100;
  await joystickPointerMove(makeEvent(1, 20, 20));
  expect(posts.length === 2, "pointermove sends again after 100 ms");

  const beforeSecondPointer = posts.length;
  await joystickPointerDown(makeEvent(2, 80, 80));
  expect(baseElement.captured.length === 1, "second pointer is not captured");
  expect(posts.length === beforeSecondPointer, "second pointer does not take control");

  fakeNow = 200;
  await joystickPointerUp(makeEvent(1, 20, 20));
  expect(posts[posts.length - 1].payload.throttle === 0, "release sends a zero throttle");
  expect(posts[posts.length - 1].payload.steering === 0, "release sends a zero steering");
  expect(elements.joystickKnob.style.transform === "translate(0px, 0px)", "release centers the knob");

  fakeNow = 300;
  await joystickPointerDown(makeEvent(3, -10, 90));
  fakeNow = 400;
  await joystickPointerCancel(makeEvent(3, -10, 90));
  expect(posts[posts.length - 1].payload.throttle === 0, "cancel sends a zero command");

  fakeNow = 500;
  await joystickPointerDown(makeEvent(4, 10, 10));
  fakeNow = 600;
  await handleJoystickBlur();
  expect(posts[posts.length - 1].payload.throttle === 0, "blur sends a zero command");

  fakeNow = 700;
  await joystickPointerDown(makeEvent(5, 10, 10));
  document.hidden = true;
  fakeNow = 800;
  await handleVisibilityChange();
  expect(posts[posts.length - 1].payload.throttle === 0, "hidden document sends a zero command");
  document.hidden = false;

  fakeNow = 900;
  await joystickPointerDown(makeEvent(6, 10, 10));
  await setMappingJoystickVisible(true);
  expect(elements.mappingJoystick.hidden === false, "joystick is visible for Mapping");
  expect(stage.classList.contains("mapping-active"), "HUD shift class is applied for Mapping");
  fakeNow = 1000;
  await setMappingJoystickVisible(false);
  expect(elements.mappingJoystick.hidden === true, "joystick is hidden away from Mapping");
  expect(!stage.classList.contains("mapping-active"), "HUD shift class is removed away from Mapping");
  expect(posts[posts.length - 1].payload.throttle === 0, "tab switch sends a zero command");

  fakeNow = 1100;
  await joystickPointerDown(makeEvent(7, 10, 10));
  handlePageHide();
  expect(beacons.length === 1, "page shutdown sends one keepalive stop");
  expect(JSON.parse(beacons[0].body).throttle === 0, "keepalive stop is a zero command");

  console.log("mapping joystick ok");
})().catch((error) => { console.error(error); process.exit(1); });
"""


def test_mapping_joystick_pointer_lifecycle_and_throttling_execute_under_node():
    if shutil.which("node") is None:
        pytest.skip("node is not installed")
    app_js = Path(__file__).resolve().parents[1] / "static" / "app.js"
    completed = subprocess.run(
        ["node", "-e", _MAPPING_JOYSTICK_NODE_HARNESS],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "APP_JS": str(app_js)},
    )
    assert completed.returncode == 0, completed.stderr
    assert "mapping joystick ok" in completed.stdout
```

- [ ] **Step 2: Run the new tests to verify RED**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_core.py -k "mapping_joystick"
```

Expected: `test_mapping_joystick_is_a_mapping_only_sibling_overlay` fails with
`ValueError: substring not found` (no `id="mappingJoystick"`), and
`test_mapping_joystick_pointer_lifecycle_and_throttling_execute_under_node`
fails with `Error: mapping joystick block not found` on stderr.

- [ ] **Step 3: Add the joystick markup**

In `static/index.html`, replace the `#view3d` block with:

```html
        <div id="view3d" class="viewport active">
          <canvas id="mapCanvas3d"></canvas>
          <div class="hud" id="hud">3D map ready</div>
          <div id="mappingJoystick" class="joystick" hidden>
            <div id="joystickBase" class="joystick-base">
              <span class="joystick-axis joystick-axis-x"></span>
              <span class="joystick-axis joystick-axis-y"></span>
              <div id="joystickKnob" class="joystick-knob"></div>
            </div>
            <p id="joystickStatus" class="joystick-status" role="status" aria-live="polite">idle</p>
          </div>
        </div>
```

The joystick stays a sibling of `#mapCanvas3d`, so canvas pan, zoom, and pose
selection are unaffected.

- [ ] **Step 4: Add the browser joystick block**

In `static/app.js`, insert this block immediately after the
`// --- End navigation pause/resume controls` line (before
`function maybeClearGoalFromActionStatus()`):

```javascript
// --- Mapping joystick
const JOYSTICK_SEND_INTERVAL_MS = 100;
const JOYSTICK_DEADZONE = 0.08;

const teleop = {
  pointerId: null,
  active: false,
  throttle: 0,
  steering: 0,
  lastSentAt: -Infinity,
};

function joystickAxes(offsetX, offsetY, radius) {
  if (!Number.isFinite(radius) || radius <= 0) return { throttle: 0, steering: 0 };
  const clamp = (value) => Math.max(-1, Math.min(1, value));
  const deadzone = (value) => (Math.abs(value) < JOYSTICK_DEADZONE ? 0 : value);
  // Screen y grows downward, so up is forward; left is positive steering.
  return {
    throttle: deadzone(clamp(-offsetY / radius)),
    steering: deadzone(clamp(-offsetX / radius)),
  };
}

function joystickOffset(event, base) {
  const rect = base.getBoundingClientRect();
  const radius = Math.max(1, Math.min(rect.width, rect.height) / 2);
  let dx = event.clientX - (rect.left + rect.width / 2);
  let dy = event.clientY - (rect.top + rect.height / 2);
  const distance = Math.hypot(dx, dy);
  if (distance > radius) {
    dx = (dx / distance) * radius;
    dy = (dy / distance) * radius;
  }
  return { dx, dy, radius };
}

function renderJoystickKnob(dx, dy) {
  const knob = document.getElementById("joystickKnob");
  if (!knob) return;
  knob.style.transform = `translate(${dx}px, ${dy}px)`;
}

function setJoystickStatus(text, level) {
  const element = document.getElementById("joystickStatus");
  if (!element) return;
  element.textContent = text;
  element.className = `joystick-status ${level}`;
}

function resetJoystick() {
  teleop.pointerId = null;
  teleop.throttle = 0;
  teleop.steering = 0;
  renderJoystickKnob(0, 0);
}

async function sendTeleopCommand(throttle, steering, force = false) {
  const now = Date.now();
  if (!force && now - teleop.lastSentAt < JOYSTICK_SEND_INTERVAL_MS) return false;
  teleop.lastSentAt = now;
  try {
    const response = await postJson("/api/teleop", { throttle, steering });
    const accepted = response && response.teleop ? response.teleop : null;
    if (accepted && accepted.active) setJoystickStatus("active", "active");
    else setJoystickStatus("stopped", "idle");
    return true;
  } catch (error) {
    teleop.active = false;
    resetJoystick();
    teleop.lastSentAt = -Infinity;
    setJoystickStatus(`error: ${error.message}`, "error");
    return false;
  }
}

async function stopTeleop(reason) {
  const wasActive = teleop.active || teleop.pointerId !== null;
  resetJoystick();
  teleop.active = false;
  teleop.lastSentAt = -Infinity;
  if (!wasActive) {
    setJoystickStatus("idle", "idle");
    return;
  }
  setJoystickStatus("stopped", "idle");
  await sendTeleopCommand(0, 0, true);
}

function sendTeleopKeepaliveStop() {
  const body = JSON.stringify({ throttle: 0, steering: 0 });
  if (navigator && typeof navigator.sendBeacon === "function") {
    navigator.sendBeacon("/api/teleop", body);
    return;
  }
  fetchWithTimeout("/api/teleop", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body,
    keepalive: true,
  }).catch(() => {});
}

async function joystickPointerDown(event) {
  if (teleop.pointerId !== null) return;
  event.preventDefault();
  event.stopPropagation();
  teleop.pointerId = event.pointerId;
  teleop.active = true;
  const base = event.currentTarget;
  if (base && typeof base.setPointerCapture === "function") {
    base.setPointerCapture(event.pointerId);
  }
  const offset = joystickOffset(event, base);
  renderJoystickKnob(offset.dx, offset.dy);
  const axes = joystickAxes(offset.dx, offset.dy, offset.radius);
  teleop.throttle = axes.throttle;
  teleop.steering = axes.steering;
  await sendTeleopCommand(axes.throttle, axes.steering, true);
}

async function joystickPointerMove(event) {
  if (!teleop.active || teleop.pointerId === null || event.pointerId !== teleop.pointerId) return;
  event.preventDefault();
  event.stopPropagation();
  const offset = joystickOffset(event, event.currentTarget);
  renderJoystickKnob(offset.dx, offset.dy);
  const axes = joystickAxes(offset.dx, offset.dy, offset.radius);
  teleop.throttle = axes.throttle;
  teleop.steering = axes.steering;
  await sendTeleopCommand(axes.throttle, axes.steering);
}

async function joystickPointerUp(event) {
  if (teleop.pointerId === null || event.pointerId !== teleop.pointerId) return;
  event.preventDefault();
  event.stopPropagation();
  await stopTeleop("release");
}

async function joystickPointerCancel(event) {
  if (teleop.pointerId === null || event.pointerId !== teleop.pointerId) return;
  event.preventDefault();
  event.stopPropagation();
  await stopTeleop("cancel");
}

async function handleJoystickBlur() {
  await stopTeleop("blur");
}

async function handleVisibilityChange() {
  if (document.hidden) await stopTeleop("hidden");
}

function handlePageHide() {
  if (!teleop.active && teleop.pointerId === null) return;
  resetJoystick();
  teleop.active = false;
  teleop.lastSentAt = -Infinity;
  sendTeleopKeepaliveStop();
}

async function setMappingJoystickVisible(visible) {
  const joystick = document.getElementById("mappingJoystick");
  if (joystick) joystick.hidden = !visible;
  const stage = document.querySelector(".stage");
  if (stage) stage.classList.toggle("mapping-active", visible);
  if (!visible && teleop.active) await stopTeleop("tab");
}
// --- End mapping joystick
```

- [ ] **Step 5: Wire the joystick into the tab and events**

In `setTab`, add this call next to the other class toggles (immediately after
the `document.querySelector(".app").classList.toggle("system-mode", tab === "system");`
line):

```javascript
  setMappingJoystickVisible(tab === "mapping");
```

In `attachEvents`, add these listeners immediately before the
`mapCanvas3d.addEventListener("contextmenu", ...)` line:

```javascript
  const joystickBase = document.getElementById("joystickBase");
  if (joystickBase) {
    joystickBase.addEventListener("pointerdown", joystickPointerDown);
    joystickBase.addEventListener("pointermove", joystickPointerMove);
    joystickBase.addEventListener("pointerup", joystickPointerUp);
    joystickBase.addEventListener("pointercancel", joystickPointerCancel);
    joystickBase.addEventListener("lostpointercapture", joystickPointerCancel);
    joystickBase.addEventListener("contextmenu", (event) => event.preventDefault());
  }
  window.addEventListener("blur", handleJoystickBlur);
  window.addEventListener("pagehide", handlePageHide);
  document.addEventListener("visibilitychange", handleVisibilityChange);
```

The joystick block must not read `state.localizationRunning`,
`state.navigationRunning`, `state.managedProcessPending`, or any managed-process
state; visibility depends only on the active tab.

- [ ] **Step 6: Add the styling**

In `static/styles.css`, add this block after the `.hud` rule:

```css
.joystick {
  position: absolute;
  left: 12px;
  bottom: 12px;
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

.joystick-base {
  position: relative;
  width: 128px;
  height: 128px;
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
  left: 10px;
  right: 10px;
  top: 50%;
  height: 1px;
}

.joystick-axis-y {
  top: 10px;
  bottom: 10px;
  left: 50%;
  width: 1px;
}

.joystick-knob {
  position: absolute;
  left: 50%;
  top: 50%;
  width: 44px;
  height: 44px;
  margin: -22px 0 0 -22px;
  border-radius: 50%;
  border: 1px solid var(--liquid-glass-edge);
  background: var(--glass-strong);
  box-shadow: var(--shadow-soft);
  pointer-events: none;
  transition: transform 90ms linear;
}

.joystick-status {
  margin: 0;
  color: var(--muted);
  font-size: 12px;
}

.joystick-status.active {
  color: var(--accent);
}

.joystick-status.error {
  color: var(--bad);
}

.stage.mapping-active .hud {
  left: 156px;
}
```

Inside the existing `@media (max-width: 900px)` block, after the `.hud`
adjustment, add the compact Mapping offset so the HUD still clears the
joystick on small screens:

```css
  .stage.mapping-active .hud {
    left: 148px;
  }
```

- [ ] **Step 7: Run the new tests to verify GREEN**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test/test_core.py -k "mapping_joystick"
```

Expected: both tests pass and the Node harness prints `mapping joystick ok`.

- [ ] **Step 8: Commit**

```bash
git add static/index.html static/app.js static/styles.css test/test_core.py
git commit -m "feat: add mapping joystick control"
```

---

## Final Verification

Run after all three tasks are committed. Do not run `colcon`.

- [ ] **Step 1: Focused teleop tests**

Run:

```bash
python3 -m pytest -q test/test_teleop.py
```

Expected: all controller tests pass.

- [ ] **Step 2: One full package regression with proxies unset**

Run:

```bash
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  python3 -m pytest -q test
```

Expected: the complete suite (including the new teleop and joystick tests)
passes with zero failures. The proxy variables are unset because the HTTP
endpoint tests open real loopback sockets, which an environment proxy can
intercept.

- [ ] **Step 3: Syntax and formatting gates**

Run:

```bash
node --check static/app.js
python3 -m py_compile web_control/*.py
git diff --check
```

Expected: each command exits zero. `git diff --check` reports no whitespace
errors.

- [ ] **Step 4: Confirm Navigation coexistence by inspection**

Run:

```bash
grep -n -i "navigation" web_control/teleop.py
```

Expected: no output. The controller has no Navigation dependency, so it never
detects, blocks, pauses, or stops Navigation. The one-shot zero semantics are
already proven by the controller tests and by
`test_teleop_timer_publishes_active_command_then_exactly_one_zero` and
`test_teleop_cleanup_publishes_exactly_one_zero_on_shutdown`: while idle the
timer callback returns early, so an idle joystick does not stream zeros onto
`/cmd_vel`.
