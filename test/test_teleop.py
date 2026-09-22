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
