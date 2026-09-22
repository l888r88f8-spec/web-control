# Mapping Joystick Design

## Goal

Add a spring-return virtual joystick to the Mapping map view so an operator can
drive the Hunter vehicle from a mouse or touchscreen while building a map.

## Placement and Visibility

The joystick floats over the lower-left corner of the shared 3D map viewport.
It is visible and enabled whenever the Mapping tab is active. It does not depend
on whether Mapping, Localization, or Navigation managed processes are running.
Switching away from Mapping hides the joystick and requests a stop.

The existing lower-left map HUD moves far enough to the right while Mapping is
active to avoid overlapping the joystick. The joystick remains a sibling of the
Three.js canvas so its pointer events cannot trigger map pan, zoom, or pose
selection.

## Interaction Model

The control is a circular base with horizontal and vertical center lines and a
spring-return knob. The lines divide it into four continuous quadrants:

- upper-left: forward and left;
- upper-right: forward and right;
- lower-left: reverse and left;
- lower-right: reverse and right.

Vertical displacement from the horizontal center line determines signed linear
speed. Horizontal displacement from the vertical center line determines signed
steering angle. Drag magnitude is continuous. Pure horizontal displacement has
zero linear speed, so the vehicle does not move. Releasing or cancelling the
pointer returns the knob to center and requests a stop.

The browser supports mouse, pen, and single-touch Pointer Events. The joystick
captures only its active pointer and stops propagation so the map canvas does
not receive the same gesture. A second pointer does not take control.

## Axis Mapping

The browser normalizes the knob position to `throttle` and `steering` values in
the inclusive range `[-1, 1]`. Positive throttle means forward; positive
steering means left. Values inside a configurable center dead zone are zero.

The backend converts normalized input into Hunter commands:

```text
speed = throttle * max_linear_speed
steering_input = sign(steering) * abs(steering) ^ steering_exponent
steering_angle = steering_input * max_steer_angle
angular_velocity = speed / wheelbase * tan(steering_angle)
```

The default steering exponent is `0.5`. This amplifies small horizontal
displacements while remaining continuous, so a diagonal drag produces a clear
turn without a discontinuous minimum-angle jump. The backend validates finite
numbers and clamps both normalized axes before applying the formula.

Hunter defaults come from the installed base driver:

- maximum forward and reverse speed: `1.5 m/s`;
- wheelbase: `0.65 m`;
- maximum central steering angle: `0.461 rad` for Hunter V2.

The base driver detects its protocol version and applies its own final steering
limit. All web limits remain configurable for field tuning.

## Browser-to-ROS Data Flow

While the joystick is held outside the dead zone, the browser posts normalized
axes to `/api/teleop` at approximately 10 Hz:

```json
{"throttle": 0.6, "steering": -0.4}
```

The HTTP response reports the accepted normalized axes and calculated Twist.
The browser uses the existing finite request timeout and displays a small
joystick status for active, stopped, or error states.

The ROS node owns a `geometry_msgs/msg/Twist` publisher on the configured
`/cmd_vel` topic. A 20 Hz ROS timer publishes the latest nonzero active command.
This matches the message type already consumed by `hunter_base`; the base
driver converts Twist into Ackermann steering internally.

No rosbridge, `ackermann_msgs`, or new ROS process is required.

## Stop and Timeout Behavior

The browser sends a zero command when any of these events occurs:

- pointer release or pointer cancellation;
- browser window blur;
- document becomes hidden;
- switching away from Mapping;
- page shutdown when a keepalive request can still be queued.

The backend also tracks command arrival using a monotonic clock. If no fresh
command arrives within `0.4 seconds`, the timer publishes one zero Twist and
deactivates teleoperation. Node shutdown publishes one zero Twist before the
publisher is destroyed.

After release or timeout the backend does not continuously publish zero. This
prevents an idle web joystick from continuously overriding another `/cmd_vel`
publisher. Per the accepted requirement, the feature does not detect, block,
pause, or stop Navigation. If Navigation publishes `/cmd_vel` while the
joystick is actively held, both publishers remain active and command ordering
is determined by ROS delivery timing.

## Configuration

The web node declares and reads these parameters from `web_control.yaml`:

```yaml
teleop_cmd_vel_topic: /cmd_vel
teleop_max_linear_speed: 1.5
teleop_wheelbase: 0.65
teleop_max_steer_angle: 0.461
teleop_steering_exponent: 0.5
teleop_deadzone: 0.08
teleop_command_timeout: 0.4
teleop_publish_rate: 20.0
```

Startup rejects nonpositive speed, wheelbase, steering angle, exponent,
timeout, or publish rate. The dead zone must be within `[0, 0.5)`.

## Component Boundaries

A small Python teleoperation component owns input validation, axis mapping,
latest-command state, timeout state, and stop transitions. It receives a clock
function so its mapping and watchdog behavior can be unit-tested without ROS.

`WebControlNode` owns ROS parameters, the Twist publisher and timer, and adapts
the component output into `geometry_msgs/msg/Twist`. The HTTP handler only
parses JSON, calls the node method, and returns its result.

The browser module owns joystick geometry, pointer lifecycle, send throttling,
visibility, and UI status. It sends normalized axes and never constructs ROS
messages.

## Error Handling

Malformed, missing, nonnumeric, or nonfinite axes return HTTP 400 and do not
replace the current command. Browser request failures center the visible knob
and stop the send loop; the backend watchdog supplies the final stop if the
explicit request cannot reach the server.

An unavailable `/cmd_vel` subscriber does not make ROS publication fail. The UI
therefore reports successful command acceptance rather than claiming vehicle
motion.

## Verification

Automated tests will cover:

- axis validation, clamping, dead zone, speed and Ackermann angular mapping;
- exact timeout boundary, one-shot timeout stop, release stop, and reactivation;
- `/api/teleop` success and invalid-payload responses;
- ROS Twist construction and stop publication during node shutdown;
- joystick placement, Mapping-only visibility, quadrant axes, and no
  process-state gating;
- pointer capture, continuous normalized motion, 10 Hz throttling, release,
  cancel, blur, visibility change, tab switch, and page-shutdown stop paths;
- JavaScript syntax, Python compilation, and the complete package test suite.

The user will run the ROS workspace build, so implementation verification will
not run `colcon`.
