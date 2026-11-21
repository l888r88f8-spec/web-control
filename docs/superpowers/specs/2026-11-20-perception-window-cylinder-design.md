# Perception window overlay: body-frame cylinder

Date: 2026-11-20
Status: approved

## Problem

The 3D view draws the perception window as a dashed cuboid of
+/- `window` in x/y and `[min_height, max_height]` in z.  The shared cloud
preprocessor does not crop a box: it transforms each scan into the robot body
frame and keeps points with `x*x + y*y <= r*r` and
`min_obstacle_height <= z <= marking_height`.  The configured
`perception_window_size: 10.0` is a radius in metres, not a half extent, so the
overlay currently overstates the cropped area by the corners of the square.

The overlay also only applies the odometry yaw.  The crop plane is
perpendicular to the body z axis, so on a pitch or roll the real window tilts
with the vehicle while the drawn box stays horizontal.

## Behaviour

- Draw a dashed wire cylinder aligned with the body frame: one ring at body
  `z = min_height`, one at `z = max_height`, both of radius `window`, plus
  eight vertical generatrices every 45 degrees.
- Place the cylinder at the body origin from localization odometry and orient
  it with the full body attitude (roll, pitch, yaw).
- Keep the existing colour (`0x35c8ff`), dashed styling (`dashSize` 0.8 m,
  `gapSize` 0.5 m) and opacity (0.85).
- Label the window as a radius: `r 10.0 m · z -0.20…2.00 m`.
- When the odometry payload carries no quaternion (older server), fall back to
  the yaw-only rotation used today.
- No C++ change: the preprocessor already crops in the body frame.

## Components

### Backend (`web_control_node.py`)

`odom_callback` and `mapping_odom_callback` already build the orientation
quaternion to derive yaw.  Add `qx`, `qy`, `qz`, `qw` to the cached pose
payload next to `x`, `y`, `z`, `yaw`, `frame_id`.  The payload flows to
`/api/state`, `/api/pose` and the inline SSE pose entries unchanged.

### Frontend (`static/app.js`)

1. `poseRevision()` must include the quaternion so a roll/pitch-only change is
   not discarded as a duplicate revision.
2. `notePoseEntry()` accepts the optional quaternion and passes it through
   `applyPose()`; missing or non-finite components leave it unset.
3. `perceptionWindowSegmentPositions(window, minHeight, maxHeight)` is a pure
   function returning the flat LineSegments vertex array:
   - axis mapping is `rosToThree`'s `(x, y, z) -> (x, z, -y)`, so the rings lie
     in the three.js XZ plane;
   - each ring uses 96 segments; the two rings share vertices in azimuth;
   - the eight generatrices connect `(0, min)` to `(0, max)` at every 45
     degrees.
4. `buildPerceptionWindowGeometry()` keeps the `window|min|max` cache key and
   builds the `LineSegments` from that array with the existing material.
5. `updatePerceptionWindow()` sets the group position from the pose and the
   group quaternion from `(qx, qz, -qy, qw)` normalised; without a valid
   quaternion it keeps `rotation.y = yaw`.
6. The section comment describing a body-frame box becomes a description of
   the body-frame cylinder.

## Parameters

- `perception_window_size` — radius of the crop circle in metres, mirrored from
  `/shared_cloud_preprocessor/get_parameters`; the overlay radius equals it.
- `min_obstacle_height` / `marking_height` — lower and upper crop limits along
  the body z axis in metres; they become the two ring heights.
- `perception_window_refresh_sec` — seconds between parameter re-queries;
  unchanged.

## Error handling

- Missing `available`/`window` keeps the overlay hidden (existing behaviour).
- Missing quaternion components fall back to yaw; a degenerate quaternion
  (norm near zero) also falls back to yaw instead of producing NaNs.
- The group is still hidden outside the 3D view.

## Testing

- Node contract test executing the extracted
  `perceptionWindowSegmentPositions()`: every ring vertex satisfies
  `x*x + z*z == r*r` at one of the two heights; generatrix endpoints are paired
  at equal azimuth; both rings have the same segment count.
- Python test: `odom_callback` and `mapping_odom_callback` payloads carry a
  normalised quaternion and keep the existing yaw.
- Frontend pose test: a change confined to the quaternion produces a new pose
  revision and reaches `state.odom.pose`.
