# Hesai driver publish statistics

This directory documents the canonical-driver side of the LiDAR
"驱动发布频率" feature. The driver implementation is in the canonical workspace
`/home/robot/hesailidar_ws`.

There is intentionally no patch or copied header here: the canonical driver
workspace owns `src/utility/publish_stats_collector.hpp`, and keeping a second
copy would drift from the implementation being built.

## Build (low priority, single job)

The driver change has already been applied to the canonical workspace.  When a
rebuild is needed:

```bash
source /opt/ros/humble/setup.bash
cd /home/robot/hesailidar_ws
nice -n 15 env MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  colcon build --packages-select hesai_ros_driver --parallel-workers 1
source /home/robot/hesailidar_ws/install/local_setup.bash
```

The package installs under `/home/robot/hesailidar_ws/install/hesai_ros_driver`.
Source `install/local_setup.bash` **before** starting `web_control` so this
overlay wins over an older `hesai_ros_driver` in `hunter_navigation_arm64` or
another install prefix.  Do not use `/home/robot/hunter_navigation_arm64` or
`/home/robot/old` for this feature; those are older installed copies.

## Standalone collector test

With a local copy of `publish_stats_collector_test.cc`, build against the canonical driver headers:

```bash
g++ -std=c++17 -O2 -pthread \
  -I /home/robot/hesailidar_ws/src/HesaiLidar_ROS_2.0/src \
  /home/robot/web_ws/src/web_control/docs/driver_publish_stats/publish_stats_collector_test.cc \
  -o /tmp/publish_stats_test
/tmp/publish_stats_test
```

Expected output: `publish_stats_collector tests passed`.

## Published interface

Topic: derived from the configured point-cloud topic by appending
`/statistics` (default `/lidar_points/statistics`).  Optional YAML override:

```yaml
ros:
  ros_send_point_statistics_topic: /lidar_points/statistics
```

One `diagnostic_msgs/msg/DiagnosticArray` per second, QoS
`BEST_EFFORT`/`VOLATILE`/depth 1.  It contains one
`diagnostic_msgs/msg/DiagnosticStatus`:

| Field | Value |
| --- | --- |
| `name` | `hesai_lidar_publish` |
| `hardware_id` | configured point-cloud topic |
| `level` | `OK` when `frames_in_window > 0`, else `WARN` |
| `message` | `publishing` / `idle` / `waiting` |

`status.values` (all strings; web parses strictly):

| Key | Meaning |
| --- | --- |
| `publish_rate_hz` | successful point-cloud publish calls / measured window seconds |
| `window_sec` | elapsed driver monotonic time for this report window |
| `frames_in_window` | successful publish calls in this window |
| `total_frames` | successful publish calls since driver process start |
| `last_publish_age_sec` | seconds since the last successful publish, or `-1` when none |
| `source_topic` | configured point-cloud topic |
| `session_id` | driver-process start timestamp (nanoseconds); changes on restart |
| `sequence` | per-session heartbeat sequence; duplicates/older values are ignored |
| `state` | `publishing` / `idle` / `waiting` |

`publish_rate_hz` is the driver-side **application publish-call rate**.  It is
not guaranteed DDS delivery, not the subscriber's received rate, and not the
LiDAR hardware RPM.  `ros2 topic hz` measures a subscriber's receipt rate;
QoS, DDS/network loss, load, serialization, and discovery can make the two
differ.

## Runtime behavior

- The existing SDK `SendPointCloud` path stores a monotonic timestamp and
  increments an atomic counter after `pub_->publish(...)` returns.  It performs
  no point-cloud copy, XYZ processing, formatting, logging, or mutex work.
- Device-mode `node_ptr_` is not spun, so the heartbeat is a dedicated
  1 Hz worker thread, not a ROS timer.  It keeps reporting while cloud frames
  stop (`publish_rate_hz: 0`, growing `last_publish_age_sec`).
- One message per second is formatted on the worker only.  The report contains
  no point data.
- `Stop()` stops the SDK/parser first, then joins the statistics worker and
  releases its publisher before node destruction. Stop is idempotent because
  `NodeManager` calls it during both normal shutdown and destruction. Starting
  the hardware again uses a new driver process; the existing SDK deletes its
  internal LiDAR instance on Stop.
- Each `SourceDriver`/LiDAR instance owns its own topic, counters, session and
  sequence; there is no global cross-device counter.
- If `send_point_cloud_ros` is false, no cloud publisher or statistics
  publisher is created.
