# LiDAR 发布频率与网页显示

本文说明 Hesai 驱动侧“发布频率”统计、`web_control` 默认订阅方式，以及它与
`ros2 topic hz`/网页原始接收频率的区别。驱动实现位于 canonical 驱动仓库
`/home/robot/hesailidar_ws`；独立 C++ 测试仍保留在
[`docs/driver_publish_stats/`](driver_publish_stats/)，供驱动侧构建/验证使用。

> 网页与 canonical 驱动仓库 `/home/robot/hesailidar_ws` 均已应用改动。
> 构建完成后，按本文第 5 节加载新 overlay；本次未重启运行中的节点。

## 1. 两种频率口径

| 口径 | 来源 | 含义 |
| --- | --- | --- |
| 驱动发布频率 `publish_rate_hz` | 驱动在点云 `publish()` 返回后计数 | 应用层成功调用发布 API 的次数/驱动单调时钟窗口 |
| 网页接收频率 `received_hz` | 网页仅在 `diagnostics_lidar_receive_enabled: true` 时订阅 `PointCloud2` | 网页进程实际收到的消息数/网页单调时钟窗口 |
| `ros2 topic hz` | CLI 自己创建的订阅 | 该 CLI 订阅端实际收到消息的周期统计 |

三者**不需要相等**。QoS 匹配、DDS/网络丢包、发现/启动延迟、系统负载、
executor/序列化延迟都会造成差异。驱动发布频率也不是雷达硬件 RPM，不能用来
反推转速。网页主显示默认采用驱动发布频率；原始接收频率只作为 opt-in 的调试
字段单独暴露，二者不会互相冒充。

## 2. 驱动发布接口

默认话题：配置的点云话题 + `/statistics`，默认
`/lidar_points/statistics`。可在驱动的 `ros:` 段覆盖：

```yaml
ros:
  ros_send_point_cloud_topic: /lidar_points
  ros_send_point_statistics_topic: /lidar_points/statistics  # optional
```

消息类型：`diagnostic_msgs/msg/DiagnosticArray`，约 1 Hz，
`BEST_EFFORT`/`VOLATILE`/depth 1，只有一个
`diagnostic_msgs/msg/DiagnosticStatus`。如果覆盖驱动的话题，必须把网页的
`diagnostics_lidar_stats_topic` 设为同一话题，并保持 `diagnostics_lidar_topic`
等于驱动 `ros_send_point_cloud_topic`（网页会校验 `source_topic`）。

| 字段 | 值 |
| --- | --- |
| `status[0].name` | `hesai_lidar_publish` |
| `status[0].hardware_id` | 点云话题 |
| `status[0].level` | 本窗口有发布为 `OK`，否则 `WARN` |
| `status[0].message` | `publishing` / `idle` / `waiting` |

`status[0].values`：

| key | 说明 |
| --- | --- |
| `publish_rate_hz` | `frames_in_window / window_sec` |
| `window_sec` | 驱动单调时钟窗口实际秒数，必须 > 0 |
| `frames_in_window` | 本窗口成功发布调用数 |
| `total_frames` | 进程启动以来成功发布调用总数 |
| `last_publish_age_sec` | 距上次成功发布的秒数；从未发布为 `-1` |
| `source_topic` | 点云话题；网页要求与 `diagnostics_lidar_topic` 相等 |
| `session_id` | 驱动进程启动标识；变化表示驱动重启 |
| `sequence` | 本会话心跳序号；重复/回退值被忽略 |
| `state` | `publishing` / `idle` / `waiting` |

驱动在既有 `SendPointCloud` 热路径上只做一次原子计数和一次
monotonic 时间戳存储；不复制点云、不做 XYZ/格式化/日志/互斥量。设备模式下
`node_ptr_` 不 spin，因此不能使用 ROS timer；实现使用独立 1 Hz worker，
在点云停止时仍然发送心跳（`publish_rate_hz: 0`、`last_publish_age_sec` 增长）。
停驱动时先停 SDK/parser，再 join worker 并释放统计 publisher，避免与节点销毁
竞争。每个 `SourceDriver`/LiDAR 实例有独立话题、计数、session 与 sequence。
现有 SDK 在 Stop 时销毁内部雷达实例；再次启动硬件应创建新的驱动进程。

## 3. 网页订阅与状态

`web_control` 新增参数：

```yaml
diagnostics_lidar_stats_topic: "/lidar_points/statistics"
diagnostics_lidar_stats_stale_timeout_sec: 3.0
diagnostics_lidar_receive_enabled: false   # 默认仍不接收原始点云
```

- 默认只订阅小 `DiagnosticArray`（`BEST_EFFORT`/`VOLATILE`/depth 1）。回调按
  `status.name == hesai_lidar_publish` 且 `source_topic` 等于配置点云话题过滤；
  其他诊断发布者不会覆盖 LiDAR 行。
- 严格校验：`publish_rate_hz` 有限且 `0..1000`，`window_sec` 有限且
  `> 0..3600`，`frames_in_window`/`total_frames`/`sequence` 为非负十进制整数，
  `frames_in_window <= total_frames`，`last_publish_age_sec` 为 `>= 0` 或 `-1`
  （仅从未发布），`session_id` 非空且有界，`state` 只接受已知值。
- 新鲜度只由网页收到**有效**统计的 monotonic 时间决定。坏消息/无关消息只增加
  `invalid_count`，不崩溃、不刷新 `stats_age_sec`、不产生假 0 Hz。
- 同一 `session_id` 下 `sequence` 不递增的重复/乱序包被忽略；`session_id` 变化
  视为驱动重启，重置序号并保留有界计数（不保存历史数组）。
- 超过超时未收到有效统计：`stats_status: stale`、顶层 `status: stats_stale`、
  实时 `hz: null`，历史 `last_publish_hz` 仅供离线参考。
- 统计从未到达且统计话题无发布者：`stats_status: no_publisher`；有发布者但尚未
  到达：`waiting`。旧驱动不发布统计话题时网页继续使用原始发布者存在性回退，
  显示“未测频（有发布者）”，`hz` 保持 `null`，不会伪造 0。
- 新鲜统计 `publish_rate_hz == 0` 是合法测量：顶层 `status: idle`，界面显示
  `0.00 Hz · 驱动发布（当前无发布）`，与 `null`/过期/无发布者/未知区分。
- 统计数据到达**不会**增加原始 `received_count`、不会设置
  `last_received_stamp`，也不会打开 `PointCloud2` 订阅。

### LiDAR 行主要字段

`GET /api/diagnostics` 的 `topics[]` 中 `key == "lidar"` 的行新增：

| 字段 | 说明 |
| --- | --- |
| `status` | `active` / `idle` / `stats_stale` / `publisher_only` / `no_publisher` / `unknown` / `waiting` / `unavailable` |
| `hz` | 主显示频率；驱动统计新鲜时为 `publish_rate_hz`，过期/无统计为 `null` |
| `hz_source`, `rate_source` | `driver_publish` / `received` / `null` |
| `publish_hz`, `publish_hz_source` | 驱动窗口频率及来源 |
| `received_hz`, `received_hz_source` | 仅在原始 opt-in 接收时的网页接收频率 |
| `stats_status`, `stats_age_sec` | 统计心跳状态与网页侧年龄 |
| `driver_stats` | 完整驱动报告块（窗口、总数、session、sequence、state、cloud age 等） |
| `driver_window_sec`, `driver_window_count`, `driver_total_frames` | 驱动窗口/累计计数 |
| `driver_cloud_age_sec`, `driver_stats_age_sec` | 最后点云年龄、统计年龄 |
| `driver_session_id`, `driver_sequence`, `driver_state` | 重启/去重/状态 |
| `last_publish_hz` | 最近一次驱动报告的历史频率（过期时保留） |
| `received_count`, `last_received_stamp` | 仍是原始点云接收字段，统计心跳不修改 |

顶层还有 `lidar_rate_source` 与 `lidar_publish_stats`；`/api/diagnostics` 还返回
`lidar_stats_topic`、`lidar_stats_stale_timeout_sec`、`lidar_stats_available`。

## 4. 界面与日志

- Navigation/Mapping 的 LiDAR chip 在驱动统计新鲜时显示 `Lidar 发布 10.0 Hz`；
  新鲜 0 Hz 显示 `Lidar 发布 0.00 Hz`；统计过期显示 `Lidar 统计过期`；旧驱动
  或无统计仍显示 `Lidar 未测频 · N 个发布者`。IMU 与底盘 odom 行为不变。
- System → 话题与运动表格列“接收 Hz”已改为“频率（来源）”，按来源显示
  `10.0 Hz · 驱动发布`、`9.50 Hz · 网页接收`、`0.00 Hz · 驱动发布（当前无发布）`
  或 `--`/`未测频`；不会把驱动频率标成网页接收。
- 约 1 Hz 的 ROS graph 发布者计数仍保留（点云话题 + 统计话题），它只证明
  graph 中存在发布者，不是数据流证明。
- 日志 ON 时 snapshot/startup/shutdown/topic_status 包含上述 source/mode/
  counters/window/cloud-age 字段；日志 OFF 在写边界删除 `topics`、
  `lidar_publish_stats`、`lidar_rate_source`、`driver_stats` 及
  `publish_hz`/`received_hz`/`hz_source`/`rate_source`/`stats_status`/
  `driver_*` 等话题字段，运动状态/速度/里程仍照常写入。`topic_status`
  转换行本身携带同一组有界字段，并在 shutdown 排空时同样自包含。

## 5. 激活与验证

1. 驱动侧改动已应用到 canonical 仓库 `/home/robot/hesailidar_ws`。
   需要重建时，使用低优先级单任务构建：
   ```bash
   source /opt/ros/humble/setup.bash
   cd /home/robot/hesailidar_ws
   nice -n 15 env MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
     colcon build --packages-select hesai_ros_driver --executor sequential --parallel-workers 1
   ```
2. 启动网页前 source `/home/robot/hesailidar_ws/install/local_setup.bash`，
   确认 `ros2 pkg prefix hesai_ros_driver` 指向该 workspace，而不是
   `hunter_navigation_arm64` 等旧安装副本。
3. 重启驱动与网页节点（本次实现未重启任何运行中进程）。
4. 只读检查：`ros2 topic echo /lidar_points/statistics`；网页 System 页应显示
   驱动发布频率。需要调试网页实际接收频率时，将
   `diagnostics_lidar_receive_enabled` 设为 `true` 并重启网页节点；默认保持
   `false`，不订阅原始点云。

## 6. 测试与限制

- C++ 独立测试覆盖：初始 waiting、10 Hz 窗口、精确 total、停止后 0 Hz/年龄增长、
  新 session、4 线程并发计数、worker start/stop join、stats 心跳不计入点云。
- Web 测试覆盖：DiagnosticArray 严格解析、无关/坏消息、重复序号、session 重启、
  新鲜 0 Hz、过期、无发布者、旧驱动回退、原始/驱动频率分离、日志过滤、
  fake ROS 订阅（默认无 `PointCloud2`，opt-in 仍有 raw/depth1 QoS），以及
  Node 执行的前端行/chip 渲染。
- 未在真实机器人上运行、未启动/重启任何节点、未做实时 CPU 压测。统计功能有
  每帧两次原子操作和一次单调时钟读取、每秒一个 worker 格式化/发布，
  但**不是零成本**；实际 CPU 百分比未在目标机器测量，不作百分比承诺。
