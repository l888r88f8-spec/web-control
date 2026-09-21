# DDDMR Web Control

启动网页控制节点前，先加载建图包和网页工作空间：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/slam_ws/install/setup.bash
source /home/robot/web_ws/install/setup.bash
ros2 launch web_control web_control.launch.py
```

## 网页保存地图

打开 **Mapping** 页面，启动建图并等待 **Save Map** 按钮可用。点击后，在弹窗中填写**相对于机器人网页节点用户主目录的路径**（默认 `/map/hesai_outdoor`，例如 `/map/site`），选择是否导出地面地图和 tiled 地图，再确认覆盖。

## 选择地图（Map 选择器）

**Navigation** 页面中的 **Map** 区域用于选择下一次由网页启动的 localization/navigation 使用哪张地图。

- 地图根目录固定为网页节点用户主目录下的 `~/map`。只有在 `~/map` 下直接包含可读的 `GlobalMap.pcd` 和 `GroundMap.pcd` 的子目录才会出现在列表中，网页不会接受任意文件路径。
- 地图内若存在合法的 `tiled/manifest.yaml`（schema version 1，且引用的分块文件都在 `tiled/` 内并存在），localization 会以 tiled 方式加载；否则回退使用 `GlobalMap.pcd`。tiled 清单损坏时该地图仍可选，并显示警告。
- 含 `sc_db.bin` 的地图显示自动重定位可用；缺少 `sc_db.bin` 的地图会显示 **Manual initial pose required**，需要在启动后手动发布初始位姿。
- 选择结果持久化在 `~/.config/web_control/selected_map.json`，网页节点重启后仍然保留。启动 localization 时，网页节点把选中的地图直接作为 `map_path:=` 传给 launch（tiled 目录或 `GlobalMap.pcd`），不再生成 `~/.cache` 下的参数文件；缺少 `sc_db.bin` 时同时传入 `auto_relocalization:=false`，要求手动发布初始位姿。
- 选择只影响**下一次**由网页启动的 localization/navigation，不会热重载正在运行的地图。localization 或 navigation 运行期间无法更改选择（服务端返回 409），需要先停止相应进程。
- 点击 **Refresh** 可重新扫描地图目录；保存地图成功后列表也会自动刷新。
- 通过命令行直接执行 `ros2 launch`（例如 `~/bringup.sh`）时仍使用各自配置中的默认地图，不受网页选择影响。

## 暂停与恢复导航（Pause / Resume / Clear）

**Navigation** 页面的 **Target Pose** 区域提供导航目标控制：

- **Pause Navigation**：在当前目标 accepted/executing 时可用。网页节点先把目标保存为 `pausing`，再通过 `cancel_goal_service`（默认 `/p2p_move_base/_action/cancel_goal`，使用零 UUID）取消当前 Action 目标；收到 Action 的 `canceled` 后进入 **Paused**，不会停止 navigation launch。
- **Resume Navigation**：在 **Paused** 且当前所选地图与保存地图一致时可用。网页节点用保存的 x/y/z/yaw/frame 重新发布目标，由导航从机器人当前位姿重新规划；新目标被 accepted/executing 后才删除保存记录。
- **Clear Goal**：取消当前目标或删除已暂停的目标。删除已暂停目标不会调用 Action 服务；取消进行中（**Pausing**）时点击 Clear 会复用同一次取消请求。**Resuming** 期间 Clear 返回 HTTP 409。
- 状态显示为 `Idle`、`Navigating`、`Pausing`、`Paused`、`Resuming` 或 `Clearing`；过渡状态期间会禁用可能重复发布目标的按钮。
- 暂停目标持久化在 `~/.config/web_control/paused_navigation.json`，浏览器刷新或网页节点重启后仍可 Resume。重启时若检测到 active 但无保存目标，显示 **Navigating**，允许 Clear，Pause 禁用。
- 网页节点重启后若 Action 状态数组为空（当前没有任何目标），持久化的 **Pausing**/**Resuming** 会一次性归一为 **Paused**，保存目标保留、可继续 Resume；已处于 **Paused** 的记录保持不变。
- 选择另一张地图会清除上一张地图的暂停目标；取消服务超时返回 HTTP 504，状态保持 **Pausing**，前端继续轮询直到 Action 状态确定。
- `GET /api/state` 的 `navigation_control` 字段是按钮文案与可用状态的唯一权威来源。
