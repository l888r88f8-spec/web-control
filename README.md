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

## 可调摇杆布局（Mapping）

**Mapping** 页面左下角的虚拟摇杆带一个跟随摇杆移动的 **Settings** 按钮：

- 点击 **Settings** 会先无条件请求一次 teleop 零速停止并结束发送循环（即使当前空闲也会发出），随后进入布局模式；布局模式下车速控制被禁用，**Done** 退出布局模式并恢复车速控制。若在等待停止返回期间切走标签页或点击 **Done**，进入流程不会再恢复布局模式。
- 布局模式提供摇杆上方的移动手柄和右下角的缩放手柄；两者使用 Pointer Events 与 pointer capture，事件不会传递到 Three.js 画布，且可聚焦并带 `aria-label`。
- 拖动缩放手柄实时改变摇杆底座直径，范围 `140px`–`260px`，未保存过设置时默认 `180px`；旋钮、十字轴、设置控件、状态文字与缩放手柄都通过 `--joystick-size` 等比缩放。
- 拖动移动手柄或缩放手柄时只在浏览器内实时预览，指针释放或取消时各写入一次后端（每个完整手势一次保存）。位置以视口归一化中心坐标 `x_ratio`、`y_ratio` 保存；摇杆连同右下角外伸的缩放手柄都被夹在可见地图区域内，在移动、恢复、窗口缩放、侧边栏动画与切换标签页时都会重新计算。默认位置靠近左下角，不同分辨率下仍然可用。
- 只有最新一次保存或读取的响应能应用到界面：新手势或新的保存会使在途响应失效，较早返回的旧尺寸/位置不会覆盖当前预览。
- 摇杆设置保存在 `~/.config/web_control/selected_map.json` 的 `ui.joystick` 下，例如 `{"name": "hesai_sim2", "ui": {"joystick": {"size": 180, "x_ratio": 0.05, "y_ratio": 0.75}}}`。旧格式 `{"name": "..."}` 仍然可用并回退到默认值；地图选择只更新 `name`，摇杆设置只更新 `ui.joystick`，两者都不会覆盖未知键。读取时单个非法字段（非有限、越界或超大整数）只回退该字段。
- 设置对该 web-control 服务是全局的：所有浏览器加载同一份布局，最后一次完成的调整生效。
- 后端接口：`GET /api/ui/preferences` 返回校验后的摇杆设置；`POST /api/ui/preferences` 接受完整摇杆设置对象 `{"size": ..., "x_ratio": ..., "y_ratio": ...}`，校验并原子合并后返回保存结果。非法、非有限或越界值（含超大整数）返回 HTTP 400 且不修改文件；保存失败时摇杆界面显示 `Save failed` 并在布局模式中保留当前预览，可继续调整后重试。
- HUD 默认停在左下角；当摇杆区域与左下角重叠时移到右下角，与右下角重叠时回到左下角，其他标签页沿用原有 HUD 位置。

## 折叠右侧信息栏

- 地图区域与右侧信息栏交界处（垂直居中）有一个切换按钮；折叠与展开复用同一个 DOM 按钮，只有箭头方向与 `aria-expanded` / `aria-label` 变化。
- 折叠时网格中的侧边栏轨道在约 `260ms` 内过渡到 `0px`，面板内容同时淡出并轻微右移；展开时反向播放，切换按钮跟随移动的边界，不会跳到新位置。
- 地图区域上的 `ResizeObserver` 在过渡期间调用现有的 Three.js `resizeAll()` 并重新计算摇杆布局，让画布宽高比与分辨率保持同步。
- 折叠后 `#controlPanel` 设为 `inert`，其中的控件不可聚焦或点击；展开时恢复。
- 折叠状态只保存在当前页面内存中，不写入后端、`sessionStorage` 或 `localStorage`；每次页面加载都从展开状态开始。
- **System** 标签页继续使用现有的隐藏面板布局并隐藏该切换按钮；返回 **Navigation** 或 **Mapping** 时恢复当前页面内的折叠状态并显示同一个按钮。

## 暂停与恢复导航（Pause / Resume / Clear）

**Navigation** 页面的 **Target Pose** 区域提供导航目标控制：

- **Pause Navigation**：在当前目标 accepted/executing 时可用。网页节点先把目标保存为 `pausing`，再通过 `cancel_goal_service`（默认 `/p2p_move_base/_action/cancel_goal`，使用零 UUID）取消当前 Action 目标；收到 Action 的 `canceled` 后进入 **Paused**，不会停止 navigation launch。
- **Resume Navigation**：在 **Paused** 且当前所选地图与保存地图一致时可用。网页节点用保存的 x/y/z/yaw/frame 重新发布目标，由导航从机器人当前位姿重新规划；新目标被 accepted/executing 后才删除保存记录。
- **Clear Goal**：取消当前目标或删除已暂停的目标。删除已暂停目标不会调用 Action 服务；本进程内取消进行中（**Pausing**）时点击 Clear 会复用同一次取消请求。网页节点重启后恢复的 **Pausing**、或取消服务已经超时的 **Pausing**，没有可复用的在途取消，Clear 会重新发送一次取消。**Resuming** 期间 Clear 返回 HTTP 409。
- 状态显示为 `Idle`、`Navigating`、`Pausing`、`Paused`、`Resuming` 或 `Clearing`；过渡状态期间会禁用可能重复发布目标的按钮。
- 暂停目标持久化在 `~/.config/web_control/paused_navigation.json`，浏览器刷新或网页节点重启后仍可 Resume。重启时若检测到 active 但无保存目标，显示 **Navigating**，允许 Clear，Pause 禁用。
- 网页节点重启后若 Action 状态数组为空（当前没有任何目标），持久化的 **Pausing**/**Resuming** 会一次性归一为 **Paused**，保存目标保留、可继续 Resume；已处于 **Paused** 的记录保持不变。
- 选择另一张地图会清除上一张地图的暂停目标；取消服务超时返回 HTTP 504，状态与保存记录保持不变以便轮询，同时调度**一次有界后台重试**。若重试仍然不可用/超时/被拒绝/（目标已确认 active 后）匹配不到目标，则状态如实回到 **Navigating** 并保留当前目标（Clear 意图会丢弃暂停记录），浏览器可再次点击 **Clear Goal** 重试。自动重试运行期间，新的 Resume 或目标发布会被拒绝，直到重试租约释放。
- `GET /api/state` 的 `navigation_control` 字段是按钮文案与可用状态的唯一权威来源。
