# DDDMR Web Control

启动网页控制节点前，先加载建图包和网页工作空间：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/slam_ws/install/setup.bash
source /home/robot/web_ws/install/setup.bash
source /home/robot/hesailidar_ws/install/local_setup.bash
ros2 launch web_control web_control.launch.py
```

## 网页保存地图

打开 **Mapping** 页面，启动建图并等待 **Save Map** 按钮可用。点击后选择机器人上的**父目录**（绝对路径，可点击 **Browse...** 浏览，默认 `~/map`），填写地图文件夹名称（默认 `hesai_clean`），选择是否导出地面地图和 tiled 地图（默认勾选），再确认覆盖。最终请求的 `destination` 是父目录与文件夹名拼接出的绝对路径，例如 `/home/robot/map/hesai_clean`，并且会原样发送给保存服务：`/map/site` 就是 `/map/site`，网页节点不会拼接主目录。保存服务请求字段为 `destination`、`resolution`、`ground_resolution`、`save_ground_map`、`save_tiled_map`，响应为 `success`、`message`。DUFOMap 由建图配置 `dufomap.enable` 统一控制，保存时仅导出当时的在线结果，网页不提供开关。过期 API 请求若仍带 `use_dufomap` 或旧的 `dufomap_mode`，会被明确拒绝并提示由建图配置控制。

## 网页录制 rosbag

在 **Mapping** 页点击 **开始录制**，勾选需要的话题，选择机器人上已存在且可写的保存目录，并填写数据包文件夹名称。支持手动输入绝对路径或点击 **浏览...** 选择目录，包括挂载磁盘；默认目录为网页节点用户的主目录。点击 **刷新话题** 可重新读取当前 ROS 话题。

录制时页面显示状态和完整保存路径；点击 **停止录制** 后，等待保存完成再使用数据包。刷新网页不会中断录制，页面会恢复当前状态。同名数据包不会被覆盖。`recording_root` 参数可限制 rosbag 与地图保存共用的目录浏览/保存范围，默认为 `/`。

更新后需要重启网页节点并强制刷新浏览器，才能同时加载后端接口和页面。当前工作空间采用 symlink install，源码修改无需重新构建；重新安装时应确保已安装 `rosbag2` 运行依赖。

## 选择地图（Map 选择器）

**Navigation** 页面中的 **Map** 区域用于选择下一次由网页启动的 localization/navigation 使用哪张地图。

- 地图根目录固定为网页节点用户主目录下的 `~/map`。只有在 `~/map` 下直接包含可读的 `GlobalMap.pcd` 和 `GroundMap.pcd` 的子目录才会出现在列表中，网页不会接受任意文件路径。
- 地图内若存在合法的 `tiled/manifest.yaml`（schema version 1，且引用的分块文件都在 `tiled/` 内并存在），localization 会以 tiled 方式加载。Map 卡片中的 **加载方式** 可选 `自动（有分块优先）`、`分块`、`全局`：自动在 tiled 可用时用 tiled、否则回退 `GlobalMap.pcd`；手动 `分块` 在 tiled 缺失或损坏时明确报错，绝不静默降级；`全局` 始终使用完整的 `GlobalMap.pcd`。tiled 清单损坏时该地图仍可选，并显示警告。
- Map 卡片中的 **自动重定位** 按钮控制下一次 localization 启动是否使用 AUTOMATIC RELOCALIZATION（按钮采用与启动/停止日志一致的双态样式，`aria-pressed` 表示实际生效状态）。能力来自所选地图是否包含可读的 `sc_db.bin`：缺少数据库时按钮禁用并给出简短原因，已保存的偏好保留但不生效，不会阻止选择或启动该地图；显式开启不支持的地图返回 HTTP 400。该设置只影响下一次启动，不做热更新。
- 地图名称、加载方式和自动重定位偏好一起持久化在 `~/.config/web_control/selected_map.json`（顶层 `mode` 为 `auto`、`tiled` 或 `global`，缺失/未知值按 `auto` 读取；`auto_relocalization` 为严格 JSON 布尔值，缺失/非法值按 `true` 读取），网页节点重启后仍然保留；更新只修改请求涉及的字段并保留未知键、`ui.joystick` 和 `ui.topic_logging_enabled`。启动 localization 时，网页节点按已解析的方式把 `map_path:=`（tiled 目录或 `GlobalMap.pcd`）传给 launch，并在启动前重新校验地图和 tiled 目录，同时总是显式传入 `auto_relocalization:=true|false`（用户偏好且所选地图有 `sc_db.bin` 时为 `true`，否则为 `false`），覆盖配置中的冲突值且不产生重复参数；不再生成 `~/.cache` 下的参数文件。
- 地图、加载方式和自动重定位的改动只影响**下一次**由网页启动的 localization（加载方式不改变 navigation 的 `GlobalMap` 所在目录语义），不会热重载正在运行的算法。localization 或 navigation 运行期间无法更改这些设置（服务端返回 409），需要先停止相应进程；仅建图运行不阻止选择。仅切换自动重定位不会使已缓存的地图几何失效。
- 点击 **Refresh** 可重新扫描地图目录；保存地图成功后列表也会自动刷新。
- 通过命令行直接执行 `ros2 launch`（例如 `~/bringup.sh`）时仍使用各自配置中的默认地图，不受网页选择影响。
- **Navigation → Layers → global map** 直接订阅 localization 的 `/lio_sam/localization/global_map`（`global_map_topic` 可配置；RELIABLE + TRANSIENT_LOCAL，depth 1，与现有静态地图一致），以按高度高对比着色的小点显示完整全局地图，默认开启，不依赖当前的 `auto`/`tiled`/`global` 加载方式。停止 navigation 不会清除它；localization 重启或切换所选地图时会清空，避免显示上一份全局地图。
- **Navigation → Layers → current lidar** 订阅 localization 的 `/lio_sam/localization/current_lidar`（`current_lidar_topic` 可配置；BEST_EFFORT + VOLATILE，depth 1，也与 reliable 发布者兼容），以与地图点云相同的 2 px 高度着色样式显示当前扫描，默认开启。SLAM 发布前已把该点云变换到 map 坐标并设置 `header.frame_id=map`，因此网页直接复用现有 map-frame 渲染管线，不逐点变换、不伪造原点。该层跟随每个新发布帧并用单一最新待处理槽合并，不套 1 Hz 限制；关闭勾选或页面隐藏时不续期需求，后台不执行重解码。localization 重启或切换所选地图时清空；停止 navigation 不清除该 localization 图层。
- **Navigation → Layers → obstacles** 订阅导航的实时障碍聚合点云 `/aggregated_pc`（`obstacle_topic` 可配置；BEST_EFFORT + VOLATILE，depth 1，不重放旧的 latched 数据），以平面纯红 `#ff0000`、5 px 大点显示（普通地图点为 2 px）。空点云会立即清空该层；停止/重启 navigation、localization 重启或切换地图时也会清除，避免残留旧障碍。该层复用现有 PointCloud2 读取与 TF frame 信息，不修改建图页面或定位模式。
- **点云高度高对比着色（仅浏览器）**：Navigation 的 `global_map`、`local_map`、`current_lidar`、`mapcloud` 与 Mapping 的 `global_map` 按每条新点云的 ROS `z` 自动取有限高度的最小/最大值并归一化；低高度区域（归一化 0–1.5%）保持蓝色，3% 即转为饱和黄色，20% 橙色，45% 红橙，75% 红紫，100% 高品红。这样低处与略高处、最高处与近最高处都有明显色差。这里的“低高度”只表示当前点云内的相对高度，不是地面匹配或地面估计；坡道/斜坡的地面点 Z 可能高于该点云的最低点。相同高度、空点云、单点或全非有限高度安全回退到低高度蓝色；该蓝色只是有效显示颜色，不表示这些点在实际地面上。`mapground`、Mapping 的 `ground` 和红色障碍层保持原有纯色。颜色在创建新点云时一次性计算（仅新增每点 RGB 顶点缓冲），不改路径/车辆、点大小、透明度、深度、后端接口或轮询频率。
- **浏览器刷新（SSE 版本通知）**：`static/app.js` 只对当前可见 Navigation/Mapping 页面保持一个原生 `EventSource`，由 `/api/cloud/events` 的版本/状态元数据驱动点云 GET；同一流还携带已配置 path/轨迹的轻量版本元数据和很小的车辆 pose 内联数据（不携带大数组）。每层/每条 path 最多一个在途请求，新版本到达时只在上一次完成后拉取最新一次，不排队、不按固定 1/5 Hz 或 100 ms 轮询。版本未变化的重复通知不会重复下载。Navigation 的 `global_map`、`mapcloud`、`mapground` 首次非空有效点云后缓存（真实地图/lifecycle 重置产生新版本时才重新加载）；`local_map`、`current_lidar`、`obstacles` 和 Mapping 的 `global_map`/`ground` 跟随发布帧。paths/trajectory 和车辆位姿改为收到新消息才更新；路径数组仍只通过现有 GET 获取，pose 直接在 SSE 内联以避免每个里程计帧一次 HTTP。切换标签/勾选图层会先关闭旧流再按新选择重连，隐藏页面关闭流；断线由 EventSource 有界重连并重新获取 snapshot，通知通道不可用时回退到 1 s 有界轮询。实际切换地图、localization 运行状态变化会递增 generation 并使旧响应失效，navigation 生命周期仍清除其拥有的图层/路径。

## 预测轨迹（/best_trajectory）

Navigation 3D 视图显示局部规划器发布的预测轨迹 `/best_trajectory`。该话题是 `geometry_msgs/PoseArray`，与 `path_topics` 中的 `nav_msgs/Path` 话题类型不同，因此由独立参数 `trajectory_topic`（默认 `/best_trajectory`）订阅，设为空字符串可关闭。提取每个位姿的 `position` 后，它与普通路径共用同一份缓存、`/api/path` 接口和 SSE 版本刷新，并显示为琥珀色，以便和蓝色的全局路径 `/global_path` 区分。Layers 中新增 **best trajectory** 勾选框，与其它话题勾选一致：取消后立即隐藏该轨迹、从 SSE 订阅中移除并且不再请求该话题，重新勾选后只拉取最新版本。局部规划器只在 `debug_publish.best_trajectory` 为 `true` 且存在订阅者时发布，网页节点启动后即满足订阅条件。

## 可调双摇杆布局（Mapping）

**Mapping** 页面地图视口下方有两个独立的虚拟摇杆：左下角 **throttle** 摇杆只识别垂直方向（前/后），右下角 **steering** 摇杆只识别水平方向（左/右）。两个摇杆各自独立拖动、独立缩放，并支持鼠标/触控的双指针同时操作：

- 每个摇杆只改变自己的轴：throttle 摇杆的横向位移被忽略，steering 摇杆的纵向位移被忽略；中心死区内的值归零。两个摇杆的当前值合并为同一个既有 `/api/teleop` 请求 `{throttle, steering, client_id, seq}`（后端遥操作契约不变）。松开或取消一个摇杆只把该轴归零并立即发送剩余轴的合并状态；两个都松开时发送零速。10 Hz keepalive 发送循环与 0.4s 后端看门狗保持不变。
- 地图视口右上角有一个共享的小齿轮图标进入布局模式（`aria-label`/`title` 为 “Joystick settings”，带 `aria-pressed`）；进入时先无条件请求一次 teleop 零速停止并结束发送循环，布局模式下车速控制被禁用，共享的勾选图标确认并退出布局模式、恢复车速控制。若在等待停止返回期间切走标签页或点击勾选，进入流程不会再恢复布局模式。
- 布局模式下，每个摇杆上方有移动手柄，右下角有缩放手柄；两者使用 Pointer Events 与 pointer capture，事件不会传递到 Three.js 画布，且可聚焦并带 `aria-label`。摇杆容器盒就是底座本身，移动/缩放控件绝对定位、不参与布局，因此布局模式预览的几何与按勾选确认后应用的几何完全一致。
- 拖动缩放手柄实时改变**该摇杆**底座直径，范围 `140px`–`260px`，未保存过设置时默认 `180px`；两个摇杆尺寸互不影响。旋钮与轴线通过各自的 `--stick-size` 缩放。
- 拖动移动手柄或缩放手柄时只在浏览器内实时预览，指针释放或取消时各写入一次后端（每个完整手势一次保存）。位置以**稳定的视口像素偏移** `x`、`y`（相对地图视口左上角）保存，而不是按比例归一化。侧边栏折叠/展开只改变视口大小：只要保存的像素位置仍在视口内就完全不动，只有真正越界时才夹取；夹取只影响本次显示，不会回写或重定基准保存值。视口/侧边栏过渡期间不写入后端。默认位置为左下（throttle）与右下（steering）。
- 只有最新一次保存或读取的响应能应用到界面：新手势或新的保存会使在途响应失效，较早返回的旧尺寸/位置不会覆盖当前预览。
- 摇杆设置保存在 `~/.config/web_control/selected_map.json` 的 `ui.joystick` 下，例如 `{"name": "hesai_sim2", "ui": {"joystick": {"throttle": {"size": 180, "x": 12, "y": 408}, "steering": {"size": 180, "x": 708, "y": 408}}}}`。`null` 偏移表示使用该摇杆的角色默认位置。旧版单摇杆格式 `{"size": ..., "x_ratio": ..., "y_ratio": ...}` 会被迁移：保留其尺寸作为两个摇杆的尺寸，位置回退到默认值。旧格式 `{"name": "..."}` 仍然可用；地图选择只更新 `name`，摇杆设置只更新 `ui.joystick`，两者都不会覆盖未知键。读取时单个非法字段（非有限、负数、越界或超大整数）只回退该字段。
- 设置对该 web-control 服务是全局的：所有浏览器加载同一份布局，最后一次完成的调整生效。
- 后端接口：`GET /api/ui/preferences` 返回校验后的双摇杆设置；`POST /api/ui/preferences` 接受完整对象 `{"throttle": {"size": ..., "x": ..., "y": ...}, "steering": {"size": ..., "x": ..., "y": ...}}`，校验并原子合并后返回保存结果。非法、非有限、负数或越界值（含超大整数）返回 HTTP 400 且不修改文件；保存失败时通过 `#joystickSettingsError` 错误表面显示并在布局模式中保留当前预览，可继续调整后重试。teleop 运行时错误也通过同一可访问错误表面显示，不再有摇杆下方的 idle/active/error 状态文字。
- HUD 默认停在左下角；Mapping 激活时两个下角都被摇杆占用，HUD 停在两者之间的底部中央，其他标签页沿用原有 HUD 位置。

## 折叠右侧信息栏

- 地图区域与右侧信息栏交界处（垂直居中）有一个切换按钮；折叠与展开复用同一个 DOM 按钮，只有箭头方向与 `aria-expanded` / `aria-label` 变化。
- 折叠时网格中的侧边栏轨道在约 `650ms` 内过渡到 `0px`，面板内容在 `480ms` 内淡出并轻微右移；展开时反向播放，切换按钮跟随移动的边界，不会跳到新位置。
- `.app` 在折叠/视图动画期间裁剪水平方向由面板位移产生的绘制/滚动溢出（优先 `overflow-x: clip`，回退 `overflow-x: hidden`），面板自身的垂直滚动仍由 `.panel` 内部承担。
- 地图区域上的 `ResizeObserver` 与窗口 resize 在布局过渡期间只标记“待刷新”，不直接重建；当地图视图正处于会改变网格的侧栏/视图过渡时，渲染循环每帧在更新相机与绘制之前调用一次带缓存的 `resizeAll()`，保证绘制缓冲区与相机投影同帧校正、旧画面不被拉伸；过渡结算时仍执行最终 `resizeAll()` 校正，摇杆布局与 HUD 位置只在结算帧更新。纯视图过渡与降低动态模式下仍走原有的下一帧结算路径。
- 折叠后 `#controlPanel` 设为 `inert`，其中的控件不可聚焦或点击；展开时恢复。
- 折叠状态只保存在当前页面内存中，不写入后端、`sessionStorage` 或 `localStorage`；每次页面加载都从展开状态开始。
- **System** 标签页与侧栏折叠共用同一个有效折叠值（`sidebarState.collapsed || state.view === "system"`）和同一段 `--panel-width` 过渡，不再用 display 隐藏面板；侧栏切换按钮在 System 中淡出并置为不可点击，返回 **Navigation** 或 **Mapping** 时恢复当前页面内的折叠状态并显示同一个按钮。

## Liquid Glass 页面过渡

视图切换与侧栏折叠共用同一套布局过渡状态机，玻璃材质分三档，只由 `static/liquid-glass.js`（档位判定）、`static/styles.css`（材质与动效）和 `static/app.js`（选择与持久化）实现，不新增依赖。

- **三档材质**：`glass-full` 下左上角 `#viewTabs` 整条标签轨道使用按当前 border-box 尺寸与 DPR 动态生成的 `url(#liquidTrackFilter)`（`R`/`G` 通道位移、中心与丸形外部中性、边缘沿圆角矩形解析法线向内折射：长边只动 `G`、端帽只动 `R`），并叠加 `blur(0.25px) contrast(1.2) brightness(1.05) saturate(1.1)`；模式滑块 `#modeSegment::before` 与侧栏切换镜片继续使用静态 `url(#liquidDrop)`。`glass-light` 为轨道提供一层强透明 `blur()`/`saturate()`，其余结构面保持一层 `blur()`；`glass-off` 使用不透明高对比度颜色。半透明模式下车图视图是深色画布，未选中标签继续用浅色 `--track-glass-label`；System 视图位于自身浅色/深色表面，其未选中标签在三种档位下都用专用的 `--track-system-label`（浅色方案 `#111827`、深色方案 `#f5f7fb`）。WebGL 画布、终端/日志文本、密集数据、实时摇杆旋钮和安全操作按钮不做位移或背景模糊。
- **手动 Glass Strength**：**System** 视图提供 **Glass Strength** 选择器，只有 `Full`、`Light`、`Off` 三档，未保存过选择时默认 `Full`。选择保存在浏览器的 `localStorage`（键 `web_control_glass_strength`），刷新后仍然生效，并且按浏览器/设备各自独立、不写入后端。该设置只决定材质，不会因为浏览器品牌或能力探测而自动把 `Full` 改成 `Light`；浏览器不支持某个 CSS 效果时按 CSS 自身规则自然退化。
- **档位优先级**：运行时/手动的 System 选择（`window.__webControlSetGlassMode(...)` 或 System 选择器）→ 显式 `?glass=` URL 参数 → 浏览器保存的有效选择 → 默认 `full`。档位完全由显式选择决定，辅助功能的降低透明度设置不会改变档位，也不再依据 Chromium、`navigator.connection.saveData` 或 `CSS.supports` 等浏览器身份/能力自动选择档位。
- **`?glass=` 覆盖**：URL 参数 `?glass=full`、`?glass=light`、`?glass=off`，或运行时 `window.__webControlSetGlassMode("full" | "light" | "off")` 可强制档位。URL 参数对当前页面加载确定且优先于已保存的选择，但不写入 `localStorage`；在 System 选择器里做出的选择会持久化并优先于 URL 参数，直到下次重新加载。
- **降低动态**：`prefers-reduced-motion: reduce` 把 `--layout-slide-ms`、`--fade-ms`、`--thumb-ms`、`--joystick-reveal-ms`、`--joystick-hide-ms` 和 `--glass-drop-ms` 压到 `1ms`、关闭关键帧与背景动画，状态机在下一帧直接结算；该媒体查询只影响动画时长，不改变 `Full`、`Light`、`Off` 档位。隐藏视图与折叠面板使用 `inert` 和 `aria-hidden`，焦点移到可见的标签或侧栏按钮，`:focus-visible` 为 2px 焦点环，粗指针设备点击目标不小于 44px。
- **摇杆显隐生命周期**：进入 **Mapping** 时摇杆层先以 `opacity: 0` 挂载并按最终尺寸与像素位置完成一次测量，再在 `--joystick-reveal-ms`（`420ms`）内淡入；离开 **Mapping** 时立即停止 teleop 并禁用指针交互，随后在 `--joystick-hide-ms`（`220ms`）内淡出，最后才设为 `hidden`。`--layout-ease: cubic-bezier(0.4, 0, 0.2, 1)` 为页面滑动、淡入淡出和摇杆显隐提供无过冲的收尾；快速切换标签页时用同一令牌作废过期的 `requestAnimationFrame`、隐藏定时器与结算回调，过期的显示帧不会把已隐藏的摇杆重新显示。显隐状态只存在当前页面内存，摇杆尺寸与位置持久化行为不变。
- **挂载式 System 过渡**：`#view3d` 与 `#viewSystem` 始终挂载，通过 `opacity`/`visibility`/`pointer-events`/`inert` 交叉淡入淡出而不是 `display` 切换；进入 System 与手动折叠共用同一个有效折叠值和 `--panel-width` 网格过渡，离开 System 时恢复页面内折叠状态。
- **无白闪的实时 resize**：`ResizeObserver` 与窗口 resize 在布局过渡期间只标记待刷新（结算时做最终校正）。当地图视图处于会改变网格的过渡（`pageTransition.active && expectsGrid`）且未降低动态时，渲染循环每帧在 `updateCamera()` 与 `renderer.render()` 之前调用带缓存的 `resizeAll()`——它仍是唯一的 `renderer.setSize` 调用点——因此绘制缓冲区与相机投影总在同一动画帧内先校正再绘制，旧缓冲区不会被拉伸到过渡结束。摇杆布局与 HUD 位置仍只在结算帧更新，纯视图过渡与 `prefers-reduced-motion: reduce` 仍走下一帧结算路径。画布清屏色 `setClearColor(0x0b0f19, 1)` 与 CSS `--scene-bg` 都匹配 `scene.background`；地图视图隐藏时暂停渲染循环，恢复时先同步渲染一帧。
- **动态与静态位移贴图**：标签轨道在 `glass-full` 下于 `#liquidGlassDefs defs` 注入 SVG 滤镜 `#liquidTrackFilter`（`feImage` + `feDisplacementMap`），贴图按 `#viewTabs` 的 border-box 尺寸与当前 DPR 逐像素生成（`R`/`G` 通道分别编码水平/垂直位移，中性值 `128`，中心与丸形外部中性，边缘沿解析圆角矩形法线向内折射：长边只动 `G`、端帽只动 `R`，`feDisplacementMap` 的 `scale` 以 CSS 像素计），由 `ResizeObserver`、窗口 resize 与 DPR 变化合并刷新，尺寸/DPR 未变时去重；初始化会先移除所有同名 `#liquidTrackFilter` 节点，避免残留的不完整滤镜留下重复 id；模式滑块与侧栏镜片继续使用 `static/liquid-glass.js` 中一次性内嵌的静态 `128x64` `LIQUID_DROP_MAP`。
- **上游改编与许可**：动态轨道位移生成器改编自 [shuding/liquid-glass](https://github.com/shuding/liquid-glass)（MIT，Copyright (c) 2025 Shu Ding）的 `smoothStep`/`roundedRectSDF`、Canvas R/G 贴图与 `feImage` + `feDisplacementMap` 技术，完整许可见根目录 `THIRD_PARTY_NOTICES.md`；静态 `LIQUID_DROP_MAP`、`#liquidDrop` 滤镜标记、能力探测、CSS 与过渡状态机仍为独立实现。

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

## 持久化启动日志

所有由网页启动的 6 个受管目标（`chassis`、`imu`、`lidar`、`localization`、`navigation`、`mapping`）的实时日志在保留内存环形缓冲（`managed_log_lines`，默认 400 行）之外，会同步逐行写入磁盘文件，因此网页刷新、进程退出或 `web_control` 节点重启后仍可查看。

- 日志根目录由 ROS 参数 `managed_log_dir` 指定，支持 `~` 展开；留空时默认 `~/.local/state/web_control/logs`（若设置了绝对的 `XDG_STATE_HOME`，则使用 `$XDG_STATE_HOME/web_control/logs`）。
- 目录结构为 `<root>/<target>/<YYYYmmdd-HHMMSS>-<session>.log`，按目标与启动时间+唯一标识分文件；同一次启动不会创建空的新文件，不同次启动也不会互相覆盖。
- 每条记录依次为命令、stdout/stderr、失败与退出信息，并带启动时间头。底盘的两条 CAN 前置命令和随后的 ROS launch 共用同一个会话文件。
- 行内容沿用实时日志的 ANSI 控制符清理，写入时同步 flush；正常停止或进程退出后立即关闭文件句柄。
- 写入失败（目录不可创建、磁盘满等）不会终止已运行的设备进程，也不会留下 reader 子进程；错误会通过 `/api/logs` 的 `sessions[<target>].persist_error` 与 System 页的 Saved Logs 状态明确显示，不会静默声称保存成功。文件资源在会话结束时释放。
- **System → Saved Logs** 卡片选择目标/运行会话后可刷新列表、查看有界 tail 或下载完整文件。服务端只接受受管 target 与合法 session id，拒绝任意路径、`..` 穿越和指向日志根之外的符号链接；下载按块流式发送，不会把大文件整体读入内存。日志不会放入 `static/` 目录。
- 实时 API `GET /api/logs` 的既有字段保持不变，仅新增 `sessions`（每个目标的持久化会话/错误元数据）与 `log_dir`。持久化专用接口：`GET /api/logs/sessions?target=...`、`GET /api/logs/tail?target=...&session=...`、`GET /api/logs/download?target=...&session=...`。

## 话题监控与轮速里程

**System → 话题与运动** 面板与后台计时器持续监控 IMU/底盘里程计，并从 Hesai 驱动发布的轻量 `diagnostic_msgs/DiagnosticArray` 心跳获取 LiDAR 的**驱动发布频率**，不依赖浏览器是否打开。默认不接收原始 `PointCloud2`，避免日常运行时持续复制完整点云；只有显式 opt-in 时才额外统计网页端实际接收频率。驱动侧接口字段、QoS 与启用步骤见 [Hesai 驱动发布统计](docs/driver_publish_stats/README.md)。

- `diagnostics_imu_topic` 默认 `/imu`（FDI `sensor_msgs/Imu`）。监控订阅统一使用 `BEST_EFFORT`/`VOLATILE`/depth 10，可同时接收 reliable 与 best-effort 发布者；仅额外显示最后一次加速度与角速度，作为辅助观察，IMU 单独不能证明运动。数据过期或不可用时明确标注“最近一次/不可用”，不伪装成实时观测。
- `diagnostics_lidar_topic` 默认 `/lidar_points`（Hesai `sensor_msgs/PointCloud2`）。`diagnostics_lidar_stats_topic` 默认 `/lidar_points/statistics`，由驱动按点云话题派生（点云话题 + `/statistics`）。网页默认只订阅这个约 1 Hz 的小消息，QoS 为 `BEST_EFFORT`/`VOLATILE`/depth 1；回调再按状态名 `hesai_lidar_publish` 和 `source_topic`（必须等于配置的点云话题）过滤，其他 `DiagnosticArray` 发布者不能覆盖 LiDAR 行。到达新鲜度只用网页单调时钟，数值字段要求有限、非负且窗口大于 0；坏消息/无关消息只计入 `invalid_count`，不崩溃、不伪造 0 Hz、不刷新新鲜度。超过 `diagnostics_lidar_stats_stale_timeout_sec`（默认 3.0 s）未收到有效统计时显示 `统计过期`，实时 Hz 为 `null`，历史 `last_publish_hz` 仅作离线参考。
- `publish_rate_hz` 是驱动**成功调用点云 `publish()` 的应用层频率**（按驱动自己的单调时钟窗口统计），不是 DDS 必达、不是订阅端收到频率，也不等于雷达硬件转速。`ros2 topic hz` 与网页原始接收 `received_hz` 是订阅端口径；QoS、DDS/网络丢包、系统负载、executor/序列化延迟都会让两者不相等，差值是传输/负载现象，不能反推硬件 RPM。
- `diagnostics_lidar_receive_enabled` 默认 `false`：不创建 `PointCloud2` 订阅，仍保留现有约 1 Hz 的 ROS graph 发布者计数（`publisher_only`/`no_publisher`/`unknown`）。旧驱动不发布统计话题时继续显示“未测频（有发布者）”，Hz 保持 `null`，不会把发布者存在或缺失的统计当成 0 Hz。需要手动诊断**网页实际接收频率**时才将该参数设为 `true` 并重启；开启后恢复 `raw=True`、`BEST_EFFORT`/`VOLATILE`/depth 1 的接收计数，不做 Python 点云反序列化或点处理，但仍存在 DDS/网络字节复制开销，并非零拷贝。此时 `received_hz` 单独暴露，主显示仍优先驱动发布频率；默认关闭的实际 CPU 节省比例未在机器人上测量，本仓库不作百分比承诺。
- `diagnostics_chassis_odom_topic` 默认 `/odom1`（Hunter `nav_msgs/Odometry`，50 Hz）。运动状态、速度方向与里程只来自标准嵌套字段 `twist.twist.linear.x`（`geometry_msgs/TwistWithCovariance.twist.linear`）实测轮速；`/odom` 监控订阅同样使用 `BEST_EFFORT`/`VOLATILE`/depth 10，不影响现有可视化/控制订阅的 QoS。不发布/监听 `cmd_vel` 作为里程，不使用纯 IMU 推算，也不依赖 LIO-SAM。
- 每个话题显示发布者数量（约 1 Hz 查询）、频率及来源、最后数据年龄，以及 `无发布者`/`等待数据`/`正常`/`发布暂停`/`统计数据过期`/`数据过期`/`不可用` 状态。表格列由“接收 Hz”改为“频率（来源）”：驱动统计新鲜时显示 `10.0 Hz · 驱动发布`，驱动心跳仍在但点云发布为 0 时显示 `0.00 Hz · 驱动发布（当前无发布）` 并标为“发布暂停”，统计过期显示 `--`/“统计过期”，旧驱动或无统计时保持“未测频”，无发布者/未知仍分别显示。原始 opt-in 接收正常时显示 `9.50 Hz · 网页接收`；不会把驱动发布频率写成网页接收频率。新鲜 0 Hz 与 `null`/过期/无发布者/未知严格区分，缺少统计时不伪造 0。
- 里程采用相邻有效消息的实测速度绝对值积分，支持前进/倒车：同向时用 `dt*(|v0|+|v1|)/2`，跨零反向时用 `dt*(v0²+v1²)/(2*(|v0|+|v1|))`（线性变化假设），不会把 +1→-1 的反向区间抵消为零；时间基准使用 header stamp，过期判断使用单调到达时钟，stamp 为 0 也按有效值显式处理。NaN/Inf、时间戳非递增、超过 `motion_max_gap_sec`（默认 0.5 s）的空档、frame 变化、异常高速以及网页受管底盘启动/退出边界都会中断积分并重置基线，不跨丢失时间补里程；会话累计值保留并标记不完整。
- 运动状态使用 0.05/0.02 m/s 启动/停止阈值与 0.3 s 去抖。从未收到里程计时累计里程为 `null`（界面 `--`）；收到后按**本次网页节点会话的轮速估计**显示累计/本段距离。数据过期时状态为未知、积分暂停，已累计值保留并明确标记 paused/incomplete，绝不把数据丢失称为静止。浏览器刷新不会重置服务端累计值。
- 后台约 1 Hz 采样，每 5 s 写一次快照，并在状态变化、运动开始/停止/中断、启动和关闭时追加 JSONL 记录；启动/快照/关闭记录以及 `topic_status` 状态转换行包含 `lidar_receive_enabled`/`receive_enabled`，LiDAR 行还包含 `hz_source`/`rate_source`（`driver_publish`/`received`/`null`）、`publish_hz`、`received_hz`、`stats_status`、`stats_age_sec`、`driver_window_sec`、`driver_window_count`、`driver_total_frames`、`driver_cloud_age_sec`、`driver_session_id`、`driver_sequence`、`driver_state`。默认模式下 `received_count`/`last_received_stamp` 只由真实点云接收更新；统计心跳不会增加它们。关闭时先排空自上次计时器以来积压的话题/运动事件，再写入包含完整 topics/motion/IMU 快照及最终速度、本段与累计里程、有效性的 shutdown 记录，然后关闭文件。受管进程先停止、后关闭诊断日志，底盘退出生命周期事件不会丢失。日志位于 `<managed_log_dir>/diagnostics/<session>.log`（默认根目录 `~/.local/state/web_control/logs`），沿用受管日志目录；每日或 10 MiB 轮转，仅清理 `diagnostics` 目标下已完成且超过 7 天或令已完成总量超过 100 MiB 的旧诊断文件，不删除活动文件或任何硬件进程日志。写入失败通过 `GET /api/diagnostics` 的 `logging.error` 和 Saved Logs 显示，不中断遥操作或监控。
- **Navigation / Mapping → Startup → 开始/停止记录话题** 两处按钮只控制诊断 JSONL 的话题/IMU 数据写入，默认开启以兼容旧行为；两页共享同一权威状态，按钮在首次读取成功前及保存期间同时禁用，标签、颜色和 `aria-pressed` 同步。关闭后所有快照、启动/关闭记录、`topic_status` 行以及运动事件中的 `topic_hz`/`topic_age_sec`/`publisher_count`/`topic_status`、`publish_hz`/`received_hz`/`hz_source`/`rate_source`/`stats_status`/`driver_*` 等话题元数据和 IMU 观测都不再持久化，运动状态/速度/来源/里程/时长/有效性仍每 5 s 及状态转换时写入。实时频率显示、接收监控、运动统计与 Mapping 的 rosbag 录制完全独立。选择由后端保存到共享 `selected_map.json` 的 `ui.topic_logging_enabled`，浏览器刷新、其他客户端和网页节点重启后保持；`GET /api/diagnostics` 返回 `topic_logging_enabled`，`POST /api/diagnostics/logging` 仅接受严格 JSON `{"enabled": true|false}`（拒绝字符串/数字/null/未知字段），保存失败不会翻转当前生效值。
- `GET /api/diagnostics` 返回话题、IMU 辅助观察、运动/距离有效性及日志状态；新增 `lidar_stats_topic`、`lidar_stats_stale_timeout_sec`、`lidar_stats_available`，LiDAR 行返回 `driver_stats` 块及扁平别名 `driver_publish_hz`、`driver_window_*`、`driver_total_frames`、`driver_cloud_age_sec`、`driver_stats_age_sec`、`driver_session_id`、`driver_sequence`、`driver_state`、`publish_hz`、`received_hz`、`hz_source`、`rate_source`。界面只在运动有效时显示“当前速度”，暂停/过期时保留历史累计里程并明确标注，IMU 显示为最近一次观测及年龄。Saved Logs 新增 `话题与运动日志` 类别，沿用现有 tail/download 接口。既有 `/api/logs` 与六个受管启动目标不变。
- 参数集中在 `web_control` 节点的 `diagnostics_*` / `motion_*` 参数段；`diagnostics_lidar_receive_enabled` 默认为 `false`，没有新增界面开关。修改后需重启 `web_control` 网页节点并强制刷新浏览器。重启网页节点会通过现有生命周期停止其管理的驱动进程；之后请在网页界面重新启动所需驱动。非由网页节点启动的驱动不受影响。本功能不会启动或移动任何硬件；真实数据频率与时间行为必须在目标机器上确认，本仓库测试使用合成时钟与消息。
- **运行激活**：统计发布端改动已应用到 canonical 驱动仓库 `/home/robot/hesailidar_ws`。构建完成后，启动网页前先 source `/home/robot/hesailidar_ws/install/local_setup.bash`，确保加载的是这次构建的 `hesai_ros_driver`，而不是 `hunter_navigation_arm64` 等旧安装副本；再重启驱动与网页节点。可用 `ros2 topic echo /lidar_points/statistics`（只读观察）确认状态名/字段。本次实现没有在运行中的机器人上启动、重启或发布任何节点。

## 发布者驱动的点云/路径/位姿刷新（无新增面板）

点云订阅回调不再在单线程 ROS executor 内同步解析全部点。每个图层只保留**最新一个待处理帧引用**，由单个后台线程（`web_control_pointcloud`）完成 NumPy 二进制解析、NaN/Inf 过滤和 3D 降采样；解析不持有 `node.lock`。路径/轨迹和车辆位姿复用现有订阅与 SSE 元数据路由，不再由浏览器固定频率轮询。这不等于零影响：worker 仍与节点共享 Python GIL、CPU 和分配器，HTTP 响应序列化、SSE 元数据与路径快照本身也仍是独立工作；实际控制/导航影响需在目标机器部署后测量。

- **动态图层跟随发布帧**：`local_map`、`current_lidar`、`obstacles` 和 Mapping 的 `global_map`/`ground` 不再有 1 Hz/5 Hz 输出上限。发布者每到达一帧就触发处理；解码忙时只用新帧替换唯一待处理槽，不排队。因此频率在机器跟得上时跟随输入，过载时会丢弃中间帧，**不保证每一帧都显示**。三个 latched navigation 地图（`global_map`、`mapcloud`、`mapground`）仍是静态地图：每个新输入最多按 1 Hz 处理一次，正常使用中只加载一次。
- **SSE 更新通知（含轻量位姿）**：现有 threaded HTTP 服务使用 `GET /api/cloud/events?group=nav|mapping&layers=...&paths=...&pose=nav|mapping`，使用 stdlib `text/event-stream`。每个浏览器只订阅当前页面启用图层、已配置 path/轨迹和 pose；可同时连接多个客户端，各自维护版本游标，事件不会从一个客户端“偷走”。路径名只接受后端已配置话题，绝不会因浏览器请求而新建 ROS 订阅；path/pose 更新不会续期点云需求窗口。事件顶层带不可变的服务器会话 `epoch`，cloud/path 条目只含 `group`/`layer`/`version`/`generation`/`status`/`frame_id`/`count`，pose 条目额外携带很小的 pose 字典；不包含点数组或路径数组。ROS 回调和通知路径不做点云/路径 JSON 序列化，也不在持锁时复制大数组。空闲时约 1 s 心跳同时续期 2 s 的动态点云需求窗口并检测断开。
- **浏览器按版本取数**：`static/app.js` 使用一个 EventSource；每层最多一个在途 `GET /api/map3d` 或 `GET /api/mapping/map3d`，每条 path/轨迹最多一个在途 `GET /api/path` 或 `GET /api/mapping/path`，新版本只在上一请求完成后拉取最新一次，不排队。版本未变化、重复通知、旧代次或已清空事件都不会重复下载/复活旧几何；心跳/状态变化等稀疏元数据仍会传播。快照顺序在节点锁内分配，较旧的并发刷新不能把版本或同版本状态回滚。初始 snapshot 与客户端版本对齐，重连获取最新 snapshot；同一服务器会话的重连不会重新下载已应用版本，而网页服务重启后的新 `epoch` 会重置两侧版本游标并接受更低的计数器。`global_map`/`mapcloud`/`mapground` 首次非空后缓存，不在无新版本时周期重下。隐藏的 path/轨迹和 System 页面停止取数，恢复时只取最新版本；`/api/pose` 只作为通知通道不可用时的有界回退，正常路径不再有 100 ms 位姿定时器，`/api/state`、`/api/mapping/state` 的旧 pose 字段也不会覆盖较新的推送。SSE 不可用时使用有界重连，通知通道不可用时回退到 1 s 有界轮询。
- **按需解码**：`/api/map2d`、`/api/map3d`、`/api/mapping/map3d` 会激活对应图层的短需求窗口（`cloud_demand_ttl_sec`，默认 2 s），并立即返回已有缓存；HTTP 不等待 worker。没有可见 SSE 客户端时 `local_map`、`current_lidar`、`obstacles` 和 Mapping 的 `global_map`/`ground` 停止解码，只保留最多一个最新原始帧引用；之后打开/恢复对应页面时才解码该最新帧。`/api/system` 和 viewer heartbeat 不会驱动点云工作。只有三个 latched navigation 地图在没有浏览器时仍会各处理一次以准备早期缓存，随后释放原始 ROS 大 buffer，只保留受 `max_3d_points_per_layer` 限制的 3D 缓存；所有图层成功、解析失败、代次失效或结果被拒后都会释放 raw frame/临时点列表。
- **序列化缓存**：默认每个逻辑接口只保留**一个当前版本**的已编码响应，多个客户端或未变化数据复用同一字节串，`max_entries` 限制整体数量；显式要求保留多个版本的调用仍受支持。状态变为 `stale`/`waiting`、清空或新帧会改变 key，不会返回旧状态。2D 仍是请求时从 3D 懒投影的旧 API 形状。
- **生命周期**：clear、导航/定位/建图停止、手动切换地图和销毁都会递增代次并丢弃队列/在途结果、待处理原始帧和序列化字节，并通过元数据通知使浏览器拒绝旧代次；空障碍帧清空图层，停止后迟到的帧不会复活旧数据。销毁时先有界停止 worker/SSE hub，再关闭日志/节点。坏点云只记录错误并丢弃该帧，不影响控制。
- **依赖**：`package.xml` 已声明 `python3-numpy`；使用系统现有 NumPy，无需安装新包。SSE 使用 Python 标准库和浏览器原生 `EventSource`，没有新增第三方依赖。若遇到 NumPy 不支持的字段类型，才回退到旧 `sensor_msgs_py` 读取路径；布局损坏会明确报错而不是静默误解码。
- **基准（仅合成数据，非实机 CPU）**：20 万点、5 次迭代下，解码墙钟/进程 CPU 约从 0.237 s 降到 0.0516 s（约 4.59x），`tracemalloc` 峰值约从 33.1 MB 降到 11.8 MB（约 2.8x），输出与旧算法逐点一致。基准脚本为 `benchmarks/pointcloud_benchmark.py`。这是隔离合成数据的解码基准，不是实机/节点 CPU 或持续负载结果；真实收益取决于点云大小、话题频率和是否打开地图页，需在目标机器上确认。

## TF 树

**System** 页新增 **TF Tree** 区域，真实订阅 ROS `/tf`（动态，`BEST_EFFORT`/`VOLATILE`）与 `/tf_static`（`TRANSIENT_LOCAL`/`RELIABLE`），依赖 `tf2_msgs`（已在 `package.xml` 声明）。网页节点只读取，不发布 TF，也不启动任何驱动或演示关系。

- 每个动态边显示 `parent -> child`、`dynamic`、`age`/`stale` 标志与最近更新时间；静态边始终标记 `static` 且不会因时间变旧而失效。
- 树按父/子关系以原生 `<details>` 层级列表渲染，使用 `textContent` 构建 DOM，frame 名不会触发 XSS；无外部图渲染或 CDN 依赖。
- 支持多 root、无 `map` 根、失去连接的 frame、parent 变更、畸形 frame、自环与更长的 cycle：环检测与深度上限避免无限递归，无法放入树的边仍在 "Unplaced edges" 中显示并给出原因；任何 frame 都不会因为缺少 `map` 根而消失。
- 视图隐藏时不轮询；仅在 System 可见时以低频（约 3 s）轮询，并用请求令牌保证快速切页/过时响应不会覆盖当前状态。`GET /api/tf` 返回 `status`、`frame_count`、`trees`、`detached` 等字段；ROS 不可用时返回 `unavailable` 而不是崩溃。
