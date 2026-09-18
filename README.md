# DDDMR Web Control

该 ROS 2 包位于 `web_ws/src/web_control`。启动网页控制节点前，先加载建图包和网页工作空间：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/slam_ws/install/setup.bash
source /home/robot/web_ws/install/setup.bash
ros2 launch web_control web_control.launch.py
```

如需从网页启动导航，再在 `web_ws` 之前加载 `navigation_ws/install/setup.bash`。

网页的 **Start** 按钮启动定位、导航和建图 launch 时，只传递各自配置的 `*_launch_args`，不会自动附加 `use_sim`。仿真时间由被启动的 launch 自己决定；若它支持 `USE_SIM_TIME` 环境变量或显式 `use_sim:=...` 参数，这些设置仍可生效。

## 网页保存地图

打开 **Mapping** 页面，启动建图并等待 **Save Map** 按钮可用。点击后，在弹窗中填写**相对于机器人网页节点用户主目录的路径**（默认 `/map`，例如 `/map/site`），选择是否导出地面地图和 tiled 地图，再确认覆盖。网页会把 `/map` 解析为 `$HOME/map`（机器人上通常为 `/home/robot/map`），`/map/site` 解析为 `$HOME/map/site`；请不要填写已经带主目录前缀的绝对路径（如 `/home/robot/map`），也不要使用 `..` 路径段，否则保存请求会被拒绝。

网页调用 `/lio_sam/save_map_options` 服务；网页先按当前运行用户的主目录把填写路径解析成绝对目录，再传给建图节点，分辨率沿用建图节点当前配置。确认后，目标目录中已有的地图产物会被覆盖，其他文件保留。页面会显示保存中、成功或失败的结果。建图服务未启动或已有保存任务进行时，不能提交新任务。
