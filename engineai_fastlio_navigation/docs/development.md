# 配置与接口

简体中文 | [English](development.en.md)

## 配置覆盖

默认值以根目录 `config/` 中的文件为准。

| 要调整的内容 | 默认文件 | 启动参数 |
| --- | --- | --- |
| Nav2 规划、控制与代价地图 | `sim/nav2.yaml`、`hardware/nav2.yaml` | `nav2_params` |
| 障碍点云与实时栅格 | 对应平台的 `obstacles.yaml`、`online_map.yaml` | `obstacle_params`、`map_params` |
| 扫描、建图与旧图定位 | `matching/scan.yaml`、`matching/slam.yaml`、`matching/amcl.yaml` | `scan_params`、`slam_params`、`amcl_params` |
| 雷达网络与驱动 | `hardware/livox_network.json`、`hardware/livox_driver.yaml` | `lidar_config`、`driver_params` |
| 雷达外参与 LIO | `pm01_lidar_tf.yaml`、对应平台的 `lio.yaml` | `extrinsics`、`lio_params` |

参数文件从 `config/` 下选取。使用完整文件作为覆盖配置。LIO 先加载上游基础参数，再加载 `lio_params`。调整障碍高度时同时核对 `obstacles.yaml` 和 `online_map.yaml`。

例如，在仿真电脑复制完整 Nav2 配置，修改副本中 `controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance` 的值为 `0.20` 米。

```bash
cd ~/engineai_tutorials/engineai_fastlio_navigation
mkdir -p ~/.config/pm01_navigation/tuning
cp -n config/sim/nav2.yaml ~/.config/pm01_navigation/tuning/nav2-sim.yaml
nano ~/.config/pm01_navigation/tuning/nav2-sim.yaml
```

结束当前仿真，然后使用副本启动。

```bash
source scripts/env.sh
ros2 launch pm01_navigation_bringup navigation.launch.py \
  profile:=sim mode:=fresh \
  nav2_params:="$HOME/.config/pm01_navigation/tuning/nav2-sim.yaml"
```

在已加载仿真环境的另一个终端运行 `ros2 param get /controller_server general_goal_checker.xy_goal_tolerance`，确认实际加载值。

真机覆盖文件放在机器人 `~/.config/pm01_navigation/docker/`，在同目录 `session.json` 的 `launch_args` 中写入参数名和文件的机载绝对路径，再重启会话。使用自定义部署时，查看 `container.json` 中的 `session_config`。

## 应用接口

这些 ROS 2 接口供上层程序使用，例如让机器人去指定位置、在应用里显示地图，或保存采集到的地图。Action 用于提交任务并获取结果，话题用于发送或接收消息，服务用于调用一次操作并获取回复。

| 接口 | 用途 |
| --- | --- |
| `/navigate_to_pose` · `nav2_msgs/action/NavigateToPose` | 发送目标位置和朝向，获取导航进度与最终结果，也可取消目标。 |
| `/compute_path_to_pose` · `nav2_msgs/action/ComputePathToPose` | 给定目标位姿，获取规划出的路径。 |
| `/initialpose` · `geometry_msgs/msg/PoseWithCovarianceStamped` | 发布机器人在旧图上的大致位置和朝向，供 AMCL 初始化。 |
| `/navigation_map` · `nav_msgs/msg/OccupancyGrid` | 订阅导航使用的二维栅格地图，显示空闲、占用和未知区域。 |
| `/odom` · `nav_msgs/msg/Odometry` | 订阅里程计，读取机器人底座的局部位姿和速度。 |
| `/pm01/save_navigation_map` · `std_srvs/srv/Trigger` | 保存当前地图，获取地图 YAML 路径。 |
| `/pm01/navigation/enable_motion`、`/pm01/navigation/release_motion` · `std_srvs/srv/Trigger` | 分别接通和释放本项目的运动输出。 |

Python 应用使用 `pm01_navigation_client.Navigator`。按任务选择 [Python 客户端示例](../src/pm01_navigation_client/examples/README.md)。
