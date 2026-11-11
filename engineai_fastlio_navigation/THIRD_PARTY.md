# 第三方来源与许可

简体中文 | [English](THIRD_PARTY.en.md)

根目录 LICENSE 适用于项目自有代码。下列第三方组件采用表中所列许可。

| 内容 | 来源与版本 | 许可入口 | 本项目修改范围 |
|---|---|---|---|
| FAST_LIO_ROS2 | 本仓库 vendored 快照；对应源码随发布提交固定 | [GPL-2.0](src/FAST_LIO_ROS2/LICENSE) | ROS2 接入、仿真辅助估计、诊断、去畸变点云输出及现有恢复检查；演示资料通过上游链接提供 |
| livox_ros_driver2 | 本仓库 vendored 快照 | [上游分文件条款](src/livox_ros_driver2/LICENSE.txt) | ROS2 构建与消息接入；保留原始内部布局 |
| interface_protocol | 本仓库 PM01 消息快照 | [BSD-3-Clause](src/interface_protocol/LICENSE) | 真机消息构建接入 |
| EngineAI Navigation | `7ee29a3ede5ecd7d884a1a53b258cd5a0147bd81`，见 dependencies.repos | [上游 LICENSE](src/engineai_navigation/LICENSE)、嵌入组件许可 | [内置仿真源码适配](src/engineai_navigation/PM01_ADAPTATION.md)：速度话题、步态配置、初始化退出与站立控制交接 |
| Livox SDK2 | 上述公开版本中的树 `bc4597a34a4d0220f6112cb3e7156281499487e4` | SDK `source/LICENSE.txt` | 构建集成，不修改 SDK 算法 |
| SLAM Toolbox / Karto | `8293d21fe5c816d0405e0cc1f3eb46c70659c267`（2.6.10） | [根 LICENSE](src/slam_toolbox/LICENSE) 为 LGPL-2.1；Karto 修改文件声明 LGPL-3.0-or-later；ROS 文件保留上游头部 | [内置源码适配](src/slam_toolbox/PM01_ADAPTATION.md)：局部匹配验收、匹配与地图修订元数据；裁剪未使用的实验后端 |
| Nav2 启动组合 | Navigation2 1.1.20 `navigation_launch.py` | Apache-2.0，见 [保留的文件头](src/pm01_navigation_bringup/launch/nav2_stack.launch.py) | 默认资源路径与项目组合 |
| ROS2 / Nav2 / AMCL / Gazebo | Ubuntu 22.04 / ROS Humble 系统包 | 各包 LICENSE 与系统 copyright 文件 | 使用标准接口；实际版本随镜像包清单交付 |
| MNN 与步行模型 | 上述固定 EngineAI Navigation 版本 | 上游第三方与模型说明 | 仅用于 x86_64 仿真 |

镜像与离线包附带源码、依赖版本、修改说明和许可证。

仿真构建脚本从固定的上游版本自动获取并校验模型网格、步行策略、MNN 库和 Livox SDK。运行所需的资源在构建时安装到对应的包，原始许可随资源保留。

ROS、Nav2、AMCL 和 Gazebo 由 Ubuntu / ROS 软件源提供。机载镜像保留系统包的版权文件。安装版本见交付包中的 `container-packages.tsv`。
