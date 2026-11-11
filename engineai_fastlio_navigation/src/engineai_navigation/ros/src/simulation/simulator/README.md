# PM01 Gazebo 仿真器

简体中文 | [English](README.en.md)

提供 ROS 2 Humble / Gazebo Classic 的 PM01 模型、DART 物理、Mid360 GPU 射线、IMU 与关节控制。随项目根目录的 `bash scripts/build.sh sim` 构建，统一启动方式见[教程](../../../../../../docs/tutorial.md)。

- `/livox/lidar/pointcloud`：PointCloud2，雷达坐标系 `livox_frame`。
- `/livox/lidar`：Livox CustomMsg，供 FAST-LIO 接入。
- `/livox/imu`：雷达 IMU；`/pm01/imu`：步行策略使用的躯干 IMU。
- `/joint_states`：关节位置与速度。
- `/pm01_sdk_pd_controller/commands`：24 个关节的位置指令，单位为弧度，顺序与模型关节声明一致。

启动先保持站立，控制器激活及策略就绪后交接。平台初始化不代表项目运动输出已经接通。运行参数与资源由项目统一启动入口选择。

来源、版权和许可见[上层说明](../../../../README.md)及[适配说明](../../../../PM01_ADAPTATION.md)。
