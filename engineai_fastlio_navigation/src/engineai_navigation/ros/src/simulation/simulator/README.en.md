# PM01 Gazebo simulator

[简体中文](README.md) | English

Provides the PM01 model, DART physics, Mid360 GPU rays, IMU, and joint control for ROS 2 Humble / Gazebo Classic. Built through `bash scripts/build.sh sim` at the project root. See the [tutorial](../../../../../../docs/tutorial.en.md) for unified startup.

- `/livox/lidar/pointcloud`: PointCloud2 in the lidar frame `livox_frame`.
- `/livox/lidar`: Livox CustomMsg for FAST-LIO.
- `/livox/imu`: lidar IMU; `/pm01/imu`: torso IMU used by the walking policy.
- `/joint_states`: joint positions and velocities.
- `/pm01_sdk_pd_controller/commands`: position commands for 24 joints, in radians and in the order declared by the model.

Startup begins with a standing hold and hands over after the controllers are active and the policy is ready. Project motion output is enabled separately from platform initialization. The unified project launch entry selects runtime parameters and resources.

See the [parent README](../../../../README.en.md) and [adaptation notice](../../../../PM01_ADAPTATION.en.md) for sources, copyright, and licensing.
