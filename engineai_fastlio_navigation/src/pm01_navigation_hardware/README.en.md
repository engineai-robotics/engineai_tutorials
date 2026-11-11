# pm01_navigation_hardware

[简体中文](README.md) | English

PM01 hardware adaptation. motion/cmd_vel_bridge converts navigation velocity commands to the robot's motion-control protocol. odometry/odom_relay uses fixed extrinsics to convert LIO poses into base odometry and TF.

Nav2 Twist → /motion/body_vel_cmd; /Odometry → /odom and odom→base_footprint.

This is the only functional package that depends on the vendor's interface_protocol motion messages. It runs through the unified launch entry and motion manager, with output disabled by default.
