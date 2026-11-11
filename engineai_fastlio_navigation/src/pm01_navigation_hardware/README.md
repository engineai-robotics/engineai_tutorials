# pm01_navigation_hardware

简体中文 | [English](README.en.md)

PM01 真机适配。motion/cmd_vel_bridge 将导航速度转换为真机运控协议；odometry/odom_relay 根据固定外参，将 LIO 位姿转换为底座里程计和 TF。

Nav2 Twist → /motion/body_vel_cmd；/Odometry → /odom 与 odom→base_footprint。

仅此功能包依赖厂商 interface_protocol 运控消息。通过统一启动和运动管理器运行，保留默认输出关闭。
