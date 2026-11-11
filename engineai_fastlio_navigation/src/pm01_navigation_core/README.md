# pm01_navigation_core

简体中文 | [English](README.en.md)

地面、实时地图与定位接入。ground、mapping、localization 保存算法实现，nodes 负责 ROS 消息、TF、参数和服务。纯 Python 包不编译 Nav2 插件。

去畸变点云与地面模型 → 障碍点云、/navigation_map；SLAM 已验收位姿与修订 → map→odom；/pm01/save_navigation_map 保存原有文件格式。

AMCL 自行发布 map→odom，本包仅诊断。动态底图按源时间和修订关联；实时证据保存在采集时的地图位置，图优化时按对应关键帧修正。
