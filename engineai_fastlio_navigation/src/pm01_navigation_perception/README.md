# pm01_navigation_perception

简体中文 | [English](README.en.md)

传感数据与扫描转换。转换 Livox 消息和 LIO 去畸变点云，生成水平扫描及建图关键帧。复用 core 的纯 Python 点云与几何函数。

`/livox/lidar` → `/livox/points`；`/cloud_registered_body` → `/pm01/deskewed_points`；`/pm01/obstacle_points` → `/scan`、`/mapping_scan`。

planar_scan 唯一发布 odom→navigation_base；保留测量时间，缺同刻 TF 时不使用最新 TF。
