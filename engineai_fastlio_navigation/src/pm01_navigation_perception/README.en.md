# pm01_navigation_perception

[简体中文](README.md) | English

Sensor data and scan conversion. Converts Livox messages and LIO deskewed clouds into horizontal scans and mapping keyframes. Reuses core's pure Python point-cloud and geometry functions.

`/livox/lidar` → `/livox/points`; `/cloud_registered_body` → `/pm01/deskewed_points`; `/pm01/obstacle_points` → `/scan`, `/mapping_scan`.

planar_scan is the sole publisher of odom→navigation_base. Measurement timestamps are preserved; a missing transform at that timestamp is not replaced with the latest TF.
