# pm01_navigation_core

[简体中文](README.md) | English

Ground processing, live mapping, and localization integration. ground, mapping, and localization contain the algorithms; nodes handle ROS messages, TF, parameters, and services. This pure Python package does not compile Nav2 plugins.

Deskewed clouds and the ground model → obstacle clouds and /navigation_map; accepted SLAM poses and revisions → map→odom; /pm01/save_navigation_map saves maps in the existing file format.

AMCL publishes map→odom itself; this package provides diagnostics for it. Dynamic base maps are associated by source timestamp and revision. Live evidence is stored at its map position when observed and corrected using the corresponding keyframe after graph optimization.
