# pm01_navigation_runtime

[简体中文](README.md) | English

Configuration, processes, builds, and deployment. system handles processes, networking, the environment, and clocks. build maintains the single build manifest and dependencies. deployment, containers, and release handle setup, containers, and delivery respectively.

The top-level scripts are thin entry points that pass the workspace explicitly to these modules. Offline bundles carry Python modules generated from the same source.

Importing modules does not start processes, access the robot, or import ROS, Gazebo, Nav2, or the vendor SDK. Initial remote helpers require only the Python standard library.
