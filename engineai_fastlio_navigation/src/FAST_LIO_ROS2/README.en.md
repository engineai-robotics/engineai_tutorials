# FAST-LIO ROS 2

[简体中文](README.md) | English

FAST-LIO provides local LiDAR / IMU odometry and deskewed point clouds. The project's unified launch entry selects inputs, extrinsics, and diagnostic parameters. See the [project tutorial](../../docs/tutorial.en.md) for installation and startup, and [Configuration overrides](../../docs/development.en.md#configuration-overrides) for configuration.

Sources: [Ericsii's ROS 2 port](https://github.com/Ericsii/FAST_LIO_ROS2) and [HKU-MARS FAST-LIO](https://github.com/hku-mars/FAST_LIO). Git commits pin this repository's snapshot and project modifications. See [Third-party notices](../../THIRD_PARTY.en.md) for the modification scope.

Authors: Wei Xu, Yixi Cai, Dongjiao He, Fangcheng Zhu, Jiarong Lin, Zheng Liu, and Borong Yuan; ROS 2 port maintainer: Ericsii. Per-file copyright notices are retained. See [GPL-2.0](LICENSE) for the license.

Papers and demonstrations:

- [FAST-LIO2: Fast Direct LiDAR-inertial Odometry](https://arxiv.org/abs/2107.06829)
- [FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package](https://arxiv.org/abs/2010.08196)
- [Upstream demonstrations and algorithm resources](https://github.com/hku-mars/FAST_LIO/tree/main/doc)

`include/ikd-Tree` is included in the source tree, with its author and license notices retained in that directory. Git submodule initialization is not required.
