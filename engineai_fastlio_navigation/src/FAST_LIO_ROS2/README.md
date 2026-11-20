# FAST-LIO ROS 2

简体中文 | [English](README.en.md)

FAST-LIO 提供 LiDAR / IMU 局部里程计与去畸变点云。本项目通过统一启动入口选择输入、外参和诊断参数；安装与启动见[项目教程](../../docs/tutorial.md)，配置见[开发参考](../../docs/development.md#配置覆盖)。

来源：[Ericsii 的 ROS 2 移植](https://github.com/Ericsii/FAST_LIO_ROS2)与 [HKU-MARS FAST-LIO](https://github.com/hku-mars/FAST_LIO)。本仓库快照及项目修改随 Git 提交固定，修改范围见[第三方声明](../../THIRD_PARTY.md)。

作者：Wei Xu、Yixi Cai、Dongjiao He、Fangcheng Zhu、Jiarong Lin、Zheng Liu、Borong Yuan；ROS 2 移植维护者 Ericsii。保留源码中的各文件版权声明，许可见 [GPL-2.0](LICENSE)。

论文与演示：

- [FAST-LIO2: Fast Direct LiDAR-inertial Odometry](https://arxiv.org/abs/2107.06829)
- [FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package](https://arxiv.org/abs/2010.08196)
- [上游演示与算法资料](https://github.com/hku-mars/FAST_LIO/tree/main/doc)

`include/ikd-Tree` 已随源码内置，其作者与许可说明保留在该目录，不需要初始化 Git 子模块。
