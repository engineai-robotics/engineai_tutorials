# pm01_navigation_runtime

简体中文 | [English](README.en.md)

配置、进程、构建与部署。system 管理进程／网络／环境／时钟；build 管理唯一构建清单与依赖；deployment、containers、release 分别管理准备、容器和交付。

顶层 scripts 是薄入口，向模块显式传入工作空间。离线包携带同一源码生成的 Python 模块。

导入不启动进程、不访问机器人，不导入 ROS、Gazebo、Nav2 或厂商 SDK；远程初始辅助程序只要求 Python 标准库。
