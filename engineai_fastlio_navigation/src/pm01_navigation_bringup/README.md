# pm01_navigation_bringup

简体中文 | [English](README.en.md)

系统组合与配置资源。根据 profile 和 mode 选择平台、地图后端、Nav2 及运动 worker。算法与进程管理实现由各功能包提供。

`navigation.launch.py` 保留仿真／真机、mapping／fresh／saved 主入口；`pm01_session` 调用 runtime。

默认参数维护在仓库根目录 [config/](../../config)，世界文件在本包 `worlds/`。参数安装到本包的 `share/pm01_navigation_bringup/config`；覆盖顺序见[配置覆盖](../../docs/development.md#配置覆盖)。
