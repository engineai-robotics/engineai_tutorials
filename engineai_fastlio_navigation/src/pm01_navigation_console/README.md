# pm01_navigation_console

简体中文 | [English](README.en.md)

可选操作界面。界面通过 client、runtime 和会话接口操作导航，不复制地图状态和底层输出管理实现。

pm01_console 读取用户配置，通过 SSH 管理会话、通过 ROS 管理任务；真机只支持 Docker，会话原生入口仅用于仿真。

模板位于仓库根目录 [config/console/console.example.yaml](../../config/console/console.example.yaml)，构建时安装到本包的 share 目录。实际配置由准备向导生成并保存在用户目录。
