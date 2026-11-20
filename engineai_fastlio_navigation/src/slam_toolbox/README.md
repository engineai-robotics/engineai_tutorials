# SLAM Toolbox / Karto

简体中文 | [English](README.en.md)

本项目内置 SLAM Toolbox 2.6.10，提供二维扫描匹配、姿态图、回环和地图生成。项目 `fresh` / `mapping` 模式使用该后端，`saved` 模式使用 AMCL。统一入口与操作见[教程](../../docs/tutorial.md)。

保留同步、异步、定位后端以及地图合并、RViz 和 Ceres 插件。未提供上游实验性的 lifelong 与 map-and-localization 后端。项目匹配验收、恢复和地图修订接口见 [PM01_ADAPTATION.md](PM01_ADAPTATION.md)，公共导航接口见[开发参考](../../docs/development.md#应用接口)。

来源：[SLAM Toolbox 固定版本](https://github.com/SteveMacenski/slam_toolbox/tree/8293d21fe5c816d0405e0cc1f3eb46c70659c267)。上游作者 Steve Macenski，维护者 Steve Macenski、Michel Hidalgo；Karto 作者见 [Authors](lib/karto_sdk/Authors)。

保留上游文件级版权与许可声明：[SLAM Toolbox LICENSE](LICENSE)、[Karto LICENSE](lib/karto_sdk/LICENSE)。项目适配不改变第三方许可，详见[第三方声明](../../THIRD_PARTY.md)。上游完整参数与演示请参阅固定版本的上游说明。
