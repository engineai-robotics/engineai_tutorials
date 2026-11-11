# EngineAI Navigation 仿真源码

简体中文 | [English](README.en.md)

本目录包含 PM01 使用的 `simulator` 和 `rl_walking`，分别提供 Gazebo 传感器、关节控制与步行策略。网格、策略和 MNN 资源按固定版本获取，安装后不依赖下载缓存。

从项目根目录执行 `bash scripts/build.sh sim`，完整操作见[教程](../../docs/tutorial.md)。不要单独启动步行策略与项目导航争用速度输入。

来源：[EngineAI Navigation](https://github.com/engineai-robotics/engineai_navigation/tree/7ee29a3ede5ecd7d884a1a53b258cd5a0147bd81)。集成说明见 [PM01_ADAPTATION.md](PM01_ADAPTATION.md)，资源来源与许可见[第三方声明](../../THIRD_PARTY.md)。

EngineAI 原创代码使用 [Apache-2.0](LICENSE)，嵌入组件保留自身许可。

Copyright 2026 Shenzhen Zhongqing Robot Technology Co., Ltd. (EngineAI)
