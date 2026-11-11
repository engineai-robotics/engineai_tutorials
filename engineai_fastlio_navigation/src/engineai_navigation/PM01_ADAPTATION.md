# PM01 仿真源码适配

简体中文 | [English](PM01_ADAPTATION.en.md)

来源：EngineAI Navigation `7ee29a3ede5ecd7d884a1a53b258cd5a0147bd81`。内置 `ros/src/simulation/simulator` 与 `rl_walking` 源码子集，保留上游 LICENSE、包名、目录和逐文件声明。

项目修改包括：速度话题与步态配置选择、初始站立节点正常退出、控制器激活后开始初始化、初始化完成后交接策略，以及等待策略就绪期间的站立保持与释放状态。修改位于两个包的 launch、simulator 的初始化脚本、站立插件、URDF 与构建声明。

网格、策略模型及 MNN 仍来自同一固定上游提交，获取到 `.deps/sim_assets/`。`scripts/build.sh sim` 显式传入 `PM01_SIM_ASSETS_ROOT`；两个包的 CMake 将资源和许可证复制到安装目录，运行时不读取下载缓存。Livox SDK 复用固定 Git 树获取器。

源码变更通过 Git 维护，不在构建时覆盖。正常与延迟启动、站立释放、地图和默认关闭输出通过仓库 `tests/integration/test_simulation_startup.py` 验证。
