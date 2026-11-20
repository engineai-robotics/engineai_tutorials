# PM01 SLAM 后端适配

简体中文 | [English](PM01_ADAPTATION.en.md)

项目集成了固定版本的 SLAM Toolbox 2.6.10。主要改动包括 Karto 的局部扫描匹配验收与失配恢复，以及供实时导航地图使用的匹配、地图修订和关键帧位姿信息。[上游源码](https://github.com/SteveMacenski/slam_toolbox/tree/8293d21fe5c816d0405e0cc1f3eb46c70659c267)

匹配和恢复参数集中在项目配置中。[config/matching/slam.yaml](../../config/matching/slam.yaml)

上游版权和许可声明随源码保留。修改后的 Karto 文件和新增的匹配校验代码声明 LGPL-3.0-or-later。ROS 接入文件沿用各自的上游许可。[根目录许可文本 LGPL-2.1](LICENSE)
