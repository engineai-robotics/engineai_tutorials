# PM01 SLAM 后端适配

简体中文 | [English](PM01_ADAPTATION.en.md)

上游：[SLAM Toolbox 2.6.10](https://github.com/SteveMacenski/slam_toolbox/tree/8293d21fe5c816d0405e0cc1f3eb46c70659c267)，固定提交 `8293d21fe5c816d0405e0cc1f3eb46c70659c267`。项目适配后的源码直接纳入本仓库，由 Git 管理。上游源码目录结构与许可证保留。

局部匹配验收在 Karto 普通匹配写入扫描缓存、运行扫描列表和图节点之前，检查同刻 LIO 预测残差、有效量程比例、响应、竞争峰和转向段累计修正。邻近链匹配与候选融合也相对原始 LIO 预测检查，通过后才添加约束。局部响应与竞争峰按有限测量数归一化；原始相关分数、协方差及全局回环阈值保持不变。拒绝后保留最后接受扫描，使用三个不同测量时间的连续可信观测恢复；普通缓存无法匹配时，只在同刻 LIO 预测周围 4 m 内选择最多 30 个已接受参考，验收前不修改历史或图。全局回环仍经过独立的上游验证路径。

后端发布内部 `/pm01/slam_match_update` 和 `/pm01/slam_map_revision` JSON 元数据。源时间使用整数纳秒；地图修订元数据与地图栅格在同一后端锁内生成。公共地图和位姿消息类型不变。接入层不接受缺少后端验收元数据的 SLAM 位姿。已接受更新携带 `anchor_id`；图优化完成时，`/pm01/slam_graph_anchors` 可靠保留一份同代关键帧位姿表，供实时证据逐锚重投影。普通更新不重复发送整表，表中不包含点云或扫描量程。

校验参数在现有 `config/matching/slam.yaml` 中配置，不增加教程步骤。普通跟踪和邻近链的平移残差上限为 0.20 m；失配后的主恢复候选可使用 0.40 m，但必须有三个不同测量时间的观测，与首次恢复候选经 LIO 增量预测的位姿一致，第三次通过后才写图。单帧航向残差仍限制为约 2°；主恢复候选的累计修正上限为约 5°，普通跟踪与邻近链仍为约 2°。仅完成三帧一致验收后的写图提交建立新的累计修正段，拒绝和前两帧不重置参考。支持率与歧义限制保持不变。

包内 `test/` 覆盖局部验收、稀疏扫描评分和恢复时的图写入边界，通过 `colcon test --packages-select slam_toolbox` 执行。

许可：保留上游全部版权和许可声明。上游根 LICENSE 为 LGPL-2.1 文本；Karto 被修改文件声明 LGPL-3.0-or-later，新增同目录校验代码沿用该声明。ROS 接入文件保留其上游文件头。本项目不将第三方代码改许可为项目自有代码的 Apache-2.0。

项目修改文件：`lib/karto_sdk/{include/karto_sdk/Mapper.h,include/karto_sdk/LocalMatchValidation.h,src/Mapper.cpp}`、`include/slam_toolbox/slam_toolbox_common.hpp`、`src/{slam_mapper.cpp,slam_toolbox_common.cpp}`；`CMakeLists.txt` 注册包内测试并安装许可与适配说明。参数路径相对于导航仓库根目录。
