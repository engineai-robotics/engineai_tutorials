^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
karto_sdk 更新记录
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

简体中文 | `English <CHANGELOG.rst>`_

1.1.4 (2016-03-02)
------------------
* 更新构建状态徽章。
* 新增 LocalizedRangeScanWithPoints 扫描类型。
* 贡献者：Michael Ferguson、Russell Toris。

1.1.3 (2016-02-16)
------------------
* 链接 boost，并将其声明为导出依赖。
* 贡献者：Hai Nguyen、Michael Ferguson。

1.1.2 (2015-07-13)
------------------
* 为 Mapper 内部参数加入读取和设置方法，支持通过 ROS 参数服务器修改。
* 贡献者：Luc Bettaieb、Michael Ferguson。

1.1.1 (2015-05-07)
------------------
* 改进 FindValidPoints 对扫描首个点为 NaN 的处理。
* 提高最低 CMake 版本要求。
* 修复 cppcheck 警告。
* 统一换行符（dos2unix），移除多余空白。
* 使用互斥锁保护移除 const 限定的函数（检查 dirty 标志）。
* 回环检查改为操作扫描副本，保留原扫描。
* 移除无用的 return，消除 cppcheck 错误。
* 新增 Mapper::SetUseScanMatching。
* 移除日志输出中的 HTML 实体。
* 修复 NaN 导致射线追踪无法结束的问题。
* 贡献者：Daniel Pinyol、Michael Ferguson、Paul Mathieu、Siegfried-A. Gevatter Pujals、liz-murphy。

1.1.0 (2014-06-15)
------------------
* 作为纯 catkin ROS 包发布。
