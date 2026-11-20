# pm01_navigation_motion

简体中文 | [English](README.en.md)

运动输出管理。管理输出接通、释放、桥接进程生命周期和状态，默认不启动桥接进程。通用进程管理来自 runtime。

保留 /pm01/navigation/enable_motion、release_motion、motion_status。取消导航任务与撤下输出分别报告。

worker_executable、worker_arguments、output_topic 由 bringup 按平台提供；本包不编译平台桥接器。

接通为有界的启动事务：每次分配独立就绪服务，等待 worker 确认输出及接收端，再报告 enabled；超时或提前退出会停止该进程并保留失败原因，允许重试。release 可中断等待。就绪确认只表示通信条件成立，物理控制权仍标记 unknown。
