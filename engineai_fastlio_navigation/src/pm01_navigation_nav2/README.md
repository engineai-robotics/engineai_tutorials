# pm01_navigation_nav2

简体中文 | [English](README.en.md)

导航控制扩展，包含控制器、到点检查、步行评分和观测失效时的导航暂停。

保留 pm01_fastlio_nav::LatestTfRotationShimController、SettlingDWBController、SettledGoalChecker、WalkingCommandCritic 原有标识。

本包唯一导出 pm01_nav_controllers、pm01_goal_checkers、pm01_dwb_critics；既有 Nav2 YAML 无需更换插件名。

`navigation_command_gate` 汇总地面、扫描、定位和地图状态，统一控制 Nav2 对 `/cmd_vel` 的输出。默认行为树通过 `NavigationReadyWait` 暂停规划、控制及恢复行为；观测连续恢复 1 秒后重新规划原目标，等待 30 秒仍未恢复则目标失败。`/pm01/navigation_readiness` 和 `/pm01/navigation_wait_status` 提供原因及等待状态。仅建图不启动这条导航输出链路。
