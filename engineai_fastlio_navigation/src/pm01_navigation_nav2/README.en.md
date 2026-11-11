# pm01_navigation_nav2

[简体中文](README.md) | English

Navigation control extensions: controllers, goal checking, walking trajectory scoring, and navigation pauses when observations become unavailable.

Retains the plugin identifiers pm01_fastlio_nav::LatestTfRotationShimController, SettlingDWBController, SettledGoalChecker, and WalkingCommandCritic.

This package is the sole exporter of pm01_nav_controllers, pm01_goal_checkers, and pm01_dwb_critics. Existing Nav2 YAML files keep their plugin names.

`navigation_command_gate` combines ground, scan, localization, and map status to control Nav2 output on `/cmd_vel`. The default behavior tree uses `NavigationReadyWait` to pause planning, control, and recovery behaviors. After observations remain healthy for 1 second, navigation replans to the original goal. The goal fails if recovery has not occurred after 30 seconds. `/pm01/navigation_readiness` and `/pm01/navigation_wait_status` report the reason and waiting state. Mapping-only mode leaves this navigation output chain inactive.
