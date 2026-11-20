# pm01_navigation_console

[简体中文](README.md) | English

Optional operator interface. Uses client, runtime, and session interfaces to operate navigation, sharing their map state and motion output management.

pm01_console reads the user configuration, manages sessions through SSH, and manages tasks through ROS. Hardware sessions use Docker; the native session entry point is used for simulation.

The template is in the root [config/console/console.example.yaml](../../config/console/console.example.yaml) file and is installed into this package's share directory during the build. The setup wizard generates the active configuration in the user's configuration directory.
