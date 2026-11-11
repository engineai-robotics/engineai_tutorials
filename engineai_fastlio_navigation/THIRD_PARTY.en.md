# Third-party sources and licenses

[简体中文](THIRD_PARTY.md) | English

The root LICENSE applies to project-owned code. The third-party components below use the licenses listed in the table.

| Component | Source and version | License | Project modifications |
|---|---|---|---|
| FAST_LIO_ROS2 | Vendored snapshot in this repository; the release commit pins the corresponding source | [GPL-2.0](src/FAST_LIO_ROS2/LICENSE) | ROS 2 integration, auxiliary simulation estimation, diagnostics, deskewed cloud output, and existing recovery checks; demonstrations are linked upstream |
| livox_ros_driver2 | Vendored snapshot in this repository | [Upstream per-file terms](src/livox_ros_driver2/LICENSE.txt) | ROS 2 build and message integration; original internal layout retained |
| interface_protocol | PM01 message snapshot in this repository | [BSD-3-Clause](src/interface_protocol/LICENSE) | Hardware message build integration |
| EngineAI Navigation | `7ee29a3ede5ecd7d884a1a53b258cd5a0147bd81`, listed in dependencies.repos | [Upstream LICENSE](src/engineai_navigation/LICENSE) and embedded component licenses | [Vendored simulation adaptations](src/engineai_navigation/PM01_ADAPTATION.en.md): velocity topics, gait configuration, initialization shutdown, and standing-control handoff |
| Livox SDK2 | Tree `bc4597a34a4d0220f6112cb3e7156281499487e4` from the public version above | SDK `source/LICENSE.txt` | Build integration; SDK algorithms are unchanged |
| SLAM Toolbox / Karto | `8293d21fe5c816d0405e0cc1f3eb46c70659c267` (2.6.10) | The [root LICENSE](src/slam_toolbox/LICENSE) contains LGPL-2.1; modified Karto files declare LGPL-3.0-or-later; ROS files retain their upstream headers | [Vendored source adaptations](src/slam_toolbox/PM01_ADAPTATION.en.md): local-match validation and match/map revision metadata; unused experimental backends removed |
| Nav2 launch composition | Navigation2 1.1.20 `navigation_launch.py` | Apache-2.0; see the [retained file header](src/pm01_navigation_bringup/launch/nav2_stack.launch.py) | Default resource paths and project composition |
| ROS 2 / Nav2 / AMCL / Gazebo | Ubuntu 22.04 / ROS Humble system packages | Each package's LICENSE and system copyright files | Standard interfaces; installed versions are supplied with the image package list |
| MNN and walking models | The fixed EngineAI Navigation version above | Upstream third-party and model notices | Used for x86_64 simulation only |

Images and offline bundles include source code, dependency versions, modification notices, and licenses.

The simulation build script automatically downloads and verifies model meshes, walking policies, MNN libraries, and the Livox SDK from the fixed upstream version. Runtime resources are installed into their packages during the build, with their original licenses retained.

ROS, Nav2, AMCL, and Gazebo come from the Ubuntu / ROS package repositories. The onboard image retains system package copyright files. Installed versions are listed in `container-packages.tsv` in the delivery bundle.
