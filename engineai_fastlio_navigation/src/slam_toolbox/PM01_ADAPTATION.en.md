# PM01 SLAM backend adaptations

[简体中文](PM01_ADAPTATION.md) | English

The project integrates a pinned version of SLAM Toolbox 2.6.10. The adaptations cover Karto's local scan-match validation and recovery, plus match, map revision, and keyframe pose metadata used by the live navigation map. [Upstream source](https://github.com/SteveMacenski/slam_toolbox/tree/8293d21fe5c816d0405e0cc1f3eb46c70659c267)

Matching and recovery parameters are in the project configuration. [config/matching/slam.yaml](../../config/matching/slam.yaml)

The source retains upstream copyright and license notices. Modified Karto files and the added match validation code declare LGPL-3.0-or-later. ROS integration files retain their respective upstream licenses. [Root license text (LGPL-2.1)](LICENSE)
