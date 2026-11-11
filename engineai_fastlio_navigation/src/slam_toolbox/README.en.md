# SLAM Toolbox / Karto

[简体中文](README.md) | English

The project vendors SLAM Toolbox 2.6.10 for 2D scan matching, pose graphs, loop closure, and map generation. The `fresh` and `mapping` modes use this backend; `saved` uses AMCL. See the [tutorial](../../docs/tutorial.en.md) for the unified entry point and operating steps.

The synchronous, asynchronous, and localization backends are retained, along with map merging, RViz, and Ceres plugins. The upstream experimental lifelong and map-and-localization backends are not included. See [PM01_ADAPTATION.en.md](PM01_ADAPTATION.en.md) for match validation, recovery, and map revision interfaces, and [Application interfaces](../../docs/development.en.md#application-interfaces) for public navigation interfaces.

Source: [SLAM Toolbox at the fixed version](https://github.com/SteveMacenski/slam_toolbox/tree/8293d21fe5c816d0405e0cc1f3eb46c70659c267). Upstream author: Steve Macenski; maintainers: Steve Macenski and Michel Hidalgo. Karto contributors are listed in [Authors](lib/karto_sdk/Authors).

Upstream per-file copyright and license notices are retained: [SLAM Toolbox LICENSE](LICENSE) and [Karto LICENSE](lib/karto_sdk/LICENSE). Project adaptations retain the third-party licenses; see [Third-party notices](../../THIRD_PARTY.en.md). The fixed upstream version documents the full upstream parameters and demonstrations.
