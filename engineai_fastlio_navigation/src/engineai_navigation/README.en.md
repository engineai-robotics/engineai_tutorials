# EngineAI Navigation simulation source

[简体中文](README.md) | English

This directory contains the `simulator` and `rl_walking` packages used by PM01. They provide Gazebo sensors, joint control, and the walking policy. Meshes, policies, and MNN resources are fetched from a fixed version; installed packages run independently of the download cache.

Run `bash scripts/build.sh sim` from the project root. See the [tutorial](../../docs/tutorial.en.md) for the complete workflow. Run the walking policy through the project launch entry so its velocity input is coordinated with navigation.

Source: [EngineAI Navigation](https://github.com/engineai-robotics/engineai_navigation/tree/7ee29a3ede5ecd7d884a1a53b258cd5a0147bd81). See [PM01_ADAPTATION.en.md](PM01_ADAPTATION.en.md) for integration details and [Third-party notices](../../THIRD_PARTY.en.md) for resource sources and licenses.

Original EngineAI code uses [Apache-2.0](LICENSE). Embedded components retain their own licenses.

Copyright 2026 Shenzhen Zhongqing Robot Technology Co., Ltd. (EngineAI)
