# PM01 simulation source adaptations

[简体中文](PM01_ADAPTATION.md) | English

Source: EngineAI Navigation `7ee29a3ede5ecd7d884a1a53b258cd5a0147bd81`. The source subset includes `ros/src/simulation/simulator` and `rl_walking`, retaining the upstream LICENSE, package names, directories, and per-file notices.

Project modifications cover velocity-topic and gait-configuration selection, clean shutdown of the initial-standing node, initialization after controller activation, handoff to the policy after initialization, and standing-hold/release states while waiting for the policy. Changes are in the two packages' launch files and in the simulator's initialization scripts, standing plugin, URDF, and build declarations.

Meshes, policy models, and MNN still come from the same fixed upstream commit and are downloaded to `.deps/sim_assets/`. `scripts/build.sh sim` explicitly passes `PM01_SIM_ASSETS_ROOT`. The two packages' CMake files copy resources and licenses into the installation directory; runtime does not read the download cache. Livox SDK uses the same fixed Git-tree fetcher.

Source changes are maintained in Git and are not overlaid during the build. `tests/integration/test_simulation_startup.py` verifies normal and delayed startup, release of the standing hold, map output, and motion output disabled by default.
