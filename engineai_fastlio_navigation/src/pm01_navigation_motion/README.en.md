# pm01_navigation_motion

[简体中文](README.md) | English

Motion output management. Manages output enable/release, the bridge process lifecycle, and status. The bridge process is disabled by default. Runtime provides general process management.

Provides /pm01/navigation/enable_motion, release_motion, and motion_status. Goal cancellation and output removal report their results separately.

Bringup supplies worker_executable, worker_arguments, and output_topic for each platform. This package does not compile the platform bridges.

Enabling output is a startup operation with a time limit. Each attempt allocates a separate readiness service and waits for the worker to confirm its output and receiver before reporting enabled. A timeout or early exit stops the process and retains the failure reason so the operation can be retried. release can interrupt the wait. Readiness confirms communication conditions; physical control ownership remains marked unknown.
