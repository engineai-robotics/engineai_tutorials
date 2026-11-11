# Configuration and interfaces

[简体中文](development.md) | English

## Configuration overrides

The files under the root `config/` directory define the defaults.

| What to adjust | Default files | Launch arguments |
| --- | --- | --- |
| Nav2 planning, control, and costmaps | `sim/nav2.yaml`, `hardware/nav2.yaml` | `nav2_params` |
| Obstacle clouds and the live occupancy grid | `obstacles.yaml` and `online_map.yaml` for the selected platform | `obstacle_params`, `map_params` |
| Scans, mapping, and localization on saved maps | `matching/scan.yaml`, `matching/slam.yaml`, `matching/amcl.yaml` | `scan_params`, `slam_params`, `amcl_params` |
| Lidar networking and driver | `hardware/livox_network.json`, `hardware/livox_driver.yaml` | `lidar_config`, `driver_params` |
| Lidar extrinsics and LIO | `pm01_lidar_tf.yaml`, `lio.yaml` for the selected platform | `extrinsics`, `lio_params` |

Select parameter files from `config/` and use a complete file for each override. LIO loads the upstream base parameters first, followed by `lio_params`. When adjusting obstacle heights, check both `obstacles.yaml` and `online_map.yaml`.

For example, on the simulation computer, copy the complete Nav2 configuration and set `controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance` in the copy to `0.20` meters.

```bash
cd ~/engineai_tutorials/engineai_fastlio_navigation
mkdir -p ~/.config/pm01_navigation/tuning
cp -n config/sim/nav2.yaml ~/.config/pm01_navigation/tuning/nav2-sim.yaml
nano ~/.config/pm01_navigation/tuning/nav2-sim.yaml
```

End the current simulation, then start it with the copied configuration.

```bash
source scripts/env.sh
ros2 launch pm01_navigation_bringup navigation.launch.py \
  profile:=sim mode:=fresh \
  nav2_params:="$HOME/.config/pm01_navigation/tuning/nav2-sim.yaml"
```

In another terminal with the simulation environment loaded, run `ros2 param get /controller_server general_goal_checker.xy_goal_tolerance` to confirm the active value.

For hardware, place override files in `~/.config/pm01_navigation/docker/` on the robot. Add the parameter name and the file's absolute onboard path to `launch_args` in `session.json` in the same directory, then restart the session. For a custom deployment, check `session_config` in `container.json`.

## Application interfaces

These ROS 2 interfaces let applications send the robot to a location, display a map, or save a collected map. Actions submit tasks and return progress and results. Topics send or receive messages. Services perform an operation and return a response.

| Interface | Purpose |
| --- | --- |
| `/navigate_to_pose` · `nav2_msgs/action/NavigateToPose` | Send a target position and heading, receive navigation progress and the final result, or cancel the goal. |
| `/compute_path_to_pose` · `nav2_msgs/action/ComputePathToPose` | Request a planned path to a target pose. |
| `/initialpose` · `geometry_msgs/msg/PoseWithCovarianceStamped` | Publish the robot's approximate position and heading on a saved map to initialize AMCL. |
| `/navigation_map` · `nav_msgs/msg/OccupancyGrid` | Subscribe to the 2D navigation map to display free, occupied, and unknown areas. |
| `/odom` · `nav_msgs/msg/Odometry` | Subscribe to odometry to read the robot base's local pose and velocity. |
| `/pm01/save_navigation_map` · `std_srvs/srv/Trigger` | Save the current map and receive its YAML path. |
| `/pm01/navigation/enable_motion`, `/pm01/navigation/release_motion` · `std_srvs/srv/Trigger` | Enable and release this project's motion output. |

Python applications use `pm01_navigation_client.Navigator`. Choose a [Python client example](../src/pm01_navigation_client/examples/README.en.md) for your task.
