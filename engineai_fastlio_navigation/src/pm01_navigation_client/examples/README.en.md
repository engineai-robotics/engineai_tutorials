# Python client examples

[简体中文](README.md) | English

| Task | Example |
| --- | --- |
| Navigate to one goal and inspect the final result | [`single_goal.py`](single_goal.py) |
| Visit several goals in sequence | [`continuous_goals.py`](continuous_goals.py) |
| Check the remaining distance and cancel the current goal | [`feedback_cancel.py`](feedback_cancel.py) |
| Set the robot's initial pose on a saved map in `saved` mode | [`initial_pose.py`](initial_pose.py) |
| Save the current map and get its YAML path | [`save_map.py`](save_map.py) |
| Enable and release motion output and inspect its state in simulation | [`motion_output.py`](motion_output.py) |

Start navigation or mapping with the [tutorial](../../../docs/tutorial.en.md). From the project root on the development computer, load the environment: `source scripts/env.sh` for simulation, or `source ~/.config/pm01_navigation/active.env` for hardware. To save the current map, run:

```bash
python3 src/pm01_navigation_client/examples/save_map.py
```

Before running a navigation example, change the goal coordinates in the script to a reachable location on the current map, then enable motion output. When setting the initial pose, use the robot's actual position and heading on the saved map.
