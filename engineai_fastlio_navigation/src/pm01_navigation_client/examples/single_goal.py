#!/usr/bin/env python3
"""Run after sourcing scripts/env.sh; coordinates are examples, inspect your map."""
from example_utils import rclpy, Navigator, response, time

rclpy.init();nav=Navigator()
try:
    if not nav.wait_until_ready(30.): raise RuntimeError('Map/TF/Nav2 not ready')
    task=nav.navigate_to_pose(1.,0.,0.)
    print('task UUID:',task.uuid)
    if not nav.wait(task,90.): raise TimeoutError('Goal still unresolved; explicit cancel required')
    print(task.state)
finally:
    nav.destroy_node()
    if rclpy.ok():rclpy.shutdown()
