#!/usr/bin/env python3
"""Run after sourcing scripts/env.sh; coordinates are examples, inspect your map."""
from example_utils import rclpy, Navigator, response, time

rclpy.init();nav=Navigator()
try:
    if not nav.wait_until_ready(30.): raise RuntimeError('Map/TF/Nav2 not ready')
    for x,y,yaw in [(1.,0.,0.),(1.,-1.,-1.57)]:
        task=nav.navigate_to_pose(x,y,yaw)
        if not nav.wait(task,120.): raise TimeoutError('Task unresolved; do not submit the next goal')
        if task.state!='succeeded': raise RuntimeError(task.state)
        print('Arrived:',task.uuid,'output remains enabled for next goal')
finally:
    nav.destroy_node()
    if rclpy.ok():rclpy.shutdown()
