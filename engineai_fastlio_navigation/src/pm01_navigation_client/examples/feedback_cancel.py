#!/usr/bin/env python3
"""Run after sourcing scripts/env.sh; coordinates are examples, inspect your map."""
from example_utils import rclpy, Navigator, response, time

rclpy.init();nav=Navigator()
try:
    if not nav.wait_until_ready(30.): raise RuntimeError('Map/TF/Nav2 not ready')
    task=nav.navigate_to_pose(2.,0.,0.)
    end=time.monotonic()+3.
    next_report=time.monotonic()
    while not task.done and time.monotonic()<end:
        nav.poll(.1)
        if task.feedback and time.monotonic()>=next_report:
            print('remaining metres:',task.feedback.distance_remaining)
            next_report=time.monotonic()+.5
    nav.cancel(task)  # Per-task UUID; not a global release of motion control.
    if not nav.wait(task,5.): print('Cancel outcome unknown; UUID:',task.uuid)
    else: print('Final result:',task.state)
finally:
    nav.destroy_node()
    if rclpy.ok():rclpy.shutdown()
