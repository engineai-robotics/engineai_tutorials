#!/usr/bin/env python3
"""Run after sourcing scripts/env.sh; coordinates are examples, inspect your map."""
from example_utils import rclpy, Navigator, response, time

rclpy.init();nav=Navigator()
try:
    # In saved mode, replace these coordinates with the actual pose in that map.
    end=time.monotonic()+5.
    while (not nav.initial_pub.get_subscription_count() or (nav.get_parameter('use_sim_time').value and nav.get_clock().now().nanoseconds==0)) and time.monotonic()<end: nav.poll(.1)
    if not nav.initial_pub.get_subscription_count(): raise RuntimeError('Start saved mode first')
    nav.set_initial_pose(0.,0.,0.)
    end=time.monotonic()+.5
    while time.monotonic()<end:nav.poll(.05)
    print('Published. Check /pm01/localization_status for alignment acknowledgment.')
finally:
    nav.destroy_node()
    if rclpy.ok():rclpy.shutdown()
