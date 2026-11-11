#!/usr/bin/env python3
"""Run after sourcing scripts/env.sh; coordinates are examples, inspect your map."""
from example_utils import rclpy, Navigator, response, time

rclpy.init();nav=Navigator()
try:
    # SIMULATION EXAMPLE. Hardware enablement requires an operator and stop access.
    if not nav.trigger_clients['enable_motion'].wait_for_service(timeout_sec=5.): raise RuntimeError('Motion manager missing')
    print('Enabled:',response(nav,nav.enable_motion()))
    print('Released:',response(nav,nav.release_motion()))
    # Release acknowledgment reports process removal separately from goal cancel.
    end=time.monotonic()+1.5
    while time.monotonic()<end:nav.poll(.05)
    print(nav.get_status())
finally:
    nav.destroy_node()
    if rclpy.ok():rclpy.shutdown()
