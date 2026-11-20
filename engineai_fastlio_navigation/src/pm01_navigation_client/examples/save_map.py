#!/usr/bin/env python3
"""Run after sourcing scripts/env.sh; coordinates are examples, inspect your map."""
from example_utils import rclpy, Navigator, response, time

rclpy.init();nav=Navigator()
try:
    if not nav.trigger_clients['save_map'].wait_for_service(timeout_sec=5.): raise RuntimeError('Map service missing')
    print('Saved map YAML:',response(nav,nav.save_map()))
finally:
    nav.destroy_node()
    if rclpy.ok():rclpy.shutdown()
