#!/usr/bin/env python3
"""Enable and release simulated motion after startup reports ready."""
import signal
from rclpy.signals import SignalHandlerOptions
from example_utils import rclpy, Navigator, response, time
from pm01_navigation_client.startup_wait import wait_for_startup

rclpy.init(signal_handler_options=SignalHandlerOptions.NO);nav=Navigator();submitted=False
try:
    deadline=time.monotonic()+120.
    wait_for_startup(nav,deadline,print,require_motion=True)
    remaining=max(.1,deadline-time.monotonic())
    pending=nav.enable_motion(timeout=remaining);submitted=True
    print('Enabled:',response(nav,pending,timeout=remaining))
    print('Released:',response(nav,nav.release_motion()))
    end=time.monotonic()+1.5
    while time.monotonic()<end:nav.poll(.05)
    print(nav.get_status())
finally:
    try:
        if submitted:
            # Withdraw an enable request whose response was lost or interrupted.
            signal.signal(signal.SIGINT,signal.SIG_IGN)
            cancel=nav.cancel_pending_enable();end=time.monotonic()+4.
            while not cancel.done() and time.monotonic()<end:nav.poll(.05)
            if not cancel.done() or not cancel.result().result.successful:
                print('Cancellation unconfirmed; check status and run release-motion')
    finally:
        nav.destroy_node()
        if rclpy.ok():rclpy.shutdown()
