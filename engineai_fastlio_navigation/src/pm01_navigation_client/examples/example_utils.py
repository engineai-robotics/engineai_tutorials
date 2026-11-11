import time
import rclpy
from pm01_navigation_client import Navigator


def response(nav, future, timeout=5.):
    end=time.monotonic()+timeout
    while rclpy.ok() and not future.done() and time.monotonic()<end: nav.poll(.05)
    if not future.done(): raise TimeoutError('Response unknown; task/output state has not been confirmed')
    result=future.result()
    if not result.success: raise RuntimeError(result.message)
    return result.message
