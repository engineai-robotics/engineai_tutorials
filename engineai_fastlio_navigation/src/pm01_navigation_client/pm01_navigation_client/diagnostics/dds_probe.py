#!/usr/bin/env python3
"""Bounded bidirectional ROS check; no sensor, navigation or actuator topic."""
import json
import re
import sys
import time


def namespace(nonce):
    if not re.fullmatch('[a-f0-9]{12}', nonce):
        raise ValueError('Invalid probe identifier')
    return '/pm01_setup_probe/p_'+nonce


def main():
    import rclpy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    role, nonce = sys.argv[1:3]
    if role not in ('server', 'client') or not re.fullmatch('[a-f0-9]{12}', nonce):
        raise ValueError('Invalid probe arguments')
    rclpy.init()
    node = rclpy.create_node('pm01_setup_'+role+'_'+nonce)
    # A ROS namespace segment cannot begin with a digit; UUID hex often does.
    prefix = namespace(nonce)
    received = 0
    def receive(message):
        nonlocal received
        if message.data == nonce:
            received += 1
    outgoing, incoming = ('pong', 'ping') if role == 'server' else ('ping', 'pong')
    pub = node.create_publisher(String, prefix+'/'+outgoing, 5)
    node.create_subscription(String, prefix+'/'+incoming, receive, 5)
    node.create_timer(.2, lambda: pub.publish(String(data=nonce)))
    if role == 'server':
        def reply(request, response):
            response.success = received > 0
            response.message = nonce
            return response
        node.create_service(Trigger, prefix+'/ping_service', reply)
    else:
        service = node.create_client(Trigger, prefix+'/ping_service')
    deadline = time.monotonic()+(35 if role == 'server' else 18)
    future = None
    success = False
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if role == 'client':
                if future is None and received >= 2 and service.service_is_ready():
                    future = service.call_async(Trigger.Request())
                if future is not None and future.done():
                    response = future.result()
                    success = response.success and response.message == nonce
                    break
        print(json.dumps(dict(ok=success if role == 'client' else True,
                              received=received, service_confirmed=success,
                              navigation_started=False, motion_started=False)))
        return 0 if success or role == 'server' else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
