"""Gate simulated LIO initialization on active controllers and a standing reset."""
import math
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64MultiArray
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data


def standing_sample_ready(observations, before_reset_stamp, now):
    stamp = observations['imu_stamp']
    return (observations['startup_held'] is False and stamp is not None
            and before_reset_stamp is not None and stamp > before_reset_stamp
            and observations['tilt'] < 0.25 and now-observations['imu_time'] < 0.5
            and now-observations['targets_time'] < 0.5)


def main():
    rclpy.init()
    node = rclpy.create_node('pm01_stand_ready')
    controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
    reset = node.create_client(Empty, '/pm01/reset_stand')
    policy = node.create_client(Empty, '/pm01_walking_policy/reset')
    observations = {'imu_time': 0.0, 'imu_stamp': None, 'targets_time': 0.0,
                    'targets_since': None, 'tilt': math.inf, 'startup_held': None}

    def imu(message):
        q = message.orientation
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        observations['tilt'] = (math.acos(max(-1.0, min(1.0, 1.0-2.0*(q.x*q.x+q.y*q.y)/norm)))
                                if math.isfinite(norm) and norm > 1e-8 else math.inf)
        observations['imu_time'] = time.monotonic()
        observations['imu_stamp'] = message.header.stamp.sec + message.header.stamp.nanosec*1e-9

    def targets(message):
        if len(message.data) != 24 or not all(math.isfinite(v) for v in message.data):
            return
        now = time.monotonic()
        if now-observations['targets_time'] > 0.5:
            observations['targets_since'] = now
        observations['targets_time'] = now

    node.create_subscription(Imu, '/pm01/imu', imu, qos_profile_sensor_data)
    node.create_subscription(Float64MultiArray, '/pm01_sdk_pd_controller/commands', targets, 10)
    node.create_subscription(Bool, '/pm01/sim_startup_held',
                             lambda msg: observations.update(startup_held=msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        deadline = time.monotonic() + 90.0
        required = {'joint_state_broadcaster', 'pm01_sdk_pd_controller'}
        ready = False
        active_since = None
        while time.monotonic() < deadline:
            if all(client.service_is_ready() for client in (controllers, reset, policy)):
                future = controllers.call_async(ListControllers.Request())
                end = time.monotonic() + 2.0
                while not future.done() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.1)
                if future.done() and future.result() is not None:
                    active = {c.name for c in future.result().controller if c.state == 'active'}
                    now = time.monotonic()
                    if required <= active:
                        if active_since is None:
                            active_since = now
                    else:
                        active_since = None
                    stream_ready = (observations['targets_since'] is not None and
                                    now-observations['targets_since'] >= 2.0 and
                                    now-observations['targets_time'] < 0.5 and
                                    now-observations['imu_time'] < 0.5)
                    if active_since is not None and now-active_since >= 2.0 and stream_ready:
                        ready = True
                        break
                else:
                    controllers.remove_pending_request(future)
            rclpy.spin_once(node, timeout_sec=0.2)
        if not ready:
            raise RuntimeError('Simulation controllers/policy did not become ready')
        before_reset_stamp = observations['imu_stamp']
        future = reset.call_async(Empty.Request())
        end = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Do not repeat a mutating request whose acknowledgment was lost.
        if not future.done() or future.result() is None:
            raise RuntimeError('Standing reset acknowledgment missing; navigation was not started')
        end = time.monotonic() + 30.0
        upright_since = None
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            stamp = observations['imu_stamp']
            if standing_sample_ready(observations, before_reset_stamp, now):
                if upright_since is None or stamp < upright_since:
                    upright_since = stamp
                if stamp-upright_since >= 2.0:
                    break
            else:
                upright_since = None
        else:
            raise RuntimeError('Robot did not remain upright after reset; LIO was not started')
        node.get_logger().info('Controllers active and standing reset acknowledged; starting navigation sensors')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
