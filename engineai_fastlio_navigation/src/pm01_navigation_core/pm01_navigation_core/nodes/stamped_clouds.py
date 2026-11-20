"""Select fresh clouds with both transforms available at the measurement time."""
from collections import deque
import time

from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class _Listener(TransformListener):
    def __init__(self, buffer, node, wake):
        self.wake = wake
        super().__init__(buffer, node)

    def callback(self, data):
        super().callback(data)
        self.wake()

    def static_callback(self, data):
        super().static_callback(data)
        self.wake()


class StampedClouds:
    """Called by one node executor. Coalesce input notifications outside TF callbacks."""
    def __init__(self, node, update, depth):
        self.node = node
        self.queue = deque(maxlen=depth)
        self.next_fit_ns = 0
        self.period_ns = round(1e9 / node.config['frequency'])
        self.max_age_ns = round(1e9 * node.config['max_data_age'])
        self.last_used_ns = -1
        self.waiting_tf = False
        self.state = 'waiting_for_new_cloud'
        self.last_input_ns = self.arrival_age = self.queue_wait = None
        self.guard = node.create_guard_condition(update)
        self.tf = Buffer()
        self.listener = _Listener(self.tf, node, self.on_tf)

    def append(self, msg):
        when = Time.from_msg(msg.header.stamp)
        received = self.node.get_clock().now().nanoseconds
        self.last_input_ns = when.nanoseconds
        self.arrival_age = (received - when.nanoseconds) / 1e9
        self.queue.append((msg, when, received, time.monotonic()))
        self.wake()

    def wake(self):
        if self.queue and self.node.get_clock().now().nanoseconds >= self.next_fit_ns:
            self.guard.trigger()

    def on_tf(self):
        if self.waiting_tf:
            self.wake()

    def select(self, now_ns):
        self.waiting_tf = False
        self.state = 'waiting_for_new_cloud'
        if now_ns < self.next_fit_ns:
            self.state = 'between_updates'
            return None
        c = self.node.config
        newest_reason = None
        for msg, when, received, arrival in reversed(self.queue):
            stamp = when.nanoseconds
            if stamp <= self.last_used_ns:
                continue
            if stamp <= 0 or now_ns - stamp > self.max_age_ns:
                reason = ('cloud_stale_on_arrival' if received - stamp > self.max_age_ns
                          else 'cloud_expired_in_queue')
            elif stamp > now_ns:
                reason = 'waiting_for_cloud_time'
            else:
                reason = 'waiting_for_lidar_tf'
                try:
                    sensor = self.tf.lookup_transform(c['reference_frame'], msg.header.frame_id, when)
                    reason = 'waiting_for_base_tf'
                    base = self.tf.lookup_transform(c['reference_frame'], c.get('base_frame', 'base_footprint'), when)
                except TransformException:
                    self.waiting_tf = True
                else:
                    self.state = 'ready'
                    self.queue_wait = time.monotonic() - arrival
                    return msg, stamp / 1e9, sensor.transform, base.transform.translation
            if newest_reason is None:
                newest_reason = reason
        if newest_reason:
            self.state = newest_reason
        return None

    def consumed(self, msg, now_ns):
        self.last_used_ns = Time.from_msg(msg.header.stamp).nanoseconds
        self.next_fit_ns = now_ns + self.period_ns
        while self.queue and self.queue[0][1].nanoseconds <= self.last_used_ns:
            self.queue.popleft()

    def diagnostics(self):
        return dict(input_state=self.state,
                    last_input_stamp=None if self.last_input_ns is None else self.last_input_ns / 1e9,
                    input_age_at_receive_sec=self.arrival_age, selected_cloud_wait_sec=self.queue_wait)
