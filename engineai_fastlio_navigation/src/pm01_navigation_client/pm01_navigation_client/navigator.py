"""Thin Nav2 client. Call poll() or add this node to your own executor.

No background executor is created. Do not concurrently call poll()/blocking
helpers if an external executor already owns this node; use the async methods.
"""
import json
import math
import os
import time
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from unique_identifier_msgs.msg import UUID
import rclpy
from .task import NavigationTask


def diagnostic_fields(status):
    values = {}
    for item in status.values:
        try:
            values[item.key] = json.loads(item.value)
        except (ValueError, TypeError):
            values[item.key] = item.value
    values['message'] = status.message
    return values


def has_free_cells(data):
    # Both OccupancyGrid and its binary representation encode free as 0.
    # Weighted maps can also represent a traversable cell with values 1..64.
    if hasattr(data, 'tobytes') and data.tobytes().find(b'\0') >= 0:
        return True
    return any(0 <= v < 65 for v in data)


class Navigator(Node):
    def __init__(self, node_name='pm01_navigation_client', **kwargs):
        if 'PM01_USE_SIM_TIME' in os.environ:
            from rclpy.parameter import Parameter
            kwargs.setdefault('parameter_overrides', [Parameter('use_sim_time', value=os.environ['PM01_USE_SIM_TIME'].lower()=='true')])
        super().__init__(node_name, **kwargs)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.relocalize_client = self.create_client(Empty, '/reinitialize_global_localization')
        self.trigger_clients = {name: self.create_client(Trigger, endpoint) for name, endpoint in {
            'save_map': '/pm01/save_navigation_map',
            'enable_motion': '/pm01/navigation/enable_motion',
            'release_motion': '/pm01/navigation/release_motion'}.items()}
        self.motion_request_client = self.create_client(SetParametersAtomically, '/pm01/navigation/request_motion')
        self.startup = {}
        self.startup_received = None
        self.tasks = {}
        self.motion = {}
        self.motion_received = None
        self.localization = {}
        self.localization_received = None
        self.navigation_readiness = {}
        self.navigation_readiness_received = None
        self.navigation_wait = {}
        self.navigation_wait_received = None
        self.map_stamp = None
        self.map_received = None
        self.map_has_free_cells = False
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(DiagnosticArray, '/pm01/startup_status',
                                 lambda msg: self._navigation_diagnostic('startup', msg), QoSProfile(depth=1))
        self.create_subscription(DiagnosticArray, '/pm01/navigation/motion_status', self._motion, 10)
        self.create_subscription(DiagnosticArray, '/pm01/localization_status', self._localization, 10)
        self.create_subscription(DiagnosticArray, '/pm01/navigation_readiness',
                                 lambda msg: self._navigation_diagnostic('navigation_readiness', msg), 10)
        self.create_subscription(DiagnosticArray, '/pm01/navigation_wait_status',
                                 lambda msg: self._navigation_diagnostic('navigation_wait', msg),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(OccupancyGrid, '/navigation_map', self._map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def poll(self, timeout=0.1):
        rclpy.spin_once(self, timeout_sec=max(0., timeout))

    def _map(self, msg):
        self.map_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.map_received = time.monotonic()
        self.map_has_free_cells = has_free_cells(msg.data)

    def _motion(self, msg):
        for status in msg.status:
            self.motion_received = time.monotonic()
            self.motion = diagnostic_fields(status)

    def _localization(self, msg):
        for status in msg.status:
            self.localization_received = time.monotonic()
            self.localization = diagnostic_fields(status)

    def _navigation_diagnostic(self, name, msg):
        # These describe the current navigation server, not a task result.
        # Only the Action result may finish a NavigationTask.
        for status in msg.status:
            values = diagnostic_fields(status)
            setattr(self, name, values)
            setattr(self, name+'_received', time.monotonic())

    def startup_ready(self):
        return (self.startup_received is not None and time.monotonic()-self.startup_received < 2.5 and
                self.startup.get('ready') is True and bool(self.startup.get('session_id')) and
                bool(self.startup.get('manager_id')) and self.startup.get('release_generation') is not None)

    def wait_until_ready(self, timeout=30., require_map=True):
        """Wait for the shared readiness decision and the action server."""
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            if self.action.server_is_ready() and (not require_map or self.startup_ready()):
                return True
            self.poll(min(.1, max(0., end-time.monotonic())))
        return False

    def pose(self, x, y, yaw=0., frame='map'):
        if not all(math.isfinite(float(v)) for v in (x, y, yaw)):
            raise ValueError('Pose must be finite; yaw is in radians')
        p = PoseStamped()
        p.header.frame_id = frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        return p

    def set_initial_pose(self, x, y, yaw=0., position_std=.25, yaw_std=.2):
        """Publish a timestamped /initialpose. Publication is not alignment ACK."""
        if self.get_parameter('use_sim_time').value and self.get_clock().now().nanoseconds==0:
            raise RuntimeError('Simulation clock not received; poll before publishing initial pose')
        p = self.pose(x, y, yaw)
        msg = PoseWithCovarianceStamped(header=p.header)
        msg.pose.pose = p.pose
        msg.pose.covariance[0] = msg.pose.covariance[7] = position_std**2
        msg.pose.covariance[35] = yaw_std**2
        self.initial_pub.publish(msg)
        return msg

    def navigate_to_pose(self, x, y=None, yaw=0.):
        """Return immediately with a task UUID, even before server acceptance."""
        if any(not t.done for t in self.tasks.values()):
            raise RuntimeError('Finish or cancel the current task before submitting another')
        if not self.action.server_is_ready() or not self.startup_ready():
            raise RuntimeError('Navigation is not ready: '+self.startup.get('reason', 'wait_until_ready first'))
        p = x if isinstance(x, PoseStamped) else self.pose(x, y, yaw)
        task = NavigationTask()
        self.tasks[task.uuid] = task
        goal = NavigateToPose.Goal(pose=p)
        task.send_future = self.action.send_goal_async(goal,
            feedback_callback=lambda msg: setattr(task, 'feedback', msg.feedback),
            goal_uuid=UUID(uuid=list(bytes.fromhex(task.uuid))))
        task.send_future.add_done_callback(lambda future: self._accepted(task, future))
        return task

    def _accepted(self, task, future):
        try:
            task.goal_handle = future.result()
            if not task.goal_handle.accepted:
                task.state = 'rejected'
                return
            task.state = 'canceling' if task.cancel_requested else 'executing'
            task.result_future = task.goal_handle.get_result_async()
            task.result_future.add_done_callback(lambda f: self._result(task, f))
            if task.cancel_requested:
                self._send_cancel(task)
        except Exception as exc:
            task.state, task.error = 'unknown', str(exc)

    def _result(self, task, future):
        try:
            response = future.result()
            task.result = response
            task.state = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                          GoalStatus.STATUS_CANCELED: 'canceled',
                          GoalStatus.STATUS_ABORTED: 'aborted'}.get(response.status, 'unknown')
        except Exception as exc:
            task.state, task.error = 'unknown', str(exc)

    def cancel(self, task):
        """Cancel this UUID, including a goal whose acceptance arrives later."""
        if isinstance(task, str):
            task = self.tasks[task]
        if task.done:
            return task
        task.cancel_requested = True
        task.state = 'canceling'
        if task.goal_handle is not None and task.cancel_future is None:
            self._send_cancel(task)
        return task

    def _send_cancel(self, task):
        task.cancel_future = task.goal_handle.cancel_goal_async()
        def acknowledged(future):
            try:
                task.cancel_response = future.result()
            except Exception as exc:
                task.error = 'Cancel acknowledgment unknown: ' + str(exc)
            # Only the final action result can confirm canceled.
        task.cancel_future.add_done_callback(acknowledged)

    def wait(self, task, timeout=120.):
        end = time.monotonic()+timeout
        while rclpy.ok() and not task.done and time.monotonic() < end:
            self.poll(min(.1, max(0., end-time.monotonic())))
        return task.done

    def _trigger(self, name):
        client = self.trigger_clients[name]
        if not client.service_is_ready():
            raise RuntimeError(name + ' service unavailable')
        return client.call_async(Trigger.Request())

    def save_map(self):
        """Future result: success plus saved map.yaml path in message."""
        return self._trigger('save_map')

    def enable_motion(self, timeout=8.):
        """Submit one session-bound request; result retains the Trigger response API."""
        if not self.startup_ready():
            raise RuntimeError('Navigation startup incomplete: '+self.startup.get('reason', 'waiting for startup status'))
        if not self.motion_request_client.service_is_ready():
            raise RuntimeError('Motion request service unavailable')
        self._enable_context = {k: self.startup[k] for k in ('session_id', 'manager_id', 'release_generation')}
        values = dict(self._enable_context, operation='enable', timeout_sec=float(timeout))
        request = SetParametersAtomically.Request(parameters=[
            Parameter(k, value=v).to_parameter_msg() for k, v in values.items()])
        result = Future()
        future = self.motion_request_client.call_async(request)
        def completed(f):
            try:
                response = f.result().result
                result.set_result(Trigger.Response(success=response.successful, message=response.reason))
            except Exception as exc:
                result.set_exception(exc)
        future.add_done_callback(completed)
        return result

    def cancel_pending_enable(self):
        """Fence an uncertain request without affecting a replacement session."""
        values = dict(self._enable_context, operation='cancel', timeout_sec=0.)
        return self.motion_request_client.call_async(SetParametersAtomically.Request(parameters=[
            Parameter(k, value=v).to_parameter_msg() for k, v in values.items()]))

    def release_motion(self):
        return self._trigger('release_motion')

    def relocalize(self):
        """Request AMCL global particle initialization, without sending motion.

        The Empty service ACK is not convergence. Inspect localization status
        and map->base alignment before sending the next navigation goal.
        """
        if not self.relocalize_client.service_is_ready():
            raise RuntimeError('AMCL global relocalization service unavailable')
        return self.relocalize_client.call_async(Empty.Request())

    def get_status(self):
        now = self.get_clock().now().nanoseconds*1e-9
        clock_ready = not self.get_parameter('use_sim_time').value or now>0
        transforms = {}
        for parent,child in [('map','odom'),('odom','base_footprint'),('map','base_footprint')]:
            key = parent + '->' + child
            try:
                msg = self.tf.lookup_transform(parent, child, Time())
                p,q = msg.transform.translation, msg.transform.rotation
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
                transforms[key] = {'xyz_m': [p.x,p.y,p.z],
                    'yaw_rad': math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                    'stamp_sec': stamp, 'age_sec': None if stamp==0 or not clock_ready else now-stamp}
            except TransformException as exc:
                transforms[key] = {'unavailable': str(exc)}
        return {'startup': dict(self.startup),
                'startup_receive_age_sec': None if self.startup_received is None else time.monotonic()-self.startup_received,
                'action_server_ready': self.action.server_is_ready(), 'tf': transforms, 'clock_ready': clock_ready,
                'map_age_sec': None if self.map_stamp is None or not clock_ready else now-self.map_stamp,
                'map_has_free_cells': self.map_has_free_cells,
                'map_receive_age_sec': None if self.map_received is None else time.monotonic()-self.map_received,
                'motion': dict(self.motion),
                'motion_receive_age_sec': None if self.motion_received is None else time.monotonic()-self.motion_received,
                'localization': dict(self.localization),
                'localization_receive_age_sec': (None if self.localization_received is None else
                                                time.monotonic()-self.localization_received),
                'navigation_readiness': dict(self.navigation_readiness),
                'navigation_readiness_receive_age_sec': (None if self.navigation_readiness_received is None else
                                                        time.monotonic()-self.navigation_readiness_received),
                'navigation_wait': dict(self.navigation_wait),
                'navigation_wait_receive_age_sec': (None if self.navigation_wait_received is None else
                                                   time.monotonic()-self.navigation_wait_received),
                'tasks': {key: {'state': t.state, 'error': t.error} for key,t in self.tasks.items()}}

    def destroy_node(self):
        self.action.destroy()
        return super().destroy_node()
