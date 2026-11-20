#!/usr/bin/env python3
"""ROS-only output ownership. No SSH, sudo, systemd, or broad process kills."""
import json
import math
import os
import signal
from pathlib import Path
import threading
import time
import uuid
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from pm01_navigation_runtime.system.processes import OutputProcess


class MotionManager(Node):
    def __init__(self):
        super().__init__('motion_manager')
        self.declare_parameter('adapter', 'sim')
        self.declare_parameter('session_id', '')
        self.session_id = self.get_parameter('session_id').value
        self.manager_id = uuid.uuid4().hex
        self.startup = {}
        self.startup_received = None
        self.control_lock = threading.RLock()
        self.shutting_down = False
        self.declare_parameter('worker_executable', '')  # Test/custom adapter path, never shell-evaluated.
        self.declare_parameter('worker_arguments', ['--ros-args'])
        self.declare_parameter('output_topic', '/pm01/walking_cmd_vel')
        self.declare_parameter('enable_timeout_sec', 8.0)
        self.declare_parameter('log_root', str(Path.home()/'.local/share/pm01_navigation/logs'))
        self.worker = OutputProcess()
        self.lock = threading.Lock()
        self.active_goals = set()
        self.cmd_nonzero = False
        self.cmd_seen = None
        self.state = 'disabled'
        self.cancel_state = 'unknown'
        self.release_generation = 0
        self.cancel_deadline = None
        self.log_file = None
        self.enable_cancel = threading.Event()
        self.transport_ready = False
        self.enable_error = ''
        group = self.worker_group = ReentrantCallbackGroup()
        self.create_service(SetParametersAtomically, '/pm01/navigation/request_motion', self.request_motion, callback_group=group)
        self.create_subscription(DiagnosticArray, '/pm01/startup_status', self.startup_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL), callback_group=group)
        self.create_service(Trigger, '/pm01/navigation/enable_motion', self.enable, callback_group=group)
        self.create_service(Trigger, '/pm01/navigation/release_motion', self.release, callback_group=group)
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal', callback_group=group)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL), callback_group=group)
        self.create_subscription(Twist, '/cmd_vel', self.command_callback, 1, callback_group=group)
        self.pub = self.create_publisher(DiagnosticArray, '/pm01/navigation/motion_status', 10)
        self.create_timer(.2, self.publish_status, callback_group=group,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def status_callback(self, msg):
        self.active_goals = {bytes(s.goal_info.goal_id.uuid).hex() for s in msg.status_list if s.status in (1,2,3)}
        if not self.active_goals and self.cancel_state == 'accepted':
            self.cancel_state = 'confirmed'

    def command_callback(self, msg):
        self.cmd_nonzero = any(abs(v)>1e-5 for v in (msg.linear.x,msg.linear.y,msg.angular.z))
        self.cmd_seen = time.monotonic()

    def output_publishers(self):
        return self.get_publishers_info_by_topic(self.get_parameter('output_topic').value)

    def snapshot(self):
        if self.cancel_deadline and time.monotonic()>self.cancel_deadline and self.cancel_state in ('pending','accepted'):
            self.cancel_state = 'unknown_timeout'
        publishers = self.output_publishers()
        return {'session_id': self.session_id, 'manager_id': self.manager_id,
                'release_generation': self.release_generation, 'state': self.state, 'output_process_running': self.worker.running,
                'output_pid': self.worker.process.pid if self.worker.running else None,
                'output_publishers': len(publishers),
                'output_detached': not self.worker.running and not publishers,
                'goal_cancel': self.cancel_state, 'active_goals': sorted(self.active_goals),
                'output_transport_ready': self.transport_ready and self.worker.running,
                'last_enable_error': self.enable_error,
                'physical_controller_owner': 'unknown'}

    def await_worker_ready(self, service, timeout):
        """A unique per-attempt service cannot be satisfied by an old worker."""
        client = self.create_client(Trigger, service, callback_group=self.worker_group)
        future = None
        deadline = time.monotonic()+timeout
        reason = 'worker_ready_service_unavailable'
        received_reply = False
        requested = None
        try:
            while time.monotonic() < deadline:
                if self.enable_cancel.is_set():
                    raise RuntimeError('Enable interrupted by release')
                if not self.worker.running:
                    raise RuntimeError('Output worker exited before ready; see motion-output.log')
                if future is None and client.service_is_ready():
                    future = client.call_async(Trigger.Request())
                    requested = time.monotonic()
                    if not received_reply:
                        reason = 'waiting_for_worker_ready_response'
                if future is not None and future.done():
                    result = future.result()
                    if result.success:
                        return
                    reason = result.message
                    received_reply = True
                    future = None
                elif future is not None and time.monotonic()-requested >= 1.:
                    # Readiness is an idempotent query on the same worker. DDS
                    # can discover its request reader before the response writer.
                    client.remove_pending_request(future)
                    future.cancel()
                    future = None
                time.sleep(.05)
            raise TimeoutError('Output not ready: '+reason+'; check DDS/consumer, then retry enable-motion')
        finally:
            if future is not None and not future.done():
                client.remove_pending_request(future)
                future.cancel()
            self.destroy_client(client)

    def startup_callback(self, msg):
        for status in msg.status:
            try:
                values = {v.key: json.loads(v.value) for v in status.values}
            except (ValueError, TypeError):
                continue
            if values.get('session_id') == self.session_id:
                self.startup, self.startup_received = values, time.monotonic()

    def request_motion(self, request, response):
        try:
            values = {p.name: Parameter.from_parameter_msg(p).value for p in request.parameters}
            if set(values) != {'session_id', 'manager_id', 'release_generation', 'operation', 'timeout_sec'}:
                raise ValueError('Invalid motion request')
            with self.control_lock:
                if (values['session_id'] != self.session_id or values['manager_id'] != self.manager_id or
                        values['release_generation'] != self.release_generation):
                    response.result = SetParametersResult(successful=values.get('operation') == 'cancel',
                        reason='Motion request canceled by release or session change')
                    return response
                generation = self.release_generation
            if values['operation'] == 'cancel':
                result = self.release(None, Trigger.Response(), generation=generation)
            elif values['operation'] == 'enable':
                result = self.enable(None, Trigger.Response(), generation=generation, budget=values['timeout_sec'])
            else:
                raise ValueError('Unknown motion operation')
            response.result = SetParametersResult(successful=result.success, reason=result.message)
        except Exception as exc:
            response.result = SetParametersResult(successful=False, reason=str(exc))
        return response

    def enable(self, request, response, generation=None, budget=None):
        if not self.lock.acquire(blocking=False):
            response.success, response.message = False, 'Output transition in progress'
            return response
        try:
            with self.control_lock:
                if self.shutting_down or (generation is not None and generation != self.release_generation):
                    raise RuntimeError('Motion request canceled')
                if self.session_id and (self.startup_received is None or time.monotonic()-self.startup_received > 2.5 or
                                        not self.startup.get('ready')):
                    raise RuntimeError('Startup not ready: '+self.startup.get('reason', 'waiting for status'))
                self.enable_cancel.clear()
            if self.worker.running:
                response.success, response.message = False, 'Already enabled; duplicate output refused'
            elif self.active_goals:
                response.success, response.message = False, 'Old task is still active; cancel and await its final result'
            elif self.cmd_nonzero and self.cmd_seen and time.monotonic()-self.cmd_seen<.5:
                response.success, response.message = False, 'Nonzero cached command; wait for idle /cmd_vel'
            elif self.output_publishers():
                response.success, response.message = False, 'An actuator publisher already exists; output ownership unknown'
            elif self.cancel_state in ('pending','accepted','unknown_timeout'):
                response.success, response.message = False, 'Previous task cancellation is unconfirmed; release again after server recovery'
            else:
                executable = self.get_parameter('worker_executable').value
                if not executable:
                    raise ValueError('bringup must supply worker_executable for the selected platform')
                args = list(self.get_parameter('worker_arguments').value)
                timeout = float(self.get_parameter('enable_timeout_sec').value)
                if budget is not None:
                    timeout = min(120., float(budget))
                if not math.isfinite(timeout) or timeout <= 0:
                    raise ValueError('enable_timeout_sec must be finite and positive')
                ready_service = '/pm01/navigation/output_ready_'+uuid.uuid4().hex
                args += ['--ros-args', '-p', 'ready_service:='+ready_service]
                # A new process starts with no cached Twist. It publishes only zero
                # until a fresh command arrives; no cached Twist is forwarded here.
                root = Path(self.get_parameter('log_root').value).expanduser()
                root.mkdir(parents=True, exist_ok=True)
                if self.log_file: self.log_file.close()
                self.log_file = open(root/'motion-output.log','a', buffering=1)
                self.state, self.transport_ready, self.enable_error = 'enabling', False, ''
                self.worker.start([executable, *args], self.log_file)
                try:
                    self.await_worker_ready(ready_service, timeout)
                    if self.enable_cancel.is_set():
                        raise RuntimeError('Enable interrupted by release')
                except Exception:
                    stopped = self.worker.stop(2.)
                    self.state = 'enable_failed' if stopped else 'stop_unconfirmed'
                    raise
                self.transport_ready = True
                self.state = 'enabled'
                self.cancel_state = 'not_requested'
                response.success, response.message = True, json.dumps(self.snapshot())
        except Exception as exc:
            self.enable_error = str(exc)
            if self.state == 'enabling':
                self.state = 'enable_failed'
            self.get_logger().error(self.enable_error)
            response.success, response.message = False, json.dumps(self.snapshot())
        finally:
            self.lock.release()
        return response

    def release(self, request, response, generation=None):
        # Never wait for a navigation action result while holding output ownership.
        with self.control_lock:
            if generation is not None and generation != self.release_generation:
                response.success, response.message = True, 'Request already canceled'
                return response
            self.enable_cancel.set()
            self.release_generation += 1
        if not self.lock.acquire(timeout=2.5):
            response.success, response.message = False, 'Output transition in progress; retry release'
            return response
        try:
            self.state = 'releasing'
            self.transport_ready = False
            stopped = self.worker.stop(2.)
            self.state = 'disabled' if stopped else 'stop_unconfirmed'
            generation = self.release_generation
            if self.cancel_client.service_is_ready():
                self.cancel_state = 'pending'
                self.cancel_deadline = time.monotonic()+3.
                future = self.cancel_client.call_async(CancelGoal.Request())
                def done(f):
                    if generation != self.release_generation: return
                    try:
                        result = f.result()
                        if result.return_code == 0:
                            self.cancel_state = 'accepted' if result.goals_canceling else 'confirmed'
                        elif result.return_code == 2 and not self.active_goals:
                            self.cancel_state = 'confirmed'
                        else: self.cancel_state = 'unknown_rejected'
                    except Exception:
                        self.cancel_state = 'unknown_error'
                future.add_done_callback(done)
            else:
                self.cancel_state = 'unknown_server_unavailable'
            # success means our process exited, NOT that the controller physically
            # handed control to a gamepad or the navigation goal was canceled.
            response.success, response.message = stopped, json.dumps(self.snapshot())
        finally:
            self.lock.release()
        return response

    def publish_status(self):
        if self.state=='enabled' and not self.worker.running:
            self.state='output_exited'
            self.transport_ready = False
        values = self.snapshot()
        level = DiagnosticStatus.ERROR if self.state in ('enable_failed', 'output_exited', 'stop_unconfirmed') else DiagnosticStatus.OK
        status=DiagnosticStatus(name='pm01_motion_output', level=level, message=self.state,
            values=[KeyValue(key=k,value=json.dumps(v)) for k,v in values.items()])
        self.pub.publish(DiagnosticArray(header=__import__('std_msgs.msg',fromlist=['Header']).Header(
            stamp=self.get_clock().now().to_msg()), status=[status]))

    def begin_shutdown(self):
        with self.control_lock:
            self.shutting_down = True
            self.enable_cancel.set()
            self.release_generation += 1

    def destroy_node(self):
        self.begin_shutdown()
        self.worker.stop(2.)
        if self.log_file: self.log_file.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node=MotionManager(); executor=MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    try: executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.begin_shutdown()
        try:
            executor.shutdown(timeout_sec=4.)
        finally:
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
