"""Session startup progress using the navigation gate's authoritative decision."""
import json
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Header
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from .navigator import diagnostic_fields, has_free_cells


def startup_decision(mode, *, observations, nodes, motion, map_loaded, localized):
    """Orchestration only; observation health belongs to navigation_command_gate."""
    # Nav2 costmaps can wait for map->base TF while activating. Saved mode
    # therefore prompts for initialization as soon as AMCL and its map are ready.
    if (mode == 'saved' and map_loaded and not localized and motion and
            all(nodes.get(name, False) for name in ('map_server', 'amcl'))):
        return 'initial_pose', '地图已加载，请在 RViz 设置初始位置 / Set the initial pose in RViz'
    missing = [name for name, active in nodes.items() if not active and
               name not in ('smoother_server', 'waypoint_follower')]
    if missing:
        return 'waiting_nodes', '等待节点激活 / Waiting for nodes: '+', '.join(missing)
    if not motion:
        return 'waiting_motion', '等待运动管理服务 / Waiting for motion manager'
    if not observations.get('ready'):
        return 'waiting_data', '等待可靠观测 / Waiting for observations: '+observations.get('reason', 'sensors, ground, map and TF')
    if not map_loaded or not localized:
        return 'waiting_map', '等待地图与 TF / Waiting for map and TF'
    if mode == 'mapping':
        return 'ready', '建图已就绪，可以开始采集 / Mapping ready'
    prefix = '定位与导航已就绪' if mode == 'saved' else '导航已就绪'
    return 'ready', prefix+'，可以执行 enable-motion / Ready for enable-motion'


class StartupMonitor(Node):
    def __init__(self):
        super().__init__('navigation_startup')
        self.declare_parameter('mode', 'fresh')
        self.declare_parameter('session_id', '')
        self.declare_parameter('status_file', '')
        self.status_file = self.get_parameter('status_file').value
        self.mode = self.get_parameter('mode').value
        self.session_id = self.get_parameter('session_id').value
        names = [] if self.mode == 'mapping' else [
            'controller_server', 'smoother_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother']
        if self.mode == 'saved':
            names += ['map_server', 'amcl']
        self.motion = {}
        self.motion_received = None
        self.navigation_readiness = {}
        self.navigation_readiness_received = None
        self.map_has_free_cells = False
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.motion_request_client = self.create_client(SetParametersAtomically, '/pm01/navigation/request_motion')
        self.motion_clients = [self.create_client(Trigger, '/pm01/navigation/'+name)
                               for name in ('enable_motion', 'release_motion')]
        self.create_subscription(DiagnosticArray, '/pm01/navigation/motion_status',
                                 lambda msg: self.receive_status('motion', msg), 1)
        self.create_subscription(DiagnosticArray, '/pm01/navigation_readiness',
                                 lambda msg: self.receive_status('navigation_readiness', msg), 1)
        self.seed_loaded = False
        if self.mode != 'saved':
            self.create_subscription(OccupancyGrid, '/navigation_map', self.live_map,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self.mode == 'saved':
            self.create_subscription(OccupancyGrid, '/map', self.seed_map,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.lifecycle = {name: self.create_client(GetState, '/'+name+'/get_state') for name in names}
        self.lifecycle_values = {}
        self.lifecycle_pending = {}
        self.lifecycle_last_request = {}
        self.startup_pub = self.create_publisher(DiagnosticArray, '/pm01/startup_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.since = time.monotonic()
        self.has_been_ready = False
        self.last_message = None
        self.get_logger().info('正在启动传感器与定位，运动输出关闭 / Starting sensors and localization; output disabled')
        self.create_timer(.2, self.update_startup, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive_status(self, key, msg):
        if msg.status:
            setattr(self, key, diagnostic_fields(msg.status[-1]))
            setattr(self, key+'_received', time.monotonic())

    def live_map(self, msg):
        self.map_has_free_cells = has_free_cells(msg.data)

    def seed_map(self, msg):
        self.seed_loaded = msg.info.width > 0 and msg.info.height > 0 and bool(msg.data)

    def update_startup(self):
        now = time.monotonic()
        for name, client in self.lifecycle.items():
            pending = self.lifecycle_pending.get(name)
            if pending:
                future, requested = pending
                if future.done():
                    try:
                        self.lifecycle_values[name] = (future.result().current_state.id == 3, now)
                    except Exception:
                        self.lifecycle_values[name] = (False, now)
                    del self.lifecycle_pending[name]
                elif now-requested > 2.:
                    client.remove_pending_request(future)
                    future.cancel()
                    del self.lifecycle_pending[name]
            if (name not in self.lifecycle_pending and client.service_is_ready() and
                    now-self.lifecycle_last_request.get(name, -float('inf')) >= 1.):
                self.lifecycle_pending[name] = (client.call_async(GetState.Request()), now)
                self.lifecycle_last_request[name] = now
        nodes = {name: self.lifecycle_values.get(name, (False, 0))[0] and
                 now-self.lifecycle_values[name][1] < 2.5 for name in self.lifecycle}
        motion = (self.motion.get('session_id') == self.session_id and self.motion_received is not None and
                  now-self.motion_received < 2.5 and self.motion_request_client.service_is_ready() and
                  all(c.service_is_ready() for c in self.motion_clients))
        observations = self.navigation_readiness if self.navigation_readiness_received is not None and (
            now-self.navigation_readiness_received < 2.5) else {'ready': False, 'reason': 'readiness_status_stale'}
        localized = self.tf.can_transform('map', 'base_footprint', rclpy.time.Time())
        phase, message = startup_decision(self.mode, observations=observations, nodes=nodes,
            motion=motion, map_loaded=self.seed_loaded if self.mode == 'saved' else self.map_has_free_cells, localized=localized)
        # The navigation readiness publisher already applies the stable period.
        if phase == 'ready':
            self.has_been_ready = True
        elif self.has_been_ready and phase != 'initial_pose':
            phase = 'waiting_recovery'
            message = '导航暂未就绪 / Navigation temporarily unavailable: '+message
        elif now-self.since >= 120. and phase not in ('initial_pose', 'settling'):
            phase = 'startup_delayed'
            message = '启动等待超时 / Startup delayed: '+message
        values = {'session_id': self.session_id, 'mode': self.mode, 'phase': phase,
                  'ready': phase == 'ready', 'reason': message, 'nodes': nodes,
                  'manager_id': self.motion.get('manager_id', ''),
                  'release_generation': self.motion.get('release_generation'),
                  'output_enabled': self.motion.get('state') == 'enabled',
                  'elapsed_sec': now-self.since}
        if self.status_file:
            try:
                path = Path(self.status_file)
                temporary = path.with_suffix('.tmp')
                temporary.write_text(json.dumps(dict(values, heartbeat_wall_sec=time.time())))
                temporary.replace(path)
            except OSError as exc:
                self.get_logger().warning('Startup status file: '+str(exc), throttle_duration_sec=10.)
        status = DiagnosticStatus(name='pm01/startup', level=DiagnosticStatus.OK if phase == 'ready' else DiagnosticStatus.WARN,
            message=message, values=[KeyValue(key=k, value=json.dumps(v)) for k, v in values.items()])
        self.startup_pub.publish(DiagnosticArray(header=Header(stamp=self.get_clock().now().to_msg()), status=[status]))
        if message != self.last_message:
            self.get_logger().info(message)
            self.last_message = message


def main():
    rclpy.init()
    node = StartupMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
