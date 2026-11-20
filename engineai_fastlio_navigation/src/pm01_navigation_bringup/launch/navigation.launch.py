"""Public profile/mode assembly. LIO, map and actuator output have distinct owners."""
import asyncio
import importlib.util
import os
import signal
import uuid
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as share, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def setup(context):
    from launch.substitutions import LaunchConfiguration

    def value(key):
        return LaunchConfiguration(key).perform(context)
    (profile, mode) = (value('profile'), value('mode'))
    sim = profile == 'sim'
    session_id = value('session_id') or uuid.uuid4().hex
    if mode != 'saved':
        from pm01_navigation_runtime.build.slam_backend import require_backend
        require_backend(get_package_prefix('slam_toolbox'))
    clock = {'use_sim_time': sim}
    root = Path(share('pm01_navigation_bringup'))

    def selected(key, filename):
        return value(key) or str(root / 'config' / profile / filename)
    nav = selected('nav2_params', 'nav2.yaml')
    online = selected('map_params', 'online_map.yaml')
    ground = selected('obstacle_params', 'obstacles.yaml')
    auto = True
    # The hardware point operations are N x 3. Extra BLAS workers compete
    # with LIO and Nav2 on the Orin housekeeping cores.
    numeric_env = {} if sim else {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}
    nodes = [Node(package='pm01_navigation_client', executable='pm01_startup', output='screen',
                  parameters=[clock, {'mode': mode, 'session_id': session_id, 'status_file': value('startup_status_file')}])]
    sensor_nodes = []
    if value('start_sensors') == 'true':
        if sim:
            spec = importlib.util.spec_from_file_location('pm01_gazebo_stack', root / 'launch/sim/gazebo_stack.py')
            stack = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(stack)
            nodes += stack.sim_arguments(str(root / 'config/sim/rl_walking_pm01_nav.yaml'))
            nodes += [SetLaunchConfiguration('profile', ''), *stack.walking(start_adapter=False)]
            stand = stack.stand_up()
            (extr, transforms) = stack.lidar_tf()
            sensor_nodes += transforms + stack.fast_lio('map', 'navigation_sim.rviz', save_pcd=False, lio_params=selected('lio_params', 'lio.yaml'))[:-1]
        else:
            sensor_nodes += [Node(package='livox_ros_driver2', executable='livox_ros_driver2_node', name='livox_lidar_publisher', output='screen', parameters=[selected('driver_params', 'livox_driver.yaml'), {'user_config_path': selected('lidar_config', 'livox_network.json')}, clock])]
            with open(value('extrinsics') or root / 'config/pm01_lidar_tf.yaml') as f:
                extr = yaml.safe_load(f)
            extr['base_frame'] = 'base_footprint'
            for (child, parent, prefix) in [('livox_frame', 'base_footprint', 'lidar'), ('livox_imu', 'livox_frame', 'imu')]:
                (xyz, rpy) = (extr[prefix + '_xyz'], extr[prefix + '_rpy'])
                sensor_nodes.append(Node(package='tf2_ros', executable='static_transform_publisher', arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]), '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]), '--frame-id', parent, '--child-frame-id', child], parameters=[clock]))
            affinity = {}
            cpu = value('lio_cpu')
            if cpu:
                if not cpu.isdecimal() or int(cpu) not in os.sched_getaffinity(0):
                    raise ValueError('lio_cpu must be one CPU available to the navigation container')
                affinity = {'prefix': ['taskset -c ' + str(int(cpu))],
                            'additional_env': {'OMP_THREAD_LIMIT': '1', 'OMP_WAIT_POLICY': 'PASSIVE'}}
            sensor_nodes += [Node(package='fast_lio', executable='fastlio_mapping', **affinity, parameters=[str(Path(share('fast_lio')) / 'config/mid360.yaml'), selected('lio_params', 'lio.yaml'), clock]), Node(package='pm01_navigation_perception', executable='custommsg_to_cloud', parameters=[clock]), Node(package='pm01_navigation_hardware', executable='odom_relay', parameters=[extr, clock])]
        sensor_nodes += [Node(package='pm01_navigation_perception', executable='lio_cloud_adapter', parameters=[extr, clock])]
    pipeline = []
    if mode == 'saved':
        path = str(Path(value('map')).expanduser())
        if not value('map') or not Path(path).is_file():
            raise RuntimeError('saved mode requires map:=/absolute/path/map.yaml')
        pipeline += [Node(package='nav2_map_server', executable='map_server', name='map_server', parameters=[clock, {'yaml_filename': path, 'frame_id': 'map', 'topic_name': '/map'}]), Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', parameters=[clock, {'autostart': True, 'node_names': ['map_server', 'amcl']}])]
        pipeline += [Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[value('amcl_params') or str(root / 'config/matching/amcl.yaml'), clock])]
    else:
        pipeline += [Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', parameters=[value('slam_params') or str(root / 'config/matching/slam.yaml'), clock], remappings=[('pose', '/pm01/slam_pose')])]
    if auto:
        pipeline += [Node(package='pm01_navigation_core', executable='automatic_ground.py', additional_env=numeric_env, parameters=[str(root / 'config/automatic_ground.yaml'), clock])]
    pipeline += [Node(package='pm01_navigation_perception', executable='planar_scan.py', additional_env=numeric_env, parameters=[value('scan_params') or str(root / 'config/matching/scan.yaml'), clock, {'publish_mapping_scan': mode != 'saved', 'session_id': session_id}]), Node(package='pm01_navigation_core', executable='map_matching_adapter.py', parameters=[value('scan_params') or str(root / 'config/matching/scan.yaml'), clock, {'backend': 'amcl' if mode == 'saved' else 'slam', 'session_id': session_id}])]
    pipeline += [Node(package='pm01_navigation_core', executable='navigation_obstacle_cloud.py', additional_env=numeric_env, parameters=[ground, clock, {'use_shared_ground': auto}]), Node(package='pm01_navigation_core', executable='online_navigation_map.py', additional_env=numeric_env, parameters=[online, clock, {'fresh_map': False, 'prior_alignment_mode': 'map_matching', 'dynamic_seed': mode != 'saved', 'use_shared_ground': auto, 'save_root': str(Path(value('save_root')).expanduser())}])]
    if mode != 'mapping':
        pipeline += [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root / 'launch/nav2_stack.launch.py')), launch_arguments={'params_file': nav, 'use_sim_time': str(sim).lower(), 'autostart': 'true', 'use_composition': 'False', 'use_respawn': 'False'}.items())]
    if mode == 'mapping':
        pipeline += [Node(package='pm01_navigation_nav2', executable='navigation_command_gate',
                          parameters=[clock, {'publish_commands': False}])]
    worker = ['--ros-args', '-p', 'use_sim_time:=' + str(sim).lower()]
    if not sim:
        worker += ['-p', 'limit_velocity:=false', '-p', 'release_on_goal_success:=false', '-p', 'base_frame:=base_footprint']
    nodes += [Node(package='pm01_navigation_motion', executable='motion_manager.py', parameters=[clock, {'session_id': session_id, 'adapter': 'hardware' if not sim else 'sim', 'worker_arguments': worker, 'worker_executable': str(Path(get_package_prefix('pm01_navigation_sim' if sim else 'pm01_navigation_hardware')) / 'lib' / ('pm01_navigation_sim' if sim else 'pm01_navigation_hardware') / ('sim_cmd_vel_adapter' if sim else 'cmd_vel_bridge')), 'output_topic': '/pm01/walking_cmd_vel' if sim else '/motion/body_vel_cmd'}])]
    critical_nodes = sensor_nodes + [a for a in pipeline if isinstance(a, Node)] + [nodes[0], nodes[-1]]
    if sim:
        # Interactive launch assumes children already received the terminal's
        # SIGINT. A child still being spawned can miss it. Track these two
        # session owners by pidfd, including starts delivered after shutdown.
        owners = (nodes[0], nodes[-1])
        live = {}
        closing = False

        def stop_owner(fd):
            try:
                signal.pidfd_send_signal(fd, signal.SIGTERM)
            except ProcessLookupError:
                pass

        def started(event, context):
            try:
                fd = os.pidfd_open(event.pid)
            except ProcessLookupError:
                return
            live[event.action] = fd
            if closing or context.is_shutdown:
                stop_owner(fd)

        def exited(event, context):
            fd = live.pop(event.action, None)
            if fd is not None:
                os.close(fd)

        def close_owners(event, context):
            nonlocal closing
            closing = True
            # Give children that already received the terminal SIGINT time to
            # finish their normal shutdown. A child may have missed that signal.
            def finish(owner, fd):
                if live.get(owner) == fd:
                    stop_owner(fd)
            for owner, fd in live.items():
                asyncio.get_running_loop().call_later(2., finish, owner, fd)

        nodes[:0] = [RegisterEventHandler(OnShutdown(on_shutdown=close_owners)),
                      RegisterEventHandler(OnProcessStart(target_action=lambda a: a in owners, on_start=started)),
                      RegisterEventHandler(OnProcessExit(target_action=lambda a: a in owners, on_exit=exited))]
    if value('rviz') == 'true':
        pipeline += [Node(package='rviz2', executable='rviz2', parameters=[clock], arguments=['-d', str(root / 'config' / ('sim/navigation_sim.rviz' if sim else 'hardware/navigation.rviz'))])]
    if sim and value('start_sensors') == 'true':
        def after_standing(event, context):
            if context.is_shutdown:
                return []
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='Simulation standing preparation failed'))]
            return sensor_nodes + [TimerAction(period=3.0, cancel_on_shutdown=True, actions=[
                OpaqueFunction(function=lambda ctx: [] if ctx.is_shutdown else pipeline)])]
        nodes += [RegisterEventHandler(OnProcessExit(target_action=stand, on_exit=after_standing)), stand]
    else:
        nodes += sensor_nodes + pipeline
    # A failed runtime process ends this owned launch rather than leaving a partial stack.
    for action in critical_nodes:
        if isinstance(action, Node):
            def ended(event, context):
                if context.is_shutdown:
                    return []
                return [EmitEvent(event=Shutdown(reason='Navigation process exited: '+str(event.action.process_details.get('name', 'node'))))]
            nodes.insert(0, RegisterEventHandler(OnProcessExit(target_action=action, on_exit=ended)))
    return nodes

def generate_launch_description():
    return LaunchDescription([SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'), DeclareLaunchArgument('profile', default_value='sim', choices=['sim', 'hardware']), DeclareLaunchArgument('session_id', default_value=''), DeclareLaunchArgument('startup_status_file', default_value=''), DeclareLaunchArgument('mode', default_value='fresh', choices=['mapping', 'fresh', 'saved']), DeclareLaunchArgument('localization_backend', default_value='slam_amcl', choices=['slam_amcl']), DeclareLaunchArgument('slam_params', default_value=''), DeclareLaunchArgument('amcl_params', default_value=''), DeclareLaunchArgument('scan_params', default_value=''), DeclareLaunchArgument('map', default_value=''), DeclareLaunchArgument('start_sensors', default_value='true', choices=['true', 'false']), DeclareLaunchArgument('gui', default_value='true'), DeclareLaunchArgument('render_backend', default_value='auto', choices=['auto', 'nvidia', 'system'], description='Simulation graphics selection; hardware profile is unchanged'), DeclareLaunchArgument('rviz', default_value='true'), DeclareLaunchArgument('world', default_value=str(Path(share('pm01_navigation_bringup')) / 'worlds/ground_dense.world')), DeclareLaunchArgument('nav2_params', default_value=''), DeclareLaunchArgument('map_params', default_value=''), DeclareLaunchArgument('obstacle_params', default_value=''), DeclareLaunchArgument('driver_params', default_value=''), DeclareLaunchArgument('lidar_config', default_value=''), DeclareLaunchArgument('lio_params', default_value=''), DeclareLaunchArgument('lio_cpu', default_value='', description='Optional CPU allocated to hardware LIO'), DeclareLaunchArgument('extrinsics', default_value=''), DeclareLaunchArgument('save_root', default_value=str(Path.home() / '.local/share/pm01_navigation/maps')), OpaqueFunction(function=setup)])
