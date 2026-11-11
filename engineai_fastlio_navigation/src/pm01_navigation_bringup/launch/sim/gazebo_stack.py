"""Shared Gazebo pieces for the Fast-LIO navigation check.

The simulator and walking policy live in the engineai_navigation workspace.
Source that overlay before this one. These launches never start livox_ros_driver2.
"""

import os
import importlib.util
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def rendering_actions(context):
    spec = importlib.util.spec_from_file_location(
        'pm01_sim_rendering', Path(__file__).with_name('rendering.py'))
    rendering = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rendering)
    overrides, reason = rendering.select_rendering(
        LaunchConfiguration('render_backend').perform(context), context.environment)
    return [LogInfo(msg='PM01 simulation rendering: ' + reason),
            *[SetEnvironmentVariable(key, val) for key, val in overrides.items()]]


def sim_arguments(walking_config_file=None):
    return [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('render_backend', default_value='auto',
                              choices=['auto', 'nvidia', 'system'],
                              description='Simulation graphics: prefer available NVIDIA OpenGL'),
        OpaqueFunction(function=rendering_actions),
        DeclareLaunchArgument('gazebo_master_uri', default_value=os.environ.get(
            'GAZEBO_MASTER_URI', 'http://127.0.0.1:11358')),
        DeclareLaunchArgument('gui_ready_timeout', default_value='45.0'),
        DeclareLaunchArgument('gui_max_restarts', default_value='1'),
        DeclareLaunchArgument('world', default_value='ground.world'),
        DeclareLaunchArgument('walking_config_file', default_value=walking_config_file or os.path.join(
            get_package_share_directory('rl_walking'), 'config', 'rl_walking.yaml')),
    ]


def walking(start_adapter=True):
    share = get_package_share_directory('rl_walking')
    launch_path = os.path.join(share, 'launch', 'pm01_walking.launch.py')
    with open(launch_path, encoding='utf-8') as stream:
        upstream_launch = stream.read()
        if 'walking_cmd_vel_topic' not in upstream_launch:
            raise RuntimeError('Rebuild repository rl_walking; missing command topic support in '
                               'engineai_navigation and rebuild rl_walking first')
        if 'walking_config_file' not in upstream_launch:
            raise RuntimeError('Rebuild repository rl_walking; missing configuration support in '
                               'engineai_navigation before using this launch')
        if 'after_initial_pose' not in upstream_launch:
            raise RuntimeError('Simulation startup handoff is missing; run bash scripts/build.sh sim '
                               'and reload scripts/env.sh before launching')
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            # Start the server first; the managed client waits for clock/IMU.
            'gui': 'false',
            'world': LaunchConfiguration('world'),
            'gazebo_master_uri': LaunchConfiguration('gazebo_master_uri'),
            'walking_config_file': LaunchConfiguration('walking_config_file'),
            'enable_lidar': 'true',
            'enable_joystick': 'false',
            'hold_until_stand_reset': 'true',
            'walking_cmd_vel_topic': '/pm01/walking_cmd_vel',
            # ros2 topic pub defaults to 1 Hz. The policy drops a command
            # after this many seconds, so keep the window longer than 1 s.
            'command_timeout': '2.0',
        }.items(),
    )
    return [
        Node(package='pm01_navigation_sim', executable='sim_cmd_vel_adapter', output='screen',
             condition=IfCondition(str(start_adapter).lower()),
             parameters=[{'use_sim_time': True}]),
        # Evaluate the user's gui flag before the nested launch sets gui=false.
        Node(package='pm01_navigation_sim', executable='sim_gazebo_gui.py',
             output='screen', condition=IfCondition(LaunchConfiguration('gui')),
             parameters=[{'use_sim_time': True,
                          'ready_timeout_sec': ParameterValue(
                              LaunchConfiguration('gui_ready_timeout'), value_type=float),
                          'max_restarts': ParameterValue(
                              LaunchConfiguration('gui_max_restarts'), value_type=int)}]),
        include,
    ]


def stand_up():
    """A delayed controller service must not leave LIO initialized on a fall."""
    return Node(package='pm01_navigation_sim', executable='sim_stand_ready.py',
                output='screen', parameters=[{'use_sim_time': True}])


def body_tf(output_frame='camera_init', publish_odom=False):
    """Attach the Gazebo URDF root to a Fast-LIO pose.

    Mapping hangs base_footprint on camera_init so RViz can draw the body in
    the cloud. Navigation hangs it on odom instead: the localizer owns map to
    odom, and a link cannot have both parents.
    """
    return Node(
        package='pm01_navigation_sim', executable='sim_body_tf',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_frame': output_frame,
            'publish_odom': publish_odom,
        }],
    )


def fast_lio(rviz_frame='camera_init', rviz_config='fastlio_sim.rviz', save_pcd=True,
             lio_params=None):
    share = get_package_share_directory('fast_lio')
    nav_share = get_package_share_directory('pm01_navigation_bringup')
    return [
        Node(package='fast_lio', executable='fastlio_mapping', output='screen',
             parameters=[os.path.join(share, 'config', 'mid360.yaml'),
                         lio_params or os.path.join(nav_share, 'config', 'sim', 'lio.yaml'),
                         {'use_sim_time': True, 'pcd_save.pcd_save_en': save_pcd,
                          'publish.map_en': save_pcd}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(nav_share, 'config', 'sim', rviz_config),
                        '-f', rviz_frame], parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def lidar_tf():
    share = get_package_share_directory('pm01_navigation_bringup')
    with open(os.path.join(share, 'config', 'pm01_lidar_tf.yaml')) as stream:
        config = yaml.safe_load(stream)
    # The URDF already parents every link, including livox_frame. A second
    # base_link -> LINK_TORSO_YAW parent splits the tree. Nav2 follows
    # base_footprint, which sim_body_tf parents from odom.
    config = dict(config)
    config['base_frame'] = 'base_footprint'
    return config, [
        # sim_body_tf changes the world-frame name, not its coordinates.
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'odom', '--child-frame-id', 'camera_init'],
             parameters=[{'use_sim_time': True}]),
        Node(package='pm01_navigation_perception', executable='custommsg_to_cloud',
             parameters=[{'use_sim_time': True}], output='screen'),
        body_tf('odom', True),
    ]
