# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Adapted from Navigation2 1.1.20, nav2_bringup/launch/navigation_launch.py.
# Upstream: https://github.com/ros-navigation/navigation2/tree/1.1.20
# Project defaults add a shared readiness pause and a guarded navigation output.

import os
import json
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitution import Substitution
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.frontend.parse_substitution import parse_substitution
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.utilities import perform_substitutions
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def navigation_bt_node_parameters(parameters, namespace=''):
    current = parameters
    for part in namespace.strip('/').split('/'):
        if part and part in current:
            current = current[part]
    return current.get('bt_navigator', {}).get('ros__parameters', {})


def navigation_bt_plugins(parameters, defaults, namespace=''):
    """Extend an existing user list without replacing Humble's default plugins."""
    values = navigation_bt_node_parameters(parameters, namespace)
    libraries = values.get('plugin_lib_names')
    if libraries is None:
        libraries = defaults['bt_navigator']['ros__parameters']['plugin_lib_names']
    if not isinstance(libraries, list) or any(not isinstance(name, str) for name in libraries):
        raise ValueError('bt_navigator.plugin_lib_names must be a list of library names')
    return list(dict.fromkeys([*libraries, 'pm01_navigation_ready_wait_bt_node']))


def guard_navigation_tree(source, destination):
    """Keep a selected tree's behavior and guard its entire main child subtree."""
    source = Path(source).resolve()
    tree = ET.parse(source, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    root = tree.getroot()
    trees = root.findall('BehaviorTree')
    selected = root.get('main_tree_to_execute')
    if selected:
        matches = [item for item in trees if item.get('ID') == selected]
    else:
        matches = trees if len(trees) == 1 else []
    if len(matches) != 1:
        raise ValueError(f'Cannot identify a unique main BehaviorTree in {source}')
    main = matches[0]
    children = [item for item in main if isinstance(item.tag, str)]
    if len(children) != 1:
        raise ValueError(f'Main BehaviorTree must contain exactly one root node: {source}')
    child = children[0]
    if child.tag != 'NavigationReadyWait':
        position = list(main).index(child)
        main.remove(child)
        guard = ET.Element('NavigationReadyWait', timeout='30.0', heartbeat_timeout='0.5')
        guard.append(child)
        main.insert(position, guard)
    # Includes retain their own nested include resolution relative to their file.
    for include in root.findall('include'):
        path = include.get('path')
        if path and not Path(path).is_absolute():
            include.set('path', str((source.parent / path).resolve()))
    tree.write(destination, encoding='utf-8', xml_declaration=True)
    return str(destination)


class NavigationBtTrees:
    def __init__(self, params_file, namespace, defaults_dir):
        self.params_file, self.namespace = params_file, namespace
        self.defaults_dir = defaults_dir
        self.temporary = None
        self.paths = {}

    def selected(self, context, kind):
        if kind in self.paths:
            return self.paths[kind]
        with open(self.params_file.perform(context), encoding='utf-8') as source:
            params = navigation_bt_node_parameters(yaml.safe_load(source) or {},
                                                   self.namespace.perform(context))
        key = 'default_nav_to_pose_bt_xml' if kind == 'navigate_to_pose' else 'default_nav_through_poses_bt_xml'
        configured = params.get(key, '')
        if not configured:
            result = os.path.join(self.defaults_dir, kind + '.xml')
        else:
            if not isinstance(configured, str):
                raise ValueError(f'{key} must be an XML file path')
            source = perform_substitutions(context, parse_substitution(configured))
            if self.temporary is None:
                self.temporary = tempfile.TemporaryDirectory(prefix='pm01-navigation-bt-')
            result = guard_navigation_tree(os.path.expanduser(source),
                                           Path(self.temporary.name) / (kind + '.xml'))
        self.paths[kind] = result
        return result

    def cleanup(self, context=None):
        if self.temporary is not None:
            self.temporary.cleanup()
            self.temporary = None


class NavigationBtTree(Substitution):
    def __init__(self, trees, kind):
        super().__init__()
        self.trees, self.kind = trees, kind

    def perform(self, context):
        return self.trees.selected(context, self.kind)


class NavigationBtPlugins(Substitution):
    """Resolve the final selected YAML after launch arguments have been applied."""
    def __init__(self, params_file, namespace):
        super().__init__()
        self.params_file = params_file
        self.namespace = namespace

    def perform(self, context):
        with open(self.params_file.perform(context), encoding='utf-8') as source:
            parameters = yaml.safe_load(source) or {}
        default_path = os.path.join(get_package_share_directory('nav2_bringup'),
                                    'params', 'nav2_params.yaml')
        with open(default_path, encoding='utf-8') as source:
            defaults = yaml.safe_load(source)
        return json.dumps(navigation_bt_plugins(parameters, defaults, self.namespace.perform(context)))


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('pm01_navigation_bringup')
    trees_dir = os.path.join(get_package_share_directory('pm01_navigation_nav2'), 'behavior_trees')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    selected_trees = NavigationBtTrees(params_file, namespace, trees_dir)
    navigation_bt_parameters = {
        'default_nav_to_pose_bt_xml': ParameterValue(
            NavigationBtTree(selected_trees, 'navigate_to_pose'), value_type=str),
        'default_nav_through_poses_bt_xml': ParameterValue(
            NavigationBtTree(selected_trees, 'navigate_through_poses'), value_type=str),
        'plugin_lib_names': ParameterValue(NavigationBtPlugins(params_file, namespace))}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'hardware', 'nav2.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'navigation_cmd_vel_requested')]),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, navigation_bt_parameters],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'navigation_cmd_vel_requested')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'navigation_cmd_vel_requested')]),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params, navigation_bt_parameters],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'navigation_cmd_vel_requested')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=selected_trees.cleanup)])))
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    # This outlet runs in both composition modes and is absent from mapping-only bringup.
    ld.add_action(Node(
        package='pm01_navigation_nav2', executable='navigation_command_gate',
        name='navigation_command_gate', output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level]))

    return ld
