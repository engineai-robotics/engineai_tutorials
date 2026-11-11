#!/usr/bin/env python3
# Copyright 2026 EngineAI
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

import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('simulator')
    package_lib = os.path.join(get_package_prefix('simulator'), 'lib')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    world_path = PathJoinSubstitution([
        package_dir, 'worlds', LaunchConfiguration('world')
    ])
    xacro_path = os.path.join(
        package_dir,
        'config',
        'robot',
        'pm01_lidar_v1',
        'pm01_lidar_v1.urdf.xacro',
    )
    controllers_path = os.path.join(
        package_dir, 'config', 'pm01_controllers.yaml'
    )
    mesh_prefix = 'file://' + os.path.join(
        package_dir, 'config', 'robot', 'pm01_lidar_v1', 'meshes'
    )

    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo_master_uri = LaunchConfiguration('gazebo_master_uri')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_path,
            ' mesh_prefix:=', mesh_prefix,
            ' controllers_file:=', controllers_path,
            ' enable_gazebo_plugins:=true',
            ' enable_sim_sensors:=',
            LaunchConfiguration('enable_sim_sensors'),
            ' enable_lidar:=', LaunchConfiguration('enable_lidar'),
            ' lidar_pointcloud_topic:=',
            LaunchConfiguration('lidar_pointcloud_topic'),
            ' lidar_frame:=', LaunchConfiguration('lidar_frame'),
            ' imu_topic:=', LaunchConfiguration('imu_topic'),
            ' imu_frame:=', LaunchConfiguration('imu_frame'),
            ' policy_imu_topic:=', LaunchConfiguration('policy_imu_topic'),
            ' policy_imu_frame:=', LaunchConfiguration('policy_imu_frame'),
            ' lidar_update_rate:=',
            LaunchConfiguration('lidar_update_rate'),
            ' lidar_horizontal_samples:=',
            LaunchConfiguration('lidar_horizontal_samples'),
            ' lidar_vertical_samples:=',
            LaunchConfiguration('lidar_vertical_samples'),
            ' lidar_visualize:=', LaunchConfiguration('lidar_visualize'),
            ' enable_ros2_control:=true',
            ' hold_until_stand_reset:=', LaunchConfiguration('hold_until_stand_reset'),
        ]),
        value_type=str,
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'pause': 'true',
        }.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'pm01_lidar_v1',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--switch-timeout', '30',
        ],
    )
    sdk_pd_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'pm01_sdk_pd_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--switch-timeout', '30',
        ],
    )
    initial_pose = Node(
        package='simulator',
        executable='pm01_initial_pose.py',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'duration': 8.0,
            'publish_rate': 100.0,
        }],
    )
    reset_stand = Node(
        package='simulator',
        executable='pm01_reset_stand.py',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'model_name': 'pm01_lidar_v1',
            'controller_reset_service': LaunchConfiguration(
                'controller_reset_service'
            ),
            'x': ParameterValue(
                LaunchConfiguration('x'), value_type=float
            ),
            'y': ParameterValue(
                LaunchConfiguration('y'), value_type=float
            ),
            'z': ParameterValue(
                LaunchConfiguration('z'), value_type=float
            ),
            'yaw': ParameterValue(
                LaunchConfiguration('yaw'), value_type=float
            ),
        }],
    )
    livox_bridge = Node(
        package='simulator',
        executable='livox_pointcloud_bridge',
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': LaunchConfiguration('lidar_pointcloud_topic'),
            'output_topic': LaunchConfiguration('lidar_custom_topic'),
            'scan_period': ParameterValue(
                PythonExpression([
                    '1.0 / float(',
                    LaunchConfiguration('lidar_update_rate'),
                    ')',
                ]),
                value_type=float,
            ),
        }],
    )
    unpause_gazebo = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/unpause_physics', 'std_srvs/srv/Empty', '{}',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('hold_until_stand_reset', default_value='false'),
        DeclareLaunchArgument(
            'world',
            default_value='empty.world',
            description=(
                'World file in simulator/worlds: empty.world or ground.world'
            ),
        ),
        DeclareLaunchArgument(
            'enable_sim_sensors',
            default_value='true',
            description='Enable Gazebo IMU and odometry plugins',
        ),
        DeclareLaunchArgument(
            'enable_lidar',
            default_value='true',
            description='Enable the Gazebo Mid360 point-cloud sensor',
        ),
        DeclareLaunchArgument(
            'lidar_pointcloud_topic',
            default_value='/livox/lidar/pointcloud',
        ),
        DeclareLaunchArgument(
            'lidar_custom_topic', default_value='/livox/lidar'
        ),
        DeclareLaunchArgument('lidar_frame', default_value='livox_frame'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        DeclareLaunchArgument('policy_imu_topic', default_value='/pm01/imu'),
        DeclareLaunchArgument(
            'policy_imu_frame', default_value='LINK_TORSO_YAW'
        ),
        DeclareLaunchArgument('lidar_update_rate', default_value='10.0'),
        DeclareLaunchArgument(
            'lidar_horizontal_samples', default_value='600'
        ),
        DeclareLaunchArgument('lidar_vertical_samples', default_value='64'),
        DeclareLaunchArgument(
            'lidar_visualize',
            default_value='false',
            description='Show Mid360 rays in the Gazebo client',
        ),
        DeclareLaunchArgument(
            'controller_reset_service',
            default_value='/pm01_joint_control_test/reset',
            description='Optional high-level controller reset service',
        ),
        DeclareLaunchArgument(
            'gazebo_master_uri',
            default_value='http://127.0.0.1:11358',
            description='Dedicated Gazebo Classic master URI',
        ),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.82'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MASTER_URI', gazebo_master_uri),
        SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            package_lib + ':' + os.environ.get('GAZEBO_PLUGIN_PATH', ''),
        ),
        gzserver,
        gzclient,
        robot_state_publisher,
        reset_stand,
        livox_bridge,
        spawn_robot,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    initial_pose,
                    TimerAction(
                        period=0.2,
                        actions=[joint_state_broadcaster],
                    ),
                    # Controller switching is serviced by Gazebo's update loop,
                    # so physics must run while the spawners activate.
                    TimerAction(period=0.5, actions=[unpause_gazebo]),
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[sdk_pd_controller],
            )
        ),
    ])
