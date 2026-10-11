# SPDX-FileCopyrightText: 2026 FZ_XiaoXi
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_example = get_package_share_directory('livox_lidar_example')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    display_xacro = os.path.join(
        pkg_example, 'urdf', 'livox_display.urdf.xacro')

    gz_verbosity = DeclareLaunchArgument(
        'gz_verbosity',
        default_value='3',
        description='Gazebo verbosity. Use 4 for debug output.')
    rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2.')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                '-r -v ', LaunchConfiguration('gz_verbosity'), ' ',
                os.path.join(pkg_example, 'worlds', 'livox_example.sdf')
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(
                pkg_example, 'config', 'livox_example_bridge.yaml')
        }],
        output='screen',
    )

    def display_description(root_frame, base_link, mesh_file):
        return ParameterValue(
            Command([
                'xacro ', display_xacro,
                ' root_frame:=', root_frame,
                ' base_link:=', base_link,
                ' mesh_file:=', mesh_file,
            ]),
            value_type=str,
        )

    left_model = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='livox_left_model',
        parameters=[{
            'robot_description': display_description(
                'livox_left_frame', 'base_link_left_visual',
                'package://livox_lidar_gz_sensor/meshes/mid360.dae'),
        }],
        remappings=[
            ('robot_description', '/livox_left_robot_description'),
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
        output='screen',
    )

    right_model = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='livox_right_model',
        parameters=[{
            'robot_description': display_description(
                'livox_right_frame', 'base_link_right_visual',
                'package://livox_lidar_gz_sensor/meshes/mid70.dae'),
        }],
        remappings=[
            ('robot_description', '/livox_right_robot_description'),
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_example, 'config', 'livox_example.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        gz_verbosity,
        rviz,
        gz_sim,
        bridge,
        left_model,
        right_model,
        rviz_node,
    ])
