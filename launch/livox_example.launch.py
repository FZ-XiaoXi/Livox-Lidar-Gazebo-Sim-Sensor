import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_example = get_package_share_directory('livox_lidar_example')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

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
            ]
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

    lidar_left_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-1', '--y', '0', '--z', '1.2',
            '--roll', '0', '--pitch', '0', '--yaw', '1.5708',
            '--frame-id', 'world',
            '--child-frame-id', 'livox_left_frame',
        ],
        name='livox_left_world_tf',
        output='screen',
    )

    lidar_right_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '1', '--y', '0', '--z', '1.2',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'livox_right_frame',
        ],
        name='livox_right_world_tf',
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
        lidar_left_world_tf,
        lidar_right_world_tf,
        rviz_node,
    ])
