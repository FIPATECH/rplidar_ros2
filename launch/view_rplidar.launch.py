#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rplidar_ros2')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'rplidar.rviz')
    rviz_config = LaunchConfiguration('rviz_config', default=rviz_config_file)

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_file,
            description='Absolute path to the RViz configuration file'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'rplidar.launch.py')
            )
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
