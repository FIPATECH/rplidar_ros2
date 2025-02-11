#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    channel_type     = LaunchConfiguration('channel_type', default='serial')
    serial_port      = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate  = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id         = LaunchConfiguration('frame_id', default='laser')
    inverted         = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode        = LaunchConfiguration('scan_mode', default='Express')
    auto_standby     = LaunchConfiguration('auto_standby', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifies the channel type (serial, tcp, etc.)'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifies the serial port connected to the LIDAR'
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifies the baud rate for the serial port'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifies the frame ID for the LIDAR scan'
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Indicates whether to invert the scan data'
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Enables angle compensation for the scan data if true'
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifies the LIDAR scan mode'
        ),
        DeclareLaunchArgument(
            'auto_standby',
            default_value=auto_standby,
            description='Enable auto-standby mode (stops scanning when no subscribers are present)'
        ),
        Node(
            package='rplidar_ros2',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
                'auto_standby': auto_standby
            }]
        )
    ])
