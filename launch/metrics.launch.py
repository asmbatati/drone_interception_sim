#!/usr/bin/env python3
"""Launch the interception metrics logger against the running sim.

Method-agnostic: run it alongside any controller (PN/FSM/BT/RL) to record range,
capture and time-to-intercept. Set method:= to label the CSV/run.

  ros2 launch drone_interception_sim metrics.launch.py method:=pn
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    method = LaunchConfiguration('method')
    return LaunchDescription([
        DeclareLaunchArgument('method', default_value='unknown'),
        DeclareLaunchArgument('capture_radius', default_value='1.0'),
        DeclareLaunchArgument('csv_path', default_value='/tmp/interception_metrics.csv'),
        DeclareLaunchArgument('interceptor_odom',
                              default_value='/interceptor/mavros/local_position/odom'),
        DeclareLaunchArgument('target_odom',
                              default_value='/target/mavros/local_position/odom'),
        Node(
            package='drone_interception_sim', executable='interception_metrics',
            name='interception_metrics', output='screen',
            parameters=[{
                'method': method,
                'capture_radius': LaunchConfiguration('capture_radius'),
                'csv_path': LaunchConfiguration('csv_path'),
                'use_sim_time': True,
            }],
            remappings=[
                ('interceptor_odom', LaunchConfiguration('interceptor_odom')),
                ('target_odom', LaunchConfiguration('target_odom')),
            ],
        ),
    ])
