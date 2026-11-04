#!/usr/bin/env python3
# Copyright 2026 Abdulrahman S. Al-Batati
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

"""
Launch the interception metrics logger against the running sim.

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
    return LaunchDescription(
        [
            DeclareLaunchArgument('method', default_value='unknown'),
            DeclareLaunchArgument('capture_radius', default_value='1.0'),
            DeclareLaunchArgument('csv_path', default_value='/tmp/interception_metrics.csv'),
            DeclareLaunchArgument(
                'summary_csv_path',
                default_value='/tmp/interception_results.csv',
                description='per-run summary table (appended)',
            ),
            DeclareLaunchArgument('scenario', default_value=''),
            DeclareLaunchArgument('trial', default_value='-1'),
            DeclareLaunchArgument(
                'referee',
                default_value='odom',
                description='odom | gazebo: judge range and capture on MAVROS odometry or on '
                'the poses Gazebo itself reports',
            ),
            DeclareLaunchArgument(
                'interceptor_odom', default_value='/interceptor/mavros/local_position/odom'
            ),
            DeclareLaunchArgument(
                'target_odom', default_value='/target/mavros/local_position/odom'
            ),
            Node(
                package='drone_interception_sim',
                executable='interception_metrics',
                name='interception_metrics',
                output='screen',
                parameters=[
                    {
                        'method': method,
                        'capture_radius': LaunchConfiguration('capture_radius'),
                        'csv_path': LaunchConfiguration('csv_path'),
                        'summary_csv_path': LaunchConfiguration('summary_csv_path'),
                        'scenario': LaunchConfiguration('scenario'),
                        'trial': LaunchConfiguration('trial'),
                        'referee': LaunchConfiguration('referee'),
                        'use_sim_time': True,
                    }
                ],
                remappings=[
                    ('interceptor_odom', LaunchConfiguration('interceptor_odom')),
                    ('target_odom', LaunchConfiguration('target_odom')),
                ],
            ),
        ]
    )
