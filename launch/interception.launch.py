#!/usr/bin/env python3
"""Top-level drone-to-drone interception scene.

Brings up ONE Gazebo Harmonic world containing both the interceptor and the
target, each as its own PX4 SITL instance with its own MAVROS stack.

Anti-two-Gazebo-GUI design (the failure mode of the old run_sim.launch.py):
  * Both drones use the IDENTICAL `world` value, so PX4 reuses one gz server.
  * The interceptor is launched first and owns the server + world + RViz.
  * The target is launched after `target_spawn_delay` seconds, by which time
    the single server is up, so PX4 only spawns the target MODEL into it.

Usage:
  ros2 launch drone_interception_sim interception.launch.py
  ros2 launch drone_interception_sim interception.launch.py headless:=1 use_rviz:=false
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    target_spawn_delay = LaunchConfiguration('target_spawn_delay')
    px4_dir = LaunchConfiguration('px4_dir')

    pkg = FindPackageShare('drone_interception_sim')

    interceptor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg, 'launch', 'interceptor.launch.py'])
        ]),
        launch_arguments={
            'headless': headless,
            'gz_world': world,
            'use_rviz': use_rviz,
            'px4_dir': px4_dir,
        }.items())

    target = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg, 'launch', 'target.launch.py'])
        ]),
        launch_arguments={
            'headless': headless,
            'gz_world': world,
            'px4_dir': px4_dir,
        }.items())

    # Delay the target so the interceptor's PX4 brings up the single gz server first.
    delayed_target = TimerAction(period=target_spawn_delay, actions=[target])

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='0',
                              description='1 = run Gazebo server-only (no GUI)'),
        DeclareLaunchArgument('world', default_value='interception',
                              description='Gazebo world name shared by both drones'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('target_spawn_delay', default_value='8.0',
                              description='Seconds to wait before spawning the target'),
        DeclareLaunchArgument('px4_dir',
                              default_value='/home/asmbatati/drone_interception_ws/PX4-Autopilot',
                              description='PX4-Autopilot dir (in-tree by default)'),
        interceptor,
        delayed_target,
    ])
