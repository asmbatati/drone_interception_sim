#!/usr/bin/env python3
"""Bring up the TARGET drone (PX4 instance 1).

This launch must run AFTER interceptor.launch.py so that PX4 attaches the
target model to the already-running Gazebo server instead of spawning a second
one. It therefore does NOT carry the world or RViz - only the model, MAVROS,
TF, and the scripted trajectory node that flies the target autonomously.
"""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Target identity (see plan: spawn scheme table)
NS = 'target'
MODEL = 'x3_uav'
AUTOSTART_ID = '4021'
INSTANCE_ID = '1'
FCU_URL = 'udp://:14542@127.0.0.1:14559'
TGT_SYSTEM = '2'   # PX4 MAV_SYS_ID = instance_id + 1


def launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration('headless').perform(context)
    gz_world = LaunchConfiguration('gz_world').perform(context)
    xpos = LaunchConfiguration('xpos').perform(context)
    ypos = LaunchConfiguration('ypos').perform(context)
    zpos = LaunchConfiguration('zpos').perform(context)

    pkg_share = get_package_share_directory('drone_interception_sim')

    actions = []

    # PX4 SITL (attaches to the interceptor's already-running Gazebo server)
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uav_gz_sim'),
                                  'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_ns': NS,
            'headless': headless,
            'gz_model_name': MODEL,
            'gz_world': gz_world,
            'px4_autostart_id': AUTOSTART_ID,
            'instance_id': INSTANCE_ID,
            'xpos': xpos, 'ypos': ypos, 'zpos': zpos,
        }.items()))

    # MAVROS
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uav_gz_sim'),
                                  'launch', 'mavros.launch.py'])
        ]),
        launch_arguments={
            'mavros_namespace': NS + '/mavros',
            'tgt_system': TGT_SYSTEM,
            'fcu_url': FCU_URL,
            'pluginlists_yaml': os.path.join(pkg_share, 'config', 'mavros',
                                             'target_px4_pluginlists.yaml'),
            'config_yaml': os.path.join(pkg_share, 'config', 'mavros',
                                        'target_px4_config.yaml'),
            'base_link_frame': NS + '/base_link',
            'odom_frame': NS + '/odom',
            'map_frame': 'map',
        }.items()))

    # --- TF tree (target only; map/global/map_frd come from interceptor launch) ---
    actions.append(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map2px4_' + NS + '_tf_node',
        arguments=['0', '0', '0', '0', '0', '0', 'map', NS + '/odom'],
        parameters=[{'use_sim_time': True}], output='log'))

    actions.append(Node(
        package='uav_gz_sim', executable='tf_relay',
        name='odom2base_tf_relay', namespace=NS,
        parameters=[
            {'use_sim_time': True},
            {'source_topic': f'/{NS}/mavros/local_position/pose'},
            {'target_frame_id': f'{NS}/odom'},
            {'child_frame_id': f'{NS}/base_link'},
            {'queue_size': 50},
            {'publish_rate': 50.0},
        ], output='log'))

    # Scripted autonomous target flight
    actions.append(Node(
        package='drone_interception_sim', executable='target_trajectory',
        name='target_trajectory_node', namespace=NS,
        parameters=[
            os.path.join(pkg_share, 'config', 'target_trajectory.yaml'),
            {'use_sim_time': True},
        ], output='screen'))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='0'),
        DeclareLaunchArgument('gz_world', default_value='default'),
        DeclareLaunchArgument('xpos', default_value='10.0'),
        DeclareLaunchArgument('ypos', default_value='0.0'),
        DeclareLaunchArgument('zpos', default_value='0.1'),
        OpaqueFunction(function=launch_setup),
    ])
