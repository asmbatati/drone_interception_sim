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

# Project's in-tree PX4 (the one beside ros2_ws). Override with px4_dir:=/path.
DEFAULT_PX4_DIR = '/home/asmbatati/drone_interception_ws/PX4-Autopilot'

# Must match the interceptor's gz_partition so the target attaches to the SAME
# (already-running) Gazebo server.
DEFAULT_GZ_PARTITION = 'd2d_intercept'

# Target identity (see plan: spawn scheme table)
NS = 'target'
MODEL = 'x3_uav'
# In the in-tree PX4, x3_uav is airframe 4022 (4021 there is x500_lidar_camera).
AUTOSTART_ID = '4022'
INSTANCE_ID = '1'
# PX4 SITL port convention: instance i -> listen 14540+i, remote 14557+i.
FCU_URL = 'udp://:14541@127.0.0.1:14558'
TGT_SYSTEM = '2'   # PX4 MAV_SYS_ID = instance_id + 1


def launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration('headless').perform(context)
    gz_world = LaunchConfiguration('gz_world').perform(context)
    xpos = LaunchConfiguration('xpos').perform(context)
    ypos = LaunchConfiguration('ypos').perform(context)
    zpos = LaunchConfiguration('zpos').perform(context)
    px4_dir = LaunchConfiguration('px4_dir').perform(context)
    gz_partition = LaunchConfiguration('gz_partition').perform(context)

    # Use the project's in-tree PX4 (overridable via px4_dir:=).
    if px4_dir:
        os.environ['PX4_DIR'] = px4_dir

    # Same partition as the interceptor so we attach to its running gz server.
    if gz_partition:
        os.environ['GZ_PARTITION'] = gz_partition

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
    # Offset map->target/odom by the spawn pose so RViz shows the target at its
    # true world location (its EKF/odom origin is at its spawn point).
    actions.append(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map2px4_' + NS + '_tf_node',
        arguments=[xpos, ypos, zpos, '0', '0', '0', 'map', NS + '/odom'],
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

    # Scripted autonomous target flight (evasion optional; see config)
    actions.append(Node(
        package='drone_interception_sim', executable='target_trajectory',
        name='target_trajectory_node', namespace=NS,
        parameters=[
            os.path.join(pkg_share, 'config', 'target_trajectory.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[('interceptor_odom', '/interceptor/mavros/local_position/odom')],
        output='screen'))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='0'),
        DeclareLaunchArgument('gz_world', default_value='interception'),
        DeclareLaunchArgument('xpos', default_value='10.0'),
        DeclareLaunchArgument('ypos', default_value='0.0'),
        DeclareLaunchArgument('zpos', default_value='0.1'),
        DeclareLaunchArgument('px4_dir', default_value=DEFAULT_PX4_DIR,
                              description='PX4-Autopilot dir (in-tree by default)'),
        DeclareLaunchArgument('gz_partition', default_value=DEFAULT_GZ_PARTITION,
                              description='Gazebo transport partition (must match interceptor)'),
        OpaqueFunction(function=launch_setup),
    ])
