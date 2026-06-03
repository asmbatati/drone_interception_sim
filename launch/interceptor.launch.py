#!/usr/bin/env python3
"""Bring up the INTERCEPTOR drone (PX4 instance 0).

This launch starts the PX4 SITL instance that brings up the single shared
Gazebo server for the world, plus MAVROS, the TF tree, the depth-camera bridge
and (optionally) RViz. The TARGET drone (target.launch.py) is started AFTER this
one so it attaches to the already-running Gazebo server instead of spawning a
second one (the bug that made the old run_sim.launch.py unusable).
"""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Interceptor identity (see plan: spawn scheme table)
NS = 'interceptor'
MODEL = 'x500_d435'
AUTOSTART_ID = '4020'
INSTANCE_ID = '0'
FCU_URL = 'udp://:14541@127.0.0.1:14558'
TGT_SYSTEM = '1'   # PX4 MAV_SYS_ID = instance_id + 1


def launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration('headless').perform(context)
    gz_world = LaunchConfiguration('gz_world').perform(context)
    xpos = LaunchConfiguration('xpos').perform(context)
    ypos = LaunchConfiguration('ypos').perform(context)
    zpos = LaunchConfiguration('zpos').perform(context)

    pkg_share = get_package_share_directory('drone_interception_sim')

    actions = []

    # Make headless real: PX4's gz backend honours HEADLESS=1 (runs `gz sim -s`).
    if headless == '1':
        actions.append(SetEnvironmentVariable('HEADLESS', '1'))

    # PX4 SITL + Gazebo (this instance spawns the shared server)
    gz_launch = IncludeLaunchDescription(
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
        }.items()
    )
    actions.append(gz_launch)

    # MAVROS
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uav_gz_sim'),
                                  'launch', 'mavros.launch.py'])
        ]),
        launch_arguments={
            'mavros_namespace': NS + '/mavros',
            'tgt_system': TGT_SYSTEM,
            'fcu_url': FCU_URL,
            'pluginlists_yaml': os.path.join(pkg_share, 'config', 'mavros',
                                             'interceptor_px4_pluginlists.yaml'),
            'config_yaml': os.path.join(pkg_share, 'config', 'mavros',
                                        'interceptor_px4_config.yaml'),
            'base_link_frame': NS + '/base_link',
            'odom_frame': NS + '/odom',
            'map_frame': 'map',
        }.items()
    )
    actions.append(mavros_launch)

    # --- TF tree ---
    actions.append(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map2global_tf_node',
        arguments=['0', '0', '0', '0', '0', '0', 'global', 'map'],
        parameters=[{'use_sim_time': True}], output='log'))

    actions.append(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map2map_frd_tf_node',
        arguments=['0', '0', '0', '1.5708', '0', '1.5708', 'map', 'map_frd'],
        parameters=[{'use_sim_time': True}], output='log'))

    actions.append(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map2px4_' + NS + '_tf_node',
        arguments=['0', '0', '0', '0', '0', '0', 'map', NS + '/odom'],
        parameters=[{'use_sim_time': True}], output='log'))

    # Dynamic odom->base_link from MAVROS local pose (reuse uav_gz_sim tf_relay)
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

    # Depth-camera bridge (RealSense D435 publishes on global /d435/* gz topics)
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='interceptor_depthcam_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/d435/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/d435/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/d435/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/d435/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '--ros-args',
            '-r', '/d435/depth_image:=' + NS + '/depth_image',
            '-r', '/d435/image:=' + NS + '/image',
            '-r', '/d435/points:=' + NS + '/points',
            '-r', '/d435/camera_info:=' + NS + '/camera_info',
        ],
        parameters=[{'use_sim_time': True}], output='log'))

    # RViz (optional)
    rviz_file = os.path.join(pkg_share, 'rviz', 'interception.rviz')
    actions.append(Node(
        package='rviz2', executable='rviz2', name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True},
                    {'tf_buffer_cache_time_ms': 60000},
                    {'transform_tolerance': 5.0}],
        output='log'))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='0'),
        DeclareLaunchArgument('gz_world', default_value='default'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('xpos', default_value='0.0'),
        DeclareLaunchArgument('ypos', default_value='0.0'),
        DeclareLaunchArgument('zpos', default_value='0.1'),
        OpaqueFunction(function=launch_setup),
    ])
