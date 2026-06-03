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
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Project's in-tree PX4 (the one beside ros2_ws), used by default instead of any
# ~/PX4-Autopilot. Override with px4_dir:=/path on the command line.
DEFAULT_PX4_DIR = '/home/asmbatati/drone_interception_ws/PX4-Autopilot'

# Isolate this sim's Gazebo transport so it never collides with another PX4+gz
# sim on the same machine (e.g. a different workspace). Set empty to share the
# default partition. External gz clients (gz gui/topic) must use the same value.
DEFAULT_GZ_PARTITION = 'd2d_intercept'

# Interceptor identity (see plan: spawn scheme table)
NS = 'interceptor'
MODEL = 'x500_d435'
AUTOSTART_ID = '4020'
INSTANCE_ID = '0'
# PX4 SITL port convention: instance i -> listen 14540+i, remote 14557+i.
FCU_URL = 'udp://:14540@127.0.0.1:14557'
TGT_SYSTEM = '1'   # PX4 MAV_SYS_ID = instance_id + 1


def launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration('headless').perform(context)
    gz_world = LaunchConfiguration('gz_world').perform(context)
    xpos = LaunchConfiguration('xpos').perform(context)
    ypos = LaunchConfiguration('ypos').perform(context)
    zpos = LaunchConfiguration('zpos').perform(context)
    px4_dir = LaunchConfiguration('px4_dir').perform(context)
    gz_partition = LaunchConfiguration('gz_partition').perform(context)

    # Use the project's in-tree PX4 (overridable via px4_dir:=). gz_sim.launch.py
    # reads PX4_DIR from the environment, so set it here before the include.
    if px4_dir:
        os.environ['PX4_DIR'] = px4_dir

    # Isolate Gazebo transport so PX4 starts its OWN gz server for this world
    # instead of attaching to another sim's server on the default partition.
    if gz_partition:
        os.environ['GZ_PARTITION'] = gz_partition

    pkg_share = get_package_share_directory('drone_interception_sim')

    actions = []

    # RMW is inherited from the environment (whatever RMW_IMPLEMENTATION is
    # exported), so all ROS nodes here already use it. Zenoh additionally needs
    # its router daemon running for discovery, so start it if it isn't already.
    rmw = os.environ.get('RMW_IMPLEMENTATION', '')
    if 'zenoh' in rmw:
        actions.append(ExecuteProcess(
            cmd=['bash', '-c',
                 'pgrep -f rmw_zenohd >/dev/null 2>&1 || '
                 'exec ros2 run rmw_zenoh_cpp rmw_zenohd'],
            name='zenoh_router', output='log'))

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
        DeclareLaunchArgument('gz_world', default_value='interception'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('xpos', default_value='0.0'),
        DeclareLaunchArgument('ypos', default_value='0.0'),
        DeclareLaunchArgument('zpos', default_value='0.1'),
        DeclareLaunchArgument('px4_dir', default_value=DEFAULT_PX4_DIR,
                              description='PX4-Autopilot dir (in-tree by default)'),
        DeclareLaunchArgument('gz_partition', default_value=DEFAULT_GZ_PARTITION,
                              description='Gazebo transport partition (isolates this sim; '
                                          'empty = default partition)'),
        OpaqueFunction(function=launch_setup),
    ])
