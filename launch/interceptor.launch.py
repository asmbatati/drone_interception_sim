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
                            IncludeLaunchDescription, OpaqueFunction)
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

# Isolate the ROS graph (esp. /clock) from other simulators. Every terminal that
# talks to this sim must export the same ROS_DOMAIN_ID. Empty = inherit.
DEFAULT_ROS_DOMAIN_ID = '77'

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

    # gpu:=false -> use the camera-less x500 so the gz server never invokes GL
    # rendering (runs with no GPU / a broken GL stack, and faster for RL). The
    # depth-camera bridge is then skipped. Default keeps the x500_d435 + camera.
    gpu = LaunchConfiguration('gpu').perform(context)
    if gpu == 'false':
        model, autostart = 'x500', '4001'
    else:
        model, autostart = MODEL, AUTOSTART_ID

    # Use the project's in-tree PX4 (overridable via px4_dir:=). gz_sim.launch.py
    # reads PX4_DIR from the environment, so set it here before the include.
    if px4_dir:
        os.environ['PX4_DIR'] = px4_dir

    # Isolate Gazebo transport so PX4 starts its OWN gz server for this world
    # instead of attaching to another sim's server on the default partition.
    if gz_partition:
        os.environ['GZ_PARTITION'] = gz_partition

    # Isolate the ROS graph (esp. /clock) from other simulators on the machine.
    # /clock is a ROS topic, so GZ_PARTITION is not enough: without a distinct
    # ROS_DOMAIN_ID, multiple sims' /clock collide -> "jump back in time" and PX4
    # arming is temporarily rejected. NOTE: every terminal that talks to this sim
    # (controllers, ros2 topic echo, RViz) must use the same ROS_DOMAIN_ID.
    ros_domain_id = LaunchConfiguration('ros_domain_id').perform(context)
    if ros_domain_id:
        os.environ['ROS_DOMAIN_ID'] = ros_domain_id

    # Force CPU (Mesa llvmpipe) rendering for RViz/gz when the GPU GL stack is
    # broken (e.g. NVIDIA driver/kernel-module version mismatch before a reboot).
    # LIBGL_ALWAYS_SOFTWARE alone is NOT enough when the X server's GLX vendor is
    # the broken NVIDIA one, so also route GLX through Mesa via libglvnd.
    if LaunchConfiguration('software_gl').perform(context) == 'true':
        os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
        os.environ['__GLX_VENDOR_LIBRARY_NAME'] = 'mesa'
        os.environ['GALLIUM_DRIVER'] = 'llvmpipe'

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

    # Make headless real: PX4's px4-rc.simulator starts the gz GUI unless the
    # HEADLESS env var is non-empty. It must be in the px4 PROCESS environment,
    # so set os.environ (a SetEnvironmentVariable action does NOT propagate into
    # the included gz_sim.launch.py's ExecuteProcess).
    if headless == '1':
        os.environ['HEADLESS'] = '1'
    else:
        os.environ.pop('HEADLESS', None)

    # PX4 SITL + Gazebo (this instance spawns the shared server)
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uav_gz_sim'),
                                  'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_ns': NS,
            'headless': headless,
            'gz_model_name': model,
            'gz_world': gz_world,
            'px4_autostart_id': autostart,
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

    # Sensor bridge. /clock is always needed; the D435 depth-camera topics are
    # added only with gpu:=true (the camera needs gz GL rendering).
    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    if gpu != 'false':
        bridge_args += [
            '/d435/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/d435/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/d435/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/d435/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '--ros-args',
            '-r', '/d435/depth_image:=' + NS + '/depth_image',
            '-r', '/d435/image:=' + NS + '/image',
            '-r', '/d435/points:=' + NS + '/points',
            '-r', '/d435/camera_info:=' + NS + '/camera_info',
        ]
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='interceptor_depthcam_bridge',
        arguments=bridge_args,
        parameters=[{'use_sim_time': True}], output='log'))

    # Body markers for RViz (no URDF; attached to base_link via TF). Default to
    # the real x500 mesh; marker_mesh:=none falls back to the animated quad;
    # marker_mesh:=file://... overrides with a custom mesh.
    marker_mesh = LaunchConfiguration('marker_mesh').perform(context)
    if marker_mesh == 'none':
        mesh = ''
    elif marker_mesh:
        mesh = marker_mesh
    elif px4_dir:
        mesh = ('file://' + px4_dir +
                '/Tools/simulation/gz/models/x500_base/meshes/NXP-HGD-CF.dae')
    else:
        mesh = ''
    actions.append(Node(
        package='drone_interception_sim', executable='drone_markers',
        name='drone_markers', namespace=NS,
        parameters=[{'use_sim_time': True},
                    {'frame_id': NS + '/base_link'},
                    {'marker_ns': NS},
                    {'color': [0.1, 0.4, 1.0]},   # interceptor = blue (geom fallback)
                    {'mesh_resource': mesh}],
        output='log'))

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
        DeclareLaunchArgument('ros_domain_id', default_value=DEFAULT_ROS_DOMAIN_ID,
                              description='ROS_DOMAIN_ID isolating this sim (esp. /clock); '
                                          'empty = inherit. Match it in all terminals.'),
        DeclareLaunchArgument('software_gl', default_value='false',
                              description='true = CPU (Mesa) rendering for RViz/gz when the '
                                          'GPU GL stack is broken'),
        DeclareLaunchArgument('gpu', default_value='true',
                              description='false = camera-less x500 interceptor so gz needs no '
                                          'GL (run with no GPU; faster for RL)'),
        DeclareLaunchArgument('marker_mesh', default_value='',
                              description="RViz body mesh: '' = real x500 mesh (default), "
                                          "'none' = animated geometric quad, or a file:// URI"),
        OpaqueFunction(function=launch_setup),
    ])
