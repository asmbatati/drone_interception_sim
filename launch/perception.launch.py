#!/usr/bin/env python3
"""Optional vision perception for the interceptor (YOLOv8 + smart_track).

Pipeline: interceptor RGB image -> YOLOv8 detections -> smart_track yolo2pose
(uses depth + camera_info) -> target-detection PoseArray. Feed the output into
the Behaviour Tree (`target_detection`) or guidance/strategy nodes to replace
ground-truth target pose with a vision estimate.

Requirements (perception only; not needed for ground-truth interception):
  * `pip install ultralytics`
  * the drone model `drone_detection_v3.pt` (shipped in smart_track/config)
  * the interceptor's depth-camera bridge running (interceptor.launch.py)
  * verify msg compatibility between the installed yolov8_ros and smart_track.

  ros2 launch drone_interception_sim perception.launch.py device:=cpu
"""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_model = os.path.join(
        get_package_share_directory('smart_track'), 'config', 'drone_detection_v3.pt')

    model = LaunchConfiguration('model')
    device = LaunchConfiguration('device')
    image_topic = LaunchConfiguration('image_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    caminfo_topic = LaunchConfiguration('caminfo_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    poses_topic = LaunchConfiguration('poses_topic')

    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('yolov8_bringup'),
                                  'launch', 'yolov8.launch.py'])
        ]),
        launch_arguments={
            'model': model,
            'device': device,
            'threshold': '0.5',
            'input_image_topic': image_topic,
            'namespace': 'yolo',
        }.items())

    yolo2pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('smart_track'),
                                  'launch', 'yolo2pose.launch.py'])
        ]),
        launch_arguments={
            'depth_topic': depth_topic,
            'caminfo_topic': caminfo_topic,
            'yolo_detections_topic': detections_topic,
            'detections_poses_topic': poses_topic,
            'detector_ns': '',
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument('device', default_value='cuda:0',
                              description='cuda:0 or cpu'),
        DeclareLaunchArgument('image_topic', default_value='/interceptor/image'),
        DeclareLaunchArgument('depth_topic', default_value='/interceptor/depth_image'),
        DeclareLaunchArgument('caminfo_topic', default_value='/interceptor/camera_info'),
        DeclareLaunchArgument('detections_topic', default_value='/yolo/detections'),
        DeclareLaunchArgument('poses_topic', default_value='/interceptor/target_detection'),
        yolo,
        yolo2pose,
    ])
