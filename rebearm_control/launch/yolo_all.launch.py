#!/usr/bin/env python3
# Author: ChangWhan Lee
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    # Include launch file
    launch_file_1 = os.path.join(
        get_package_share_directory('rebearm_cv'), 'launch','usbcam.launch.py'
    )

    included_usbcam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_1),
    )

    robot_parameter = LaunchConfiguration(
        'robot_parameter',
        default=os.path.join(
        get_package_share_directory('rebearm_control'),
        'param/rebearm.yaml'
        )
    )

    node = Node(
      package='rebearm_control', executable='chase_yolo', name='chase_yolo_node',
      output='screen', emulate_tty=True,
        parameters=[robot_parameter]
    )
    
    node_yolo = Node(
      package='rebearm_yolo', executable='yolo_ros', name='yolo_ros_node',
      output='screen', emulate_tty=True,
        parameters=[robot_parameter]
    )

    return LaunchDescription([
        included_usbcam,        # Add the included launch file
        node,
        node_yolo,
    ])