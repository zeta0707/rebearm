#!/usr/bin/env python3
# Author: ChangWhan Lee
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
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

    return LaunchDescription([
        node
    ])