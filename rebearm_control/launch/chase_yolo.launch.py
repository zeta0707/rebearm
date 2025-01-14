#!/usr/bin/env python3
# Author: ChangWhan Lee
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():

    package_share_dir = get_package_share_directory('rebearm_control')
    # Define config paths using os.path.join
    config1_path = os.path.join(package_share_dir, 'param', 'rebearm.yaml')
    config2_path = os.path.join(package_share_dir, 'param', 'coco.yaml')

    # Declare an argument to choose the parameter file
    config_choice = DeclareLaunchArgument(
        'config_choice',
        default_value='rebearm',
        description='Choose either rebearm or coco'
    )

    param_file = PythonExpression([
        f'"{config1_path}" if "',
        LaunchConfiguration('config_choice'),
        f'" == "rebearm" else "{config2_path}"'
    ])

    node = Node(
      package='rebearm_control', executable='chase_yolo', name='chase_yolo_node',
      output='screen', emulate_tty=True,
      parameters=[param_file]
    )

    return LaunchDescription([
        config_choice,
        node
    ])