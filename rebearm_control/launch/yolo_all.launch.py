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
    # Get the path to default config
    config_file = os.path.join(
        get_package_share_directory('rebearm_yolo'), 'config', 'yolo_ros.yaml'
    )

    # Make sure to pass the YAML file inside a list of dictionaries
    with open(config_file, 'r') as f:
        config = {'ros__parameters': yaml.safe_load(f)}

    # Declare the parameter you want to override
    param_arg = DeclareLaunchArgument(
        'yolo_model',                   # Replace with your actual parameter name
        default_value='rebearm11n.pt',  # Replace with your default value
        description='Description of the parameter'
    )

    # Include launch file
    package_name = 'rebearm_cv'
    launch_file_1 = os.path.join(
        get_package_share_directory(package_name), 'launch','usbcam.launch.py'
    )

    included_usbcam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_1),
    )

    # Include another launch file
    package_name = 'rebearm_control'
    launch_file_2 = os.path.join(
        get_package_share_directory(package_name), 'launch', 'chase_yolo.launch.py'
    )

    included_chaseyolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_2),
    )

    node = Node(
      package='rebearm_yolo', executable='yolo_ros', name='yolo_ros_node',
      output='screen', emulate_tty=True,
        parameters=[
            config,
            {
                'yolo_model': LaunchConfiguration('yolo_model')  # Replace with your actual parameter name
            }
        ]
    )

    return LaunchDescription([
        param_arg,            # Add the launch argument
        included_usbcam,      # Add the included launch file
        included_chaseyolo,   # Add the included launch file
        node
    ])