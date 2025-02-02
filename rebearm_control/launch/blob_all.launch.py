#!/usr/bin/env python3
# Author: ChangWhan Lee
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def get_parameters(context):
    # Get the config choice
    color = LaunchConfiguration('color').perform(context)

    package_share_dir = get_package_share_directory('rebearm_cv')
    # Select the appropriate config file
    if color == 'green':
        config_path = os.path.join(package_share_dir, 'param', 'green.yaml')
    elif color == 'yellow':
        config_path = os.path.join(package_share_dir, 'param', 'yellow.yaml')
    elif color == 'red':
        config_path = os.path.join(package_share_dir, 'param', 'red.yaml')
    else:
        config_path = os.path.join(package_share_dir, 'param', 'blue.yaml')

    return [Node(
        package='rebearm_cv', executable='find_ball', name='blob_detect_node', 
        output='screen',
        parameters=[config_path]
    )]

def generate_launch_description():
    # Include launch file
    launch_file_1 = os.path.join(
        get_package_share_directory('rebearm_cv'), 'launch','usbcam.launch.py'
    )

    include_usbcam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_1),
    )

    # Include launch file
    launch_file_2= os.path.join(
        get_package_share_directory('rebearm_control'), 'launch','chase_ball.launch.py'
    )

    include_chaseball = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_2),
    )

    color = DeclareLaunchArgument(
    'color',
    default_value='green',
    description='Choose green, yellow, red, or blue'
    )

    return LaunchDescription([
        include_usbcam,
        include_chaseball,
        color,
        OpaqueFunction(function=get_parameters)
    ])