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
    package_name = 'rebearm_cv'
    launch_file_1 = os.path.join(
        get_package_share_directory(package_name), 'launch','usbcam.launch.py'
    )

    included_usbcam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_1),
    )

    # Get the path to default config
    param_file = os.path.join(
        get_package_share_directory('rebearm_control'), 'param', 'motor.yaml'
    )

    # Make sure to pass the YAML file inside a list of dictionaries
    with open(param_file, 'r') as f:
        config = {'ros__parameters': yaml.safe_load(f)}

    # Declare the parameter you want to override
    param_arg1 = DeclareLaunchArgument(
        'det_class1',                   # actual parameter name
        default_value='watermelon',     # Default value
        description='Detect class 1'
    )

    # Declare the parameter you want to override
    param_arg2 = DeclareLaunchArgument(
        'det_class2',                   # actual parameter name
        default_value='pineapple',      # Default value
        description='Detect class 2'
    )

    # Declare the parameter you want to override
    param_arg3 = DeclareLaunchArgument(
        'k_a',                      # actual parameter name
        default_value='-29.0',      # Default value
        description='slope'
    )

    # Declare the parameter you want to override
    param_arg4 = DeclareLaunchArgument(
        'k_b',                    # actual parameter name
        default_value='1.0',      # Default value
        description='offset'
    )

    node = Node(
      package='rebearm_control', executable='chase_yolo', name='chase_yolo_node',
      output='screen', emulate_tty=True,
        parameters=[
            config,
            {
                'det_class1': LaunchConfiguration('det_class1'),  #actual parameter name
                'det_class2': LaunchConfiguration('det_class2'),  #actual parameter name
                'k_a': LaunchConfiguration('k_a'),                #actual parameter name
                'k_b': LaunchConfiguration('k_b')                 #actual parameter name
            }
        ]
    )

    # Get the path to default config
    config_file = os.path.join(
        get_package_share_directory('rebearm_yolo'), 'config', 'yolo_ros.yaml'
    )

    # Make sure to pass the YAML file inside a list of dictionaries
    with open(config_file, 'r') as f:
        config_yolo = {'ros__parameters': yaml.safe_load(f)}

    # Declare the parameter you want to override
    param_arg = DeclareLaunchArgument(
        'yolo_model',                   # Replace with your actual parameter name
        default_value='rebearm11n.pt',  # Replace with your default value
        description='Description of the parameter'
    )

    node_yolo = Node(
      package='rebearm_yolo', executable='yolo_ros', name='yolo_ros_node',
      output='screen', emulate_tty=True,
        parameters=[
            config_yolo,
            {
                'yolo_model': LaunchConfiguration('yolo_model')  # Replace with your actual parameter name
            }
        ]
    )

    return LaunchDescription([
        included_usbcam,        # Add the included launch file
        param_arg1,  # Add the launch argument
        param_arg2,  # Add the launch argument
        param_arg3,  # Add the launch argument
        param_arg4,  # Add the launch argument
        node,
        param_arg,      # Add the launch argument
        node_yolo,
    ])