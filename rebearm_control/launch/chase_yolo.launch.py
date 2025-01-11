#!/usr/bin/env python3
# Author: ChangWhan Lee
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    # Get the path to default config
    config_file = os.path.join(
        get_package_share_directory('rebearm_control'), 'param', 'motor.yaml'
    )

    # Make sure to pass the YAML file inside a list of dictionaries
    with open(config_file, 'r') as f:
      config = yaml.safe_load(f) 

    # Declare the parameter you want to override
    param_arg1 = DeclareLaunchArgument(
        'det_class1',                   # actual parameter name
        default_value='cookie',         # Default value
        description='Detect class 1'
    )
    # Declare the parameter you want to override
    param_arg2 = DeclareLaunchArgument(
        'det_class2',                   # actual parameter name
        default_value='cupcake',        # Default value
        description='Detect class 2'
    )
    # Declare the parameter you want to override
    param_arg3 = DeclareLaunchArgument(
        'det_class3',                   # actual parameter name
        default_value='donut',          # Default value
        description='Detect class 3'
    )
    # Declare the parameter you want to override
    param_arg4 = DeclareLaunchArgument(
        'det_class4',                   # actual parameter name
        default_value='shortcake',      # Default value
        description='Detect class 4'
    )

    # Declare the parameter you want to override
    param_arg5 = DeclareLaunchArgument(
        'k_a',                      # actual parameter name
        default_value='-29.0',      # Default value
        description='slope'
    )

    # Declare the parameter you want to override
    param_arg6 = DeclareLaunchArgument(
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

    return LaunchDescription([
        param_arg1,  # Add the launch argument
        param_arg2,  # Add the launch argument
        param_arg3,  # Add the launch argument
        param_arg4,  # Add the launch argument
        param_arg5,  # Add the launch argument
        param_arg6,  # Add the launch argument
        node
    ])