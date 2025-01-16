#!/usr/bin/env python3
# Author: ChangWhan Lee
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def get_parameters(context):
  # Get the config choice
  color = LaunchConfiguration('color').perform(context)

  package_share_dir = get_package_share_directory('rebearm_cv')
  # Select the appropriate config file
  if color == 'green':
      config_path = os.path.join(package_share_dir, 'param', 'green.yaml')
  elif color == 'yellow':
      config_path = os.path.join(package_share_dir, 'param', 'yellow.yaml')
  elif color == 'pink':
      config_path = os.path.join(package_share_dir, 'param', 'pink.yaml')
  else:
      config_path = os.path.join(package_share_dir, 'param', 'blue.yaml')

  return [Node(
    package='rebearm_cv', executable='find_ball', name='blob_detect_node', 
    output='screen',
    parameters=[config_path]
  )]

def generate_launch_description():
    color = DeclareLaunchArgument(
        'color',
        default_value='green',
        description='Choose green, yellow, pink, or blue'
    )

    return LaunchDescription([
        color,
        OpaqueFunction(function=get_parameters)
    ])
