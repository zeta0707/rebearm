#!/usr/bin/env python3
# Author: ChangWhan Lee
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("rebearm_description"), '/launch', '/rebearm_description.launch.py'])
    ),

    Node(
      package='rebearm_control', executable='chase_moveit', name='state_control_node',
      output='screen', emulate_tty=True,
    ),
  ])
