#!/usr/bin/env python3
# Author: ChangWhan Lee
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='rebearm_teleop', executable='mimic_teleop', name='mimic_teleop_node',
      output='screen', emulate_tty=True,
    ),

  ])
