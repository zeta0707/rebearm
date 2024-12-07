#!/usr/bin/env python3
# Author: ChangWhan Lee

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='rebearm_teleop', executable='teleop_keyboard', name='teleop_keyboard_node',
      output='screen', emulate_tty=True,
      prefix='gnome-terminal --'
    ),

  ])
