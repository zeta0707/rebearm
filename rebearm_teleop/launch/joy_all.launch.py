#!/usr/bin/env python3
# Author: ChangWhan Lee
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  joy_dev = LaunchConfiguration('joy_dev')
  port = LaunchConfiguration('port')

  return LaunchDescription([
    DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
    DeclareLaunchArgument('port', default_value='/dev/buslinker2'),
    
    Node(
      package='joy', executable='joy_node', name='joy_node',
      parameters=[{
        'dev': joy_dev,
        'deadzone': 0.3,
        'autorepeat_rate': 20.0,
      }]
    ),
    Node(
      package='rebearm_teleop', executable='teleop_joy', name='teleop_joy_node',
      output='screen',
      emulate_tty=True,
      parameters=[{
        'port': port,
        'max_deg': 120,
        'step_deg': 5,
      }],
    ),
  ])