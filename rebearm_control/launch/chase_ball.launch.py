#!/usr/bin/env python3
# Author: ChangWhan Lee
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  motor_parameter = LaunchConfiguration(
    'motor_parameter',
    default=os.path.join(
      get_package_share_directory('rebearm_control'),
      'param/motor.yaml'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument('motor_parameter', default_value=motor_parameter),

    Node(
      package='rebearm_control', executable='chase_ball', name='chase_ball_node',
	    output='screen', emulate_tty=True,
      parameters=[motor_parameter],
    ),
  ])