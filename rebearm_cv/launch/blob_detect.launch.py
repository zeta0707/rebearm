#!/usr/bin/env python3
# Author: ChangWhan Lee
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  cv_parameter = LaunchConfiguration(
    'cv_parameter',
    default=os.path.join(
      get_package_share_directory('rebearm_cv'),
      'param/cvparam.yaml'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument('cv_parameter', default_value=cv_parameter),
    Node(
      package='rebearm_cv', executable='find_ball', name='blob_detect_node',
      #output='screen',
      emulate_tty=True,
      parameters=[cv_parameter],
    ),
  ])
