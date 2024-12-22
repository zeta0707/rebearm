#!/usr/bin/env python3
# Author: ChangWhan Lee
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  return LaunchDescription([
    Node(
      package='rebearm_cv', executable='usbcam_pub', name='usbcam_node',
      output='screen', emulate_tty=True,
    ),
  ])
