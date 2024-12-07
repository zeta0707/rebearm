#!/usr/bin/env python3
# Author: ChangWhan Lee

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  return LaunchDescription([
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("rebearm_teleop"), '/launch', '/teleop_joy.launch.py'])
    ),

  ])
