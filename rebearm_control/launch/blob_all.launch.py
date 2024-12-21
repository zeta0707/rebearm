#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  return LaunchDescription([
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("rebearm_cv"), '/launch', '/usbcam.launch.py'])
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("rebearm_cv"), '/launch', '/blob_detect.launch.py'])
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        FindPackageShare("rebearm_control"), '/launch', '/chase_ball.launch.py'])
    ),
  ])
