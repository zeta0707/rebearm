#!/usr/bin/env python3
# Author: ChangWhan Lee
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="rebearm_description",
            description="Description package with robot URDF/xacro files.",
        )
    )
    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "rebearm.rviz"]
    )

    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes = [
        rviz_node,
    ]

    return LaunchDescription(nodes)