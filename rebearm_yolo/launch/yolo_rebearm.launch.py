import os

import ament_index_python.packages
import launch
import launch_ros.actions
from launch import LaunchDescription
import yaml


def generate_launch_description():
    share_dir = ament_index_python.packages.get_package_share_directory('rebearm_yolo')
    
    params_file = os.path.join(share_dir, 'config', 'yolo_ros_rebearm.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['yolo_ros_node']['ros__parameters']

    yolo_ros_node = launch_ros.actions.Node(package='rebearm_yolo',
                                              executable='yolo_ros',
                                              output='both',
                                              parameters=[params]
                                              )
    ld = LaunchDescription()

    ld.add_action(yolo_ros_node)

    return ld