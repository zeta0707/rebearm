from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    yolo_parameter = LaunchConfiguration(
        'yolo_parameter',
        default=os.path.join(
        get_package_share_directory('rebearm_control'),
        'param/rebearm.yaml'
        )
    )
    
    node = Node(
      package='rebearm_yolo', executable='ncnn_ros', name='yolo_ros_node',
      output='screen', emulate_tty=True,
        parameters=[yolo_parameter]
    )

    return LaunchDescription([
        node
    ])
