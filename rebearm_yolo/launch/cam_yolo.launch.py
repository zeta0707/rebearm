from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    yolo_parameter = LaunchConfiguration(
        'yolo_parameter',
        default=os.path.join(
        get_package_share_directory('rebearm_control'),
        'param/rebearm.yaml'
        )
    )
    
    yolo_node = Node(
        package='rebearm_yolo', executable='ncnn_ros', name='yolo_ros_node',
        output='screen', emulate_tty=True,
        parameters=[yolo_parameter]
    )

    # Include launch file
    launch_file_1 = os.path.join(
        get_package_share_directory('rebearm_cv'), 'launch','usbcam.launch.py'
    )

    include_usbcam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_1),
    )

    return LaunchDescription([
        include_usbcam,
        yolo_node,
    ])
