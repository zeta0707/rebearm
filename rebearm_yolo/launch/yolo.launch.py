from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    # Get the path to default config
    config_file = os.path.join(
        get_package_share_directory('rebearm_yolo'), 'config', 'yolo_ros.yaml'
    )

    # Make sure to pass the YAML file inside a list of dictionaries
    with open(config_file, 'r') as f:
        config = {'ros__parameters': yaml.safe_load(f)}

    # Declare the parameter you want to override
    param_arg = DeclareLaunchArgument(
        'yolo_model',                   # actual parameter name
        default_value='rebearm11n.pt',  # Default value
        description='Description of the parameter'
    )

    node = Node(
      package='rebearm_yolo', executable='yolo_ros', name='yolo_ros_node',
      output='screen', emulate_tty=True,
        parameters=[
            config,
            {
                'yolo_model': LaunchConfiguration('yolo_model')  #actual parameter name
            }
        ]
    )

    return LaunchDescription([
        param_arg,  # Add the launch argument
        node
    ])