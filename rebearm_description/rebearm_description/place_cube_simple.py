#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class CubeMarkerPublisher(Node):
    def __init__(self):
        super().__init__('cube_marker_publisher')
        self.publisher = self.create_publisher(Marker, 'cube_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"  # or your robot's base frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0  
        marker.pose.position.y = 0.70
        marker.pose.position.z = 0.0  
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10  # 5cm cube
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        marker.color.a = 1.0  # alpha
        marker.color.r = 1.0  
        marker.color.g = 1.0  # yellow
        marker.color.b = 0.0  # yellow
        
        self.publisher.publish(marker)

def main():
    rclpy.init()
    node = CubeMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()