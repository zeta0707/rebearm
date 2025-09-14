#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
import math

class SmartCubePickerNode(Node):
    def __init__(self):
        super().__init__('smart_cube_picker')
        
        # Publishers
        self.marker_publisher = self.create_publisher(Marker, 'cube_marker', 10)
        
        # Subscribers
        self.joint_subscriber = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for publishing marker
        self.timer = self.create_timer(0.1, self.publish_marker)  # 10Hz
        
        # State variables
        self.cube_attached = False
        self.gripper_closed_threshold = -1.32  # Amove from -1.6 to -1.0(full close)
        self.current_joint6_position = 0.0
        
        # Original cube position (when not attached)
        self.original_cube_pos = [0.0, 0.7, 0.0]
        
        # Offset from gripper to cube when attached
        self.gripper_to_cube_offset = [0.10, 0.00, -0.30]  # 2cm in front of gripper
        
        self.get_logger().info("Smart Cube Picker Node started")
        self.get_logger().info(f"Gripper close threshold: {self.gripper_closed_threshold}")

    def joint_callback(self, msg):
        """Monitor joint6 position to detect gripper state"""
        try:
            joint6_index = msg.name.index('joint6')
            self.current_joint6_position = msg.position[joint6_index]
            
            # Check if gripper should grab/release cube
            distance_to_cube = self.calculate_distance_to_cube()
            #print("dist:", distance_to_cube, self.current_joint6_position)

            # Attach if gripper is closed AND close to cube
            if (self.current_joint6_position < self.gripper_closed_threshold and 
                distance_to_cube < 0.12 and not self.cube_attached):
                self.cube_attached = True
                self.get_logger().info("Cube ATTACHED! 🦾")
            
            # Detach if gripper opens
            elif (self.current_joint6_position > self.gripper_closed_threshold and 
                  self.cube_attached):
                self.cube_attached = False
                # Return cube to original position
                self.original_cube_pos = [0.0, 0.7, 0.0]
                self.get_logger().info("Cube RELEASED! 📦")
                
        except (ValueError, IndexError):
            pass  # joint6 not found in message

    def calculate_distance_to_cube(self):
        """Calculate distance between gripper and cube"""
        try:
            # Get gripper position from TF
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'link6', rclpy.time.Time())
            
            gripper_pos = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            
            # Distance to original cube position, consider distance to gripper end
            dx = gripper_pos[0] - self.original_cube_pos[0] + 0.10
            dy = gripper_pos[1] - self.original_cube_pos[1]
            dz = gripper_pos[2] - self.original_cube_pos[2] - 0.30
            
            return math.sqrt(dx*dx + dy*dy + dz*dz)
            
        except Exception:
            return float('inf')  # Can't calculate distance

    def get_cube_position(self):
        """Get current cube position based on attachment state"""
        if not self.cube_attached:
            return self.original_cube_pos
        
        try:
            # Get gripper transform
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'link6', rclpy.time.Time())
            
            # Apply offset to get cube position relative to gripper
            cube_x = transform.transform.translation.x + self.gripper_to_cube_offset[0]
            cube_y = transform.transform.translation.y + self.gripper_to_cube_offset[1]
            cube_z = transform.transform.translation.z + self.gripper_to_cube_offset[2]
            
            return [cube_x, cube_y, cube_z]
            
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return self.original_cube_pos

    def publish_marker(self):
        """Publish cube marker"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.id = 0
        
        # Get cube position
        cube_pos = self.get_cube_position()
        marker.pose.position.x = cube_pos[0]
        marker.pose.position.y = cube_pos[1]
        marker.pose.position.z = cube_pos[2]
        marker.pose.orientation.w = 1.0
        
        # Cube size
        marker.scale.x = 0.10 # Small enough to fit in gripper
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        
        # Color changes based on state
        marker.color.a = 1.0  # alpha
        if self.cube_attached:
            # Green when attached
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            # Red when free
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        
        self.marker_publisher.publish(marker)

def main():
    rclpy.init()
    node = SmartCubePickerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()