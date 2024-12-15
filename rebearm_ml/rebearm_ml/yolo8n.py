import sys, os
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg        import Image
from cv_bridge              import CvBridge, CvBridgeError
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from ultralytics import YOLO

def check_dependencies():
    global missing_modules
    missing_modules = []
    try:
        import cv2
    except ImportError:
        missing_modules.append("opencv-python")
    try:
        import numpy as np
    except ImportError:
        missing_modules.append("numpy")
    try:
        from ultralytics import YOLO
    except ImportError:
        missing_modules.append("ultralytics")
    try:
        import rclpy
    except ImportError:
        missing_modules.append("rclpy")
    
    if missing_modules:
        print("Error: The following required modules are missing:")
        for module in missing_modules:
            print(f"  - {module}")
        print("\nPlease install the missing modules using:")
        print(f"pip install {' '.join(missing_modules)}")
        print("\nNote: For pyrealsense2 and rclpy, you may need to follow specific installation instructions for your system.")
        sys.exit(1)

class PersonDetectionNode(Node):
    def __init__(self):
        super().__init__('person_detection_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/image_raw",self.callback, qos_profile_sensor_data)
        print ("<< Subscribed to topic /image_raw")
        self.image_pub = self.create_publisher(Image, '/image_yolo', 10)
        print ("<< Publisher to topic /image_yolo")

        # Initialize YOLO model
        rosPath = os.path.expanduser('~/ros2_ws/src/rebearm/rebearm_ml/rebearm_ml/')
        yolov8n = rosPath + "yolov8n.pt"
        self.model = YOLO(yolov8n)  # Load the smallest YOLOv8 model

    def callback(self,msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Convert to NumPy array
            numpy_array = np.asanyarray(cv_image)
            #print("Image shape:", numpy_array.shape)
        
        except Exception as e:
            self.get_logger().error(f'Conversion error: {str(e)}')

        # Run YOLO detection
        results = self.model(numpy_array)

        # Process results
        for result in results:
            boxes = result.boxes.cpu().numpy()
            for box in boxes:
                if box.cls == 0:  # Class 0 is person in COCO dataset
                    x1, y1, x2, y2 = box.xyxy[0].astype(int)
                    # Calculate the center of the bounding box
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    print(f"center_x: {center_x}")
                    print(f"center_y: {center_y}")
                    cv2.rectangle(numpy_array, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    try:
                        # Convert NumPy array to ROS Image message
                        ros_image = self.bridge.cv2_to_imgmsg(numpy_array, encoding="bgr8")
                        
                        # Publish the image
                        self.image_pub.publish(ros_image)
                        self.get_logger().info('Publishing image')
                    
                    except Exception as e:
                        self.get_logger().error(f'Error publishing image: {str(e)}')


def main(args=None):
    check_dependencies()
    rclpy.init(args=args)
    node = PersonDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()