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

        # Initialize YOLO model
        rosPath = os.path.expanduser('~/ros2_ws/src/rebearm/rebearm_ml/rebearm_ml/')
        yolov8n = rosPath + "yolov8n.pt"
        self.model = YOLO(yolov8n)  # Load the smallest YOLOv8 model

    def callback(self,data):
        # Convert images to numpy arrays
        color_image = np.asanyarray(data)
        print(color_image.shape)
        # Run YOLO detection
        results = self.model(color_image)

        # Process results
        for result in results:
            boxes = result.boxes.cpu().numpy()
            for box in boxes:
                if box.cls == 0:  # Class 0 is person in COCO dataset
                    x1, y1, x2, y2 = box.xyxy[0].astype(int)
                    # Calculate the center of the bounding box
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2

                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    print(center_x, center_y)

        # Display the image
        cv2.imshow('RealSense', data)
        cv2.waitKey(1)


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