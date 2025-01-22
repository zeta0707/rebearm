import rclpy
import time, os
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

from sensor_msgs.msg import Image
from rebearm_interfaces.msg import Detections
from cv_bridge import CvBridge
from ultralytics import YOLO
import ncnn
import numpy as np
import cv2

class YoloROS(Node):

    def __init__(self):
        super().__init__('yolo_ros_node')

        self.declare_parameter("yolo_model",                "rebearm11n_ncnn_model")
        self.declare_parameter("input_rgb_topic",           "/image_raw")
        self.declare_parameter("publish_annotated_image",   True)
        self.declare_parameter("rgb_topic",                 "/yolo_ros/rgb_image")
        self.declare_parameter("annotated_topic",           "/yolo_ros/annotated_image")
        self.declare_parameter("detailed_topic",            "/yolo_ros/detection_result")
        self.declare_parameter("threshold",                 0.25)
        self.declare_parameter("device",                    "cpu")

        self.yolo_model                 = self.get_parameter("yolo_model").get_parameter_value().string_value
        self.input_rgb_topic            = self.get_parameter("input_rgb_topic").get_parameter_value().string_value
        self.publish_annotated_image    = self.get_parameter("publish_annotated_image").get_parameter_value().bool_value
        self.rgb_topic                  = self.get_parameter("rgb_topic").get_parameter_value().string_value
        self.annotated_topic            = self.get_parameter("annotated_topic").get_parameter_value().string_value
        self.detailed_topic             = self.get_parameter("detailed_topic").get_parameter_value().string_value
        self.threshold                  = self.get_parameter("threshold").get_parameter_value().double_value
        self.device                     = self.get_parameter("device").get_parameter_value().string_value

        self.bridge = CvBridge()

        # Initialize YOLO model
        rosPath = os.path.expanduser('~/ros2_ws/src/rebearm/rebearm_yolo/weights/')
        yolomodel = rosPath + self.yolo_model

        # Initialize NCNN network
        self.model = YOLO(yolomodel)

        self.get_logger().info("Setting Up yolo_ros_node...")
        self.get_logger().info("Yolo model: %s, %s, %.2f "%(self.yolo_model , self.device, self.threshold))
        
        self.subscriber_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                                 history=QoSHistoryPolicy.KEEP_LAST,
                                                 depth=1)

        self.subscription = self.create_subscription(Image, self.input_rgb_topic, self.image_callback, qos_profile=self.subscriber_qos_profile)
        self.publisher_results  = self.create_publisher(Detections, self.detailed_topic, qos_profile_sensor_data)
        self.publisher_rgb      = self.create_publisher(Image, self.rgb_topic, qos_profile_sensor_data)

        if self.publish_annotated_image:
            self.publisher_image    = self.create_publisher(Image, self.annotated_topic, qos_profile_sensor_data)

        self.counter = 0
        self.time = 0

        self.detection_msg = Detections()
        self.class_list_set = False

    def image_callback(self, rgb_image):
        start = time.time_ns()
        cv_image = self.bridge.imgmsg_to_cv2(rgb_image, desired_encoding="bgr8")
        self.result = self.model(cv_image)

        # Calculate and log processing time
        end = time.time_ns()
        process_time = (end - start) / 1e6  # Convert to milliseconds
        self.get_logger().info(f'Processing time: {process_time:.2f}ms')
	
	    #self.detection_msg.header       = rgb_image.header
        self.detection_msg.header.stamp = self.get_clock().now().to_msg()

        if (not self.class_list_set) and (self.result is not None):
            for i in range(len(self.result[0].names)):
                self.detection_msg.full_class_list.append(self.result[0].names.get(i))
                self.class_list_set = True

        if self.result is not None:
            
            self.detection_msg.detections = True

            self.detection_msg.bbx_center_x = []
            self.detection_msg.bbx_center_y = []
            self.detection_msg.bbx_size_w   = []
            self.detection_msg.bbx_size_h   = []
            self.detection_msg.class_id     = []
            self.detection_msg.confidence   = []
        
            for bbox, cls, conf in zip(self.result[0].boxes.xywh, self.result[0].boxes.cls, self.result[0].boxes.conf):

                self.detection_msg.bbx_center_x.append(int(bbox[0]))
                self.detection_msg.bbx_center_y.append(int(bbox[1]))
                self.detection_msg.bbx_size_w.append(int(bbox[2]))
                self.detection_msg.bbx_size_h.append(int(bbox[3]))
                self.detection_msg.class_id.append(int(cls))
                self.detection_msg.confidence.append(float(conf))

            self.publisher_results.publish(self.detection_msg)
            self.publisher_rgb.publish(rgb_image)

            if self.publish_annotated_image:
                self.output_image = self.result[0].plot(conf=True, line_width=1, font_size=1, font="Arial.ttf", labels=True, boxes=True)
                result_msg        = self.bridge.cv2_to_imgmsg(self.output_image, encoding="bgr8")
                
                self.publisher_image.publish(result_msg)
        else:
            self.detection_msg.detections = False

            self.detection_msg.bbx_center_x = []
            self.detection_msg.bbx_center_y = []
            self.detection_msg.bbx_size_w   = []
            self.detection_msg.bbx_size_h   = []
            self.detection_msg.class_id     = []
            self.detection_msg.confidence   = []

            self.publisher_results.publish(self.detection_msg)

        self.counter += 1
        self.time += time.time_ns() - start

        if (self.counter == 100):
            self.get_logger().info('RGB callback execution time for 100 loops: %d ms' % ((self.time/100)/1000000))
            self.time = 0
            self.counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = YoloROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
