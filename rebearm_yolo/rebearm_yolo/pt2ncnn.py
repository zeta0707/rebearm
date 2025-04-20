import rclpy
import time, os
from rclpy.node import Node
from sensor_msgs.msg import Image
from rebearm_interfaces.msg import Detections
from cv_bridge import CvBridge
from ultralytics import YOLO

# Initialize YOLO model
rosPath = os.path.expanduser('~/ros2_ws/src/rebearm/rebearm_yolo/weights/')
yolomodel = rosPath + 'rebearm.pt'

# Initialize Yolo network
model = YOLO(yolomodel,task='detect')
#please run only once to convert from pt to ncnn
model.export(format="ncnn")    # create ncnn model 
