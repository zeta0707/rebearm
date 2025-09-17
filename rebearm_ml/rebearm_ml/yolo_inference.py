#!/usr/bin/env python3
#
# Copyright (c) 2024, ChangWhan Lee
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Gets the position of the blob and it commands to steer the wheels

referenced from tizianofiorenzani/ros_tutorials
url: https://github.com/tizianofiorenzani/ros_tutorials

Subscribes to
    /blob/point_blob

"""
from time import sleep, time
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rebearm_interfaces.msg import Detections
from rclpy.qos import qos_profile_sensor_data
import atexit
from std_msgs.msg import Float32MultiArray

from .submodules.myutil import Rebearm, setArmAgles
from .submodules.myconfig import *
from .iknet import IKNet

import os, torch

MAX_Y = 3

class IKnetYolo(Node):
    def __init__(self):
        super().__init__('nn_yolo_node')
        
        # Declare string parameters that can be overridden
        self.declare_parameter('det_class1', 'cookie')
        self.declare_parameter('det_class2', 'cupcake')
        self.declare_parameter('det_class3', 'donut')
        self.declare_parameter('det_class4', 'shortcake')
        self.declare_parameter('k_a', 0.0)
        self.declare_parameter('k_b', 0.0)
        
        self.get_logger().info("Setting Up the Node...")
        self.det_class1 = self.get_parameter('det_class1').get_parameter_value().string_value
        self.det_class2 = self.get_parameter('det_class2').get_parameter_value().string_value
        self.det_class3 = self.get_parameter('det_class3').get_parameter_value().string_value
        self.det_class4 = self.get_parameter('det_class4').get_parameter_value().string_value
        self.k_a = self.get_parameter('k_a').get_parameter_value().double_value
        self.k_b = self.get_parameter('k_b').get_parameter_value().double_value
        
        print('DETECT_CLASS1: %s, DETECT_CLASS2: %s, DETECT_CLASS3: %s, DETECT_CLASS4: %s'%
            (self.det_class1, self.det_class2, self.det_class3, self.det_class4)
        )
        print('k_a: %.2f, k_b: %.2f'%(self.k_a, self.k_b))

        atexit.register(self.set_park)

        self.blob_x = 0.0
        self.blob_y = 0.0
        self._time_detected = 0.0
        self.detect_object = 0

        self.sub_center = self.create_subscription(Detections, "/yolo_ros/detection_result", self.update_object, qos_profile_sensor_data)

        self.blob_x = 0.0
        self.blob_y = 0.0
        self._time_detected = 0.0
        self.detect_object = 0

        self.motorMsg = Float32MultiArray()
        setArmAgles(self.motorMsg, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, MOTOR4_HOME, MOTOR5_HOME, GRIPPER_OPEN)
        self.get_logger().info("Setting Up control node...")

        # Create a timer that will gate the node actions twice a second
        self.timer = self.create_timer(0.1, self.node_callback)

        self.robotarm = Rebearm()
        angles = self.robotarm.readAngle()
        print("Angles:", ' '.join(f'{x:.2f}' for x in angles))
        offset = self.robotarm.get_offsets()
        print("Offset:", ' '.join(f'{x:.2f}' for x in offset))
        
        self.armStatus = 'HOMING'
        self.robotarm.home()
        self.armStatus = 'SEARCHING'

        rosPath = os.path.expanduser('~/ros2_ws/src/rebearm/rebearm_ml/rebearm_ml/')
        modely = rosPath + "iknet_y.pth"

        self.device = torch.device('cpu')
        self.modely = IKNet(MAX_Y)
        self.modely.to(self.device)
        self.modely.load_state_dict(torch.load(modely, map_location=self.device))
        self.modely.eval()

    @property
    def is_detected(self):
        return time() - self._time_detected < 1.0

    def update_object(self, message):
        #ignore 1 second previous message
        msg_secs = message.header.stamp.sec
        now = self.get_clock().now().to_msg().sec
        if (msg_secs + 1 < now):
            self.get_logger().info("Stamp %d, %d" %(now, msg_secs ) )
            return
        
        idx = 0
        for box in message.class_id:
            print(message.full_class_list[box])
            if (message.full_class_list[box] == self.det_class1) or (message.full_class_list[box] == self.det_class2) or (message.full_class_list[box] == self.det_class3) or (message.full_class_list[box] == self.det_class4) :
                self.blob_x = float(message.bbx_center_x[idx]/PICTURE_SIZE_X*2 - 1.0)
                self.blob_y = float(message.bbx_center_y[idx]/PICTURE_SIZE_Y*2 - 1.0)
                self._time_detected = time()
                if message.full_class_list[box] == self.det_class1:
                    self.detect_object = 1
                elif message.full_class_list[box] == self.det_class2:
                    self.detect_object = 2
                elif message.full_class_list[box] == self.det_class3:
                    self.detect_object = 3
                elif message.full_class_list[box] == self.det_class4:
                    self.detect_object = 4

                self.get_logger().info("Detected: %.2f  %.2f"%(self.blob_x, self.blob_y))
            else:
                self.detect_object = 0
            idx = idx + 1

    def get_control_action(self):
        if self.armStatus != 'SEARCHING' :
            print(self.armStatus)
            return

        if self.is_detected == 1:
            self.armStatus = 'PICKUP'
            #caculate angles from linear equation
            outputx = self.k_a*self.blob_x + self.k_b
            print(f"input: {self.blob_x}")
            print(f"output: {outputx}")

            #caculate angles from IKNet
            input_ = torch.FloatTensor([self.blob_y + 1.0])
            input_ = input_.to(self.device)
            print(f"input: {input_}")
            outputy = self.modely(input_)
            print(f"output: {outputy}")

            #motor move directly
            self.get_logger().info("Go to object")
            #move x, and lift up a little
            self.motorMsg.data[0] = outputx
            self.motorMsg.data[1] = MOTOR_NOMOVE
            self.motorMsg.data[2] = MOTOR3_HOME + 5.0
            self.motorMsg.data[3] = MOTOR4_HOME + 5.0
            self.motorMsg.data[4] = MOTOR_NOMOVE
            self.robotarm.run(self.motorMsg)
            sleep(1.0)
            #compensate motor2,3
            self.motorMsg.data[0] = MOTOR_NOMOVE
            self.motorMsg.data[2] = outputy[1].item() 
            self.motorMsg.data[3] = outputy[2].item() 
            self.robotarm.run(self.motorMsg)
            sleep(0.5)
            self.motorMsg.data[0] = MOTOR_NOMOVE
            #compenstate manually for far distance
            self.motorMsg.data[1] = outputy[0].item() + self.blob_y*8.0
            print(f"output1: {self.motorMsg.data[1]}")
            self.robotarm.run(self.motorMsg)
            sleep(0.5)

            self.get_logger().info("Picking up")
            #then pick it up, need new function
            self.robotarm.picknplace(self.detect_object , 0)
            self.reset_avoid()

    def reset_avoid(self):
        self.motorMsg.data[0] = MOTOR1_HOME
        self.motorMsg.data[1] = MOTOR2_HOME
        self.motorMsg.data[2] = MOTOR3_HOME
        self.motorMsg.data[3] = MOTOR4_HOME
        self.motorMsg.data[4] = MOTOR5_HOME
        self.robotarm.run(self.motorMsg)
        sleep(1.0)
        self.get_logger().info("reset avoid")
        self.armStatus = 'SEARCHING'
        self.detect_object = 0

    def node_callback(self):
         # -- update the message
        self.get_control_action()

    def set_park(self):
        self.get_logger().info('Arm parking, be careful')
        self.robotarm.park()

def main(args=None):
    rclpy.init(args=args)
    iknet_yolo = IKnetYolo()
    rclpy.spin(iknet_yolo)
    iknet_yolo.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
