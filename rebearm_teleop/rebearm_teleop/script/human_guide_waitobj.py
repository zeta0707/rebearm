#!/usr/bin/env python3
#
# Copyright (c) 2011, Willow Garage, Inc.
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

from time import sleep, time
import os
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import atexit
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from rebearm_interfaces.msg import Detections

from .submodules.myutil import Rebearm, lists_not_close
from .submodules.myconfig import *

msg = """
Move Robot by hand
---------------------------
"""

class HumanGuideNode(Node):

    def __init__(self):
        super().__init__('human_guide_node')
        self.declare_parameters(    # bring the param from yaml file
            namespace='',
            parameters=[
                ('port', BUSLINKER2),
            ])

        print('Rebearm human guide controller')
        print(msg)
        print('CTRL-C to quit')

        # Declare string parameters that can be overridden
        self.declare_parameter('det_class1', 'cookie')
        self.declare_parameter('det_class2', 'cupcake')
        self.declare_parameter('det_class3', 'donut')
        self.declare_parameter('det_class4', 'shortcake')
        self.det_class1 = self.get_parameter('det_class1').get_parameter_value().string_value
        self.det_class2 = self.get_parameter('det_class2').get_parameter_value().string_value
        self.det_class3 = self.get_parameter('det_class3').get_parameter_value().string_value
        self.det_class4 = self.get_parameter('det_class4').get_parameter_value().string_value
        
        print('DETECT_CLASS1: %s, DETECT_CLASS2: %s, DETECT_CLASS3: %s, DETECT_CLASS4: %s'%
            (self.det_class1, self.det_class2, self.det_class3, self.det_class4)
        )

        atexit.register(self.set_park)

        self.blob_x = 0.0
        self.blob_y = 0.0
        self._time_detected = 0.0
        self.detect_object = 0

        self.sub_center = self.create_subscription(Detections, "/yolo_ros/detection_result", self.update_object, qos_profile_sensor_data)
        self.get_logger().info("Subscriber set")

        self.port = self.get_parameter_or('port', Parameter('port', Parameter.Type.STRING, BUSLINKER2)).get_parameter_value().string_value
        print('port name: %s'%
            (self.port)
        )

        self.anglePub = self.create_publisher(Float32MultiArray, 'motor_angles', qos_profile_sensor_data)
        self.motorMsg = Float32MultiArray()
        self.motorMsg.data = [MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, MOTOR4_HOME, MOTOR5_HOME, GRIPPER_OPEN]
        atexit.register(self.set_park)

        self.robotarm = Rebearm(self.port)
        #just check arm's current position
        angles = self.robotarm.readAngle()
        print("Angles:", ' '.join(f'{x:.2f}' for x in angles))
        offset = self.robotarm.get_offsets()
        print("Offset:", ' '.join(f'{x:.2f}' for x in offset))
        self.robotarm.home()

        self.prev_angles = self.robotarm.readAngle()
        # timer callback
        self.timer = self.create_timer(TIMER_HGUIDE, self.cb_timer)

        rosPath = os.path.expanduser('~/ros2_ws/src/rebearm/rebearm_teleop/rebearm_teleop/script/')
        self.fhandle = open(rosPath + 'automove.csv', 'w')

        self.prev_time = time()

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
                    self.robotarm.motors_off()
                elif message.full_class_list[box] == self.det_class2:
                    self.robotarm.motors_off()
                elif message.full_class_list[box] == self.det_class3:
                    self.robotarm.motors_on()
                    self.robotarm.home()
                elif message.full_class_list[box] == self.det_class4:
                    self.robotarm.motors_on()
                    self.robotarm.home()

                self.get_logger().info("Detected: %.2f  %.2f"%(self.blob_x, self.blob_y))
            else:
                self.detect_object = 0
            idx = idx + 1

    def cb_timer(self):
        timediff = time() - self.prev_time
        self.prev_time = time()
        angles = self.robotarm.readAngle()

        #only publish topic, write to file when change angles
        if lists_not_close(self.prev_angles, angles):
            self.motorMsg.data = angles
            print('M1= %.2f, M2=%.2f, M3= %.2f, M4=%.2f, M5=%.2f, G=%.2f'%(self.motorMsg.data[0],self.motorMsg.data[1],self.motorMsg.data[2],
                                                                        self.motorMsg.data[3],self.motorMsg.data[4],self.motorMsg.data[5]))
            self.fhandle.write(f'{self.motorMsg.data[0]:.2f}' + ',' + f'{self.motorMsg.data[1]:.2f}'  + ',' + f'{self.motorMsg.data[2]:.2f}'  + ',' + f'{self.motorMsg.data[3]:.2f}'
                            + ',' + f'{self.motorMsg.data[4]:.2f}'  + ',' + f'{self.motorMsg.data[5]:.2f}'  + ',' + f'{timediff:.2f}' + '\n')

            self.fhandle.flush()
            self.anglePub.publish(self.motorMsg)
        
        self.prev_angles = angles
        
    def set_park(self):
        self.get_logger().info('Arm parking, be careful')
        self.robotarm.park()

def main(args=None):
    rclpy.init(args=args)
    teleop_hguide =  HumanGuideNode()
    rclpy.spin(teleop_hguide)
    teleop_hguide.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
