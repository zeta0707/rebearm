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
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import atexit
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import math
from rebearm_interfaces.msg import Detections

from .submodules.myutil import Rebearm, setArmAgles
from .submodules.myconfig import *

msg = """
Mimic Human's operation!
Caution: need to be careful
"""

class JointStateFromData(Node):
    def __init__(self):
        super().__init__('joint_state_from_data')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)        
     
        # Joint names - MUST match your URDF exactly
        self.joint_names = [
            'joint1',
            'joint2', 
            'joint3',
            'joint4',
            'joint5',
            'joint6'
        ]
        
        # Current joint positions from your msg.data
        self.current_positions = [0.0] * 6
        self.get_logger().info('Joint State Publisher from msg.data started')
        
    def update_from_msg_data(self, msg_data):
        # If msg.data is in degrees, convert to radians
        self.current_positions = [math.radians(angle) for angle in msg_data]
    
    def publish_joint_states(self):
        """Publish current joint states to /joint_states topic"""
        joint_state = JointState()
        
        # Set timestamp
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        # Set joint data
        joint_state.name = self.joint_names
        joint_state.position = self.current_positions
        
        # Optional: set velocities and efforts (leave empty if not needed)
        joint_state.velocity = []
        joint_state.effort = []
        
        # Publish the message
        self.joint_pub.publish(joint_state)

class HumanGuideNode(Node):

    def __init__(self):
        super().__init__('mimic_online_node')
        self.declare_parameters(    # bring the param from yaml file
            namespace='',
            parameters=[
                ('port', BUSLINKER2),
            ])

        print('Rebearm mimic online controller')
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

        self.port = self.get_parameter_or('port', Parameter('port', Parameter.Type.STRING, BUSLINKER2)).get_parameter_value().string_value
        print('port name: %s'%
            (self.port)
        )

        self._time_detected = 0.0
        self.detect_object = 0

        self.sub_center = self.create_subscription(Detections, "/yolo_ros/detection_result", self.update_object, qos_profile_sensor_data)
        self.get_logger().info("Subscriber set")

        # motor angle changed by topics
        self.angleSub = self.create_subscription(Float32MultiArray, 'motor_angles', self.cb_angles, qos_profile_sensor_data)
        # timer callback
        self.timer = self.create_timer(TIMER_HGUIDE, self.cb_timer)

        self.motorMsg = Float32MultiArray()
        self.motorMsg.data = [MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, MOTOR4_HOME, MOTOR5_HOME, GRIPPER_OPEN]

        self.jsNode = JointStateFromData()

        atexit.register(self.set_park)

        self.robotarm = Rebearm(self.port)
        angles = self.robotarm.readAngle()
        print("Angles:", ' '.join(f'{x:.2f}' for x in angles))
        offset = self.robotarm.get_offsets()
        print("Offset:", ' '.join(f'{x:.2f}' for x in offset))
        self.robotarm.home()

        self.allow = 0

    def cb_timer(self):
        self.jsNode.update_from_msg_data(self.motorMsg.data)
        self.jsNode.publish_joint_states()

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
                self._time_detected = time()
                #cupcake
                if message.full_class_list[box] == self.det_class2:
                    self.allow = 1
                #donut
                elif message.full_class_list[box] == self.det_class3:
                    self.allow = 0
            else:
                self.detect_object = 0
            idx = idx + 1

    def cb_angles(self, msg):
        #wait yolo class2, cupcke
        if self.allow == 0:
            return
        
        control_motor1 = msg.data[0]
        control_motor2 = msg.data[1]
        control_motor3 = msg.data[2]
        control_motor4 = msg.data[3]
        control_motor5 = msg.data[4]
        control_gripper = msg.data[5]

        self.jsNode.update_from_msg_data(msg.data)
        self.jsNode.publish_joint_states()

        if control_motor1 == MOTOR_HOMING:
            self.robotarm.home()
            print("Moving HOME")
        else:
            setArmAgles(self.motorMsg, control_motor1, control_motor2, control_motor3, control_motor4, control_motor5, control_gripper)
            self.robotarm.run(self.motorMsg)
            print('M1= %.1f, M2=%.1f, M3= %.1f, M4=%.1f, M5=%.1f, G=%.1f'%(control_motor1, control_motor2, control_motor3, control_motor4, control_motor5, control_gripper))

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
