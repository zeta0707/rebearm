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

from .submodules.myutil import Rebearm, setArmAgles
from .submodules.myconfig import *

msg = """
Mimic Human's operation!
Caution: need to be careful
"""

class HumanGuideNode(Node):

    def __init__(self):
        super().__init__('mimic_online_node')
        self.declare_parameters(    # bring the param from yaml file
            namespace='',
            parameters=[
                ('max_deg', 120),
                ('step_deg', 20),
            ])

        print('Rebearm mimic online controller')
        print(msg)
        print('CTRL-C to quit')

        self.angleSub = self.create_subscription(Float32MultiArray, 'motor_angles', self.cb_angles, qos_profile_sensor_data)
        self.motorMsg = Float32MultiArray()
        self.motorMsg.data = [MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, MOTOR4_HOME, MOTOR5_HOME, GRIPPER_OPEN]

        atexit.register(self.set_park)

        self.robotarm = Rebearm()
        angles = self.robotarm.readAngle()
        print("Angles:", ' '.join(f'{x:.2f}' for x in angles))
        offset = self.robotarm.get_offsets()
        print("Offset:", ' '.join(f'{x:.2f}' for x in offset))
        self.robotarm.home()

    def cb_angles(self, msg):
        control_motor1 = msg.data[0]
        control_motor2 = msg.data[1]
        control_motor3 = msg.data[2]
        control_motor4 = msg.data[3]
        control_motor5 = msg.data[4]
        control_gripper = msg.data[5]
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
