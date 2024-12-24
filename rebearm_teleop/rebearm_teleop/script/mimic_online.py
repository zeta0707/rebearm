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
from std_msgs.msg import Int32MultiArray

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

        print('Rebearm mimic human operation')
        print(msg)
        self.max_deg = self.get_parameter_or('max_deg', Parameter('max_deg', Parameter.Type.INTEGER, 120)).get_parameter_value().integer_value
        self.step_deg = self.get_parameter_or('step_deg', Parameter('step_deg', Parameter.Type.INTEGER, 20)).get_parameter_value().integer_value
        print('max ang: %s rad/s, step: %s'%
            (self.max_deg,
            self.step_deg)
        )
        print('CTRL-C to quit')

        self.angleSub = self.create_subscription(Int32MultiArray, 'motor_angles', self.cb_angles, qos_profile_sensor_data)
        self.motorMsg = Int32MultiArray()
        self.motorMsg.data = [MOTOR0_HOME, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, GRIPPER_OPEN]

        atexit.register(self.set_park)

        self.robotarm = Rebearm()
        self.robotarm.home()

    def cb_angles(self, msg):
        control_motor0 = msg.data[0]
        control_motor1 = msg.data[1]
        control_motor2 = msg.data[2]
        control_motor3 = msg.data[3]
        control_motor4 = msg.data[4]
        control_gripper = msg.data[5]
        setArmAgles(self.motorMsg, control_motor0, control_motor1, control_motor2, control_motor3, control_motor4, control_gripper)
        self.robotarm.run(self.motorMsg)
        print('M0= %d, M1=%d, M2= %d, M3=%d, G=%d'%(control_motor0, control_motor1, control_motor2, control_motor3, control_motor4, control_gripper))

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
