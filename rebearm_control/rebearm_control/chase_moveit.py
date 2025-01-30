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
Node for control 3DOF robot arm from joint_states

Subscribes to
    /joint_states
Publishes commands to
    /cmd_motor

"""

from time import sleep, time
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger
from sensor_msgs.msg import JointState
import atexit
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32MultiArray
from .submodules.myutil import Rebearm, trimLimits, setArmAgles
from .submodules.myconfig import *

class ChaseMoveit(Node):
    def __init__(self):
        super().__init__('state_control_node')
        self.declare_parameters(
            namespace='',
            parameters=[
           ])
        self.get_logger().info("Setting Up the Node...")
        atexit.register(self.set_park)
        
        self.robotarm = Rebearm()
        angles = self.robotarm.readAngle()
        print("Angles:", ' '.join(f'{x:.2f}' for x in angles))
        offset = self.robotarm.get_offsets()
        print("Offset:", ' '.join(f'{x:.2f}' for x in offset))
        self.robotarm.home()

        self.motorMsg = Float32MultiArray()
        setArmAgles(self.motorMsg, MOTOR1_ZERO, MOTOR2_ZERO, MOTOR3_ZERO, MOTOR4_ZERO, MOTOR5_ZERO, GRIPPER_OPEN)
       
        self._joint_sub = self.create_subscription(JointState, '/joint_states', self.moveit_callback, qos_profile_sensor_data)
        self.get_logger().info("Moveit Subscriber Awaked!! Waiting for Moveit Planning...")

    def moveit_callback(self, cmd_msg):
        self.motorMsg.data[0] = trimLimits(math.degrees(cmd_msg.position[0]))
        self.motorMsg.data[1] = trimLimits(math.degrees(cmd_msg.position[1]))
        self.motorMsg.data[2] = trimLimits(math.degrees(cmd_msg.position[2]))
        self.motorMsg.data[3] = trimLimits(math.degrees(cmd_msg.position[3]))
        self.motorMsg.data[4] = trimLimits(math.degrees(cmd_msg.position[4]))
        self.motorMsg.data[5] = trimLimits(math.degrees(cmd_msg.position[5]))
        #can't control air pump, then grip=0 always
        setArmAgles(self.motorMsg, self.motorMsg.data[0] , self.motorMsg.data[1] , self.motorMsg.data[2] , self.motorMsg.data[3], self.motorMsg.data[4], self.motorMsg.data[5])
        self.robotarm.run(self.motorMsg)
        print('M1= %.2f, M2=%.2f, M3= %.2f, M4=%.2f, M5=%.2f, G=%.2f'%(self.motorMsg.data[0],self.motorMsg.data[1],self.motorMsg.data[2],
                                                                       self.motorMsg.data[3],self.motorMsg.data[4],self.motorMsg.data[5]))

    def set_park(self):
        self.get_logger().info('Arm parking, be careful')
        self.robotarm.park()

def main(args=None):
    rclpy.init(args=args)
    myMoveit = ChaseMoveit()
    rclpy.spin(myMoveit)

    myMoveit.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
