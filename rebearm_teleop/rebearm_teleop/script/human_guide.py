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
from std_msgs.msg import Int32MultiArray

from .submodules.myutil import Rebearm
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
            ])

        print('Rebearm Human Guide controller')
        print(msg)
        print('CTRL-C to quit')

        self.anglePub = self.create_publisher(Int32MultiArray, 'motor_angles', qos_profile_sensor_data)
        self.motorMsg = Int32MultiArray()
        self.motorMsg.data = [MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, MOTOR4_HOME, MOTOR5_HOME, GRIPPER_OPEN]

        atexit.register(self.set_park)

        self.robotarm = Rebearm()
        self.robotarm.home()
        offset = self.robotarm.get_offsets()
        print("Offsets:", offset)

        # timer callback
        self.timer = self.create_timer(TIMER_HGUIDE, self.cb_timer)

        rosPath = os.path.expanduser('~/ros2_ws/src/rebearm/rebearm_teleop/rebearm_teleop/script/')
        self.fhandle = open(rosPath + 'automove.csv', 'w')

        self.prev_time = time()
        self.prev_time_move = time()
        self.timediff = 0.0
        self.robotarm.motors_off()

    def cb_timer(self):
        timediff = time() - self.prev_time
        self.motorMsg.data = self.robotarm.readAngle()

        self.timediff = time() - self.prev_time
        self.prev_time = time()
                
        print('M1= %d, M2=%d, M3= %d, M4=%d, M5=%d, G=%d'%(self.motorMsg.data[0], self.motorMsg.data[1], self.motorMsg.data[2], self.motorMsg.data[3], self.motorMsg.data[4], self.motorMsg.data[5]))
        self.fhandle.write(str(self.motorMsg.data[0]) + ',' + str(self.motorMsg.data[1]) + ',' + str(self.motorMsg.data[2]) 
                           + ',' + str(self.motorMsg.data[3]) + ',' + str(self.motorMsg.data[4]) +  ',' + str(self.motorMsg.data[5]) +',' + str(timediff) + '\n')
        self.fhandle.flush()
        self.anglePub.publish(self.motorMsg)
        
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
