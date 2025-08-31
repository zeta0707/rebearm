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

import os, sys
from time import sleep, time
import rclpy
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node

from .submodules.myutil import Rebearm, setArmAgles
from .submodules.myconfig import *

msg = """
Mimic Human's operation!
Caution: need to run teleop first
"""

class MimcNode(Node):
    def __init__(self):
        super().__init__('mimic_offline_node')
        
        # Declare parameters with default values
        self.declare_parameter('port', BUSLINKER2)  # serial port

        # Get parameter values
        self.PORT = self.get_parameter('port').value

def main():
    rclpy.init()

    node = MimcNode()       # generate node
    print('Port: %s' %
            (node.PORT)
    )
    print('Rebearm Mimic offline controller')
    print(msg)
    print('CTRL-C to quit')

    robotarm = Rebearm(node.PORT)
    angles=robotarm.readAngle()
    print("Angles:", ' '.join(f'{x:.2f}' for x in angles))
    offset = robotarm.get_offsets()
    print("Offset:", ' '.join(f'{x:.2f}' for x in offset))
    robotarm.home()

    try:
        rosPath = os.path.expanduser('~/ros2_ws/src/rebearm/rebearm_teleop/rebearm_teleop/script/')
        moveHistory = open(rosPath + 'automove.csv', 'r')

        motorMsg = Float32MultiArray()
        setArmAgles(motorMsg, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, MOTOR4_HOME, MOTOR5_HOME, GRIPPER_OPEN)

        while(1):
            # Get next line from file
            line = moveHistory.readline()
            # if line is empty, end of file is reached
            if not line:
                break

            motor1, motor2, motor3, motor4, motor5, gripper, time_diff = line.split(',')
            sys.stdout.write(str(motor1) + ',' + str(motor2) + ',' + str(motor3) + ',' + str(motor4) + ',' + str(motor5) + ',' + str(gripper) + ',' + str(time_diff))
            sys.stdout.flush()

            setArmAgles(motorMsg, float(motor1), float(motor2), float(motor3), float(motor4), float(motor5), float(gripper))
            robotarm.run(motorMsg)

            try:
                sleep(float(time_diff))
            except KeyboardInterrupt:
                break

    except Exception as e:
        print(e)

    finally:  #
        #motor parking angle
        robotarm.park()

if __name__ == '__main__':
    main()
