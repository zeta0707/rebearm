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
import select
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped

from std_msgs.msg import Int32MultiArray
from .submodules.myutil import Rebearm, setArmAgles
from .submodules.myconfig import *

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

msg = """
Control Your Robot!
---------------------------
press space to make dataset
CTRL-C to quit
"""

e = """
Communications Failed
"""

class ServicenSubscriber(Node):
    def __init__(self):
        super().__init__('ClientAsyncInit')
        self.blob_subscriber = self.create_subscription(
            PointStamped,
            "/blob/point_blob",
            self.update_ball,
            qos_profile_sensor_data
        )

    def update_ball(self, message):
        #ignore 1 second previous message
        msg_secs = message.header.stamp.sec
        now = self.get_clock().now().to_msg().sec
        if (msg_secs + 1 < now):
            self.get_logger().info("Stamp %d, %d" %(now, msg_secs ) )
            return

        self.blob_x = message.point.x
        self.blob_y = message.point.y
        self.get_logger().info("Detected blob: %.2f  %.2f "%(self.blob_x, self.blob_y))
        return

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:  # if valid, save to key
        key = sys.stdin.read(1)
    else:       # else, initialize
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('dataset_node')        # generate node

    robotarm = Rebearm()
    offset = robotarm.get_offsets()
    print("Offsets:", offset)
    robotarm.home()
    print('rebearm IK dataset collector')

    # just key check
    status = 0
    svcSubscriber = ServicenSubscriber()

    #torqu on, normal status
    mStatus = 0
    blob_x = 0.0
    blob_y = 0.0
    data = [0, 0, 0, 0]
    
    rosPath = os.path.expanduser('~/ros2_ws/src/rebearm/rebearm_ml/rebearm_ml/')
    fhandle = open(rosPath + 'kinematics_pose.csv', 'w')
    fhandle.write('x,y+1,data[0],data[1],data[2],data[3]\n')

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == ' ':
                rclpy.spin_once(svcSubscriber)
                if mStatus == 0:
                    print('torque off')
                    robotarm.motors_off()
                    blob_x = svcSubscriber.blob_x
                    blob_y = svcSubscriber.blob_y
                    mStatus = 1
                else:
                    print('torque on')
                    data = robotarm.readAngle()
                    robotarm.motors_on()
                    mStatus = 0

                status = status + 1

            else:
                #Ctrl-C, then stop working
                if (key == '\x03'):
                    break
                #no valid input, then continue to read key
                else:
                    continue

            #key pressed, print motor angle
            if status == 1:
                if mStatus == 0:
                    print('x= %.2f, y=%.2f, M0= %d, M1=%d, M2=%d, M3=%d' %(blob_x, blob_y, data[0], data[1], data[2], data[3]))
                    fhandle.write(str(blob_x) + ',' + str(blob_y+1.0) + ',' + str(data[0]) + ',' + str(data[1]) + ',' + str(data[2])
                                + ',' + str(data[3]) + '\n')
                    fhandle.flush()
                    #move home to collect more data, pickup action first
                    robotarm.pickup()
                    robotarm.home()
                status = 0

    except Exception as e:
        print(e)

    finally:
        #motor parking angle
        print('Arm parking, be careful')
        robotarm.park()

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
