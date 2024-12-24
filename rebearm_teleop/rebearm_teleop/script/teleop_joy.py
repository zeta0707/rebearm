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
from sensor_msgs.msg import Joy
import atexit
from std_msgs.msg import Int32MultiArray
from rclpy.qos import qos_profile_sensor_data

from .submodules.myutil import Rebearm, clamp, setArmAgles
from .submodules.myconfig import *

msg = """
Control Your Robot!
---------------------------
Moving around:
Left Stick left/right:  Base(M0), left/light
Left Stick up/down:     shoulder(M1) move
Right Stick up/down:    Elbow(M2) move
Right Stick left/right: Wrist(M3) move

'X' : gripper open/close
L-2 : Move Home
R-2 : Motor Initiailze
"""

class TeleopJoyNode(Node):

    def __init__(self):
        super().__init__('teleop_joy_node')
        self.declare_parameters(    # bring the param from yaml file
            namespace='',
            parameters=[
                ('max_deg', 120),
                ('step_deg', 20),
            ])
        
        print('Rebearm Teleop Joystick controller')
        print(msg)
        self.max_deg = self.get_parameter_or('max_deg', Parameter('max_deg', Parameter.Type.INTEGER, 120)).get_parameter_value().integer_value
        self.step_deg = self.get_parameter_or('step_deg', Parameter('step_deg', Parameter.Type.INTEGER, 20)).get_parameter_value().integer_value
        print('max ang: %s rad/s, step: %s'%
            (self.max_deg,
            self.step_deg)
        )
        print('CTRL-C to quit')

        self.anglePub = self.create_publisher(Int32MultiArray, 'motor_angles', qos_profile_sensor_data)

        self.auto_mode = False
        self.chatCount= 0
        self.mode_button_last = 0

        self.control_motor0 = MOTOR0_HOME
        self.control_motor1 = MOTOR1_HOME
        self.control_motor2 = MOTOR2_HOME
        self.control_motor3 = MOTOR3_HOME
        self.control_gripper = GRIPPER_OPEN
        self.keystroke = 0

        atexit.register(self.set_park)

        self.robotarm = Rebearm()
        self.robotarm.home()

        self.motorMsg = Int32MultiArray()
        self.motorMsg.data = [MOTOR0_HOME, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, GRIPPER_OPEN]
        setArmAgles(self.motorMsg, MOTOR0_HOME, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, GRIPPER_OPEN)

        # generate publisher for 'joy'
        self.sub = self.create_subscription(Joy, 'joy', self.cb_joy, qos_profile_sensor_data)
        # timer callback
        self.timer = self.create_timer(TIMER_JOY, self.cb_timer)

        rosPath = os.path.expanduser('~/ros2_ws/src/rebearm/rebearm_teleop/rebearm_teleop/script/')
        self.fhandle = open(rosPath + 'automove.csv', 'w')

        self.prev_time = time()
        self.prev_time_move = time()
        self.timediff = 0.0

    def cb_joy(self, joymsg):
        status = 0

        timediff = time() - self.prev_time

        # initialize motors when motor error happens
        if joymsg.buttons[7] == 1 and self.mode_button_last == 0:
            print('Initialize motors')
            #self.int_client.send_request(0)
            self.mode_button_last = joymsg.buttons[7]

        # initialize motors when motor error happens
        elif joymsg.buttons[6] == 1 and self.mode_button_last == 0:
            print('Home position')
            self.robotarm.home()
            self.control_motor0 = MOTOR0_HOME
            self.control_motor1 = MOTOR1_HOME
            self.control_motor2 = MOTOR2_HOME
            self.control_motor3 = MOTOR3_HOME
            self.control_gripper = GRIPPER_OPEN
            self.keystroke = 0
            self.mode_button_last = joymsg.buttons[6]

        # gripper open/close
        elif joymsg.buttons[3] == 1 and self.mode_button_last == 0:
            status = status + 1
            if self.control_gripper == GRIPPER_OPEN:
                self.control_gripper = GRIPPER_CLOSE
            else:
                self.control_gripper = GRIPPER_OPEN
            self.mode_button_last = joymsg.buttons[3]

        # Make jostick -> motor agngle
        elif joymsg.axes[0] != 0:
            status = status + 1
            self.control_motor0 += joymsg.axes[0] * self.max_deg / self.step_deg
        elif joymsg.axes[1] != 0:
            status = status + 1
            self.control_motor1 += joymsg.axes[1] * self.max_deg / self.step_deg
        elif joymsg.axes[3] != 0:
            status = status + 1
            self.control_motor2 += joymsg.axes[3] * self.max_deg / self.step_deg
        elif joymsg.axes[2] != 0:
            status = status + 1
            self.control_motor3 += joymsg.axes[2] * self.max_deg / self.step_deg

        #continous key stop
        elif ((self.keystroke > 0) and (timediff > 0.1)):
            self.keystroke = 0
        else:
            #nothing to do, then return
            return True

        #key pressed, torque
        if status == 1:
            self.control_motor0 = int(clamp(self.control_motor0, MOTOR0_MIN, MOTOR0_MAX))
            self.control_motor1 = int(clamp(self.control_motor1, MOTOR1_MIN, MOTOR1_MAX))
            self.control_motor2 = int(clamp(self.control_motor2, MOTOR2_MIN, MOTOR2_MAX))
            self.control_motor3 = int(clamp(self.control_motor3, MOTOR3_MIN, MOTOR3_MAX))
            self.control_gripper = int(clamp(self.control_gripper, GRIPPER_MIN, GRIPPER_MAX))

            timediff = time() - self.prev_time
            self.prev_time = time()
            status = 0

            #continous key press, usually less than 100ms
            if (timediff < 0.10):
                self.keystroke = self.keystroke + 1
                #ignore 3 continous key
                if(self.keystroke < CONTKEY):
                    return
                
        timediff_move = time() - self.prev_time_move
        self.prev_time_move = time()

        self.keystroke = 0
        setArmAgles(self.motorMsg, self.control_motor0, self.control_motor1, self.control_motor2, self.control_motor3, self.control_gripper)
        #y, z = calculate_position_5dof(self.control_motor1, self.control_motor2,self.control_motor3)
        #print('y=%.1f,z=%.1f(cm)' %(y*100, z*100))
        self.robotarm.run(self.motorMsg)
        self.anglePub.publish(self.motorMsg)
        print('M0= %d, M1 %d, M2= %d, M3= %d, G=%d'%(self.control_motor0, self.control_motor1, self.control_motor2, self.control_motor3, self.control_gripper))
        self.fhandle.write(str(self.motorMsg.data[0]) + ',' + str(self.motorMsg.data[1]) + ',' + str(self.motorMsg.data[2]) + ',' + str(self.motorMsg.data[3])
                            + ',' + str(self.motorMsg.data[4])+ ',' + str(timediff_move) + '\n')
        self.fhandle.flush()
            
    def cb_timer(self):
        if (self.mode_button_last == 1):
            self.chatCount += 1                     # protect chattering
        if (self.chatCount > MAX_CHAT):
            self.mode_button_last = 0
            self.chatCount = 0

    def set_park(self):
        self.get_logger().info('Arm parking, be careful')
        self.robotarm.park()

def main(args=None):
    rclpy.init(args=args)
    teleop_joy =  TeleopJoyNode()
    rclpy.spin(teleop_joy)
    teleop_joy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
