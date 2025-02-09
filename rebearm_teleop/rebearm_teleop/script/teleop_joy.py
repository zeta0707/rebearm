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
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data

from .submodules.myutil import Rebearm, clamp, setArmAgles
from .submodules.myconfig import *

msg = """
Control Your Robot!
---------------------------
Moving around:
Left lever left/right:    Waist(M1), left/light
Left lever up/down:       Shoulder(M2) move
Right lever up/down:      Elbow(M3) move
L1 + Right lever up/down: Forearm(M4) move
Right lever left/right:   Wrist(M5) move 
X   :                     gripper toggle
START :                   Move Home
CTRL-C to quit
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

        self.anglePub = self.create_publisher(Float32MultiArray, 'motor_angles', qos_profile_sensor_data)

        self.auto_mode = False
        self.chatCount= 0
        self.mode_button_last = 0

        self.control_motor1 = MOTOR1_HOME
        self.control_motor2 = MOTOR2_HOME
        self.control_motor3 = MOTOR3_HOME
        self.control_motor4 = MOTOR4_HOME
        self.control_motor5 = MOTOR5_HOME
        self.control_gripper = GRIPPER_OPEN
        self.keystroke = 0

        atexit.register(self.set_park)

        self.robotarm = Rebearm()
        angles = self.robotarm.readAngle()
        print("Angles:", ' '.join(f'{x:.2f}' for x in angles))
        offset = self.robotarm.get_offsets()
        print("Offset:", ' '.join(f'{x:.2f}' for x in offset))
        self.robotarm.home()

        self.motorMsg = Float32MultiArray()
        self.motorMsg.data = [MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, MOTOR4_HOME, MOTOR5_HOME, GRIPPER_OPEN]
        setArmAgles(self.motorMsg, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, MOTOR4_HOME, MOTOR5_HOME, GRIPPER_OPEN)

        # generate publisher for 'joy'
        self.sub = self.create_subscription(Joy, 'joy', self.cb_joy, qos_profile_sensor_data)
        # timer callback
        self.timer = self.create_timer(TIMER_JOY, self.cb_timer)

        rosPath = os.path.expanduser('~/ros2_ws/src/rebearm/rebearm_teleop/rebearm_teleop/script/')
        self.fhandle = open(rosPath + 'automove.csv', 'w')

        self.prev_time = time()
        self.timediff = 0.0

    def cb_joy(self, joymsg):
        PER = 0.05    #50ms

        # go home position
        if joymsg.buttons[9] == 1 and self.mode_button_last == 0:
            print('Home position')
            self.robotarm.home()
            self.control_motor1 = MOTOR1_HOME
            self.control_motor2 = MOTOR2_HOME
            self.control_motor3 = MOTOR3_HOME
            self.control_motor4 = MOTOR4_HOME
            self.control_motor5 = MOTOR5_HOME
            self.control_gripper = GRIPPER_OPEN
            self.keystroke = 0
            self.mode_button_last = joymsg.buttons[6]

        # gripper open/close
        elif joymsg.buttons[3] == 1 and self.mode_button_last == 0:
            if self.control_gripper == GRIPPER_OPEN:
                self.control_gripper = GRIPPER_CLOSE
            else:
                self.control_gripper = GRIPPER_OPEN
            self.mode_button_last = joymsg.buttons[3]

        timediff = time() - self.prev_time
        # joystick move detected
        if ((joymsg.axes[0] != 0) or (joymsg.axes[1] != 0) or (joymsg.axes[2] != 0) or (joymsg.axes[3] != 0)):
            #too short interval, wait more till PER
            if (timediff < PER):
                return 
            
            self.keystroke = self.keystroke + 1
            #update angle, but not move yet
            if (joymsg.axes[0] != 0):
                self.control_motor1 -= joymsg.axes[0] * self.step_deg 
                self.control_motor1 = clamp(self.control_motor1, MOTOR1_MIN, MOTOR1_MAX)
            elif joymsg.axes[1] != 0:
                self.control_motor2 += joymsg.axes[1] * self.step_deg 
                self.control_motor2 = clamp(self.control_motor2, MOTOR2_MIN, MOTOR2_MAX)
            elif joymsg.axes[3] != 0:
                if joymsg.buttons[6] == 1:
                    self.control_motor4 += joymsg.axes[3] * self.step_deg 
                    self.control_motor4 = clamp(self.control_motor4, MOTOR4_MIN, MOTOR4_MAX) 
                else:
                    self.control_motor3 += joymsg.axes[3] * self.step_deg 
                    self.control_motor3 = clamp(self.control_motor3, MOTOR3_MIN, MOTOR3_MAX)                                     
            elif joymsg.axes[2] != 0:
                self.control_motor5 += joymsg.axes[2] * self.step_deg 
                self.control_motor5 = clamp(self.control_motor5, MOTOR5_MIN, MOTOR5_MAX)
                
            #over PER, then wait PER*CONT_JOY
            if (self.keystroke < CONT_JOY):
                #print("short", self.keystroke, timediff)
                return
            # over PER*CONT_JOY
            else:
                self.keystroke = 0
                #print("long", self.keystroke, timediff)

        #jostick move stopped
        elif(self.keystroke > 0) and (timediff>CONT_JOY*PER):
            self.keystroke = 0
            #print("stopped", self.keystroke, timediff)

        #no joystick, no button
        elif self.chatCount < MAX_CHAT:
            return

        self.prev_time = time()
        setArmAgles(self.motorMsg, self.control_motor1, self.control_motor2, self.control_motor3, self.control_motor4, self.control_motor5, self.control_gripper)
        self.robotarm.run(self.motorMsg)
        self.anglePub.publish(self.motorMsg)
        print('M1= %.2f, M2=%.2f, M3= %.2f, M4=%.2f, M5=%.2f, G=%.2f'%(self.control_motor1, self.control_motor2, self.control_motor3, self.control_motor4, self.control_motor5, self.control_gripper))
        self.fhandle.write(f'{self.motorMsg.data[0]:.2f}' + ',' + f'{self.motorMsg.data[1]:.2f}'  + ',' + f'{self.motorMsg.data[2]:.2f}'  + ',' + f'{self.motorMsg.data[3]:.2f}'
                           + ',' + f'{self.motorMsg.data[4]:.2f}'  + ',' + f'{self.motorMsg.data[5]:.2f}'  + ',' + f'{timediff:.2f}' + '\n')
        self.fhandle.flush()
            
    def cb_timer(self):
        if (self.mode_button_last == 1):
            self.chatCount += 1                     # protect chattering for button
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
