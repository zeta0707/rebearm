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
import os, select, sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from rclpy.qos import qos_profile_sensor_data

from .submodules.myutil import Rebearm, clamp, setArmAgles
from .submodules.myconfig import *

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

msg = """
Control Your Robot!
---------------------------
Moving around:
a/d : base(M0), left/light
w/x : shoulder(M1) move
j/l : Elbow(M2) move
i/, : Wrist(M3) move
g/G : Gripper close/open
h   : Move home
9   : 90 position, motor assemble check
z   : zero position, motor assemble check

CTRL-C to quit
"""

e = """
Communications Failed
"""

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

def check_angle_range(velocity):
    return constrain(velocity, -MAX_ANG, MAX_ANG)

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    print('Param max lin: %s deg, lin step: %s deg'%
        (MAX_ANG,
        ANG_STEP)
    )

    node = rclpy.create_node('teleop_keyboard_node')        # generate node

    anglePub = node.create_publisher(Int32MultiArray, 'motor_angles', qos_profile_sensor_data)

    print('Rebearm Teleop Keyboard controller')
    robotarm = Rebearm()
    robotarm.home()
    offset = robotarm.get_offsets()
    print("Offsets:", offset)

    status = 0
    control_motor0 = MOTOR0_HOME
    control_motor1 = MOTOR1_HOME
    control_motor2 = MOTOR2_HOME
    control_motor3 = MOTOR3_HOME
    control_motor4 = MOTOR4_HOME
    control_gripper = GRIPPER_OPEN
    OFF_STEP = 3

    motorMsg = Int32MultiArray()
    motorMsg.data = [MOTOR0_HOME, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, MOTOR4_HOME, GRIPPER_OPEN]
    setArmAgles(motorMsg, MOTOR0_HOME, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, MOTOR4_HOME, GRIPPER_OPEN)

    rosPath = os.path.expanduser('~/ros2_ws/src/rebearm/rebearm_teleop/rebearm_teleop/script/')
    fhandle = open(rosPath + 'automove.csv', 'w')

    prev_time = time()
    prev_time_move = time()
    #current key - previous key
    timediff = 0.0
    #current move - previous move
    timediff_move = 0.0
    keystroke = 0

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == 'x':              # motor1
                control_motor1 = check_angle_range(control_motor1 - ANG_STEP)
                status = status + 1
            elif key == 'w':            # motor1
                control_motor1 = check_angle_range(control_motor1 + ANG_STEP)
                status = status + 1
            elif key == 'j':            # motor2
                control_motor2 = check_angle_range(control_motor2 - ANG_STEP)
                status = status + 1
            elif key == 'l':            # motor2
                control_motor2 = check_angle_range(control_motor2 + ANG_STEP)
                status = status + 1
            elif key == ',':            # motor3
                control_motor3 = check_angle_range(control_motor3 - ANG_STEP)
                status = status + 1
            elif key == 'i':            # motor3
                control_motor3 = check_angle_range(control_motor3 + ANG_STEP)
                status = status + 1
            elif key == 'a':            # motor0
                control_motor0 = check_angle_range(control_motor0 - ANG_STEP)
                status = status + 1
            elif key == 'd':            # motor0
                control_motor0 = check_angle_range(control_motor0 + ANG_STEP)
                status = status + 1
            elif key == 'g':            # gripper close
                control_gripper = check_angle_range(control_gripper - ANG_STEP)
                status = status + 1
            elif key == 'G':            # gripper open 
                control_gripper = check_angle_range(control_gripper + ANG_STEP)
                status = status + 1

            # calibrate M0 offset
            elif key == '0' or key == ')':           
                if key == '0':
                    offset[0] += OFF_STEP
                else:
                    offset[0] -= OFF_STEP
                robotarm.set_offset(0, offset[0])
                robotarm.save_offset(0)
                print("offset0:",offset[0])
                continue
            # calibrate M1 offset
            elif key == '1' or key == '!':  
                if key == '1':       
                    offset[1]  += OFF_STEP
                else:
                    offset[1] -= OFF_STEP
                robotarm.set_offset(1, offset[1])
                robotarm.save_offset(1)
                print("offset1:",offset[1])
                continue
            # calibrate M2 offset
            elif key == '2' or key == '@':  
                if key == '2':     
                    offset[2] += OFF_STEP
                else:
                    offset[2]-= OFF_STEP
                robotarm.set_offset(2, offset[2])
                robotarm.save_offset(2)
                print("offset2:",offset[2])
                continue
            # calibrate M3 offset
            elif key == '3' or key == '#': 
                if key == '3':       
                    offset[3] += OFF_STEP
                else:
                    offset[3] -= OFF_STEP
                robotarm.set_offset(3, offset[3])
                robotarm.save_offset(3)
                print("offset3:",offset[3])
                continue
            # calibrate M4 offset  
            elif key == '4' or key == '$':         
                if key == '4': 
                    offset[4] += OFF_STEP
                else:
                    offset[4] -= OFF_STEP
                robotarm.set_offset(4, offset[4])
                robotarm.save_offset(4)
                print("offset4:",offset[4])
                continue

            elif key == '9':
                print('90degree position')
                robotarm.deg90()
                control_motor0 = MOTOR0_HOME
                control_motor1 = 0
                control_motor2 = MOTOR_RIGHT
                control_motor3 = MOTOR_RIGHT
                control_motor4 = MOTOR4_HOME
                control_gripper = GRIPPER_OPEN
                keystroke = 0

            elif key == 'z':
                print('zero position')
                robotarm.zero()
                control_motor0 = MOTOR0_HOME
                control_motor1 = 0
                control_motor2 = 0
                control_motor3 = 0
                control_motor4 = MOTOR4_ZERO
                control_gripper = GRIPPER_OPEN
                keystroke = 0

            elif key == 'h':
                print('Home position')
                robotarm.home()
                control_motor0 = MOTOR0_HOME
                control_motor1 = MOTOR1_HOME
                control_motor2 = MOTOR2_HOME
                control_motor3 = MOTOR3_HOME
                control_motor4 = MOTOR4_HOME
                control_gripper = GRIPPER_OPEN
                keystroke = 0

            else:
                timediff = time() - prev_time
                #Ctrl-C, then stop working
                if (key == '\x03'):
                    break
                #continous key stop
                elif ((keystroke > 0) and (timediff > 0.1)):
                    keystroke = 0
                #no valid input, then don't control arm
                else:
                    continue

            #key pressed
            if status == 1:
                control_motor0 = int(clamp(control_motor0, MOTOR0_MIN, MOTOR0_MAX))
                control_motor1 = int(clamp(control_motor1, MOTOR1_MIN, MOTOR1_MAX))
                control_motor2 = int(clamp(control_motor2, MOTOR2_MIN, MOTOR2_MAX))
                control_motor3 = int(clamp(control_motor3, MOTOR3_MIN, MOTOR3_MAX))
                control_motor4 = int(clamp(control_motor4, MOTOR4_MIN, MOTOR4_MAX))
                control_gripper = int(clamp(control_gripper, GRIPPER_MIN, GRIPPER_MAX))
                timediff = time() - prev_time
                prev_time = time()
                status = 0
                #print(timediff, keystroke, control_motor0, control_motor1, control_motor3)

                #continous key press, usually less than 100ms
                if (timediff < 0.10):
                    keystroke = keystroke + 1
                    #accumulate till CONT_KEY
                    if(keystroke < CONT_KEY):
                        continue

            timediff_move = time() - prev_time_move
            prev_time_move = time()

            keystroke = 0
            setArmAgles(motorMsg, control_motor0, control_motor1, control_motor2, control_motor3, control_motor4, control_gripper)
            anglePub.publish(motorMsg)
            robotarm.run(motorMsg)
            print('M0=%d, M1=%d, M2=%d, M3=%d, M4=%d, G=%d'%(control_motor0, control_motor1, control_motor2, control_motor3, control_motor4, control_gripper))
            fhandle.write(str(motorMsg.data[0]) + ',' + str(motorMsg.data[1]) + ',' + str(motorMsg.data[2]) + ',' + str(motorMsg.data[3])
                        + ',' + str(motorMsg.data[4]) + ',' + str(motorMsg.data[5]) + ',' + str(timediff_move) + '\n')
            fhandle.flush()
                

    except Exception as e:
        print(e)

    finally:  #
        #motor parking angle
        print('Arm parking, be careful')
        robotarm.park()

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
