#!/usr/bin/env python3
# Author: ChangWhan Lee
# print min, max, home angles for joint_state_publisher
import math
from myconfig import *

print("M1(MIN,MAX,HOME):", math.radians(MOTOR1_MIN), math.radians(MOTOR1_MAX), math.radians(MOTOR1_HOME),)
print("M2(MIN,MAX,HOME):", math.radians(MOTOR2_MIN), math.radians(MOTOR2_MAX), math.radians(MOTOR2_HOME),)
print("M3(MIN,MAX,HOME):", math.radians(MOTOR3_MIN), math.radians(MOTOR3_MAX), math.radians(MOTOR3_HOME),)
print("M4(MIN,MAX,HOME):", math.radians(MOTOR4_MIN), math.radians(MOTOR4_MAX), math.radians(MOTOR4_HOME),)
print("M5(MIN,MAX,HOME):", math.radians(MOTOR5_MIN), math.radians(MOTOR5_MAX), math.radians(MOTOR5_HOME),)
print("M6(MIN,MAX,HOME):", math.radians(GRIPPER_MIN), math.radians(GRIPPER_MAX), math.radians(GRIPPER_OPEN),)
