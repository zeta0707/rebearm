#!/usr/bin/env python3
# Author: ChangWhan Lee
from time import sleep, time
import math
import rclpy
from rclpy.node import Node
from . myconfig import *
from std_msgs.msg import Int32MultiArray

import serial
import lewansoul_lx16a

SERIAL_PORT = '/dev/buslinker2'
END_ID = 6
END_ID1 = (END_ID - 1)

controller = lewansoul_lx16a.ServoController(
    serial.Serial(SERIAL_PORT, 115200, timeout=1),
)

class Joint:
    def __init__(self, id):
        self.id = id
        self.prev_pos = -1
        try:
            self.start = self.end = controller.get_position(self.id)
        except lewansoul_lx16a.TimeoutError:
            print("TimeoutError, servo id: ", self.id)
            exit(0)

    def move_to(self, pos, t=0):
        if pos != self.prev_pos:
            #move time tuning, 2: too fast, 3: a little slow
            t =  int(abs(pos - self.prev_pos)*3.0)
            self.prev_pos = pos
            try:
                controller.move(self.id, pos, t)
            except:
                pass

    def get_pos(self):
        try:
            return controller.get_position(self.id)
        except:
            return None
        
    def set_offset(self, deviation):
        controller.set_position_offset(self.id, deviation)

    def save_offset(self):
        controller.save_position_offset(self.id)

    def get_offset(self):
        try:
            return controller.get_position_offset(self.id)
        except:
            return None  
           
    def motors_on(self):
        controller.motor_on(254)

    def motors_off(self):
        controller.motor_off(254)

def clamp(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n

# Sometimes servo angle goes just above 180 or just below 0 - trim to 0 or 180
def trimLimits(mtr_pos):
    if(mtr_pos > 180):
      mtr_pos = 180

    if(mtr_pos < -180):
      mtr_pos = -180

    return int(mtr_pos)

def setArmAgles(arm, ang0, ang1, ang2, ang3, ang4, grip):
    arm.data = [ang0, ang1, ang2, ang3, ang4, grip]

def moveJoint(id, joint, mMSG):
    #degree -> motor value, -120~120 => 0~1000
    CMD_TIMEOUT = 0.7
    count = 0
    tar_ang = mMSG.data[id-1]
    tar_ang_val = int(mMSG.data[id-1]*500.0/120.0 + 500.0)
    #print('id: %d, tar:%d'%(id, tar_ang))

    if tar_ang == MOTOR_NOMOVE:
        return
    #gripper, don't read angle
    if (id == END_ID):
        joint.move_to(tar_ang_val)
        sleep(1.0)
    else:
        start_time = time()
        joint.move_to(tar_ang_val)
        sleep(0.1)
        while True:
            if (time() - start_time) > CMD_TIMEOUT:
                #print(id, "!!!motor slow!!!")
                start_time = time()
                count = count + 1
                if (count == 3):
                    #print(id, "!!!motor stuck!!!")
                    break
                else:
                    joint.move_to(tar_ang_val + count)
                    continue

            cur_ang = int((float(joint.get_pos()) - 500.0)*120.0/500.0) 
            #print(cur_ang, ',', sep='', end='', flush=True)
            if abs(cur_ang - tar_ang) < 4:
                #print("Move done")
                return

def moveJointAll(m1, m2, m3, m4, m5, end, mMSG):
    #degree -> motor value, -120~120 => 0~1000
    if mMSG.data[0] != MOTOR_NOMOVE:
        m1.move_to(int(mMSG.data[0]*500.0/120.0 + 500.0))
    if mMSG.data[1] != MOTOR_NOMOVE:
        m2.move_to(int(mMSG.data[1]*500.0/120.0 + 500.0))
    if mMSG.data[2] != MOTOR_NOMOVE:
        m3.move_to(int(mMSG.data[2]*500.0/120.0 + 500.0))
    if mMSG.data[3] != MOTOR_NOMOVE:
        m4.move_to(int(mMSG.data[3]*500.0/120.0 + 500.0))
    if mMSG.data[4] != MOTOR_NOMOVE:
        m5.move_to(int(mMSG.data[4]*500.0/120.0 + 500.0))
    if mMSG.data[5] != MOTOR_NOMOVE:
        end.move_to(int(mMSG.data[5]*500.0/120.0 + 500.0))

class Rebearm(Node):
    """
    4DOF+Gripper with LX-16A motor
    """
    def __init__(self):
        super().__init__('arm_basic_node')
        self.motorMsg = Int32MultiArray()
        setArmAgles(self.motorMsg,MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, MOTOR4_HOME, MOTOR5_HOME, GRIPPER_OPEN)
        self.armStatus = 'HOMING'
        
        #create 5 joint, actually motors
        self.m1 = Joint(1)
        self.m2 = Joint(2)
        self.m3 = Joint(3)
        self.m4 = Joint(4)
        self.m5 = Joint(5)
        self.end = Joint(6)

    def run(self, mMSG):
        self.motorMsg.data[0] = mMSG.data[0]
        self.motorMsg.data[1] = mMSG.data[1]
        self.motorMsg.data[2] = mMSG.data[2]
        self.motorMsg.data[3] = mMSG.data[3]
        self.motorMsg.data[4] = mMSG.data[4]
        self.motorMsg.data[5] = mMSG.data[5]
        moveJointAll(self.m1, self.m2, self.m3, self.m4, self.m5, self.end, self.motorMsg)

    def readAngle(self):
        #motor value -> degree, -120~120 => 0~1000 => -120~120
        ang1 = int((self.m1.get_pos() - 500.0)*120.0/500.0)
        ang2 = int((self.m2.get_pos() - 500.0)*120.0/500.0)
        ang3 = int((self.m3.get_pos() - 500.0)*120.0/500.0)
        ang4 = int((self.m4.get_pos() - 500.0)*120.0/500.0)
        ang5 = int((self.m5.get_pos() - 500.0)*120.0/500.0)
        ang6 = int((self.end.get_pos() - 500.0)*120.0/500.0)
        return [ang1, ang2, ang3, ang4, ang5, ang6]
    
    def motors_off(self):
        self.m1.motors_off()

    def motors_on(self):
        self.m1.motors_on()

    def get_offsets(self):
        offset1 = self.m1.get_offset()
        offset2 = self.m2.get_offset()
        offset3 = self.m3.get_offset()
        offset4 = self.m4.get_offset()
        offset5 = self.m5.get_offset()
        offset6 = self.end.get_offset()
        return [offset1, offset2, offset3, offset4, offset5, offset6]

    def set_offset(self, id, deviation):
        if (id == 1):
            self.m1.set_offset(deviation)
        elif (id == 2):
            self.m2.set_offset(deviation)
        elif (id == 3):
            self.m3.set_offset(deviation)
        elif (id == 4):
            self.m4.set_offset(deviation)
        elif (id == 5):
            self.m5.set_offset(deviation)

    def save_offset(self, id):
        if (id == 1):
            self.m1.save_offset()
        elif (id == 2):
            self.m2.save_offset()
        elif (id == 3):
            self.m3.save_offset()
        elif (id == 4):
            self.m4.save_offset()
        elif (id == 5):
            self.m5.save_offset()

    def park(self):
        print("Parking...")
        self.m1.motors_on()
        print("Parking Done")
        return
    
        self.m1.motors_on()    
        self.motorMsg.data[END_ID1] = GRIPPER_OPEN
        moveJoint(END_ID, self.end, self.motorMsg)

        self.motorMsg.data[4] = MOTOR5_OFF
        moveJoint(5, self.m5, self.motorMsg)

        self.motorMsg.data[0] = MOTOR1_OFF
        moveJoint(1, self.m1, self.motorMsg)

        self.motorMsg.data[1] = (MOTOR2_OFF - 5)
        moveJoint(2, self.m2, self.motorMsg)
        
        self.motorMsg.data[2] = MOTOR3_OFF
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[3] = MOTOR4_OFF
        moveJoint(4, self.m4, self.motorMsg)

        self.motorMsg.data[1] = MOTOR2_OFF
        moveJoint(2, self.m2, self.motorMsg)

        #add all motor torque off
        self.m1.motors_off()
        print("Parking Done")

    def pickup(self):
        print("Pickup...")
        self.m1.motors_on()

        self.motorMsg.data[END_ID1] = GRIPPER_OPEN
        moveJoint(END_ID, self.end, self.motorMsg)

        self.motorMsg.data[1] = (MOTOR2_HOME + 30)
        moveJoint(2, self.m2, self.motorMsg)

    def home(self):
        print("Homing...")
        self.m1.motors_on()

        self.motorMsg.data[END_ID1] = GRIPPER_OPEN
        moveJoint(END_ID, self.end, self.motorMsg)

        self.motorMsg.data[3] = MOTOR4_HOME + 15
        moveJoint(4, self.m4, self.motorMsg)

        self.motorMsg.data[2] = MOTOR3_HOME - 10
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[1] = MOTOR2_HOME + 15
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[3] = MOTOR4_HOME
        moveJoint(4, self.m4, self.motorMsg)

        self.motorMsg.data[4] = MOTOR5_HOME
        moveJoint(5, self.m5, self.motorMsg)

        self.motorMsg.data[2] = MOTOR3_HOME
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[1] = MOTOR2_HOME
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[0] = MOTOR1_HOME
        moveJoint(1, self.m1, self.motorMsg)
        print("Homing Done")

    def zero(self):
        print("Zeroing...")
        self.m1.motors_on()

        self.motorMsg.data[END_ID1] = GRIPPER_OPEN
        moveJoint(END_ID, self.end, self.motorMsg)

        self.motorMsg.data[4] = MOTOR5_ZERO
        moveJoint(5, self.m5, self.motorMsg)

        self.motorMsg.data[1] = 25                      #move reverse at first
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[2] = int(MOTOR3_HOME /2)     #move 1/2
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[3] = int(MOTOR4_HOME/2)      #move 1/2
        moveJoint(4, self.m4, self.motorMsg)

        self.motorMsg.data[2] = MOTOR3_ZERO
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[3] = MOTOR4_ZERO
        moveJoint(4, self.m4, self.motorMsg)

        self.motorMsg.data[1] = MOTOR2_ZERO
        moveJoint(2, self.m2, self.motorMsg)
        print("Zoering Done")

    def deg90(self):
        print("Run 90degree")
        self.motors_on()

        self.motorMsg.data[2] = MOTOR_RIGHT
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[3] = MOTOR_RIGHT
        moveJoint(4, self.m4, self.motorMsg)
        
        self.motorMsg.data[1] = 0
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[0] = MOTOR1_HOME
        moveJoint(1, self.m1, self.motorMsg)
        print("90degree Done")

    def picknplace(self, object, down):
        self.motors_on()
        if down == 1:
            #move to pick up postion
            self.motorMsg.data[3] = MOTOR4_PICKUP
            moveJoint(4, self.m4, self.motorMsg)

            self.motorMsg.data[1] = MOTOR2_PICKUP
            moveJoint(2, self.m2, self.motorMsg)

            self.motorMsg.data[4] = MOTOR5_PICKUP
            moveJoint(5, self.m5, self.motorMsg)

        #grap action
        self.motorMsg.data[END_ID1] = GRIPPER_CLOSE
        moveJoint(END_ID, self.end, self.motorMsg)

        #lift up
        self.motorMsg.data[1] = (MOTOR2_HOME + 10)
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[2] = (MOTOR3_HOME + 10)
        moveJoint(3, self.m3, self.motorMsg)

        #move to place position
        if object == 1:
            self.motorMsg.data[0] = MOTOR1_PLACE1
        elif object == 2:
            self.motorMsg.data[0] = MOTOR1_PLACE2
        elif object == 3:
            self.motorMsg.data[0] = MOTOR1_PLACE3
        else:
            self.motorMsg.data[0] = MOTOR1_PLACE4
        
        moveJoint(1, self.m1, self.motorMsg)

        #move down postion
        self.motorMsg.data[1] = MOTOR2_PICKUP
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[3] = MOTOR4_PICKUP
        moveJoint(4, self.m4, self.motorMsg)

        self.motorMsg.data[END_ID1] = GRIPPER_OPEN
        moveJoint(END_ID, self.end, self.motorMsg)

        #place action, lift up
        self.motorMsg.data[2] =(MOTOR3_HOME + 10)
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[1] = (MOTOR2_HOME + 10)
        moveJoint(2, self.m2, self.motorMsg)

        #Home
        self.motorMsg.data[0] = MOTOR1_HOME
        moveJoint(1, self.m1, self.motorMsg)

def main(args=None):
    rclpy.init(args=args)
    myarm=  Rebearm()
    rclpy.spin(myarm)
    myarm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

