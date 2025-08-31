#!/usr/bin/env python3
# Author: ChangWhan Lee
from time import sleep, time
import math
import rclpy
from rclpy.node import Node
from . myconfig import *
from std_msgs.msg import Float32MultiArray

from . import lx16a

GRIP_ID = 6
GRIP_IDM1 = (GRIP_ID - 1)

Tfactor = 25        #value to LX16A time
T1Factor = 0.3      #300ms more wait

class Joint:
    def __init__(self, id):
        self.id = id
        #class with torque enable
        self.servo = lx16a.LX16A(id_=id, disable_torque=0)
        self.servo.set_angle_limits(0, 240)
        self.prev_pos = -1

    def move_to(self, pos, t_move=0):
        #return without doing anything
        if pos == MOTOR_NOMOVE:
            pass
        
        # only move asking 0.4 degree
        if abs(pos - self.prev_pos) > 0.3:
            #no move time parameter, then calculate
            if t_move == 0:
                t_move =  int(abs(pos - self.prev_pos)*Tfactor)
            self.prev_pos = pos
            try:
                #-120~120 => 0~240
                self.servo.move(angle=pos+120.0, time=t_move)
            except:
                pass

    def get_pos(self):
        return self.prev_pos
            
    def get_physical_pos(self):
        try:
            angle  = self.servo.get_physical_angle() 
            self.prev_pos = angle - 120.0
            # 0~240 =>-120~120 
            return self.prev_pos
        except:
            print(self.id)
            return None
     
    def set_offset(self, deviation):
        #with permanent
        self.servo.set_angle_offset(deviation,  permanent=True)

    def get_offset(self):
        try:
            #read with poll_hardware
            return self.servo.get_angle_offset(True)
        except:
            return None  
           
    def motors_on(self):
        self.servo.enable_torque_all()

    def motors_off(self):
        self.servo.disable_torque_all()

def lists_not_close(list1, list2, rel_tol=1e-5, abs_tol=0.5):
    """
    Check if any corresponding elements in two lists are NOT close
    Returns True if ANY pair of elements are not close
    """
    if len(list1) != len(list2):
        return True  # Different lengths = not close
    
    for a, b in zip(list1, list2):
        if not math.isclose(a, b, rel_tol=rel_tol, abs_tol=abs_tol):
            return True  # Found elements that are not close
    
    return False  # All elements are close

def clamp(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n

# Sometimes servo angle goes just above 180 or just below 0 - trim to 0 or 180
def trimLimits(mtr_pos):
    if(mtr_pos > 180.0):
      mtr_pos = 180.0

    if(mtr_pos < -180.0):
      mtr_pos = -180.0

    return mtr_pos

def setArmAgles(arm, ang0, ang1, ang2, ang3, ang4, grip):
    arm.data = [ang0, ang1, ang2, ang3, ang4, grip]

def moveJoint(id, joint, mMSG):
    tar_ang = mMSG.data[id-1]
    cur_ang = joint.get_pos()

    #gripper, don't read angle
    if (id == GRIP_ID):
        joint.move_to(tar_ang, 800)
        sleep(0.8)
    else:
        joint.move_to(tar_ang, 0)
        t = abs(tar_ang - cur_ang)*Tfactor*0.001 + T1Factor
        sleep(t)
        #print('id:%d, org:%.2f, tar:%.2f, cur:%.2f, time:%.2f' %(id, cur_ang, tar_ang, joint.get_physical_pos(), t))

def moveJointAll(m1, m2, m3, m4, m5, end, mMSG):
    m1.move_to(mMSG.data[0], 300)
    m2.move_to(mMSG.data[1], 300)
    m3.move_to(mMSG.data[2], 300)
    m4.move_to(mMSG.data[3], 300)
    m5.move_to(mMSG.data[4], 300)
    end.move_to(mMSG.data[5], 300)

class Rebearm(Node):
    """
    4DOF+Gripper with LX-16A motor
    """
    def __init__(self, port=BUSLINKER2):
        super().__init__('arm_basic_node')
        lx16a.LX16A.initialize(port, 0.1)
        self.motorMsg = Float32MultiArray()
        setArmAgles(self.motorMsg,MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, MOTOR4_HOME, MOTOR5_HOME, GRIPPER_OPEN)
        self.armStatus = 'HOMING'
        
        #create 6 joint, actually motors
        self.m1 = Joint(1)
        self.m2 = Joint(2)
        self.m3 = Joint(3)
        self.m4 = Joint(4)
        self.m5 = Joint(5)
        self.end = Joint(6)

    def run(self, mMSG):
        moveJointAll(self.m1, self.m2, self.m3, self.m4, self.m5, self.end, mMSG)

    def readAngle(self):
        ang1 = self.m1.get_physical_pos()    
        sleep(0.02)
        ang2 = self.m2.get_physical_pos()
        sleep(0.02)
        ang3 = self.m3.get_physical_pos()
        sleep(0.02)
        ang4 = self.m4.get_physical_pos()
        sleep(0.02)
        ang5 = self.m5.get_physical_pos()
        sleep(0.02)
        ang6 = self.end.get_physical_pos()
        sleep(0.02)
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

    def park(self):
        print("Parking...")
        self.m1.motors_on()
        print("Parking Done")
        return

    def pickup(self):
        print("Pickup...")
        self.m1.motors_on()

        self.motorMsg.data[GRIP_IDM1] = GRIPPER_OPEN
        moveJoint(GRIP_ID, self.end, self.motorMsg)

        self.motorMsg.data[1] = (MOTOR2_HOME + 30.0)
        moveJoint(2, self.m2, self.motorMsg)

    def home(self):
        print("Homing...")
        self.motors_on()

        self.motorMsg.data[GRIP_IDM1] = GRIPPER_OPEN
        moveJoint(GRIP_ID, self.end, self.motorMsg)

        self.motorMsg.data[3] = MOTOR4_HOME + 15.0
        moveJoint(4, self.m4, self.motorMsg)

        self.motorMsg.data[2] = MOTOR3_HOME - 10.0
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[1] = MOTOR2_HOME + 15.0
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

        #somehow doesn't move complete, then compensate here
        moveJointAll(self.m1, self.m2, self.m3, self.m4, self.m5, self.end, self.motorMsg)

        print("Homing Done")

    def zero(self):
        print("Zeroing...")
        self.motors_on()

        self.motorMsg.data[GRIP_IDM1] = GRIPPER_OPEN
        moveJoint(GRIP_ID, self.end, self.motorMsg)

        self.motorMsg.data[1] = MOTOR2_HOME+5.0    #move reverse at first
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[2] = MOTOR3_HOME/2.0     #move 1/2
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[3] = MOTOR4_HOME/2.0     #move 1/2
        moveJoint(4, self.m4, self.motorMsg)

        self.motorMsg.data[2] = MOTOR3_ZERO
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[3] = MOTOR4_ZERO
        moveJoint(4, self.m4, self.motorMsg)

        self.motorMsg.data[1] = MOTOR2_ZERO
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[4] = MOTOR5_ZERO
        moveJoint(5, self.m5, self.motorMsg)

        #somehow doesn't move complete, then compensate here
        moveJointAll(self.m1, self.m2, self.m3, self.m4, self.m5, self.end, self.motorMsg)
        print("Zoering Done")

    def deg90(self):
        print("Run 90degree")
        self.motors_on()

        self.motorMsg.data[3] = MOTOR_DEG90
        moveJoint(4, self.m4, self.motorMsg)
        
        self.motorMsg.data[2] = MOTOR_DEG90
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[1] = 0.0
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[0] = MOTOR1_HOME
        moveJoint(1, self.m1, self.motorMsg)

        self.motorMsg.data[GRIP_IDM1] = GRIPPER_OPEN
        self.motorMsg.data[4] = MOTOR5_HOME

        #somehow doesn't move complete, then compensate here
        moveJointAll(self.m1, self.m2, self.m3, self.m4, self.m5, self.end, self.motorMsg)
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
        self.motorMsg.data[GRIP_IDM1] = GRIPPER_CLOSE
        moveJoint(GRIP_ID, self.end, self.motorMsg)

        #lift up
        self.motorMsg.data[1] = (MOTOR2_HOME + 10.0)
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[2] = (MOTOR3_HOME + 10.0)
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

        self.motorMsg.data[GRIP_IDM1] = GRIPPER_OPEN
        moveJoint(GRIP_ID, self.end, self.motorMsg)

        #place action, lift up
        self.motorMsg.data[2] =(MOTOR3_HOME + 10.0)
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[1] = (MOTOR2_HOME + 10.0)
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

