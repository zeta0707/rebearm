from time import sleep, time
import math
import rclpy
from rclpy.node import Node
from . myconfig import *
from std_msgs.msg import Int32MultiArray

import serial
import lewansoul_lx16a

SERIAL_PORT = '/dev/buslinker2'

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
            t =  int(abs(pos - self.prev_pos)*2.7)
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

def moveJoint(id, motor, mMSG):
    #degree -> motor value, -120~120 => 0~1000
    if mMSG.data[id] != MOTOR_NOMOVE:
        #print("target:", mMSG.data[id])
        motor.move_to(int(mMSG.data[id]*500.0/120.0 + 500.0))
        #wait until 4 degree
        while True:
            ang = int((float(motor.get_pos()) - 500.0)*120.0/500.0) 
            #print(ang, ',', sep='', end='', flush=True)
            if abs(ang - mMSG.data[id]) < 4:
                #print("Done")
                return

def moveJointAll(m0, m1, m2, m3, m4, end, mMSG):
    #degree -> motor value, -120~120 => 0~1000
    if mMSG.data[0] != MOTOR_NOMOVE:
        m0.move_to(int(mMSG.data[0]*500.0/120.0 + 500.0))
    if mMSG.data[1] != MOTOR_NOMOVE:
        m1.move_to(int(mMSG.data[1]*500.0/120.0 + 500.0))
    if mMSG.data[2] != MOTOR_NOMOVE:
        m2.move_to(int(mMSG.data[2]*500.0/120.0 + 500.0))
    if mMSG.data[3] != MOTOR_NOMOVE:
        m3.move_to(int(mMSG.data[3]*500.0/120.0 + 500.0))
    if mMSG.data[4] != MOTOR_NOMOVE:
        m4.move_to(int(mMSG.data[4]*500.0/120.0 + 500.0))
    if mMSG.data[5] != MOTOR_NOMOVE:
        end.move_to(int(mMSG.data[5]*500.0/120.0 + 500.0))

class Rebearm(Node):
    """
    4DOF+Gripper with LX-16A motor
    """
    def __init__(self):
        super().__init__('arm_basic_node')
        self.motorMsg = Int32MultiArray()
        setArmAgles(self.motorMsg,MOTOR0_HOME, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, MOTOR4_HOME, GRIPPER_OPEN)
        self.armStatus = 'HOMING'
        
        #create 5 joint, actually motors
        self.m0 = Joint(1)
        self.m1 = Joint(2)
        self.m2 = Joint(3)
        self.m3 = Joint(4)
        self.m4 = Joint(5)
        self.end = Joint(6)

    def run(self, mMSG):
        self.motorMsg.data[0] = mMSG.data[0]
        self.motorMsg.data[1] = mMSG.data[1]
        self.motorMsg.data[2] = mMSG.data[2]
        self.motorMsg.data[3] = mMSG.data[3]
        self.motorMsg.data[4] = mMSG.data[4]
        self.motorMsg.data[5] = mMSG.data[5]
        moveJointAll(self.m0, self.m1, self.m2, self.m3, self.m4, self.end, self.motorMsg)

    def readAngle(self):
        #motor value -> degree, -120~120 => 0~1000 => -120~120
        ang0 = int((self.m0.get_pos() - 500.0)*120.0/500.0)
        ang1 = int((self.m1.get_pos() - 500.0)*120.0/500.0) 
        ang2 = int((self.m2.get_pos() - 500.0)*120.0/500.0)
        ang3 = int((self.m3.get_pos() - 500.0)*120.0/500.0)
        ang4 = int((self.m4.get_pos() - 500.0)*120.0/500.0)
        ang5 = int((self.end.get_pos() - 500.0)*120.0/500.0)
        return [ang0, ang1, ang2, ang3, ang4, ang5]
    
    def motors_off(self):
        self.m0.motors_off()

    def motors_on(self):
        self.m0.motors_on()

    def get_offsets(self):
        offset0 = self.m0.get_offset()
        offset1 = self.m1.get_offset()
        offset2 = self.m2.get_offset()
        offset3 = self.m3.get_offset()
        offset4 = self.m4.get_offset()
        offset5 = self.end.get_offset()
        return [offset0, offset1, offset2, offset3, offset4, offset5]

    def set_offset(self, id, deviation):
        if (id == 0):
            self.m0.set_offset(deviation)
        elif (id == 1):
            self.m1.set_offset(deviation)
        elif (id == 2):
            self.m2.set_offset(deviation)
        if (id == 3):
            self.m3.set_offset(deviation)
        if (id == 4):
            self.m4.set_offset(deviation)
        if (id == 5):
            self.end.set_offset(deviation)

    def park(self):
        print("Parking...")
        self.m0.motors_on()
        self.motorMsg.data[5] = GRIPPER_OPEN
        moveJoint(5, self.end, self.motorMsg)

        self.motorMsg.data[4] = MOTOR4_OFF
        moveJoint(4, self.m4, self.motorMsg)

        self.motorMsg.data[0] = MOTOR0_OFF
        moveJoint(0, self.m0, self.motorMsg)

        self.motorMsg.data[1] = (MOTOR1_OFF - 5)
        moveJoint(1, self.m1, self.motorMsg)
        
        self.motorMsg.data[2] = MOTOR2_OFF
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[3] = MOTOR3_OFF
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[1] = MOTOR1_OFF
        moveJoint(1, self.m1, self.motorMsg)

        #add all motor torque off
        self.m0.motors_off()
        print("Parking Done")

    def pickup(self):
        print("Pickup...")
        self.m0.motors_on()

        self.motorMsg.data[5] = GRIPPER_OPEN
        moveJoint(5, self.end, self.motorMsg)

        self.motorMsg.data[1] = (MOTOR1_HOME + 30)
        moveJoint(1, self.m1, self.motorMsg)

    def home(self):
        print("Homing...")
        self.m0.motors_on()

        self.motorMsg.data[5] = GRIPPER_OPEN
        moveJoint(5, self.end, self.motorMsg)

        self.motorMsg.data[3] = MOTOR3_HOME + 15
        moveJoint(3, self.m3, self.motorMsg)
        
        self.motorMsg.data[1] = MOTOR1_HOME + 15
        moveJoint(1, self.m1, self.motorMsg)

        self.motorMsg.data[2] = MOTOR2_HOME
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[3] = MOTOR3_HOME
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[4] = MOTOR4_HOME
        moveJoint(4, self.m4, self.motorMsg)

        self.motorMsg.data[1] = MOTOR1_HOME
        moveJoint(1, self.m1, self.motorMsg)

        self.motorMsg.data[0] = MOTOR0_HOME
        moveJoint(0, self.m0, self.motorMsg)
        print("Homing Done")

    def zero(self):
        print("Zeroing...")
        self.m0.motors_on()

        self.motorMsg.data[5] = GRIPPER_OPEN
        moveJoint(5, self.end, self.motorMsg)

        self.motorMsg.data[4] = MOTOR4_ZERO
        moveJoint(4, self.m4, self.motorMsg)

        self.motorMsg.data[1] = 25                      #move reverse at first
        moveJoint(1, self.m1, self.motorMsg)

        self.motorMsg.data[2] = int(MOTOR2_HOME /2)     #move 1/2
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[3] = int(MOTOR3_HOME/2)      #move 1/2
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[2] = MOTOR2_ZERO
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[3] = MOTOR3_ZERO
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[1] = MOTOR1_ZERO
        moveJoint(1, self.m1, self.motorMsg)
        print("Zoering Done")

    def deg90(self):
        print("Run 90degree")
        self.motors_on()

        self.motorMsg.data[2] = MOTOR_RIGHT
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[3] = MOTOR_RIGHT
        moveJoint(3, self.m3, self.motorMsg)
        
        self.motorMsg.data[1] = 0
        moveJoint(1, self.m1, self.motorMsg)

        self.motorMsg.data[0] = MOTOR0_HOME
        moveJoint(0, self.m0, self.motorMsg)
        print("90degree Done")

    def picknplace(self, object, down):
        self.motors_on()
        if down == 1:
            #move to pick up postion
            self.motorMsg.data[3] = MOTOR3_PICKUP
            moveJoint(3, self.m3, self.motorMsg)

            self.motorMsg.data[1] = MOTOR1_PICKUP
            moveJoint(1, self.m1, self.motorMsg)

            self.motorMsg.data[4] = MOTOR4_PICKUP
            moveJoint(4, self.m4, self.motorMsg)

        #grap action
        self.motorMsg.data[5] = GRIPPER_CLOSE
        moveJoint(5, self.end, self.motorMsg)

        #lift up
        self.motorMsg.data[1] = (MOTOR1_HOME + 10)
        moveJoint(1, self.m1, self.motorMsg)

        self.motorMsg.data[2] = (MOTOR2_HOME + 10)
        moveJoint(2, self.m2, self.motorMsg)

        #move to place position
        if object == 1:
            self.motorMsg.data[0] = MOTOR0_PLACE1
        else:
            self.motorMsg.data[0] = MOTOR0_PLACE2
        moveJoint(0, self.m0, self.motorMsg)

        #move down postion
        self.motorMsg.data[1] = MOTOR1_PICKUP
        moveJoint(1, self.m1, self.motorMsg)

        self.motorMsg.data[3] = MOTOR3_PICKUP
        moveJoint(3, self.m3, self.motorMsg)

        self.motorMsg.data[5] = GRIPPER_OPEN
        moveJoint(5, self.end, self.motorMsg)

        #place action, lift up
        self.motorMsg.data[2] =(MOTOR2_HOME + 10)
        moveJoint(2, self.m2, self.motorMsg)

        self.motorMsg.data[1] = (MOTOR1_HOME + 10)
        moveJoint(1, self.m1, self.motorMsg)

        #Home
        self.motorMsg.data[0] = MOTOR0_HOME
        moveJoint(0, self.m0, self.motorMsg)

def main(args=None):
    rclpy.init(args=args)
    myarm=  Rebearm()
    rclpy.spin(myarm)
    myarm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

