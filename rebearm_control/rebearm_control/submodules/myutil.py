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

def setArmAgles(arm, ang0, ang1, ang2, ang3, grip):
    arm.data = [ang0, ang1, ang2, ang3, grip]

def moveJoint(m0, m1, m2, m3, m4, mMSG):
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

class Rebearm(Node):
    """
    4DOF+Gripper with LX-16A motor
    """
    def __init__(self):
        super().__init__('arm_basic_node')
        self.motorMsg = Int32MultiArray()
        setArmAgles(self.motorMsg,MOTOR0_HOME, MOTOR1_HOME, MOTOR2_HOME, MOTOR3_HOME, GRIPPER_OPEN)
        self.armStatus = 'HOMING'
        
        #create 5 joint, actually motors
        self.m0 = Joint(1)
        self.m1 = Joint(2)
        self.m2 = Joint(3)
        self.m3 = Joint(4)
        self.m4 = Joint(5)

    def run(self, mMSG):
        self.motorMsg.data[0] = mMSG.data[0]
        self.motorMsg.data[1] = mMSG.data[1]
        self.motorMsg.data[2] = mMSG.data[2]
        self.motorMsg.data[3] = mMSG.data[3]
        self.motorMsg.data[4] = mMSG.data[4]
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)

    def readAngle(self):
        #motor value -> degree, -120~120 => 0~1000 => -120~120
        ang0 = int((self.m0.get_pos() - 500.0)*120.0/500.0)
        ang1 = int((self.m1.get_pos() - 500.0)*120.0/500.0)
        ang2 = int((self.m2.get_pos() - 500.0)*120.0/500.0)
        ang3 = int((self.m3.get_pos() - 500.0)*120.0/500.0)
        ang4 = int((self.m4.get_pos() - 500.0)*120.0/500.0)
        return [ang0, ang1, ang2, ang3, ang4]
    
    def motors_off(self):
        self.m0.motors_off()

    def motors_on(self):
        self.m0.motors_on()

    def park(self):
        print("Parking...")
        self.m0.motors_on()
        self.motorMsg.data[4] = GRIPPER_OPEN
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.3)

        self.motorMsg.data[0] = MOTOR0_OFF
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(1.5)

        self.motorMsg.data[1] = (MOTOR1_OFF + 20)
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.8)

        self.motorMsg.data[2] = MOTOR2_OFF
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.8)

        self.motorMsg.data[3] = MOTOR3_OFF
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.4)

        self.motorMsg.data[1] = MOTOR1_OFF
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.2)

        #add all motor torque off
        self.m0.motors_off()
        print("Parking Done")

    def pickup(self):
        print("Pickup...")
        self.m0.motors_on()

        self.motorMsg.data[4] = GRIPPER_OPEN
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.3)

        self.motorMsg.data[1] = (MOTOR1_HOME + 30)
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(1.0)

    def home(self):
        print("Homing...")
        self.m0.motors_on()

        self.motorMsg.data[4] = GRIPPER_OPEN
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.3)

        self.motorMsg.data[3] = MOTOR3_HOME + 5
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(1.0)
        
        self.motorMsg.data[1] = MOTOR1_HOME
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(1.0)

        self.motorMsg.data[2] = MOTOR2_HOME
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.5)

        self.motorMsg.data[3] = MOTOR3_HOME
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(1.0)

        self.motorMsg.data[0] = MOTOR0_HOME
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(1.5)

        print("Homing Done")

    def zero(self):
        print("Zeroing...")
        self.m0.motors_on()

        self.motorMsg.data[4] = GRIPPER_OPEN
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.3)

        self.motorMsg.data[1] = MOTOR1_HOME + 10
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.8)

        self.motorMsg.data[2] = MOTOR2_HOME + 20
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.8)

        self.motorMsg.data[3] = MOTOR3_HOME + 20
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.8)

        self.motorMsg.data[2] = MOTOR2_ZERO
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.8)

        self.motorMsg.data[1] = MOTOR1_ZERO
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.8)

        self.motorMsg.data[3] = MOTOR3_ZERO
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.8)

        print("Zoering Done")

    def picknplace(self, object, down):
        self.motors_on()
        if down == 1:
            #move to pick up postion
            self.motorMsg.data[3] = MOTOR3_PICKUP
            moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
            sleep(1.0)

            self.motorMsg.data[1] = MOTOR1_PICKUP
            moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
            sleep(0.5)

        #grap action
        self.motorMsg.data[4] = GRIPPER_CLOSE
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.5)

        #lift up
        self.motorMsg.data[1] = (MOTOR1_HOME + 20)
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(1.0)

        self.motorMsg.data[2] = MOTOR2_HOME
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(1.0)

        #move to place position
        if object == 1:
            self.motorMsg.data[0] = MOTOR0_PLACE1
        else:
            self.motorMsg.data[0] = MOTOR0_PLACE2
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(1.0)

        #move down postion
        self.motorMsg.data[1] = MOTOR1_PICKUP
        self.motorMsg.data[3] = MOTOR3_PICKUP
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(1.0)

        self.motorMsg.data[4] = GRIPPER_OPEN
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(0.3)
        #place action

        #lift up
        self.motorMsg.data[2] = MOTOR2_HOME
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(1.0)

        self.motorMsg.data[1] = MOTOR1_HOME
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(1.0)

        #Home
        self.motorMsg.data[0] = MOTOR0_HOME
        moveJoint(self.m0, self.m1, self.m2, self.m3, self.m4, self.motorMsg)
        sleep(1.5)

def main(args=None):
    rclpy.init(args=args)
    myarm=  Rebearm()
    rclpy.spin(myarm)
    myarm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

