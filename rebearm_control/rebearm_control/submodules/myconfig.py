MOTOR_NOMOVE = 360
MOTOR_RIGHT = -90

#turn left or right of base
MOTOR0_ZERO = 0
MOTOR0_HOME = 0
MOTOR0_OFF = MOTOR0_HOME
MOTOR0_MAX = 60
MOTOR0_MIN = -60
MOTOR0_PLACE1 = MOTOR0_MAX
MOTOR0_PLACE2 = MOTOR0_MIN

#forward or backward
#roll, 2 DOF
MOTOR1_ZERO = 0
MOTOR1_HOME = 5
MOTOR1_PICKUP = (MOTOR1_HOME - 10)
MOTOR1_OFF = 90
MOTOR1_MAX = 90
MOTOR1_MIN = (MOTOR1_HOME - 65)

MOTOR2_ZERO = 0
MOTOR2_HOME = -85
MOTOR2_PICKUP = (MOTOR2_HOME - 10)
MOTOR2_OFF = -110
MOTOR2_MAX  = (MOTOR2_HOME + 50)
MOTOR2_MIN  = (MOTOR2_HOME - 30)

MOTOR3_ZERO = 0
MOTOR3_HOME = -90
MOTOR3_PICKUP = (MOTOR3_HOME + 20)
MOTOR3_OFF = -100
MOTOR3_MAX  = (MOTOR3_HOME + 60)
MOTOR3_MIN  = -90

MOTOR4_ZERO = 0
MOTOR4_HOME = 90
MOTOR4_PICKUP = (MOTOR4_HOME + 0)
MOTOR4_OFF = 0
MOTOR4_MAX  = 90
MOTOR4_MIN  = 0
MOTOR4_PICKUP = MOTOR4_HOME

GRIPPER_CLOSE = -80
GRIPPER_OPEN = -60
GRIPPER_MIN = GRIPPER_CLOSE
GRIPPER_MAX = GRIPPER_OPEN

#yolo_chase or blob_chase
PICTURE_SIZE_X = 640.0
PICTURE_SIZE_Y = 480.0

# a*x + b for angle_x
K_a = -29.000
K_b = 31.000

Ktimer = 0.1

#teleop_joy parameter
MAX_CHAT = 5           #for protecting jostick's button chattering
TIMER_JOY = 0.1         #for protecting jostick's button chattering
CONT_JOY = 3            #don't move untill CONT_JOY

#teleop_keyboard parameter
MAX_ANG = 120
ANG_STEP = 3
CONT_KEY = 3            #don't move untill CONT_KEY*100ms

#human_guide parameter
TIMER_HGUIDE = 0.2      #how frequent angle capture
