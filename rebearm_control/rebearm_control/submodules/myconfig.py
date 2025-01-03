MOTOR_NOMOVE = 360
MOTOR_RIGHT = -90

#turn left or right of base
MOTOR1_ZERO = 0
MOTOR1_HOME = 0
MOTOR1_OFF = MOTOR1_HOME
MOTOR1_MAX = 60
MOTOR1_MIN = -60
MOTOR1_PLACE1 = MOTOR1_MAX
MOTOR1_PLACE2 = MOTOR1_MIN

#forward or backward
#roll, 3DoF
MOTOR2_ZERO = 0
MOTOR2_HOME = 5
MOTOR2_PICKUP = (MOTOR2_HOME - 15)
MOTOR2_OFF = 90
MOTOR2_MAX = 30
MOTOR2_MIN = (MOTOR2_HOME - 65)

MOTOR3_ZERO = 0
MOTOR3_HOME = -85
MOTOR3_PICKUP = (MOTOR3_HOME - 5)
MOTOR3_OFF = -110
MOTOR3_MAX  = (MOTOR3_HOME + 40)
MOTOR3_MIN  =  -120

MOTOR4_ZERO = 0
MOTOR4_HOME = -90
MOTOR4_PICKUP = (MOTOR4_HOME + 30)
MOTOR4_OFF = -100
MOTOR4_MAX  = (MOTOR4_HOME + 40)
MOTOR4_MIN  = -120

MOTOR5_ZERO = 0
MOTOR5_HOME = -90
MOTOR5_PICKUP = (MOTOR5_HOME + 0)
MOTOR5_OFF = 0
MOTOR5_MAX  = 0
MOTOR5_MIN  = -90

GRIPPER_CLOSE = -80
GRIPPER_OPEN = -60
GRIPPER_MIN = GRIPPER_CLOSE
GRIPPER_MAX = GRIPPER_OPEN

#yolo_chase or blob_chase
PICTURE_SIZE_X = 640.0
PICTURE_SIZE_Y = 480.0

CB_FREQ = 0.1

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
