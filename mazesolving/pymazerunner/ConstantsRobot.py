# Constants specific to the robot. This includes physical size of the robot and speed settings
# This would allow for different robots to have different config files. Just update the Constants.py reference
import ConstantsClasses as cc

JSONFilename = 'ConstantsRobot.json'

def LoadConfig() -> dict[str,int] :
    import json
    data = []
    with open(JSONFilename, 'r') as f:
        data = json.load(f)

    return data

def SaveConfig() -> None :
    import json
    data = LoadConfig()

    data['FRONT_REFERENCE'] =  FRONT_REFERENCE
    data['LEFT_ANGLE_CALIBRATION'] =  LEFT_ANGLE_CALIBRATION
    data['RIGHT_ANGLE_CALIBRATION'] =  RIGHT_ANGLE_CALIBRATION
    data['LEFT_FRONT_CALIBRATION'] =  LEFT_FRONT_CALIBRATION
    data['RIGHT_FRONT_CALIBRATION'] =  RIGHT_FRONT_CALIBRATION
    data['TURN_THRESHOLD'] = TURN_THRESHOLD_SS90E
    data['FWD_KP'] = (FWD_KP / PD_SCALE)
    data['FWD_KD'] = (FWD_KD / PD_SCALE)
    data['ROT_KP'] = ROT_KP / PD_SCALE
    data['ROT_KD'] = ROT_KD / PD_SCALE
    data['STEERING_KP'] = STEERING_KP / PD_SCALE
    data['STEERING_KD'] = STEERING_KD / PD_SCALE
    data['FAST_SPEED'] = FAST_RUN_SPEED
    data['SLOW_SPEED'] = SEARCH_SPEED
    data['FAST_ACC'] = FAST_RUN_ACCELERATION
    data['SLOW_ACC'] = SEARCH_ACCELERATION
    data['TURN_SPEED'] = SEARCH_TURN_SPEED
    data['SPIN_SPEED'] = OMEGA_SPIN_TURN
    data['SPIN_ACC'] = ALPHA_SPIN_TURN

    with open(JSONFilename, 'w') as json_file:
        json.dump(data, json_file)

def PrintJSON() -> str :
    import json
    data = LoadConfig()

    return json.dumps(data)


# All below are shared across multple classes as well as defining overall robot real world measurments

JSONData = LoadConfig() #Load data from our JSON file. This is so that specific configs can be updated by code and read on boot. Helps to change maze specific sensor settings

ROBOT_NAME = "G2" #this is used in the bluetooth name

# the values above which, a wall is seen
LEFT_THRESHOLD = 30   # minimum value to register a wall
RIGHT_THRESHOLD = 30  # minimum value to register a wall
FRONT_THRESHOLD = 30  # minimum value to register a wall

# Mouse radius is the distance between the contact patches of the drive wheels.
# A good starting approximation is half the distance between the wheel centres.
# After testing, you may find the working value to be larger or smaller by some
# small amount. AFTER you have the wheel diameter and gear ratio calibrated,
# have the mouse turn in place and adjust the MOUSE_RADIUS until these turns are
# as accurate as you can get them
MOUSE_DIAMETER = 76 # distance between the wheel centers (mm)
MOUSE_RADIUS = MOUSE_DIAMETER / 2 # distance from the center of the robot to the wheel (mm) 

WHEEL_DIAMETER = 32.2 
ENCODER_PULSES = 12 
GEAR_RATIO = 50 # Get this from the gear box and then verify with testing.

# Various Scale factors for converting between units. These are used in the code to convert between mm, degrees and counts.
SCALE_FACTOR = 1000
RAD_SCALE = 100000  # Increase this for precision
ACC_SCALE = 1000000
DIST_SCALE = 1000 # This is used to scale the distance values. Used for mm and degrees


PD_SCALE = 10000 # This is used to scale the PD controller values. 
VOLTS_SCALE = 10 

LOOP_FREQUENCY = 100 #Number of times per second we want to run the control loop
LOOP_INTERVAL = (1.0 / LOOP_FREQUENCY) # time interval of each system tick in seconds
LOOP_SCALE = 1000 
LOOP_INTERVAL_SCALED = int(LOOP_INTERVAL * LOOP_SCALE) # time interval of each system tick in milliseconds

PI = 3.141592654


ALLOWED_SPEEDS = []
ALLOWED_ACCELERATIONS = []
ALLOWED_SPEEDS.append(0)
for i in range(1,50):
    ALLOWED_SPEEDS.append(i*LOOP_FREQUENCY)
    ALLOWED_ACCELERATIONS.append(i*LOOP_FREQUENCY*10) 

# Pre calculating one over acceleration for the allowed accelerations in the profile class
# This is used to speed up the calculations in the profile class. The values are stored in a dictionary for quick access.
PRECOMPUTED_ONE_OVER_ACC = {
    a: (ACC_SCALE // a ) for a in ALLOWED_ACCELERATIONS
}

def FindSpeed(target):
    closest = ALLOWED_SPEEDS[0]
    for val in ALLOWED_SPEEDS:
        if abs(val - target) < abs(closest - target):
            closest = val
    return closest

def FindAcceleration(target):
    closest = ALLOWED_ACCELERATIONS[0]
    for val in ALLOWED_ACCELERATIONS:
        if abs(val - target) < abs(closest - target):
            closest = val
    return closest

TARGET_SPEED_ZERO_MAGIC = LOOP_FREQUENCY # This is a magic minimum speed in COUNTS per second to make sure we reach zero. It is used in the profile class to make sure we reach zero speed when braking. It is used in the profile class to make sure we reach zero speed when braking.





ROTATION_BIAS = -0.005  # Negative makes robot curve to left


MM_PER_COUNT = PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO) 
MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * MM_PER_COUNT
MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * MM_PER_COUNT
DEG_PER_MM_DIFFERENCE = (180.0 / (2 * MOUSE_RADIUS * PI))


DEG_PER_COUNT = (MM_PER_COUNT * 180.0) / (2 * MOUSE_RADIUS * PI) 


ENC_PER_MM = round((ENCODER_PULSES * GEAR_RATIO) / (PI * WHEEL_DIAMETER),1) # encoder counts per mm. Cannot have partial encoder counts so needs scaling to maintain precision
ENC_PER_DEG = round((MOUSE_DIAMETER * GEAR_RATIO * ENCODER_PULSES) / (180 * WHEEL_DIAMETER),1) # encoder counts per degree. Cannot have partial encoder counts


COUNT_PER_MM_SCALED = int(ENC_PER_MM * DIST_SCALE) 
COUNT_PER_DEG_SCALED = int(ENC_PER_DEG * DIST_SCALE) 

# 1 rad ≈ 57.2958 degrees


MOUSE_RADIUS_SCALED = int(((MOUSE_RADIUS * DIST_SCALE)/57.29) * RAD_SCALE) 

MAX_MOTOR_VOLTS = int(9.0 * VOLTS_SCALE)
MIN_MOTOR_VOLTS = 0

BATTERY_VOLTS = int(9 * VOLTS_SCALE)
MOTOR_MIN_PWM = 0 #constant do not change 
MOTOR_MAX_PWM = 65000 #constant do not change 

#*** MOTION CONTROLLER CONSTANTS **********************************************//

#Dynamic performance constants
#There is a video describing how to get these numbers and calculate the feedforward
#constnats here: https://youtu.be/BrabDeHGsa0

##########################################################
#
#       PD Controller Constants
#
##########################################################

# Forward Controller Variables used in PD and feed forward
FWD_KM = 625 # Slope of the graph to calculate mms per volt
BIAS_FF = int(0.27 * PD_SCALE) # calculated by the slope of the graph in excel. This is minimum Voltage to make the robot move

SPEED_FF = int((1.0 / FWD_KM) * PD_SCALE)

FWD_KP = int(JSONData.get("FWD_KP",1.1) * PD_SCALE)
FWD_KD = int(JSONData.get("FWD_KD",0.5) * PD_SCALE)

# Rotation Controller Variables
ROT_KP = int(JSONData.get("ROT_KP",0.5) * PD_SCALE) #Decrease - If the system is aggressively overshooting and oscillating 
ROT_KD = int(JSONData.get("ROT_KD",0.1) * PD_SCALE)#Increase If the system corrects too sharply or is jittery

#controller constants for the steering controller
STEERING_KP = int(JSONData.get("STEERING_KP",3) * PD_SCALE) #3 is ok
STEERING_KD = int(JSONData.get("STEERING_KD",0.25) * PD_SCALE)
STEERING_ADJUST_LIMIT = int(10 * ENC_PER_DEG)  #deg/s

#search and run speeds in mm/s and mm
MIN_SPEED = 50 # minimum speed for the robot to ensure that any profile will finish. 
SEARCH_SPEED = JSONData.get("SLOW_SPEED",250)
SEARCH_ACCELERATION = JSONData.get("SLOW_ACC",2500)
SEARCH_TURN_SPEED = JSONData.get("TURN_SPEED",250)

FAST_RUN_SPEED = JSONData.get("FAST_SPEED",500)
FAST_RUN_ACCELERATION = JSONData.get("FAST_ACC",2500)

OMEGA_SPIN_TURN = JSONData.get("SPIN_SPEED",360)
ALPHA_SPIN_TURN = JSONData.get("SPIN_ACC",4000) 

# Sensor Settings
LED_ON_TIME = 10 #TIME IN NS FOR SENSORS TO BE ON BEFORE TAKING AN ADC READING


#***** SENSOR SCALING ******************************************************//
# This is the normalised value seen by the front sensor when the mouse is
# in its calibration position
SIDE_NOMINAL = 100
FRONT_NOMINAL = 100


# The side sensors are not reliable close to a wall ahead. This value
# is the limit for the front sensor reading, above which the side sensors
# are likely to give innaccurate readings because of reflections from
# the wall ahead. This value only effects steering
FRONT_WALL_RELIABILITY_LIMIT = 950

OCCLUDED_THRESHOLD = 150 # Minimum value for hand start

# RAW values for the side sensors when the robot is centred in a cell at the back wall
# and there is no wall ahead
LEFT_ANGLE_CALIBRATION = JSONData.get("LEFT_ANGLE_CALIBRATION",2700) 
RIGHT_ANGLE_CALIBRATION = JSONData.get("RIGHT_ANGLE_CALIBRATION",4050) 

# RAW values for the front sensor when the robot is backed up to a wall, no side walls and a wall ahead
LEFT_FRONT_CALIBRATION = JSONData.get("LEFT_FRONT_CALIBRATION",3200)   
RIGHT_FRONT_CALIBRATION = JSONData.get("RIGHT_FRONT_CALIBRATION",4100)   


FRONT_REFERENCE = JSONData.get("FRONT_REFERENCE",390) #390  # reading when mouse centered in cell with wall ahead

LEFT_FRONT_SCALE = (FRONT_NOMINAL*SCALE_FACTOR) // LEFT_FRONT_CALIBRATION
RIGHT_FRONT_SCALE = (FRONT_NOMINAL*SCALE_FACTOR) // RIGHT_FRONT_CALIBRATION

LEFT_SCALE = (SIDE_NOMINAL*SCALE_FACTOR) // LEFT_ANGLE_CALIBRATION
RIGHT_SCALE = (SIDE_NOMINAL*SCALE_FACTOR) // RIGHT_ANGLE_CALIBRATION


'''
// SS90E turn thresholds. This is the front sum reading to trigger a turn
// it changes a bit if there is an adjacent wall. The threshold is set for
// when the robot is 20mm past the cell boundary. That is, the distance
// from the front of the mouse to the wall ahead is 92mm
'''
TURN_THRESHOLD_SS90E = JSONData.get("TURN_THRESHOLD",160)
EXTRA_WALL_ADJUST = 0 

# with robot against back wall, how much travel is there to the cell center?
# Robot center is defined by the wheel axles, not the front of the robot.
BACK_WALL_TO_CENTER = 48 
FRONT_WALL_TO_CENTER = 22 # distance from the front of the robot to the cell center when against a wall
# the position in the cell where the sensors are sampled.
SENSING_POSITION = 170

# The mouse enters the smooth turn at the sensing position
# The mouse exits the smooth turn at the sensing position
#Entry and Exist offsets are based on cell center
turn_params = [
#                       speed,         , acceleration,  Entry Offst, exit offset,  angle, omega (speed), alpha(acceleration), sensor threshold
    cc.TurnParameters(SEARCH_TURN_SPEED, SEARCH_ACCELERATION,    75,     75,    91,   250,  1800, TURN_THRESHOLD_SS90E), # 0 => SS90L
    cc.TurnParameters(SEARCH_TURN_SPEED, SEARCH_ACCELERATION,    75,     75,   -91,   250,  1800, TURN_THRESHOLD_SS90E), # 0 => SS90R    
    cc.TurnParameters(360, FAST_RUN_ACCELERATION,    75,     75,   91,   420,  4100, TURN_THRESHOLD_SS90E), # 0 => SS90LF
    cc.TurnParameters(360, FAST_RUN_ACCELERATION,    75,     75,   -91,   420,  4100, TURN_THRESHOLD_SS90E), # 0 => SS90RF
]
