#from typing import List
from ConstantsClasses import *
import ConstantsRobot as cr
import ConstantsMachine as cm
import machine 

from utime import sleep_us, sleep_ms, ticks_us, ticks_diff
from math import floor, sqrt

# @timed_function
# Print time taken by a function call
# https://github.com/peterhinch/micropython-samples/blob/master/timed_function/timed_func.py

def timed_function(f, *args, **kwargs):
    def new_func(*args, **kwargs):
        t = ticks_us()
        result = f(*args, **kwargs)
        delta = ticks_diff(ticks_us(), t) 
        print('Function {} Time = {:6.3f}ms'.format(f.__name__, delta/1000))
        return result
    return new_func

class Sensors :

    def __init__(self) -> None :

        #Simple wall board has 3 ADC but we mimic 4 for simple code coping
        self.LeftAngleSensor = SensorChannel()
        self.LeftFrontSensor = SensorChannel()
        self.RightAngleSensor = SensorChannel()
        self.RightFrontSensor = SensorChannel()

        self.AngleActive = True
        self.FrontActive = True

        self.see_front_wall = False

        self.see_left_angle_wall = False
        
        self.see_right_angle_wall = False

        self.m_front_sum = 0
        self.m_front_diff = 0

        self.g_steering_mode = SteeringModes.STEER_NORMAL

        self.last_steering_error = 0
        self.m_active = False
        self.m_cross_track_error = 0
        self.m_steering_adjustment = 0
        self.walls = [False,False,False]
        self.wallValues = [0,0,0]
        self.wallRawValues = [0,0,0,0]
        self.wallAmbientValues = [0,0,0,0]

        # local temp variables
        self.leftSensorValue = 0
        self.leftSensorValueOff = 0
        self.leftSensorDiff = 0
        self.rightSensorValue = 0
        self.rightSensorValueOff = 0
        self.rightSensorDiff = 0
        self.frontLeftSensorValue = 0
        self.frontLeftSensorValueOff = 0
        self.frontLeftSensorDiff = 0
        self.frontRightSensorValueOff = 0
        self.frontRightSensorDiff = 0

        self.error = 0
        self.left_error = 0
        self.right_error = 0
        self.left_front_error = 0
        self.right_front_error = 0

        self.pTerm = 0
        self.dTerm = 0
        self.steering_adjustment_temp = 0
        

    def enable(self) -> None :
        self.m_active = True

    def disable(self) -> None :
        self.m_active = False

    def get_front_sum(self) -> int :
        return int(self.m_front_sum)
    
    def get_front_diff(self) -> int :
        return int(self.m_front_diff)
    
    def get_cross_track_error(self) -> int :
        return self.m_cross_track_error    

    def get_steering_feedback(self) -> int :
        return self.m_steering_adjustment
    
    def GetWalls(self) -> list[bool] :
        self.walls[0] = self.see_left_angle_wall
        self.walls[1] = self.see_front_wall
        self.walls[2] = self.see_right_angle_wall

        return self.walls

    def GetWallValues(self) -> list[int] :
        self.wallValues[0] = self.LeftAngleSensor.value
        self.wallValues[1] = self.m_front_sum
        self.wallValues[2] = self.RightAngleSensor.value

        return self.wallValues
    
    def GetWallRawValues(self) -> list[int] :
        self.wallRawValues[0] = self.LeftAngleSensor.raw
        self.wallRawValues[1] = self.LeftFrontSensor.raw
        self.wallRawValues[2] = self.RightFrontSensor.raw
        self.wallRawValues[3] = self.RightAngleSensor.raw

        return self.wallRawValues
    
    def GetWallAmbientValues(self) -> list[int] :
        self.wallAmbientValues[0] = self.LeftAngleSensor.ambient
        self.wallAmbientValues[1] = self.LeftFrontSensor.ambient
        self.wallAmbientValues[2] = self.RightFrontSensor.ambient
        self.wallAmbientValues[3] = self.RightAngleSensor.ambient

        return self.wallAmbientValues
   
    # square roots and divisions are pretty slow. Don't call this from systick()
    # If you really want to use this in the control loop, consider pre-calculating
    # a table of k/sqrt(i) for some suitable range of i. The table would need to be
    # stored in flash.
   
    def get_distance(self, sensor_value:int,  k:int) -> float :  
             
        distance = k / sqrt(sensor_value)
        return min(200, distance)

    def calculate_steering_adjustment(self) -> None :
        self.pTerm = cr.STEERING_KP * self.m_cross_track_error
        self.dTerm = cr.STEERING_KD * (self.m_cross_track_error - self.last_steering_error)
        self.steering_adjustment_temp = ((self.pTerm + self.dTerm) * cr.LOOP_INTERVAL_SCALED) // (cr.LOOP_SCALE * cr.PD_SCALE)

        if self.steering_adjustment_temp > cr.STEERING_ADJUST_LIMIT:
            self.steering_adjustment_temp = cr.STEERING_ADJUST_LIMIT
        elif self.steering_adjustment_temp < -cr.STEERING_ADJUST_LIMIT:
            self.steering_adjustment_temp = -cr.STEERING_ADJUST_LIMIT
        
        self.last_steering_error = self.m_cross_track_error
        self.m_steering_adjustment = self.steering_adjustment_temp

        
    def set_steering_mode(self, mode : int) -> None :
        self.last_steering_error = self.m_cross_track_error
        self.m_steering_adjustment = 0
        self.g_steering_mode = mode
    
    def GetSensorRawValues(self) -> None:                

        self.leftSensorValueOff = cm.SENSOR_LEFT_ADC_PIN.read_u16()
        self.rightSensorValueOff = cm.SENSOR_RIGHT_ADC_PIN.read_u16()
        self.frontLeftSensorValueOff = cm.SENSOR_FRONT_ADC_PIN.read_u16()
        self.frontRightSensorValueOff = self.frontLeftSensorValueOff #HACK The simple wall board only has one front sensor but lets mimic 2 sensors incase of future changes

        cm.SENSOR_FRONT_ACTIVATION_PIN.on()
        sleep_us(cr.LED_ON_TIME)
                  
        self.frontLeftSensorValue = cm.SENSOR_FRONT_ADC_PIN.read_u16()
        self.frontRightSensorValue = self.frontLeftSensorValue   
        cm.SENSOR_FRONT_ACTIVATION_PIN.off()    
        

        cm.SENSOR_ANGLE_ACTIVATION_PIN.on()
        sleep_us(cr.LED_ON_TIME)
        self.leftSensorValue = cm.SENSOR_LEFT_ADC_PIN.read_u16()
        self.rightSensorValue = cm.SENSOR_RIGHT_ADC_PIN.read_u16() 
        cm.SENSOR_ANGLE_ACTIVATION_PIN.off()

        #In theory these wont be negative
        self.leftSensorDiff = self.leftSensorValue - self.leftSensorValueOff 
        self.rightSensorDiff = self.rightSensorValue - self.rightSensorValueOff 
        self.frontLeftSensorDiff = self.frontLeftSensorValue - self.frontLeftSensorValueOff 
        self.frontRightSensorDiff = self.frontRightSensorValue - self.frontRightSensorValueOff 
        
        # Just check if they are negative and set to zero. Might happen if a wall is very far away. If is used because it is faster then max()
        self.LeftFrontSensor.raw = self.frontLeftSensorDiff if self.frontLeftSensorDiff > 0 else  0 
        self.LeftFrontSensor.ambient = self.frontLeftSensorValueOff
        self.RightFrontSensor.raw = self.frontRightSensorDiff if self.frontRightSensorDiff > 0 else 0
        self.RightFrontSensor.ambient = self.frontRightSensorValueOff
        
        self.LeftAngleSensor.raw = self.leftSensorDiff if self.leftSensorDiff > 0 else  0 
        self.LeftAngleSensor.ambient = self.leftSensorValueOff
        self.RightAngleSensor.raw = self.rightSensorDiff if self.rightSensorDiff > 0 else 0
        self.RightAngleSensor.ambient = self.rightSensorValueOff

    def ScaleSensorRAWValues(self) -> None : 
        self.LeftFrontSensor.value = (self.LeftFrontSensor.raw * cr.LEFT_FRONT_SCALE) // cr.SCALE_FACTOR
        self.RightFrontSensor.value = (self.RightFrontSensor.raw * cr.RIGHT_FRONT_SCALE) // cr.SCALE_FACTOR   

        self.LeftAngleSensor.value = (self.LeftAngleSensor.raw * cr.LEFT_SCALE) // cr.SCALE_FACTOR
        self.RightAngleSensor.value = (self.RightAngleSensor.raw * cr.RIGHT_SCALE) // cr.SCALE_FACTOR

    # This will be called in the systick function
    def update(self) -> None :
        
        if not self.m_active : # Exit if sensor readings arn't needed
            self.m_cross_track_error = 0
            self.m_steering_adjustment = 0
            return

        self.GetSensorRawValues()
        self.ScaleSensorRAWValues()
                
        self.see_left_angle_wall = self.LeftAngleSensor.value > cr.LEFT_THRESHOLD
        self.see_right_angle_wall = self.RightAngleSensor.value > cr.RIGHT_THRESHOLD

        self.m_front_sum = self.LeftFrontSensor.value + self.RightFrontSensor.value 
        self.m_front_diff = self.LeftFrontSensor.value - self.RightFrontSensor.value
        # Front wall reading is different if side walls are present

        self.see_front_wall = self.m_front_sum > cr.FRONT_THRESHOLD

        self.error = 0
        self.left_error = cr.SIDE_NOMINAL - self.LeftAngleSensor.value
        self.right_error = cr.SIDE_NOMINAL - self.RightAngleSensor.value

        self.left_front_error = cr.FRONT_NOMINAL - self.LeftFrontSensor.value
        self.right_front_error = cr.FRONT_NOMINAL - self.RightFrontSensor.value
        
        if self.g_steering_mode == SteeringModes.STEER_NORMAL :
            if(self.see_front_wall) :
                # result = b and a / b or 0  # a / b
                # https://stackoverflow.com/questions/27317517/make-division-by-zero-equal-to-zero
                #Possible divide by zero error
                self.error = (self.left_front_error + self.right_front_error) and ((self.left_front_error - self.right_front_error) // (self.left_front_error + self.right_front_error)) or 0

            elif self.see_left_angle_wall and self.see_right_angle_wall :
                self.error = self.left_error - self.right_error
            elif self.see_left_angle_wall  : 
                self.error = 2 * self.left_error
            elif self.see_right_angle_wall : 
                self.error = -2 * self.right_error
            
        elif self.g_steering_mode == SteeringModes.STEER_LEFT_WALL : 
            self.error = 2 * self.left_error
        elif self.g_steering_mode == SteeringModes.STEER_RIGHT_WALL :
            self.error = -2 * self.right_error
        
        # the side sensors are not reliable close to a wall ahead.
        if self.m_front_sum > cr.FRONT_WALL_RELIABILITY_LIMIT and not self.see_front_wall : # Not sure this code will execute
            self.error = 0
        
        self.m_cross_track_error = self.error
        self.calculate_steering_adjustment()

    # Convenience functions for the sensors when used as UI components
    # such as when starting the robot by putting your hand in front

    def occluded_left(self) -> bool :
        return self.LeftAngleSensor.value > cr.OCCLUDED_THRESHOLD and self.RightAngleSensor.value < cr.OCCLUDED_THRESHOLD
    

    def occluded_right(self) -> bool :
        return self.LeftAngleSensor.value < cr.OCCLUDED_THRESHOLD and self.RightAngleSensor.value > cr.OCCLUDED_THRESHOLD
    
    
    # * The sensors will be enabled and this function will return
    # * values indicating which of the two front sensors was occluded by the
    # * user. While waiting, the user LED is flashing rapidly to show that it
    # * expects some action. The LED will go out when the user is detected.
    # *
    # * There is no timeout - the method will wait forever if necessary.
    # *
    # * There is a 250ms delay before returning to let the user get clear.
    # *
    # * Leaves the sensors disabled.
    
    def wait_for_user_start(self) -> int : 
        cm.SENSOR_LED_LEFT.on()
        self.enable()
        choice = StartStates.NO_START
        while choice == StartStates.NO_START :
            count = 0
            while self.occluded_left() :
                count += 1
                sleep_ms(20)
            
            if count > 5 :
                choice = StartStates.LEFT_START
                break
            
            count = 0
            while self.occluded_right() :
                count += 1
                sleep_ms(20)
            
            if count > 5 : 
                choice = StartStates.RIGHT_START
                break
            
            sleep_ms(25)
        self.disable()
        cm.SENSOR_LED_LEFT.off()
        sleep_ms(500)
        return choice
