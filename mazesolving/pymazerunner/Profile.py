
from math import fabs
from utime import sleep_ms

from ConstantsClasses import *
import ConstantsRobot as cr

class Profile :

    def __init__(self, scaleType:int) -> None :
        '''This class implements a trapezoidal velocity profile. The profile is used to accelerate and decelerate a motor to a target speed.
        all m_* variables are in counts. The scaleType is used to determine if the profile is in mm or degrees.'''
        self.m_state = States.IDLE
        self.m_speed = 0
        self.m_position = 0
        self.m_sign = 1
        self.m_acceleration = 0
        self.m_one_over_acc = 1
        self.m_target_speed = 0
        self.m_final_speed = 0
        self.m_final_position = 0
        self.writeDebug = False
        self.scaleType = scaleType

    def start(self,distance :int, top_speed : int, final_speed : int, acceleration : int) -> None: 
        '''All of the inputs are in mm/s or mm/s^2. The distance is the distance to travel in mm. 
        The top speed is the maximum speed in mm/s. 
        The final speed is the speed at the end of the profile in mm/s. The acceleration is the acceleration in mm/s^2.
        The core UPDATE loop of the system needs to operate in counts to avoid float math'''
        self.m_sign = -1 if (distance < 0) else +1

        if (distance < 0) :
            distance = -distance

        if (self.scaleType == ScaleType.MM) :
            distance = (distance * cr.COUNT_PER_MM_SCALED ) // cr.DIST_SCALE
            top_speed = (top_speed * cr.COUNT_PER_MM_SCALED) // cr.DIST_SCALE
            final_speed = (final_speed * cr.COUNT_PER_MM_SCALED) // cr.DIST_SCALE
            acceleration = (acceleration * cr.COUNT_PER_MM_SCALED) // cr.DIST_SCALE

            # Lets convert the calculated counts per X into a number that divides better within the profile loop. 
            # This should help prevent rounding errors from accumulating over time.
            top_speed = cr.FindSpeed(top_speed)
            final_speed = cr.FindSpeed(final_speed)
            acceleration = cr.FindAcceleration(acceleration) 
            
        else :
            distance = (distance * cr.COUNT_PER_DEG_SCALED ) // cr.DIST_SCALE
            top_speed = (top_speed * cr.COUNT_PER_DEG_SCALED) // cr.DIST_SCALE
            final_speed = (final_speed * cr.COUNT_PER_DEG_SCALED) // cr.DIST_SCALE
            acceleration = (acceleration * cr.COUNT_PER_DEG_SCALED) // cr.DIST_SCALE

                
        
        if (distance < 1) :
            self.m_state = States.FINISHED
            return
        
        if (final_speed > top_speed) :
            final_speed = top_speed        

        self.m_position = 0
        self.m_final_position = distance
        self.m_target_speed = self.m_sign * abs(top_speed)
        self.m_final_speed = self.m_sign * abs(final_speed)
        self.m_acceleration = abs(acceleration)
        if (self.m_acceleration >= 1) :
            if acceleration in cr.PRECOMPUTED_ONE_OVER_ACC:
                self.m_one_over_acc = cr.PRECOMPUTED_ONE_OVER_ACC[acceleration] 
            else :
                self.m_one_over_acc = int((1.0 / self.m_acceleration) * cr.ACC_SCALE)
        else :
            self.m_one_over_acc = 1
        
        self.m_state = States.ACCELERATING

    #Start a profile and wait for it to finish. This is a blocking call.
    def move(self,distance : int,top_speed : int,final_speed : int,acceleration : int) -> None :
        self.start(distance, top_speed, final_speed, acceleration)
        self.wait_until_finished()

    def wait_until_finished(self) -> None :
        while (self.m_state != States.FINISHED) :
            sleep_ms(2)
        
    def reset(self) -> None : 
        
        self.m_position = 0
        self.m_speed = 0
        self.m_target_speed = 0
        self.m_state = States.IDLE
      
    def is_finished(self) -> bool :
        return self.m_state == States.FINISHED
    
    def stop(self) -> None :
    
        self.m_target_speed = 0    
        self.finish()    

    # Force a profile to finish with the target speed set to the final speed
    def finish(self) -> None :    
        self.m_speed = self.m_target_speed
        self.m_state = States.FINISHED    
  
    def set_state(self,state : States) -> None :
        self.m_state = state
  
    
    def get_braking_distance(self) -> int :
        delta_sq = ((self.m_speed * self.m_speed) - (self.m_final_speed * self.m_final_speed))
        #return abs(delta_sq // 2 * self.m_one_over_acc) // cr.ACC_SCALE
        if delta_sq > 3 :
            return abs(delta_sq // (2 * self.m_acceleration))
        else :
            return 0
  
    # @brief  gets the distance travelled (mm) since the last call to start(). If there
    #         was a prior call to set_position() distance is incremented from there.
    # @return distance travelled (mm)
    def position(self) -> int :        
        pos = self.m_position  
        if (self.scaleType == ScaleType.MM) :
            pos = (pos * cr.DIST_SCALE ) // cr.COUNT_PER_MM_SCALED
        else :
            pos = (pos * cr.DIST_SCALE) // cr.COUNT_PER_DEG_SCALED

        return pos
    
    
    def speed(self) -> int :  
        '''Get the current speed in mm/s or deg/s depending on the scale type'''        
        speed = self.m_speed  
        if (self.scaleType == ScaleType.MM) :
            speed = (speed * cr.DIST_SCALE ) // cr.COUNT_PER_MM_SCALED
        else :
            speed = (speed * cr.DIST_SCALE  ) // cr.COUNT_PER_DEG_SCALED  
        return speed
    
    def acceleration(self) -> int : 
        '''Get the current acceleration in mm/s^2 or deg/s^2 depending on the scale type'''
        acc = self.m_acceleration  
        if (self.scaleType == ScaleType.MM) :
            acc = (acc * cr.DIST_SCALE ) //  cr.COUNT_PER_MM_SCALED
        else :
            acc = (acc * cr.DIST_SCALE) //  cr.COUNT_PER_DEG_SCALED 
         
        return acc
    
    def set_speed(self, speed : int) -> None : 
        '''Set the current speed in mm/s or deg/s depending on the scale type'''
        if (self.scaleType == ScaleType.MM) :
            speed = (speed * cr.COUNT_PER_MM_SCALED ) // cr.DIST_SCALE
            speed = cr.FindSpeed(speed)
        else :
            speed = (speed * cr.COUNT_PER_DEG_SCALED ) // cr.DIST_SCALE
        self.m_speed = speed
        
    def set_target_speed(self, speed : int) -> None :
        '''Set the target speed in mm/s or deg/s depending on the scale type'''
        if (self.scaleType == ScaleType.MM) :
            speed = (speed * cr.COUNT_PER_MM_SCALED ) // cr.DIST_SCALE
            speed = cr.FindSpeed(speed)
        else :
            speed = (speed * cr.COUNT_PER_DEG_SCALED ) // cr.DIST_SCALE
        self.m_target_speed = speed

    def set_final_speed(self, speed : int) -> None :
        '''Set the final speed in mm/s or deg/s depending on the scale type'''
        if (self.scaleType == ScaleType.MM) :
            speed = (speed * cr.COUNT_PER_MM_SCALED ) // cr.DIST_SCALE
            speed = cr.FindSpeed(speed)
        else :
            speed = (speed * cr.COUNT_PER_DEG_SCALED ) // cr.DIST_SCALE
        self.m_final_speed = speed
    
    # normally only used to alter position for forward error correction
    def adjust_position(self, adjustment : int) -> None : 
        '''Adjust the current position in mm or deg depending on the scale type'''
        if (self.scaleType == ScaleType.MM) :
            adjustment = (adjustment * cr.COUNT_PER_MM_SCALED ) // cr.DIST_SCALE
        else :
            adjustment = (adjustment * cr.COUNT_PER_DEG_SCALED ) // cr.DIST_SCALE
        self.m_position += adjustment
        
    def set_position(self,position : int) -> None :
        '''Set the current position in mm or deg depending on the scale type'''
        if (self.scaleType == ScaleType.MM) :
            position = (position * cr.COUNT_PER_MM_SCALED ) // cr.DIST_SCALE
        else :
            position = (position * cr.COUNT_PER_DEG_SCALED ) // cr.DIST_SCALE
        self.m_position = position
        
    def update(self) -> None : # Called during system tick cycle               
        if (self.m_state == States.IDLE) :
            return

        delta_v = (self.m_acceleration * cr.LOOP_INTERVAL_SCALED) // cr.LOOP_SCALE
        remaining = abs(self.m_final_position) - abs(self.m_position)
        
        
        if (self.m_state == States.ACCELERATING) :
            if (remaining < self.get_braking_distance()) : 
                self.m_state = States.BRAKING
                if (self.m_final_speed == 0) :
                    self.m_target_speed = self.m_sign * cr.TARGET_SPEED_ZERO_MAGIC #  // magic number to make sure we reach zero
                else :
                    self.m_target_speed = self.m_final_speed
        
        #try to reach the target speed
        if (self.m_speed < self.m_target_speed) :
            self.m_speed += delta_v

            if (self.m_speed > self.m_target_speed) :
                self.m_speed = self.m_target_speed
        
        if (self.m_speed > self.m_target_speed) :
            self.m_speed -= delta_v

            if (self.m_speed < self.m_target_speed) :
                self.m_speed = self.m_target_speed
            
        #increment the position
        # TODO if the speed is too low this will never increment and the profile will never finish
        # Int math rounds different if negative numbers so only do positive math
        postionChange = (abs(self.m_speed) * cr.LOOP_INTERVAL_SCALED) // cr.LOOP_SCALE

        if postionChange == 0 and self.m_speed != 0 :
            postionChange = 1

        self.m_position += self.m_sign * postionChange #add the negative back in if it is needed
        if (self.m_state != States.FINISHED and remaining < 3) :
            self.m_state = States.FINISHED
            self.m_target_speed = self.m_final_speed     

    def LogData(self) -> str : 
        data = "state {:>3}     speed {:>3}     pos {:>5}       sign {:>3}  accel {:>3} 1over {:>3}     trgspd {:>3}    finspd {:>3}    finpos {:>3}    breakD {:>3}".format(self.m_state
        ,self.m_speed
        ,self.m_position
        ,self.m_sign
        ,self.m_acceleration
        ,self.m_one_over_acc
        ,self.m_target_speed
        ,self.m_final_speed
        ,self.m_final_position
        , self.get_braking_distance())  

        return data