import ConstantsMachine as cm
import ConstantsRobot as cr

from Encoders import Encoders
from DCMotor import DCMotor


class Motors : 

    def __init__(self, encoders : Encoders) -> None :
        self.encoders = encoders
        self.m_controller_output_enabled = False
        self.m_feedforward_enabled = True

        self.m_fwd_error = 0
        self.m_rot_error = 0
        self.m_previous_fwd_error = 0
        self.m_previous_rot_error = 0
        
        
        self.m_velocity = 0 #in units of counts
        self.m_omega = 0 #in units of counts

        self.pos_output = 0
        self.rot_output = 0

        self.leftMotor = DCMotor(cm.LEFT_MOTOR_FWD_PWM_PIN, cm.LEFT_MOTOR_REV_PWM_PIN)
        self.rightMotor = DCMotor(cm.RIGHT_MOTOR_FWD_PWM_PIN, cm.RIGHT_MOTOR_REV_PWM_PIN)

        self.m_left_motor_volts = 0
        self.m_right_motor_volts = 0
        self.left_pwm = 0
        self.right_pwm = 0

        self.left_output = 0
        self.right_output = 0
        self.tangent_speed = 0
        self.left_speed = 0
        self.right_speed = 0

        self.DEBUG = False

    def stop(self) -> None :
        self.leftMotor.stop()
        self.rightMotor.stop()
        
    def disable_controllers(self) -> None :
        self.m_controller_output_enabled = False
        return

    def reset_controllers(self) -> None :
        self.m_fwd_error = 0
        self.m_rot_error = 0
        self.m_previous_fwd_error = 0
        self.m_previous_rot_error = 0
        return

    def enable_controllers(self) -> None :
        self.m_controller_output_enabled = True
        return

    def calcFeedForward(self, speed :int) -> int :       
        if speed > 0:
            return ((speed * cr.SPEED_FF) + cr.BIAS_FF) 
        else:
            return ((speed * cr.SPEED_FF) - cr.BIAS_FF) 

    def set_left_motor_pwm(self, pwm : int, direction : int) -> None :
        pwm = self.constrain(pwm, cr.MOTOR_MIN_PWM, cr.MOTOR_MAX_PWM)
        if direction == 1:
            self.leftMotor.forward(pwm)
        else:
            self.leftMotor.backwards(pwm)

    def set_right_motor_pwm(self, pwm : int, direction : int) -> None:
        pwm = self.constrain(pwm, cr.MOTOR_MIN_PWM, cr.MOTOR_MAX_PWM)
        if direction == 1:
            self.rightMotor.forward(pwm)
        else:
            self.rightMotor.backwards(pwm)
    
    def set_left_motor_volts(self, volts : int) -> None : 
        '''volts are being passed while being scaled by the PD_SCALE, so they are not in volts yet'''        
        self.m_left_motor_volts = self.constrain(abs(volts), cr.MIN_MOTOR_VOLTS, cr.MAX_MOTOR_VOLTS * cr.PD_SCALE)        
        #TODO this is a bit of a hack as there was as int overflow causing heap memory uses. This splits the PD_SCALE into 2 parts to prevent this
        self.m_left_motor_volts = self.m_left_motor_volts // 100
        self.left_pwm = cr.MOTOR_MAX_PWM - (cr.MOTOR_MAX_PWM * self.m_left_motor_volts ) // (cr.BATTERY_VOLTS * (cr.PD_SCALE // 100))
        #PWM needs to be inverted to get the right amount of power
        self.set_left_motor_pwm(self.left_pwm, 0 if volts < 0 else 1)
    
    def set_right_motor_volts(self, volts : int) -> None :         
        self.m_right_motor_volts = self.constrain(abs(volts), cr.MIN_MOTOR_VOLTS, cr.MAX_MOTOR_VOLTS * cr.PD_SCALE)        
        self.m_right_motor_volts = self.m_right_motor_volts // 100
        self.right_pwm = cr.MOTOR_MAX_PWM - (cr.MOTOR_MAX_PWM * self.m_right_motor_volts ) // (cr.BATTERY_VOLTS * (cr.PD_SCALE // 100))
        self.set_right_motor_pwm(self.right_pwm,0 if volts < 0 else 1)        

    def set_speeds(self, velocity : int, omega : int) -> None :
        self.m_velocity = velocity
        self.m_omega = omega    

    def update_controllers(self, velocity : int, omega : int, steering_adjustment : int) -> None :
        '''velocity is from the profile of what the robot should be doing in units of counts per second
        omega is the angular velocity from the profile of what the robot should be doing, this is in counts per second
        steering_adjustment is the steering adjustment from the PID controller
        '''
        velocitySign = -1 if (velocity < 0) else +1
        omegaSign = -1 if(omega < 0) else +1
        self.m_velocity = velocity
        self.m_omega = omega
        
        # We use the absolute value in the calculation of the error, so we need to make sure that the sign is correct
        # This is done for consistant rounding of the error regardless of the sign of the velocity and omega
        # Postion controller 
        self.m_fwd_error += (velocitySign * ((abs(self.m_velocity) * cr.LOOP_INTERVAL_SCALED) // cr.LOOP_SCALE)) - self.encoders.RobotFowardChangeCount() # The error between how far we should have gone and how far we have gone
        
        self.pos_output = ((cr.FWD_KP * self.m_fwd_error) + (cr.FWD_KD * (self.m_fwd_error - self.m_previous_fwd_error)) ) 
        self.m_previous_fwd_error = self.m_fwd_error

        # Angle controller
        self.m_rot_error += (omegaSign * ((abs(self.m_omega) * cr.LOOP_INTERVAL_SCALED) // cr.LOOP_SCALE)) - self.encoders.RobotAngleChangeCount() # The error between how far we should have turned and how far we have turned
        self.m_rot_error += steering_adjustment
        
        self.rot_output = ((cr.ROT_KP * self.m_rot_error) + (cr.ROT_KD * (self.m_rot_error - self.m_previous_rot_error)) ) 
        self.m_previous_rot_error = self.m_rot_error
    
        self.left_output = self.pos_output - self.rot_output
        self.right_output = self.pos_output + self.rot_output 
        
        if (self.m_feedforward_enabled) :
            # Convert omega of degrees per second to radians per second
            self.tangent_speed = (self.m_omega * cr.MOUSE_RADIUS_SCALED) // (cr.DIST_SCALE*cr.RAD_SCALE)
            self.left_speed = self.m_velocity - self.tangent_speed
            self.right_speed = self.m_velocity + self.tangent_speed 
        
            self.left_output += self.calcFeedForward(self.left_speed)
            self.right_output += self.calcFeedForward(self.right_speed)
        
        # these value have still been scalled by the PD_SCALE, so they are not in volts yet
        if (self.m_controller_output_enabled) :
            self.set_left_motor_volts(self.left_output)
            self.set_right_motor_volts(self.right_output) 

    def log_state(self):
        print("=== MOTOR STATE ===")
        print("velocity:", self.m_velocity, "omega:", self.m_omega)
        print("fwd_error:", self.m_fwd_error, "prev_fwd_error:", self.m_previous_fwd_error)
        print("rot_error:", self.m_rot_error, "prev_rot_error:", self.m_previous_rot_error)
        print("pos_output:", self.pos_output, "rot_output:", self.rot_output)
        print("left_output:", self.left_output, "right_output:", self.right_output)
        print("left_speed:", self.left_speed, "right_speed:", self.right_speed)
        print("tangent_speed:", self.tangent_speed)
        print("left_motor_volts:", self.m_left_motor_volts, "right_motor_volts:", self.m_right_motor_volts)

            
    def constrain(self, val, min_val, max_val):
        # Weirdly the inbuilt min and max functions are slower than the if statement
        if val < min_val:
            return min_val
        elif val > max_val:
            return max_val
        else:
            return val