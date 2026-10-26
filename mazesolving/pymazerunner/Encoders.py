
import ConstantsRobot as cr
import utime

# @timed_function
    # Print time taken by a function call
    # https://github.com/peterhinch/micropython-samples/blob/master/timed_function/timed_func.py
def timed_function(f, *args, **kwargs):
    def new_func(*args, **kwargs):
        t = utime.ticks_us()
        result = f(*args, **kwargs)
        delta = utime.ticks_diff(utime.ticks_us(), t) 
        print("Function", f.__name__, "Time =", round(delta / 1000, 3), "ms")
        return result
    return new_func

class Encoders:        
    def __init__(self, leftMotorAPin : int, rightMotorAPin : int) -> None:        
        from pio_quadrature_counter import PIOQuadratureCounter
        self.left = PIOQuadratureCounter(0, leftMotorAPin, leftMotorAPin+1)
        self.right = PIOQuadratureCounter(1, rightMotorAPin, rightMotorAPin+1)
        self._flip_sign = 1

        # Zero counts
        self.left_offset = 0
        self.right_offset = 0
        #self.get_counts(reset = True)

        self.LeftPulses = 0
        self.RightPulses = 0

        self.PrevLeftPulses = 0
        self.PrevRightPulses = 0

        self.Remainder = 0
        self.LeftRemainder = 0
        self.RightRemainder = 0

        self.delta_counts_forward = 0
        self.delta_counts_rotation = 0
       
        self.speed_buffer = [0] * 10  # 
        self.buffer_index = 0       

        # Temp variables but created here to help with dynamic memory allocation
        self.counts = [0, 0]
        

    def flip(self, flip):
        self._flip_sign = -1 if flip else 1

    def get_counts(self, reset = False):
        # Sign on the 3pi encoders is reversed from what
        # the PIO counter gives us.
        left = -self.left.read() - self.left_offset
        right = -self.right.read() - self.right_offset

        if reset:
            self.left_offset += left
            self.right_offset += right

        self.counts[0] = left * self._flip_sign
        self.counts[1] = right * self._flip_sign
    
    def reset(self) -> None:
        self.get_counts(reset=True)
        self.LeftPulses = 0
        self.RightPulses = 0   
        self.PrevLeftPulses = 0
        self.PrevRightPulses = 0
        self.Remainder = 0
        self.LeftRemainder = 0
        self.RightRemainder = 0
        return
    
    
    def update(self) -> None :
        
        self.PrevLeftPulses = self.LeftPulses
        self.PrevRightPulses = self.RightPulses

        self.get_counts(reset=True)

        l = self.counts[0]
        r = self.counts[1]

        self.LeftPulses += l
        self.RightPulses += r

        self.Remainder = (((self.LeftPulses + self.RightPulses) - (self.PrevLeftPulses + self.PrevRightPulses)) % 2)
        self.LeftRemainder = (self.LeftPulses - self.PrevLeftPulses) % 2
        self.RightRemainder = (self.RightPulses - self.PrevRightPulses) % 2

        self.delta_counts_forward = (((self.LeftPulses + self.RightPulses) - (self.PrevLeftPulses + self.PrevRightPulses)) // 2) #- self.Remainder
        self.delta_counts_rotation = (r - l)         
        
        # Update the speed buffer
        self.speed_buffer[self.buffer_index] = self.delta_counts_forward
        self.buffer_index = (self.buffer_index + 1) % len(self.speed_buffer)
        
        return
    
    def RobotFowardCount(self):
        return ((self.LeftPulses + self.RightPulses)//2)
    
    def RobotFowardChangeCount(self):
        return (self.delta_counts_forward)
    
    def RobotFowardChangeMM(self):
        return ((self.delta_counts_forward * cr.DIST_SCALE) // cr.COUNT_PER_MM_SCALED)
    
    def RobotDistanceMM(self):
        return (((self.LeftPulses + self.RightPulses) * cr.DIST_SCALE) // cr.COUNT_PER_MM_SCALED) //2

    def RobotAngleCount(self):
        return (self.RightPulses - self.LeftPulses)    

    def RobotAngleDeg(self):
        return ((self.RightPulses - self.LeftPulses) * cr.DIST_SCALE) // cr.COUNT_PER_DEG_SCALED
    
    def RobotAngleChangeCount(self):
        return (self.delta_counts_rotation)
    
    def RobotAngleChangeDeg(self):
        return ((self.delta_counts_rotation * cr.DIST_SCALE) // cr.COUNT_PER_DEG_SCALED)
    
    def RobotSpeed(self) -> int :        
        return (cr.LOOP_FREQUENCY * self.RobotFowardChangeMM())
    
    def RobotSpeedCounts(self) -> int :        
        return (cr.LOOP_FREQUENCY * self.RobotFowardChangeCount())

    def RobotAverageSpeed(self) -> int :
        # Average speed over the last 10 samples        
        return cr.LOOP_FREQUENCY * ((sum(self.speed_buffer) // len(self.speed_buffer) * cr.DIST_SCALE) // cr.COUNT_PER_MM_SCALED)
    
    def RobotAverageCounts(self) -> int :
        # Average speed over the last 10 samples        
        return cr.LOOP_FREQUENCY * (sum(self.speed_buffer) // len(self.speed_buffer) )
    


    # def RobotDistance(self) -> float:        
    #     return self.robotDistanceTraveled

    # def RobotSpeed(self) -> float :        
    #     return (cr.LOOP_FREQUENCY * self.forwardChange)

    # def RobotOmega(self) -> float : # Speed of rotation change        
    #     return (cr.LOOP_FREQUENCY * self.rotationChange)

    # def RobotForwardChange(self) -> float :
    #     return self.forwardChange

    # def RobotRotationChange(self) -> float :
    #     return self.rotationChange

    # def RobotAngle(self) -> float : 
    #     return self.robotAngleTurned
