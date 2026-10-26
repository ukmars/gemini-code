
from Motors import Motors
from Profile import Profile
import ConstantsRobot as cr
from ConstantsClasses import *

from utime import  sleep_ms

class Motion :

    def __init__(self,motors : Motors, forward : Profile, rotation : Profile) -> None :
        self.motors = motors
        self.forward = forward
        self.rotation = rotation       

    def reset_drive_system(self):
        self.motors.stop()
        self.motors.disable_controllers()
        self.motors.encoders.reset()
        self.forward.reset()
        self.rotation.reset()
        self.motors.reset_controllers()        
        self.motors.enable_controllers()

    def stop(self) -> None :
        self.motors.stop()

    def disable_drive(self) -> None:
        self.motors.disable_controllers()

    def position(self) -> int :
        '''Get the position of the forward motion. This is in mm. '''
        return self.forward.position()

    def velocity(self) -> int :
        return self.forward.speed()

    def acceleration(self) -> int :
        return self.forward.acceleration()

    def set_target_velocity(self, velocity : int):
        '''Set the target velocity for the forward motion. This is used to set the target speed for the profile.
        The velocity is in mm/s. this only effects the forward motion. The rotation is not affected.'''
        self.forward.set_target_speed(velocity)

    def set_final_speed(self, velocity : int):
        self.forward.set_final_speed(velocity)

    def angle(self) -> int :
        return self.rotation.position()

    def omega(self) -> int :
        return self.rotation.speed()

    def alpha(self) -> int :
        return self.rotation.acceleration()

    def start_move(self, distance : int, top_speed : int, final_speed : int, acceleration : int) -> None:
        self.forward.start(distance, top_speed, final_speed, acceleration )

    def move_finished(self) -> bool:
        return self.forward.is_finished()

    #This is a blocking action and will wait for the move to finish
    def move(self, distance : int, top_speed : int, final_speed : int, acceleration : int) -> None:
        self.forward.move(distance, top_speed, final_speed, acceleration)

    def start_turn(self, distance : int, top_speed : int, final_speed : int, acceleration : int) -> None:
        self.rotation.start(distance, top_speed, final_speed, acceleration)

    def turn_finished(self) -> bool:
        return self.rotation.is_finished()

    def update(self) -> None:
        self.forward.update()
        self.rotation.update()

    def set_position(self, pos : int) -> None:
        '''Set the position of the forward motion. This is in mm. '''
        self.forward.set_position(pos)

    def adjust_forward_position(self, delta : int) -> None:
        '''Adjust the position of the forward motion. This is in mm. '''
        self.forward.adjust_position(delta)

    def turn(self, distance : int, top_speed : int, final_speed : int, acceleration : int)  -> None:
        self.rotation.move(distance, top_speed, final_speed, acceleration)
  
    def spin_turn(self, angle : int, omega : int, alpha : int) -> None:
        self.forward.set_target_speed(0)
        while self.forward.speed() != 0:
            sleep_ms(2)  
        self.rotation.reset()        
        self.rotation.move(angle, omega, 0, alpha)

    def stop_at(self, position : int) -> None:
        
        remaining = position - self.forward.position()
        self.forward.move(remaining, self.forward.speed(), 0, self.forward.acceleration())

    def stop_after(self, distance : int) -> None:
        self.forward.move(distance, self.forward.speed(), 0, self.forward.acceleration())

    def wait_until_position(self, position : int) -> None:
        '''position is in mm'''        
        while self.forward.position() < position:
            sleep_ms(2) 

    def wait_until_distance(self, distance : int) -> None:
        
        target = self.forward.position() + distance
        self.wait_until_position(target)

