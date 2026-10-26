# file first executed by the pico on boot
from machine import Pin, Timer
from utime import sleep_us, sleep_ms

import ConstantsMachine as cm
from Button import *
from Motion import Motion
from Sensors import Sensors
from Motors import Motors
from Profile import Profile
from Encoders import Encoders

import ConstantsRobot as cr
from ConstantsClasses import *

#Setup global objects for controlling robot
forward = Profile(ScaleType.MM)
rotation = Profile(ScaleType.DEG)
encoders = Encoders(cm.LEFT_MOTOR_A_ENCODER_PIN, cm.RIGHT_MOTOR_A_ENCODER_PIN)
motors = Motors(encoders)

motion = Motion(motors, forward, rotation)

#motion.forward.writeDebug = True
ticks = 0
#motion.start_move(500, 800, 200, 3000)
#motion.start_move(100, cr.FAST_RUN_SPEED_MAX, 0, cr.FAST_RUN_ACCELERATION)
#motion.start_move(100, cr.SEARCH_SPEED, 0, cr.SEARCH_ACCELERATION)
#motion.start_move(1000, 500, 0, 2500)
motion.start_move(180, cr.SEARCH_TURN_SPEED, cr.SEARCH_TURN_SPEED, cr.SEARCH_ACCELERATION)

while (not motion.move_finished()) :
    
    print(motion.forward.LogData())    
    motion.update()
    ticks += 1

print("Ticks = ", ticks)
print(motion.forward.LogData())                                            
motion.update()
print(motion.forward.LogData())                                            
motion.update()
print(motion.forward.LogData())                                            
motion.update()
print(motion.forward.LogData())                                            
motion.update()
print(cr.LOOP_FREQUENCY)