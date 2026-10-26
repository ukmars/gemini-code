import ConstantsRobot as cr
import ConstantsMachine as cm
from Encoders import Encoders
from DCMotor import DCMotor

from machine import Pin, PWM
import time

rightMotor = DCMotor(cm.RIGHT_MOTOR_FWD_PWM_PIN, cm.RIGHT_MOTOR_REV_PWM_PIN)
leftMotor = DCMotor(cm.LEFT_MOTOR_FWD_PWM_PIN, cm.LEFT_MOTOR_REV_PWM_PIN)

pwmStart = 32000
pwmInc = 100
leftMotor.stop()
rightMotor.stop()

time.sleep(1)
print("Start")
waitTime = 2

for i in range(0, 3):
    print("Both Forward")
    leftMotor.forward(pwmStart)
    rightMotor.forward(pwmStart)

    time.sleep(waitTime)
    print("Left backwards right forward")
    leftMotor.backwards(pwmStart)
    rightMotor.forward(pwmStart)

    time.sleep(waitTime)
    print("Left forward right backwards")
    leftMotor.forward(pwmStart)
    rightMotor.backwards(pwmStart)

    time.sleep(waitTime)
    print("Both Backwards")
    leftMotor.backwards(pwmStart)
    rightMotor.backwards(pwmStart)
    time.sleep(waitTime)


leftMotor.stop()
rightMotor.stop()


print("Finished")
