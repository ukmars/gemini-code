
# Super simple script that stands alone to test the motors on Gemini
# Motors will be run full speed forwarf and reverse in sequence so you can prove correct motor operation
#Press and hold the left button to stop the script. This prevents motors from running when the script is not being used
from machine import Pin
from utime import sleep

ButtonPin = 15
LeftMotorPin = 2
RightMotorPin = 4

BUTTON_FUNCTION_PIN = Pin(ButtonPin, Pin.IN, Pin.PULL_UP)

LEFT_MOTOR_FWD_PWM_PIN = Pin(LeftMotorPin+1, Pin.OUT)
LEFT_MOTOR_REV_PWM_PIN = Pin(LeftMotorPin, Pin.OUT)

RIGHT_MOTOR_FWD_PWM_PIN = Pin(RightMotorPin, Pin.OUT)
RIGHT_MOTOR_REV_PWM_PIN = Pin(RightMotorPin+1, Pin.OUT)

def LeftMotorForward():
    LEFT_MOTOR_FWD_PWM_PIN.on()
    LEFT_MOTOR_REV_PWM_PIN.off()

def LeftMotorReverse():
    LEFT_MOTOR_FWD_PWM_PIN.off()
    LEFT_MOTOR_REV_PWM_PIN.on()

def RightMotorForward():
    RIGHT_MOTOR_FWD_PWM_PIN.on()
    RIGHT_MOTOR_REV_PWM_PIN.off()
def RightMotorReverse():
    RIGHT_MOTOR_FWD_PWM_PIN.off()
    RIGHT_MOTOR_REV_PWM_PIN.on()
def StopMotors():
    LEFT_MOTOR_FWD_PWM_PIN.off()
    LEFT_MOTOR_REV_PWM_PIN.off()
    RIGHT_MOTOR_FWD_PWM_PIN.off()
    RIGHT_MOTOR_REV_PWM_PIN.off()

while (BUTTON_FUNCTION_PIN.value() == 1): 
    print("Both Forward")
    LeftMotorForward()
    RightMotorForward()
    sleep(2)
    print("Left Reverse, Right forward")
    LeftMotorReverse() 
    RightMotorForward()
    sleep(2)
    print("Left Forward, Right Reverse")
    LeftMotorForward()
    RightMotorReverse()
    sleep(2)
    print("Left Reverse, Right Reverse")
    LeftMotorReverse()
    RightMotorReverse()

print("Stopping motors")
StopMotors()

