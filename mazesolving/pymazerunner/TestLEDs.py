from machine import Pin
from utime import sleep, sleep_ms, sleep_us

from ConstantsMachine import *


def sequenceLEDs():
    period_ms = 500
    SENSOR_LED_LEFT.value(1)
    sleep_ms(period_ms)
    SENSOR_LED_LEFT.value(0)
    SENSOR_LED_CENTRE.value(1)
    sleep_ms(period_ms)
    SENSOR_LED_CENTRE.value(0)
    SENSOR_LED_RIGHT.value(1)
    sleep_ms(period_ms)
    SENSOR_LED_RIGHT.value(0)
    MEZ_LED_LEFT.value(1)
    sleep_ms(period_ms)
    MEZ_LED_LEFT.value(0)
    MEZ_LED_RIGHT.value(1)
    sleep_ms(period_ms)
    MEZ_LED_RIGHT.value(0)
    for pulses in range(0,150):
        SENSOR_FRONT_ACTIVATION_PIN.value(1)
        sleep_us(15)
        SENSOR_FRONT_ACTIVATION_PIN.value(0)
        sleep_ms(2)
    sleep_ms(period_ms)
    for pulses in range(0,150):
        SENSOR_ANGLE_ACTIVATION_PIN.value(1)
        sleep_us(15)
        SENSOR_ANGLE_ACTIVATION_PIN.value(0)
        sleep_ms(2)
    sleep_ms(period_ms)


while (BUTTON_FUNCTION_PIN.value() == 1):     
    sequenceLEDs()

print("Stopping LEDs")