from utime import sleep_us, sleep_ms

import ConstantsMachine as cm
from Encoders import Encoders

encoders = Encoders(cm.LEFT_MOTOR_A_ENCODER_PIN, cm.RIGHT_MOTOR_A_ENCODER_PIN)



print("running")
while True:
    encoders.get_counts()
    print(encoders.counts[0], "   ", encoders.counts[1])
    print(encoders.LeftPulses, "   ", encoders.RightPulses)
    print("---")
    sleep_ms(20)