from machine import Timer
from utime import sleep


import ConstantsRobot as cr
import ConstantsMachine as cm
from Sensors import Sensors
import time
from Button import *
import gc

sensors = Sensors()
systick = Timer()

cm.SENSOR_LED_LEFT.off()

print("Start reading Sensor value. Press and hold left button to stop")

def Update(t) -> None : #This is called as part of the system tick rate and called every 2ms
    global sensors
    sensors.update()

systick.init(mode=Timer.PERIODIC, freq=250, callback=Update)    
sensors.disable()
sleep(1)
startTime = time.ticks_ms()
sensors.enable()

sensors.m_active = True
while (not button_pressed()):     

    walls = sensors.GetWalls()
    wallValues = sensors.GetWallValues()
    wallRawValues = sensors.GetWallRawValues()
    wallAmbientValues = sensors.GetWallAmbientValues() # Have to be carful here as this returns 4 values. We just ignore the right front

    leftWall = walls[0]
    frontWall = walls[1]
    rightWall = walls[2]

    print("-------------------")
    print("Left Ambient[", wallAmbientValues[0], "] Front Ambient[", wallAmbientValues[1], "]  Right Ambient[", wallAmbientValues[3], "]")
    
    print("L" if leftWall else "-","F" if frontWall else "-","R" if rightWall else "-",
    "Left norm[", wallValues[0], "] Font norm[", wallValues[1], "] Right norm[", wallValues[2], "] ")

    print("Left raw[", wallRawValues[0], "] Front raw[", wallRawValues[1], "]  Right raw[", wallRawValues[3], "]")
  
    print("------------------ ",gc.mem_free())
    

    sleep(0.5)                                                    
    
sensors.disable()

systick.deinit()
print("End of test")




    
    