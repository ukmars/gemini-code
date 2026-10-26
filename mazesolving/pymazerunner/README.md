# Gemini MicroPython starting code base

Here is a starting code base that has been built and tested for the UKMARS Gemini maze solving robot. This code is heavily based on the UKMARS Mazerunner-core codebase with differences only be a result of MicroPython or RP2040 capabilities. 

This is intended for readers comfortable reading and modifying Python. Expect to verify wiring, measure your robot, and calibrate it before attempting a maze.
This codebase assumes that the build guide has been followed and that you are using 50:1 motors. If you have used other components then you will need to make suitable changes.

This code is provided as a starting point in the readers maze solving journey. You are expected to go through the code to understand how and what it is doing so that you can change it to make your robot perform better.
This code is proven to get a Gemini robot into the centre of a competition 16x16 maze and return it to the start cell. 

Tested on a Pico (RP2040) running Micropython v1.25.0

# Tools

This codebase uses multiple files that all need to exists on the pico in order to run. While it is possible to use Thonny it is NOT recommended for this code base. The recommended approach is to use VSCode and the extension MicroPico. This extension allows for auto deployment of all files onto the device as well as deploying single files and execution of files while the pico is connected.

A lot of the codes functions are activated via BLE commands sent to the robot. It is recommended to use an Android phone or tablet for this using the "Serial Bluetooth Terminal" app. This will allow you to connect to the robot and issue commands as well as receive telemetry while the robot is running.


# Code overview

There are files that start with ***Testxxx.py*** these files are designed to be executed through the IDE with the robot connected.

All other files are all linked in some way with ***main.py*** being the starting point. When all files are deployed to the Pico the main.py will be executed first. This can cause you problems if you have deployed the code but want to run test files or do other debugging as the robot will go through its start-up when power is applied. If you do not want the robot to boot press and left button for 5 seconds during power up. This will stop main.py from starting the robot code loops. 

The file names of each of the .py files should give a clear indication to their purpose. As best as possible each function of the robot has been put into its own file.

Important files that should be looked at are:

- ConstantsRobot.py - This contains many values used through the code base with some critical real world physical values that you will need to take from your robot. 
	- **WHEEL_DIAMETER** - needs to be measured and confirmed
	- **MOUSE_DIAMETER** - needs to be measured and confirmed
	- **ENCODER_PULSES** - needs to be confirmed
	- **GEAR_RATIO** - needs to be confirmed
	- **LEFT/RIGHT/FRONT_THRESHOLD** - will need to be checked and validated as you test the robot.

- ConstantsRobot.json - This contains values that can be updated via the BLE interface and might be values you change during a contest. This is where calibrated values are stored for the current wall configuration.
- ConstantsMachine.py - This contains all of the pin mappings. If you have built a default Gemini then this should not need changed but you should validate the pins.
- main.py - The core code file of the robot. This is the file that is loaded by the pico when it gets power. The system control loop is configured and once executed the robot will sit and wait for a command from BLE or via the wheel menu. This file contains the high level functions of the robot like turning and maze solving along with a number of test and calibration functions.


# Getting Started

This assumes you have built the robot. Before you get started with trying to solve a maze you must confirm basic functionality of the robot. 

My approach to starting would be to go through the following Testxxx.py files and confirm that the results match your expectation.

1. TestLEDs.py - Execute this and you should see all of the LED's on the robot cycle through. Including the sensor LED's. Each LED is lit in turn before the next so you should see each LED light once. The only exception is that both side sensor LED will fire at once. This will repeat until you press the left button
2. TestEncoders2.py - Execute and manually turn each wheel. You should see the numbers increase and decrease as expected. Numbers will increase when you move the wheel in the forward position and decrease when reversed. It is possible to attach the A/B leads the wrong way round when soldering.
3. TestMotor.py - This test file does not need any additional configuration so check the pin numbers. This will cycle the motors forward and backward repeating until you press the left button. The expected direction will be printed. Ensure that your robot is supports in some way so that the wheels can rotate freely. 
4. TestMotorDirections.py - This confirms that the ConstantsMachine.py file has the correct mappings. This stops without the need for any button pressing
5. TestSensors.py - This will show sensor readings repeating until you press the left button. This will allow you to confirm that the sensors are returning values you manually move a wall towards and away from each of the 3 sensors. If your ambient and raw reading are always similar then it is possible that the photo transistor has been installed backward. 
6. TestProfileForward.py - Doesn't need to be run but shows you how the profile system works for a given command. This can be helpful to understand how the robot carries out its motion.

If test files 1-5 all work as you would expect you can then move on to more integrated tests.

Once you move to more integrated tests I would start with

- Using the BLE interface first send a '?' (without quotes) to see a full list of available functions

- Confirming the sensor threshold for wall detection. When the robot hits the sensing point you want to be very sure that you correctly detect a wall
	- Via the BLE interface you can use the 's' command to see current sensor reading. Robot must not be moving for this to work
- Confirming spin turns, confirm that when you spin 90 degrees left and right that it is 90 degrees
	- Via the BLE interface you can use the 'spin XX' or 'f 5' or 'f 6' commands to spin the robot. Robot must not be moving for this to work
- Confirm distances. Does the robot accurately drive forward 100mm, 300mm 1000mm within a suitable error margin
	- Via the BLE interface you can use the 'f 7' or 'fwd xxx' command to move the robot forward. Robot must not be moving for this to work

All of these test routines already exists in the code.

# How does the robot work

There is the control loop that is running all of the time. The control loop runs at 100hz. 

The control loop is responsible for moving the robot both forward and rotationally.

Then there is the high level code that is working out what to do and commanding new moves for the robot to conduct.

When the robot is commanded to do a move. The profile calculates a trapezoidal profile that will use the given speeds and acceleration to know what speed the robot should be doing at any given time. The control loop is then going to try and get the robot to that given speed. How quickly the robot adjust to the error in the expected position vs its currently known actual position will determine how accurate the robot is.

With Gemini it is only the wheel encoders that allow the robot to know how far it has driven. These are used to give the known position.

When searching. You start at the known 0,0 point in the maze backed up into the wall. The robot is commanded to move to the sensing point which is at the 170mm mark (the very first move assumes robot at the back of cell so moves forward 128mm to get the 170mm mark). The search loop will then take a wall reading and update the maze. The maze is then flooded and the next move is determined. The robot will either go straight, turn left or turn right each time moving to the sensing point. This is repeated until the goal is reached. 
Once the goal is reached the robot will return home

When the robot reaches the goal or back home. The current maze that has been detected is saved to the file system. This is stored in the CurrentMaze.txt file. You can read this to see how the robot views the maze and to see if you have correctly read the walls. 

Reset the maze to clear this using the inbuilt command. 

# Robot calibrations for the maze

There are 3 elements the code has built in to calibrate the sensors for a given maze. Once done the settings are saved to the JSON file and will survive reboots BUT if you update your robot from VSCode you can overwrite these settings if you haven't captured them.

Via the BLE interface.

- f 10 - Calibrates the front sensor. The robot needs to be placed at the back of a cell with a wall in front of it and no side walls. Run this command and let it finish. Reboot the robot once done.
- f 11 - Calibrates the angle sensor. The robot needs to be placed at the back of a cell with no wall in front and side wall on the left and right. Robot must be central. Reboot robot once done
- f 12 - Robot must be placed in the centre of the cell (the wheel are on the centre line). Technically this has the front of the robot 22mm from the wall. Reboot once complete

Once these are calibrated you should be able to search a maze. This assumes that the walls are consistent. You can definitely think of ways to make this more accurate. 

The robot has 2 maze search configurations built in.  Via the BLE interface there is 'f 2' Test maze Search and 'f 14' Search classic maze. The only difference between these 2 functions is the goal cell the robot is aiming for. In the test maze the robot will aim for cell 0,1, this is the cell to the right of the start maze and makes it easy to test at home. 'f 14' has a goal set to 8,8 for when you are at a contest. Both settings assume a 16x16 maze so that you are always flooding a full size maze.

# Thinking points

- The sensor LED are not ON all the time. They are being pulsed once per system control cycle (100hz).
- The sensors are used for two things. Steering while moving forward and for sensing walls. Wall sensing is only done at the sensing point.
- Steering is only enabled when the robot is moving forward. Never while turning and never when stopped.
- It is possible for the robot to headbutt a wall and then back up. It is possible with the sensor board for the front sensor readings to start to reduce as you get close to a wall. This can happen at the exact point the robot tried to detect it is in the centre of a cell. 


## Wheel menu

BLE commands remain available. With the robot idle, turn the left wheel to
select a function, then press and release the function button to execute it.
Hold the robot so the wheel can turn freely. Menu input is suspended while a
function runs; wheel movement during that function is discarded afterwards.

The three sensor LEDs show a binary number, read left to right as **4, 2, 1**.

| Selection | Left / centre / right | Function | BLE equivalent |
| --- | --- | --- | --- |
| 0 | off / off / off | Idle | — |
| 1 | off / off / on | Test maze search | `f 2` |
| 2 | off / on / off | Classic maze search | `f 14` |
| 3 | off / on / on | Reset saved maze | `f 18` |
| 4 | on / off / off | Unused | — |
| 5 | on / off / on | Unused | — |
| 6 | on / on / off | Unused | — |
| 7 | on / on / on | Unused | — |

The left mezzanine LED means the menu is ready. The right LED flashes while
waiting for reset confirmation. Click the function button a second time within
five seconds to reset; turning the left wheel or waiting cancels. BLE reset
retains its existing button prompt.

On dispatch, the right mezzanine LED lights and the menu LEDs clear. Search
functions then take control of the LEDs and retain their existing sensor
hand-start gesture (left selects run, right selects search). The function button
retains its existing search-abort role. After execution or cancellation, the menu
returns to 0. Selecting an unused slot also returns to 0 without running anything.
Selection stops at 0 and 7 without wrapping. Holding the function button at boot
still prevents normal startup.

### Adjusting or extending the menu

In `main.py`, `MENU_COUNTS_PER_STEP` sets how many raw left encoder counts make
one selection step. Its starting value is 75, approximately an eighth revolution
using the repository's nominal 12 counts and 50:1 gearing. Actual sensitivity
needs checking on the robot. Increase it for more wheel movement per step.
Set `MENU_ENCODER_DIRECTION` to `-1` if forward wheel rotation decreases selection.
These settings do not change drive calibration.

To add an existing BLE action, replace a `pass` in `RunMenuFunction` with
`RunFunction(number)`, using its BLE function number, and update the table above.
For a new action, also add its function and corresponding branch in `RunFunction`.

### Robot validation

Check forward/backward selection, all eight LED patterns, sensitivity, and both
menu endpoints. Check that slots 4–7 do nothing, that both searches retain their
hand-start and abort behaviour, and that the menu returns to 0 afterwards without
counting movement during the search. Check BLE operation as well.

For maze reset, first back up the saved maze. Check wheel cancellation and timeout
before confirming a reset, then verify the saved maze is blank. Motors should
remain stopped throughout selection and confirmation.


# Python course

Link to a Python course run by Harvard
https://cs50.harvard.edu/python/
