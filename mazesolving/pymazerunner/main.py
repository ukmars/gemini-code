"""File first executed by the pico on boot"""
#import gc
import machine

import utime
from math import floor

from ConstantsClasses import *

import ConstantsRobot as cr
import ConstantsMaze as cmaze
import ConstantsMachine as cm

from Button import *
from Motion import Motion
from Sensors import Sensors
from Motors import Motors
from Profile import Profile
from Encoders import Encoders
from Maze import Maze
from BLEModule import *

led = Pin("LED", Pin.OUT)  # Use the built-in LED alias
led.on()

_DEBUG_BUF_SIZE = 256
_debug_buf = bytearray(_DEBUG_BUF_SIZE) # Used to send debug messages over BLE

#Setup global objects for controlling robot
forward = Profile(ScaleType.MM)
rotation = Profile(ScaleType.DEG)
encoders = Encoders(cm.LEFT_MOTOR_A_ENCODER_PIN, cm.RIGHT_MOTOR_A_ENCODER_PIN)
motors = Motors(encoders)
motion = Motion(motors, forward, rotation)
sensors = Sensors()

maze = Maze()

ble = bluetooth.BLE()
BLEP = BLESimplePeripheral(ble)


BLECommands = []


m_handStart = False
sensors.set_steering_mode(SteeringModes.STEERING_OFF)
m_location = 0
m_heading = Heading.NORTH

systick = Timer()

start = 0
encoderTime = 0
motionTime = 0
sensorTime = 0
variablesTime = 0
end = 0
DEBUG = True
LOGFilename = "log.txt"
logfile = None
gcCount = 0

#----------------------------------------------------------------------------------
# Functions for main control
#----------------------------------------------------------------------------------
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

def OpenFile():
    global logfile, LOGFilename
    logfile = open(LOGFilename, 'at')  # Open the file in append mode
    return

def CloseFile():
    global logfile
    if logfile:
        logfile.close()
        logfile = None
    return

loop_counter = 0

def Update(t) -> None : 
    global motors, sensors, encoders, motion, loop_counter

    try :   
        loop_counter += 1
        encoders.update()
        motion.update()
        sensors.update()        
        motors.update_controllers(motion.forward.m_speed, motion.rotation.m_speed, sensors.m_steering_adjustment)   
        if loop_counter >= 50:  # print every 1000 update cycles
            #print("Free RAM:", gc.mem_free()) # type: ignore
            #writeDebug("SA - ", sensors.m_steering_adjustment)
            loop_counter = 0

    except Exception as e:
        print(e)
    return

def UpdateWithDebug(t) -> None : 

    global motors, sensors, encoders, motion, start, encoderTime, motionTime, sensorTime, variablesTime, end, DEBUG, gcCount       

    try :   
        start = utime.ticks_us()

        encoders.update()
        encoderTime = utime.ticks_us()
        motion.update()
        motionTime = utime.ticks_us()
        sensors.update()
        sensorTime = utime.ticks_us()
        
        variablesTime = utime.ticks_us()
        motors.update_controllers(motion.forward.m_speed, motion.rotation.m_speed, sensors.m_steering_adjustment)    
        end = utime.ticks_us()
        gcCount = gc.mem_free() # type: ignore

        if DEBUG :
            print("encoder-Time", utime.ticks_diff(encoderTime, start),
                "motion-Time", utime.ticks_diff(motionTime, encoderTime),
                "sensors-Time", utime.ticks_diff(sensorTime, motionTime),
                "variables-Time", utime.ticks_diff(variablesTime, sensorTime),
                "motors-Time", utime.ticks_diff(end, variablesTime),
                "LeftVolage", motors.m_left_motor_volts,
                "RightVoltage", motors.m_right_motor_volts,
                "gc-count", gcCount)
                                                                                                                                   
        
    except Exception :
        print(Exception)

    return


def Setup() -> None : 
    global motors, sensors, encoders, systick
    motors.stop()
    encoders.reset()
    
    sensors.disable()
    
    systick.init(mode=Timer.PERIODIC, freq=cr.LOOP_FREQUENCY, callback=Update)
    return

def toggle_led(pin_obj):
    """Toggle the state of the given GPIO pin (assumed to control an LED)."""
    pin_obj.value(not pin_obj.value())


##########################################################
#
#       CONTROL Methods - Used to control the robot
#
##########################################################   

'''
* These convenience functions will bring the robot to a halt
* before actually turning.
*
* Note that they do not change the robot's heading. That is
* the responsibility of the caller.
'''

def turn_IP180L() :
    global motion
    # print("turn_IP180L Entry")
    motion.spin_turn(-180, cr.OMEGA_SPIN_TURN, cr.ALPHA_SPIN_TURN)

def turn_IP180R() :
    global motion
    # print("turn_IP180R Entry")
    motion.spin_turn(180, cr.OMEGA_SPIN_TURN, cr.ALPHA_SPIN_TURN)

def turn_IP90R() :
    global motion
    # print("turn_IP90R Entry")
    motion.spin_turn(-90, cr.OMEGA_SPIN_TURN, cr.ALPHA_SPIN_TURN)

def turn_IP90L() :
    global motion
    # print("turn_IP90L Entry")
    motion.spin_turn(90, cr.OMEGA_SPIN_TURN, cr.ALPHA_SPIN_TURN)

'''Search turns
*
* These turns assume that the robot is crossing the cell boundary but is still
* short of the start position of the turn.
*
* The turn will be a smooth, coordinated turn that should finish short of
* the next cell boundary.
*
* Does NOT update the mouse heading but it possibly should
* use the turn_turner app to get some starting guess values and to visuale the offsets being used
*
* TODO: There is only just enough space to get down to turn speed. Increase turn speed?
'''
def turn_smooth(turn_id : int) -> None :
    global m_location, m_heading, sensors, motion

    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    
    frontWallFound = maze.cell_has_wall(m_location,m_heading)

    params = cr.turn_params[turn_id]
    motion.set_target_velocity(params.speed)

    trigger = params.trigger
    if (sensors.see_left_angle_wall) : 
        trigger += cr.EXTRA_WALL_ADJUST

    if (sensors.see_right_angle_wall) :
        trigger += cr.EXTRA_WALL_ADJUST

    positionAtStart = motion.position()

    triggered_by_sensor = False
    
    turn_point = cmaze.FULL_CELL + cmaze.HALF_CELL - params.entry_offset #Turn point is entry_offset away from the cell center
    if frontWallFound : # We know there is a wall so lets use this as the trigger. Should be more accurate
        while (not triggered_by_sensor) :
            if (sensors.get_front_sum() > trigger) :
                motion.set_target_velocity(motion.velocity())                
                triggered_by_sensor = True
                break
    else :
        while (motion.position() < turn_point) :
            if (sensors.get_front_sum() > trigger) :
                motion.set_target_velocity(motion.velocity())
                #TODO should this reset position as might have a better idea of position than the encoders
                triggered_by_sensor = True
                break
            
    positionAtEnd = motion.position()
    
    note = 's' if triggered_by_sensor else 'd'
    dir = 'R' if (turn_id & 1) else 'L'
    writeDebug("ST ", dir, note,"[", positionAtStart,"] [", positionAtEnd,"] fw ",frontWallFound)
    # finally we get to actually turn    
    motion.turn(params.angle, params.omega, 0, params.alpha)
    
    #robot should be at exit offset - run to the sensing position
    
    end_point = cmaze.HALF_CELL + params.exit_offset
    #end_point = params.run_out # Run to sensing point
    motion.move(cr.SENSING_POSITION - end_point, motion.velocity(), params.speed, params.accelleration)
    motion.set_position(cr.SENSING_POSITION)


'''
* bring the mouse to a halt in the center of the current cell. That is,
* the cell it is entering.
'''
def stop_at_center() -> bool : 
    global sensors, motion, maze, m_location, m_heading

    frontWallFound = maze.cell_has_wall(m_location,m_heading)

    maxFrontWallSensor = 0 #If we know there is a front wall the front sensor should only increase but this isn't true
    #So as soon as it starts to drop lets stop

    movefinished = False

    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    remaining = (cmaze.FULL_CELL + cmaze.HALF_CELL) - motion.position()    
    #In theory we are at 170mm of position here
    # finish at very low speed so we can adjust from the wall ahead if present
    acceleration = motion.acceleration()
    writeDebug("stop_at_center ", remaining, " ", motion.velocity(), " ", acceleration, " ", frontWallFound)
    motion.start_move(remaining, motion.velocity(), cr.MIN_SPEED, acceleration)
    if (frontWallFound) :  #front wall
        maxFrontWallSensor = sensors.get_front_sum()
        #I think there is a bug that occurs if the move finishes before we reach the Front Ref and it causes the sensors to turn off
        motion.adjust_forward_position(-45) # Give ourselves 25mm more travel so that the sensor stops us
        while(sensors.get_front_sum() < cr.FRONT_REFERENCE ) :  
            if(motion.move_finished()) :
                movefinished = True
                break
            sleep_ms(1)
    else :
        while (not motion.move_finished()) : 
            sleep_ms(1)

    # Be sure robot has come to a halt.    
    motion.stop()
    motion.reset_drive_system() 

    if(movefinished) : #We are probably in the wall
        motion.move(-(cr.FRONT_WALL_TO_CENTER), cr.MIN_SPEED, 0, cr.SEARCH_ACCELERATION)  
        motion.stop()
        motion.reset_drive_system()      


    gc.collect() # robot has stopped so we can do a garbage collection   
    return frontWallFound

'''
* The robot is already moving so it is enough to let it carry on until
* the next sensing position is reached.
* Subtracting one full cell from the current position tricks the motion
* control into thinking it is at (or just before) the start of a new cell.
* Then it just waits until it gets to the next sensing position.
'''
def move_ahead() -> None :
    global motion
    motion.adjust_forward_position(-cmaze.FULL_CELL)
    motion.wait_until_position(cr.SENSING_POSITION)


#***************************************************************************#
def turn_left(back_into_wall: bool = True) -> None :
    global m_location, m_heading, motion

    xy = maze.cell_xy(m_location)
    heading = ConvertHeadingToCompass(m_heading)
    writeDebug("L cl [", xy[0], ",", xy[1], "]", heading)

    frontwall = stop_at_center()
    turn_IP90L()

    #we are turning left so if the right wall exists we can back into it
    behindUs = right_from(m_heading)
    wallFound = maze.cell_has_wall(m_location,behindUs)

    distance = cr.SENSING_POSITION - cmaze.HALF_CELL

    if back_into_wall and wallFound : # Reset position against the wall behind the new heading.
        xy = maze.cell_xy(m_location)
        heading = ConvertHeadingToCompass(m_heading)
        writeDebug("L Back UP cl ", "[", xy[0], ",", xy[1], "]", heading)
        motion.move(-(cr.BACK_WALL_TO_CENTER+50), cr.SEARCH_SPEED, 0, cr.SEARCH_ACCELERATION)
        distance = distance + cr.BACK_WALL_TO_CENTER
    #else :
        #writeDebug("wf - ", wallFound, " fw - ",frontwall)
    
    motion.move(distance, cr.SEARCH_SPEED, cr.SEARCH_SPEED, cr.SEARCH_ACCELERATION)
    motion.set_position(cr.SENSING_POSITION)
    
    m_heading = left_from(m_heading)


#***************************************************************************#
def turn_right(back_into_wall: bool = True) -> None :
    global m_location, m_heading, motion, maze
    
    xy = maze.cell_xy(m_location)
    heading = ConvertHeadingToCompass(m_heading)
    writeDebug("R cl [", xy[0], ",", xy[1], "]", heading)

    frontwall = stop_at_center()
    turn_IP90R()

    
    #we are turning right so if the left wall exists we can back into it
    behindUs = left_from(m_heading)
    wallFound = maze.cell_has_wall(m_location,behindUs)
    
    #We are going to be in the center of the cell.
    distance = cr.SENSING_POSITION - cmaze.HALF_CELL

    if back_into_wall and wallFound : # Reset position against the wall behind the new heading.
        xy = maze.cell_xy(m_location)
        heading = ConvertHeadingToCompass(m_heading)
        writeDebug("R Back Up cl [", xy[0], ",", xy[1], "]", heading)

        motion.move(-(cr.BACK_WALL_TO_CENTER+50), cr.SEARCH_SPEED, 0, cr.SEARCH_ACCELERATION)
        distance = distance + cr.BACK_WALL_TO_CENTER
    #else :
        #writeDebug("wf - ", wallFound, " fw - ",frontwall)

    
    motion.move(distance, cr.SEARCH_SPEED, cr.SEARCH_SPEED, cr.SEARCH_ACCELERATION)
    motion.set_position(cr.SENSING_POSITION)
    m_heading = right_from(m_heading)

def smooth_turn_left(speedRun = False) -> None :
    global m_location, m_heading, motion
    if speedRun :
        turn_smooth(TurnType.SS90LF)
    else :
        turn_smooth(TurnType.SS90L)
    m_heading = left_from(m_heading)

def smooth_turn_right(speedRun = False) -> None :
    global m_location, m_heading, motion
    if speedRun :
        turn_smooth(TurnType.SS90RF)
    else :
        turn_smooth(TurnType.SS90R)
    m_heading = right_from(m_heading)
#***************************************************************************#
'''/***
* As with all the search turns, this command will be called after the robot has
* reached the search decision point and decided its next move. It is not known
* how long that takes or what the exact position will be.
*
* Turning around is always going to be an in-place operation so it is important
* that the robot is stationary and as well centred as possible.
*
* It only takes 27mm of travel to come to a halt from normal search speed.
*/'''
def turn_back(target : int, inPlaceTurnBack : bool = False) -> None :
    global m_location, m_heading, motion, maze
    
    xy = maze.cell_xy(m_location)
    heading = ConvertHeadingToCompass(m_heading)
    writeDebug("B cl [", xy[0], ",", xy[1], "]", heading)

    leftWallFound, frontWallFound, rightWallFound = maze.GetWallsFromHeading(m_heading, m_location)

    if(not inPlaceTurnBack) :
        frontWallFound = stop_at_center()
    else :
        writeDebug("InPlaceTurnBack")

    maze.flood_for_search(target)

    #If there is a left or right wall we can center off before resetting off the back wall
    writeDebug("TrnB lw", leftWallFound, "fnt", frontWallFound,"rw", rightWallFound)

    def reset_from_side_wall(use_front_sensor: bool) -> None:
        movefinished = False
        motion.start_move(cr.BACK_WALL_TO_CENTER, cr.SEARCH_SPEED, cr.MIN_SPEED, cr.SEARCH_ACCELERATION)
        if use_front_sensor:
            motion.adjust_forward_position(-25) # Give ourselves 25mm more travel so that the sensor stops us
            while(sensors.get_front_sum() < cr.FRONT_REFERENCE ) :
                if(motion.move_finished()) :
                    movefinished = True
                    break
                sleep_ms(2)
        else:
            while (not motion.move_finished()) :
                sleep_ms(2)
        motion.stop()
        motion.reset_drive_system()
        if movefinished:
            writeDebug("TrnB sensor miss")

    if leftWallFound :
        turn_IP90R()
        writeDebug("lWall rst")
        motion.move(-(cr.BACK_WALL_TO_CENTER+50), cr.SEARCH_SPEED, 0, cr.SEARCH_ACCELERATION) #backinto the wall
        reset_from_side_wall(rightWallFound)
        turn_IP90R()
    elif rightWallFound :
        turn_IP90L()
        writeDebug("rWall rst")
        motion.move(-(cr.BACK_WALL_TO_CENTER+50), cr.SEARCH_SPEED, 0, cr.SEARCH_ACCELERATION)
        reset_from_side_wall(leftWallFound)
        turn_IP90L()
    else :
        writeDebug("shouldn't happen - spin")
        turn_IP180L()


    distance = cr.SENSING_POSITION - cmaze.HALF_CELL

    if(frontWallFound) : #Lets reset off the back wall
        #log_action_status('B', 'Back Up', m_location, m_heading)
        motion.move(-(cr.BACK_WALL_TO_CENTER+50), cr.SEARCH_SPEED, 0, cr.SEARCH_ACCELERATION)
        distance = cr.BACK_WALL_TO_CENTER + cmaze.HALF_CELL - (cmaze.FULL_CELL - cr.SENSING_POSITION) 
        motion.stop()
        motion.reset_drive_system()
        
    motion.move(distance, cr.SEARCH_SPEED, cr.SEARCH_SPEED, cr.SEARCH_ACCELERATION)
    motion.set_position(cr.SENSING_POSITION)
    m_heading = behind_from(m_heading)

##########################################################
#
#       HIGH LEVEL ACTIVITY - Used to make the robot do something, Like solve a maze
#
##########################################################

'''* search_to will cause the mouse to move to the given target cell
   * using safe, exploration speeds and turns.
   *
   * During the search, walls will be mapped but only when first seen.
   * A wall will not be changed once it has been mapped.
   *
   * It is possible for the mapping process to make the mouse think it
   * is walled in with no route to the target if wals are falsely
   * identified as present.
   *
   * On entry, the mouse will know its location and heading and
   * will begin by moving forward. The assumption is that the mouse
   * is already facing in an appropriate direction.
   *
   * All paths will start with a straight.
   *
   * If the function is called with handstart set true, you can
   * assume that the mouse is already backed up to the wall behind.
   *
   * Otherwise, the mouse is assumed to be centrally placed in a cell
   * and may be stationary or moving.
   *
   * The walls for the current location are assumed to be correct in
   * the map since mapping is always done by looking ahead into the
   * cell that is about to be entered.
   *
   * On exit, the mouse will be centered in the target cell still
   * facing in the direction it entered that cell. This will
   * always be one of the four cardinal directions NESW
   *'''
def search_to(target: int, closed_maze: bool = False, back_into_wall: bool = True) -> None : 
    """Use search movements with optional closed planning and quarter-turn backing."""
    global m_location, m_heading, m_handStart, maze, motion, sensors, encoders

    # Select the maze view for both flooding and choosing exits.
    maze.set_mask(cmaze.CLOSED_MAZE_MASK if closed_maze else cmaze.OPEN_MAZE_MASK)
    
    sleep_ms(200)
    sensors.enable()
    motion.reset_drive_system()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF) # never steer from zero speed
    writeDebug("Off we go... to [", maze.cell_x(target), ",", maze.cell_y(target), "] pos", motion.position()) 

    if (not m_handStart) :
        # We can assume this is not a start cell. We could be anywhere in the goal cell and we need to reset our position
        # back up to the wall behind
        leftWallFound, frontWallFound, rightWallFound = maze.GetWallsFromHeading(m_heading, m_location)
        writeDebug("search_to not handstart. walls LFR ", leftWallFound, frontWallFound, rightWallFound)
        if (not frontWallFound) :
            if (leftWallFound) :
                turn_IP90R()
                m_heading = right_from(m_heading)
            elif (rightWallFound) :
                turn_IP90L()
                m_heading = left_from(m_heading)
        # We should now have our back to a wall so back into it
            motion.move(-(cr.BACK_WALL_TO_CENTER+120), cr.SEARCH_SPEED, 0, cr.SEARCH_ACCELERATION)
            
            distanceToMove = cr.BACK_WALL_TO_CENTER + cmaze.HALF_CELL - (cmaze.FULL_CELL - cr.SENSING_POSITION) 
            motion.move(distanceToMove, cr.SEARCH_SPEED, cr.SEARCH_SPEED, cr.SEARCH_ACCELERATION)
            motion.set_position(cr.SENSING_POSITION)
            
            m_location = maze.neighbour(m_location,m_heading)
            update_map() 
            turn_back(target)
        else : # front wall was found
            turn_back(target, True) 
    else :
        # Calculate the distance to move to the sensing position
        # If we just move to the center and then set the half position this doesn;t take into account wall thickness
        distanceToMove = cr.BACK_WALL_TO_CENTER + cmaze.HALF_CELL - (cmaze.FULL_CELL - cr.SENSING_POSITION) 
        motion.move(distanceToMove, cr.SEARCH_SPEED, cr.SEARCH_SPEED, cr.SEARCH_ACCELERATION)
        motion.set_position(cr.SENSING_POSITION)
    
    # Each iteration of this loop starts at the sensing point
    while (m_location != target) : #Check to see if we have reached target cell
        if (button_pressed()) :   # allow user to abort gracefully
            break
        toggle_led(cm.MEZ_LED_LEFT) #Toggle the sensor LED
        toggle_led(cm.MEZ_LED_RIGHT)
        sensors.set_steering_mode(SteeringModes.STEER_NORMAL)
        m_location = maze.neighbour(m_location,m_heading)  # the cell we are about to enter
        
        update_map()   
        maze.flood(target) # Respect the open or closed maze view selected above.

        newHeading = maze.direction_to_smallest(m_location, m_heading)

        writeDebug("cl [", maze.cell_x(m_location), ",", maze.cell_y(m_location), "]", ConvertHeadingToCompass(m_heading), "nh", ConvertHeadingToCompass(newHeading))

        hdgChange = (newHeading - m_heading) & 0x3 # This operation effectively keeps only the lowest two bits of the result, ensuring that hdgchange is in the range [0, 3].
        
        if (m_location != target) :
            # each of the following actions will finish with the
            # robot moving and at the sensing point ready for the
            # next loop iteration
            if(hdgChange == Direction.AHEAD) :                 
                move_ahead() #TODO here is where you could go a bit faster if you already know the maze
            elif(hdgChange == Direction.RIGHT) : 
                turn_right(back_into_wall=back_into_wall) #TODO This is where you could smooth turn
            elif(hdgChange == Direction.BACK) : 
                turn_back(target)
            elif(hdgChange == Direction.LEFT) : 
                turn_left(back_into_wall=back_into_wall) #TODO This is where you could smooth turn
    
    # we are entering the target cell so come to an orderly
    # halt in the middle of that cell
    writeDebug("Target Reached. Turn Back")
    stop_at_center()
    sensors.disable()
    writeDebug("Arrived!  ")
    sleep_ms(250)
    motion.stop()
    motion.reset_drive_system()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)

#@timed_function
def update_map() -> None:
    global m_location, m_heading, maze, sensors
    walls = sensors.GetWalls()
    wallValues = sensors.GetWallValues()

    leftWall = walls[0]
    frontWall = walls[1]
    rightWall = walls[2]
    
    writeDebug("\r\n", 
            "L" if leftWall else "-",       "F" if frontWall else "-",             "R" if rightWall else "-", 
            " L[", wallValues[0], "] F[", wallValues[1], "] R[", wallValues[2], "] ", 
            "cl [", maze.cell_x(m_location), ",", maze.cell_y(m_location), "]", ConvertHeadingToCompass(m_heading), " UM")

    cell = m_location
    
    if(m_heading == Heading.NORTH) :              
        maze.update_wall(cell,Heading.NORTH,cmaze.WALL_PRESENT if frontWall else cmaze.WALL_ABSENT)
        maze.update_wall(cell,Heading.EAST,cmaze.WALL_PRESENT if rightWall else cmaze.WALL_ABSENT) 
        maze.update_wall(cell,Heading.WEST,cmaze.WALL_PRESENT if leftWall else cmaze.WALL_ABSENT)
    elif(m_heading == Heading.EAST) :
        maze.update_wall(cell,Heading.EAST,cmaze.WALL_PRESENT if frontWall else cmaze.WALL_ABSENT)
        maze.update_wall(cell,Heading.SOUTH,cmaze.WALL_PRESENT if rightWall else cmaze.WALL_ABSENT)
        maze.update_wall(cell,Heading.NORTH,cmaze.WALL_PRESENT if leftWall else cmaze.WALL_ABSENT)
    elif(m_heading == Heading.SOUTH) :
        maze.update_wall(cell,Heading.SOUTH,cmaze.WALL_PRESENT if frontWall else cmaze.WALL_ABSENT)
        maze.update_wall(cell,Heading.WEST,cmaze.WALL_PRESENT if rightWall else cmaze.WALL_ABSENT)
        maze.update_wall(cell,Heading.EAST,cmaze.WALL_PRESENT if leftWall else cmaze.WALL_ABSENT)
    elif(m_heading == Heading.WEST) :
        maze.update_wall(cell,Heading.WEST,cmaze.WALL_PRESENT if frontWall else cmaze.WALL_ABSENT)
        maze.update_wall(cell,Heading.NORTH,cmaze.WALL_PRESENT if rightWall else cmaze.WALL_ABSENT)
        maze.update_wall(cell,Heading.SOUTH,cmaze.WALL_PRESENT if leftWall else cmaze.WALL_ABSENT)
    else :
        # This is an error. We should handle it.
        writeDebug("UM failed")

    return# wallStatus
        
def turn_to_face(newHeading : int) -> None :
    global m_heading
    
    writeDebug("t2f Ch :", ConvertHeadingToCompass(m_heading), " Nh :", ConvertHeadingToCompass(newHeading))


    hdgChange = (newHeading + Heading.HEADING_COUNT - m_heading) % Heading.HEADING_COUNT
    
    if hdgChange == Direction.BACK : 
        writeDebug("turn_to_face - Turning Back")
        turn_IP180L()
    elif hdgChange == Direction.LEFT : 
        writeDebug("turn_to_face - Turning Left")
        turn_IP90L()
    elif hdgChange == Direction.RIGHT : 
        writeDebug("turn_to_face - Turning Right")
        turn_IP90R()    

    m_heading = newHeading
    writeDebug("t2f n_h ", ConvertHeadingToCompass(newHeading), " Hchg ", ConvertHeadingToCompass(hdgChange))


def search_maze() -> int :
    global m_location, m_heading, m_handStart, maze, motion

    writeDebug("Search TO")
    maze.LoadMazeFromFile(cmaze.mazeSaveFileName)

    maze.flood_for_search(maze.goal_cell)
     
    homeCost = maze.cost[maze.home_cell]

    
    gc.collect()
    m_handStart = True
    m_location = maze.home_cell
    m_heading = Heading.NORTH
    search_to(maze.goal_cell) #Search to goal. Maze flooding has been done and will only be repeated if we reach a deadend and turn around
    maze.SaveMazeDateToFile(cmaze.mazeSaveFileName) #Save the data we have incase we crash on the way home
    
    maze.flood_for_search(maze.home_cell)

    m_handStart = False
    gc.collect()
    search_to(maze.home_cell)
    turn_to_face(Heading.NORTH)
    motion.stop()
    motion.disable_drive()
    maze.SaveMazeDateToFile(cmaze.mazeSaveFileName) #Update our save file. This might give a better route on our next run.
    return 0

def run_maze() -> None :
    global m_location, m_heading, m_handStart, maze, motion

    writeDebug("RUN TO")
    maze.LoadMazeFromFile(cmaze.mazeSaveFileName)

    maze.set_mask(cmaze.CLOSED_MAZE_MASK)
    maze.flood(maze.goal_cell) # We load a maze and see if we have a closed path. We will run a closed path if it exists     

    homeCost = maze.cost[maze.home_cell]

    writeDebug("Home Cost from closed flood = ", homeCost)

    if(homeCost == maze.MAX_COST) : 
        writeDebug("No fast route found. Need to search")
        return
    
    m_handStart = True
    m_location = maze.home_cell
    m_heading = Heading.NORTH
    gc.collect()
    # Use search movements on the known route without backing after quarter turns.
    search_to(maze.goal_cell, closed_maze=True, back_into_wall=False)
    #lets search on the way home just to be sure. 
    maze.flood_for_search(maze.home_cell) 
    
    m_handStart = False
    gc.collect()
    search_to(maze.home_cell)
    turn_to_face(Heading.NORTH)
    motion.stop()
    motion.disable_drive()
    maze.SaveMazeDateToFile(cmaze.mazeSaveFileName) #Update our save file. This might give a better route on our next run.
    gc.collect()
    


def SearchClassicMaze() -> None :
    global m_location, m_heading, m_handStart, maze, motion

    maze = Maze()   
    choice = sensors.wait_for_user_start()

    m_handStart = True
    m_location = maze.home_cell
    m_heading = Heading.NORTH

    if(choice == StartStates.LEFT_START) :
        run_maze()
    else :
        search_maze()

    


    
##########################################################
#
#       TEST METHODS - Used to Test certain aspects of the robot
#
##########################################################

def TestInfinateMove() -> None :
    #Test method to just keep moving
    global sensors, motion, DEBUG
    
    distanceToMove = 50 #mm
    counter = 0
    DEBUG = True
    print("###################################################################################################")
    #sensors.wait_for_user_start()  # cover front sensor with hand to start
    sensors.enable()
    sleep_ms(50)
    motion.reset_drive_system()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    writeDebug("Starting move")
    motion.start_move(distanceToMove, cr.SEARCH_SPEED, cr.SEARCH_SPEED, cr.SEARCH_ACCELERATION)
    #motion.start_move(distanceToMove, cr.FAST_RUN_SPEED_MAX, cr.FAST_RUN_SPEED_MAX, cr.SEARCH_ACCELERATION)
    sleep_ms(50)
    sensors.set_steering_mode(SteeringModes.STEER_NORMAL)
    
    while(not button_pressed()) :    
        frontSum = sensors.get_front_sum()
        frontDiff = sensors.m_front_sum

        sleep_ms(2)

    DEBUG = False
    writeDebug("Move finished")      
    
    motion.disable_drive() 
    motion.reset_drive_system()  
    motion.disable_drive()      
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sensors.disable()
    #gc.collect()

def TestSmoothTurns(fast=False) -> None :    
    global sensors, motion    
    
    distanceToMove = cr.BACK_WALL_TO_CENTER + cmaze.HALF_CELL - (cmaze.FULL_CELL - cr.SENSING_POSITION) # This will be the distance to move the mouse from the back of a cell to the sensing position
    turnDirection = sensors.wait_for_user_start()  # cover front sensor with hand to start

    speed = 0
    acceleration = 0
    if fast :   
        speed = cr.FAST_RUN_SPEED
        acceleration = cr.FAST_RUN_ACCELERATION
    else :  
        speed = cr.SEARCH_TURN_SPEED
        acceleration = cr.SEARCH_ACCELERATION
    
    
    sensors.enable()
    sleep_ms(50)
    motion.reset_drive_system()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    writeDebug("Starting move")
    toggle_led(cm.MEZ_LED_LEFT) 
    toggle_led(cm.MEZ_LED_RIGHT)

    motion.move(distanceToMove, speed, speed, acceleration)
    motion.set_position(cr.SENSING_POSITION)
    
    toggle_led(cm.MEZ_LED_LEFT) 
    toggle_led(cm.MEZ_LED_RIGHT)

    if turnDirection == StartStates.LEFT_START :
        if fast :
            turn_smooth(TurnType.SS90LF)
        else :
            turn_smooth(TurnType.SS90L)
    else :
        if fast :
            turn_smooth(TurnType.SS90RF)
        else :
            turn_smooth(TurnType.SS90R)
    
    toggle_led(cm.MEZ_LED_LEFT) 
    toggle_led(cm.MEZ_LED_RIGHT)
    sensor_left = sensors.LeftAngleSensor.value
    sensor_right = sensors.RightAngleSensor.value
    
    remaining = (cmaze.FULL_CELL + cmaze.HALF_CELL) - motion.position() 
    motion.move(remaining,speed, 0, acceleration)
    toggle_led(cm.MEZ_LED_LEFT) 
    toggle_led(cm.MEZ_LED_RIGHT)

    sensor_left -= sensors.LeftAngleSensor.value
    sensor_right -= sensors.RightAngleSensor.value

    writeDebug("Move finished L [", sensor_left, "]    R [", sensor_right, "]")
    writeDebug("Move finished Dist", motion.motors.encoders.RobotDistanceMM())

    motion.disable_drive() 
    motion.reset_drive_system()  
    motion.disable_drive()      
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sensors.disable()


def TestMoveFoward(dist) -> None :
    global sensors, motion

    distanceToMove = dist #in mm
    counter = 0
    sensors.wait_for_user_start()  # cover front sensor with hand to start
    sensors.enable()
    motion.reset_drive_system()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    
    motion.start_move(distanceToMove, cr.FAST_RUN_SPEED, 0, cr.FAST_RUN_ACCELERATION)
    sensors.set_steering_mode(SteeringModes.STEER_NORMAL)

    while (not motion.move_finished()) :                    
        sleep_ms(1) 

    motion.stop()
    writeDebug("motion finished")        
    motion.disable_drive() 
    writeDebug("Dist Travelled ", motion.motors.encoders.RobotDistanceMM())
    writeDebug("EncoderPulses left ", motion.motors.encoders.LeftPulses, " right ", motion.motors.encoders.RightPulses)

    motion.reset_drive_system()  
    motion.disable_drive()      
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sensors.disable()

def TestMoveFowardWithData(dist) -> None :
    global sensors, motion

    distanceToMove = dist #in mm
    counter = 0
    #sensors.wait_for_user_start()  # cover front sensor with hand to start
    #sensors.enable()
    sleep_ms(100)
    motion.reset_drive_system()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    
    motion.start_move(distanceToMove, cr.FAST_RUN_SPEED, 0, cr.FAST_RUN_ACCELERATION)
    
    #motion.motors.m_controller_output_enabled = False
    voltDataLeft = []
    voltDataRight = []
    rDist = []
    error = []
    while (not motion.move_finished()) :            
        
        counter += 1
        if counter > 400 :
            writeDebug("Counter limit. Stopping")
            break            
        rDist.append(motion.motors.encoders.RobotDistanceMM())
        error.append(motion.motors.m_rot_error)
        voltDataLeft.append(motion.motors.m_left_motor_volts)
        voltDataRight.append(motion.motors.m_right_motor_volts)
        sleep_ms(2)

    writeDebug("motion finished")        
    motion.disable_drive() 
    writeDebug("Dist Travelled ", motion.motors.encoders.RobotDistanceMM())
    writeDebug("EncoderPulses left ", motion.motors.encoders.LeftPulses, " right ", motion.motors.encoders.RightPulses)
    motion.reset_drive_system()  
    motion.disable_drive()      
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sensors.disable()
    writeDebug("--voltDataLeft--")
    writeDebug(voltDataLeft)
    writeDebug("--voltDataRight--")
    writeDebug(voltDataRight)
    writeDebug("--RobotDistance--")
    writeDebug(rDist)
    #print(motion.motors.encoders.RobotDistanceMM())
    writeDebug("--Error--")
    writeDebug(error)
    writeDebug("--Counter--")
    writeDebug(counter)
    #gc.collect()

def TestSpin(angle:int) -> None :
    global sensors, motion

    sensors.enable()
    writeDebug("Spinning ",angle) 
    sleep_ms(500)
    motion.reset_drive_system()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sleep_ms(50)

    motion.spin_turn(angle, cr.OMEGA_SPIN_TURN, cr.ALPHA_SPIN_TURN)

    writeDebug("motion finished")        
    motion.disable_drive() 
    writeDebug("Dist Travelled ", motion.motors.encoders.RobotDistanceMM())
    writeDebug("EncoderPulses left ", motion.motors.encoders.LeftPulses, " right ", motion.motors.encoders.RightPulses)
    motion.reset_drive_system()  
    motion.disable_drive()      
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sensors.disable()

def TestSpinLeft() -> None :
    global sensors, motion

    print("90 degrees Left")
    counter = 0
    #sensors.wait_for_user_start()  # cover front sensor with hand to start
    sensors.enable()
    sleep_ms(500)
    motion.reset_drive_system()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    
    print("Starting turn")
    turn_IP90L()
        
    print("motion finished")        
    motion.disable_drive() 
    writeDebug("Dist Travelled ", motion.motors.encoders.RobotDistanceMM())
    writeDebug("EncoderPulses left ", motion.motors.encoders.LeftPulses, " right ", motion.motors.encoders.RightPulses)
    motion.reset_drive_system()  
    motion.disable_drive()      
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sensors.disable()

def TestSpinRight() -> None :
    global sensors, motion

    print("90 degrees Right")
    counter = 0
    #sensors.wait_for_user_start()  # cover front sensor with hand to start
    sensors.enable()
    sleep_ms(500)
    motion.reset_drive_system()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    
    print("Starting turn")
    turn_IP90R()
        
    print("motion finished")        
    motion.disable_drive() 
    writeDebug("Dist Travelled ", motion.motors.encoders.RobotDistanceMM())
    writeDebug("EncoderPulses left ", motion.motors.encoders.LeftPulses, " right ", motion.motors.encoders.RightPulses)
    motion.reset_drive_system()  
    motion.disable_drive()      
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sensors.disable()

def TestSpin180Right() -> None :
    global sensors, motion

    print("180 degrees Right")
    counter = 0
    #sensors.wait_for_user_start()  # cover front sensor with hand to start
    sensors.enable()
    sleep_ms(500)
    motion.reset_drive_system()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    
    print("Starting turn")
    turn_IP180R()
        
    print("motion finished")        
    motion.disable_drive() 
    writeDebug("Dist Travelled ", motion.motors.encoders.RobotDistanceMM())
    writeDebug("EncoderPulses left ", motion.motors.encoders.LeftPulses, " right ", motion.motors.encoders.RightPulses)
    motion.reset_drive_system()  
    motion.disable_drive()      
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sensors.disable()

def TestSpin180Left() -> None :
    global sensors, motion

    print("180 degrees Left")
    counter = 0
    #sensors.wait_for_user_start()  # cover front sensor with hand to start
    sensors.enable()
    sleep_ms(500)
    motion.reset_drive_system()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    
    print("Starting turn")
    turn_IP180L()
        
    print("motion finished")        
    motion.disable_drive() 
    writeDebug("Dist Travelled ", motion.motors.encoders.RobotDistanceMM())
    writeDebug("EncoderPulses left ", motion.motors.encoders.LeftPulses, " right ", motion.motors.encoders.RightPulses)
    motion.reset_drive_system()  
    motion.disable_drive()      
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sensors.disable()

def TestMazeMove() -> None :
    global m_location, m_heading, m_handStart, maze, motion

    x = 16
    y = 16
    maze = Maze()
    maze.set_goal(maze.cell_id(1,0))
    choice = sensors.wait_for_user_start()

    m_handStart = True
    m_location = maze.home_cell
    m_heading = Heading.NORTH

    if(choice == StartStates.LEFT_START) :
        run_maze()
    else :
        search_maze()
    
    motion.stop()
    motion.disable_drive()    
    
    return

def TestStopAtCenter() -> None : #Robot starts 80MM from start of a cell
    global sensors, motion

    writeDebug("Searching to Center")
    distanceToMove = 100 #in mm   
    
    sleep_ms(100)      

    sensors.enable()
    sleep_ms(500)
    motion.reset_drive_system()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    motion.move(10, cr.SEARCH_SPEED, cr.SEARCH_SPEED, cr.SEARCH_ACCELERATION)

    sensors.set_steering_mode(SteeringModes.STEER_NORMAL)

    #motion.move(40, cr.SEARCH_SPEED, cr.SEARCH_SPEED, cr.SEARCH_ACCELERATION)
    motion.set_position(90)

    motion.wait_until_position(cr.SENSING_POSITION)
    wall = stop_at_center()
        
    writeDebug("motion finished hadwall ", wall)        
    motion.disable_drive() 
    
    motion.reset_drive_system()  
    motion.disable_drive()      
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sensors.disable()
    writeDebug("Should be in center")    

def TestMotorInputVsSpeed(distance_mm=300, pwm=None, volts=None):
    """
    Run both motors at a fixed PWM or voltage for a set distance, logging speed.
    Only one of pwm or volts should be set.
    """
    global motion, sensors

    assert (pwm is not None) != (volts is not None), "Set either pwm or volts, not both"

    sensors.disable()
    motion.reset_drive_system()
    motion.motors.disable_controllers()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)

    # Reset encoders and set start position
    motion.motors.encoders.reset()
    start_pos = motion.motors.encoders.RobotDistanceMM()

    # Set both motors to the same value
    if pwm is not None:
        motion.motors.leftMotor.forward(pwm)
        motion.motors.rightMotor.forward(pwm)
    else:
        # volts is in scaled units, see set_left_motor_volts for scaling
        motion.motors.set_left_motor_volts(volts * cr.PD_SCALE*10)
        motion.motors.set_right_motor_volts(volts * cr.PD_SCALE*10)

    try:
        last_debug = utime.ticks_ms()
        while True:
            curr_pos = motion.motors.encoders.RobotDistanceMM()
            travelled = abs(curr_pos - start_pos)
            speed = motion.motors.encoders.RobotSpeed()  # mm/s
            speedCounts = motion.motors.encoders.RobotSpeedCounts()  # counts/s
            avgCounts = motion.motors.encoders.RobotAverageCounts()  # counts/s

            now = utime.ticks_ms()
            if utime.ticks_diff(now, last_debug) >= 20:
                writeDebug("Dist:", travelled, " SpeedCounts:", speedCounts, " AvgspeedCounts:", avgCounts)
                last_debug = now

            if travelled >= distance_mm:
                break

            sleep_ms(2)
    finally:
        motion.motors.stop()
        motion.reset_drive_system()  
        motion.disable_drive() 
        sensors.disable()
        writeDebug("Test complete. Final dist:", travelled, "mm, Final speed:", speed, "mm/s")


def TestMotorVoltages(dist, speed) :
    global  motion, sensors, DEBUG

    #gc.collect()
    choice = sensors.wait_for_user_start()    
    sensors.enable()
    DEBUG = False
    writeDebug("Running")
    motion.reset_drive_system()
    motion.motors.m_feedforward_enabled = False
    sensors.set_steering_mode(SteeringModes.STEERING_OFF) # never steer from zero speed
        
     
    motion.start_move(dist, speed, 0, cr.FAST_RUN_ACCELERATION)
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)  

    leftPWMMax = 0
    rightPWMMax = 0

    sleep_ms(400) # wait for the motors to start moving

    while (not motion.move_finished()) :  
        if motion.motors.left_pwm > leftPWMMax :
            leftPWMMax = motion.motors.left_pwm
        if motion.motors.right_pwm > rightPWMMax :
            rightPWMMax = motion.motors.right_pwm
        writeDebug("L ", motion.motors.left_pwm, " R ", motion.motors.right_pwm)
        sleep_ms(20) 
    
    motion.stop()
    dist = motion.motors.encoders.RobotDistanceMM()
    writeDebug("Left PWM ", leftPWMMax, " Right PWM ", rightPWMMax ) # pure hacks to get a voltage number based on the scaling 
    writeDebug("Dist Travelled ",dist)
    motion.reset_drive_system()  
    motion.disable_drive()      
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sensors.disable()
    DEBUG = False
    writeDebug("Stoped")
    #gc.collect()

# Drive from back of cell forward x cells and stop at center.
def TestDrive(cells, speed, acc=cr.SEARCH_ACCELERATION, sensor=True) :
    global  motion, sensors, DEBUG

    #gc.collect()
    choice = sensors.wait_for_user_start()    
    sensors.enable()
    DEBUG = False
    writeDebug("Running")
    motion.reset_drive_system()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF) # never steer from zero speed
    
    motion.move(cr.BACK_WALL_TO_CENTER, speed, speed, acc)
    motion.set_position(cmaze.HALF_CELL)
    if(sensor) :
        sensors.set_steering_mode(SteeringModes.STEER_NORMAL)   

    motion.move(((cells)*cmaze.FULL_CELL), speed, 0, acc)
    
    motion.stop()
    dist = motion.motors.encoders.RobotDistanceMM()
    writeDebug("Dist Travelled ",dist)
    writeDebug("Left Encoders ",motion.motors.encoders.LeftPulses, " Right Encoders ", motion.motors.encoders.RightPulses)
    motion.reset_drive_system()  
    motion.disable_drive()      
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sensors.disable()
    DEBUG = False
    writeDebug("Stoped")
    #gc.collect()

def TestDriveXCells(cells, speed, acc=cr.SEARCH_ACCELERATION) :
    global  motion, sensors

    #gc.collect()
    choice = sensors.wait_for_user_start()    
    sensors.enable()
    motion.reset_drive_system()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF) # never steer from zero speed
    
    motion.move(cr.BACK_WALL_TO_CENTER, speed, speed, acc)
    motion.set_position(cmaze.HALF_CELL)
    sensors.set_steering_mode(SteeringModes.STEER_NORMAL)   

    motion.wait_until_position(cr.SENSING_POSITION)


    for x in range(0,cells-1) :        
        move_ahead()       
    
    motion.move(cmaze.HALF_CELL, speed, 0, acc)
    motion.stop()

    dist = motion.motors.encoders.RobotDistanceMM()
    writeDebug("Dist Travelled ", dist)
    motion.reset_drive_system()  
    motion.disable_drive()      
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sensors.disable()
    writeDebug("Stoped")
    #gc.collect()

##########################################################
#
#       CONFIG METHODS - Used to help configure robot
#
##########################################################

def ConfLogFrontSensors() -> None :
    global sensors, motion, encoders

    positions = []
    frontSums = []
    frontDiffs = []

    sensors.enable()
    motion.reset_drive_system()
    sleep_ms(10)

    writeDebug(" dist front_sum front_diff, distance")

    motion.start_move(-200, 100, 100, 500)
    target = -2
    posLast = 0
    while (not motion.move_finished()) :              
        pos = encoders.RobotDistanceMM()
        pos = floor(pos)

        fSum = sensors.get_front_sum()
        fDiff = sensors.get_front_diff()  
        if(pos != posLast) :
            positions.append(pos)
            frontSums.append(fSum)
            frontDiffs.append(fDiff)
            posLast = pos
        
        sleep_ms(1)

    motion.stop()
    motion.reset_drive_system()
    motion.disable_drive()
    sensors.set_steering_mode(SteeringModes.STEERING_OFF)
    sensors.disable()

    writeDebug("START", len(positions), "elements")

    for i in range(len(positions)) :
        writeDebug(positions[i],"  ", frontSums[i],"  ", frontDiffs[i])

        sleep_ms(50)
    
    writeDebug("")
    writeDebug("end")    

def CalibrateFrontSensor() -> None :
    global sensors

    frontLeftRaw = [0]*10
    frontRightRaw = [0]*10
    counter = 0
    sensors.enable()
    sleep_ms(200)    
    writeDebug("Robot should be backed to a wall with a wall in front and no side walls")    

    while counter < 10 :
        frontLeftRaw[counter] = sensors.LeftFrontSensor.raw
        frontRightRaw[counter] = sensors.RightFrontSensor.raw
        counter += 1
        sleep_ms(200)

    sensors.disable()
    # Exclude min and max values    

    frontLeftRaw.sort()
    frontRightRaw.sort()
    frontLeftRawTrimmed = frontLeftRaw[1:-1]  # Exclude the first (min) and last (max) values
    frontRightRawTrimmed = frontRightRaw[1:-1]

    FrontLeftRawAvg = sum(frontLeftRawTrimmed) / len(frontLeftRawTrimmed)
    FrontRightRawAvg = sum(frontRightRawTrimmed) / len(frontRightRawTrimmed)


    FrontLeftRawAvg = sum(frontLeftRaw) / len(frontLeftRaw)
    FrontRightRawAvg = sum(frontRightRaw) / len(frontRightRaw)

    writeDebug("new LEFT_FRONT value [", FrontLeftRawAvg, "]")
    writeDebug("new RIGHT_FRONT value [", FrontRightRawAvg, "]")


    cr.LEFT_FRONT_CALIBRATION = floor(FrontLeftRawAvg)
    cr.RIGHT_FRONT_CALIBRATION = floor(FrontRightRawAvg)
    cr.SaveConfig()
    writeDebug("Values have been saved to config. Reboot to use")

def CalibrateAngleSensor() -> None :
    global sensors

    LeftRaw = [0]*10
    RightRaw = [0]*10
    counter = 0
    sensors.enable()
    sleep_ms(200)    
    writeDebug("Robot should be backed to a wall with side walls and no front wall. The start cell is good")
    
    while counter < 10 :
                
        
        LeftRaw[counter] = sensors.LeftAngleSensor.raw
        RightRaw[counter] = sensors.RightAngleSensor.raw
        counter += 1
        sleep_ms(200)

    sensors.disable()

    LeftRaw.sort()
    RightRaw.sort()
    LeftRawTrimmed = LeftRaw[1:-1]  # Exclude the first (min) and last (max) values
    RightRawTrimmed = RightRaw[1:-1]

    LeftRawAvg = sum(LeftRawTrimmed) / len(LeftRawTrimmed)
    RightRawAvg = sum(RightRawTrimmed) / len(RightRawTrimmed)

    writeDebug("new LEFT value [", LeftRawAvg, "]")
    writeDebug("new RIGHT value [", RightRawAvg, "]")
    
    cr.LEFT_ANGLE_CALIBRATION = floor(LeftRawAvg)
    cr.RIGHT_ANGLE_CALIBRATION = floor(RightRawAvg)
    cr.SaveConfig()
    writeDebug("Values have been saved to config. Reboot to use")


def CalibrateFrontReference() -> None :
    global sensors

    FrontSum = [0]*10
    counter = 0
    sensors.enable()
    sleep_ms(200)    
    writeDebug("Robot should be in the middle of a cell with front wall and no side walls. Use the 22mm spacer")
    

    while counter < 10 :
                
        
        FrontSum[counter] = sensors.get_front_sum()
        counter += 1
        sleep_ms(100)

    sensors.disable()

    FrontSum.sort()
    frontsumTrimmed = FrontSum[1:-1]  # Exclude the first (min) and last (max) values
    FrontSumAvg = sum(frontsumTrimmed) / len(frontsumTrimmed)

    writeDebug("new Front Reference value [", FrontSumAvg, "]")
    
    cr.FRONT_REFERENCE = floor(FrontSumAvg)
    cr.SaveConfig()
    writeDebug("Value have been saved to config. Reboot to use")

def CalibrateTurnThreshold() -> None :
    global sensors

    FrontSum = [0]*10
    counter = 0
    sensors.enable()
    sleep_ms(200)    
    writeDebug("Robot should be 92mm away from a front wall and no side walls")
    

    while counter < 10 :
                
        
        FrontSum[counter] = sensors.get_front_sum()
        counter += 1
        sleep_ms(200)

    sensors.disable()
    FrontSum.sort()
    frontsumTrimmed = FrontSum[1:-1]  # Exclude the first (min) and last (max) values
    FrontSumAvg = sum(frontsumTrimmed) / len(frontsumTrimmed)

    writeDebug("new Turn Threshold value [", FrontSumAvg, "]")
    
    cr.TURN_THRESHOLD_SS90E = floor(FrontSumAvg)
    cr.SaveConfig()
    writeDebug("Value have been saved to config. Reboot to use")

def GetSensorAverageSnapShot() -> None :
    global sensors

    FrontSum = [0]*10
    FrontDiff = [0]*10
    Left = [0]*10
    Right = [0]*10
    counter = 0
    sensors.enable()
    sleep_ms(200)    
    writeDebug("Getting value of all sensors")
    

    while counter < 10 :
                
        
        FrontSum[counter] = sensors.get_front_sum()
        FrontDiff[counter] = sensors.get_front_diff()
        Left[counter] = sensors.LeftAngleSensor.value
        Right[counter] = sensors.RightAngleSensor.value
        counter += 1
        sleep_ms(100)

    sensors.disable()
    FrontSumAvg = sum(FrontSum) / len(FrontSum)
    FrontDiffAvg = sum(FrontDiff) / len(FrontDiff)
    LeftAvg = sum(Left) / len(Left)
    RightAvg = sum(Right) / len(Right)

    writeDebug("Front Sum value [", FrontSumAvg, "]")
    writeDebug("Front Diff value [", FrontDiffAvg, "]")
    writeDebug("Left value [", LeftAvg, "]")
    writeDebug("Right value [", RightAvg, "]")

    

def PrintRobotConstants() -> None :

    writeDebug("JSON values pulled loaded into python file")
    writeDebug("FRONT_REFERENCE", cr.FRONT_REFERENCE)
    writeDebug("LEFT_ANGLE_CALIBRATION", cr.LEFT_ANGLE_CALIBRATION)
    writeDebug("RIGHT_ANGLE_CALIBRATION", cr.RIGHT_ANGLE_CALIBRATION)
    writeDebug("LEFT_FRONT_CALIBRATION", cr.LEFT_FRONT_CALIBRATION)
    writeDebug("RIGHT_FRONT_CALIBRATION", cr.RIGHT_FRONT_CALIBRATION)
    writeDebug("")
    writeDebug("JSON dump", cr.PrintJSON())

    

def StreamSensorData(loopSensors : bool = True) -> None : 
    global sensors
    
    sensors.enable()
    sleep_ms(200)
    while(not button_pressed()) :
        walls = sensors.GetWalls()
        wallValues = sensors.GetWallValues()
        wallRawValues = sensors.GetWallRawValues()
        wallAmbientValues = sensors.GetWallAmbientValues()

        leftWall = walls[0]
        frontWall = walls[1]
        rightWall = walls[2]

        # Build wall label buffer safely
        writeDebug("----")

        writeDebug("L" if leftWall else "-","F" if frontWall else "-","R" if rightWall else "-",
            "L[", wallValues[0], "] F[", wallValues[1], "] R[", wallValues[2], "] ")

        writeDebug("L[", wallRawValues[0], "] FL[", wallRawValues[1], "]  FR[", wallRawValues[2], "] R[", wallRawValues[3], "]")

        writeDebug("La[", wallAmbientValues[0], "] FLa[", wallAmbientValues[1], "]  FRa[", wallAmbientValues[2], "] Ra[", wallAmbientValues[3], "]")

        writeDebug("fs =", sensors.m_front_sum, "fd =", sensors.m_front_diff)

        writeDebug("----")


        if(not loopSensors) : 
            break
        sleep_ms(1000)
    
    sensors.disable()

def ResetMazeToBlank(confirmed=False) -> None :
    global maze
    writeDebug("Resetting Maze to blank")
    # BLE keeps its existing button prompt; the menu confirms before calling.
    if not confirmed:
        writeDebug("Press button and release to continue or reset mouse")
        while not button_pressed():
            sleep_ms(10)

    #maze.LoadMazeFromFile(cmaze.blankMazeFileName)
    maze = Maze()

    maze.SaveMazeDateToFile(cmaze.mazeSaveFileName)

    writeDebug("Maze has been reset")

##########################################################
#
#       CONFIG METHODS - END
#
##########################################################

##########################################################
#
#       Mouse OS METHODS - These are the methods that work out what commands to send to the robot
#
##########################################################


def writeDebug(*args):
    '''
    This should be a heap-safe debug function that can be called from anywhere.
    It will send the debug message to the console and over BLE if connected.
    You should not use .format() as it will allocate memory.
    '''
    global _debug_buf
    idx = 0
    # Iterate over each argument
    for i, arg in enumerate(args):
        s = str(arg)  # Convert argument to string
        for c in s:
            if idx < _DEBUG_BUF_SIZE - 2:  # reserve space for \r\n
                _debug_buf[idx] = ord(c)
                idx += 1
            else:
                break  # stop if buffer full
        # Add a space if not the last argument
        if i < len(args) - 1 and idx < _DEBUG_BUF_SIZE - 2:
            _debug_buf[idx] = ord(' ')
            idx += 1
    # Append CRLF at the end if space permits
    if idx < _DEBUG_BUF_SIZE - 2:
        _debug_buf[idx] = 13  # '\r'
        _debug_buf[idx + 1] = 10  # '\n'
        idx += 2

    # Output to the USB/console
    print(*args)

    # Send over BLE if connected. 
    if BLEP.is_connected():
        # Slice the buffer up to the current index (no extra allocation)
        BLEP.send(_debug_buf[:idx])



def ConvertHeadingToCompass (heading : int) -> str :
    hdg_letters = "NESW"

    hdg = ""
    if (heading < 4) :
        hdg = hdg_letters[heading]
    else :
        hdg = "!"

    return hdg     

#Setup main control loop
def RunFunction(mode : int, maze_reset_confirmed=False) -> None : # Get the mouse to do something 
    cm.MEZ_LED_LEFT.off()  # Menu ready light is off while a function runs.
    writeDebug("running function ", mode)

     
    if mode == 0 :
        return
    elif mode == 1 :
        choice = sensors.wait_for_user_start()
        if(StartStates.LEFT_START == choice) :
            writeDebug("Left Start")
        elif(StartStates.RIGHT_START == choice) :
            writeDebug("Right Start")
        else :
            writeDebug("This shouldn't have happened")
        return
    elif mode == 2 :
        TestMazeMove()
        return
    elif mode == 3 :
        TestSpin180Right()
        return
    elif mode == 4 :
        TestSpin180Left()
        return
    elif mode == 5 :
        TestSpinRight()
        return
    elif mode == 6 :
        TestSpinLeft()
        return
    elif mode == 7 :
        TestMoveFoward(300)
        return
    elif mode == 8 :
        TestStopAtCenter()
        return
    elif mode == 9 :   
        ConfLogFrontSensors()     
        return
    elif mode == 10 :
        CalibrateFrontSensor()
        return
    elif mode == 11 :
        CalibrateAngleSensor()
        return
    elif mode == 12 :
        CalibrateFrontReference()
        return
    elif mode == 13 :
        PrintRobotConstants()
        return
    elif mode == 14 :
        SearchClassicMaze()
        return
    elif mode == 15 :
        GetSensorAverageSnapShot()
        return
    elif mode == 16 :
        return
    elif mode == 17 :
        StreamSensorData(False)
        return
    elif mode == 18 :
        ResetMazeToBlank(confirmed=maze_reset_confirmed)
        return    
    elif mode == 19 :
        TestInfinateMove()        
        return
    elif mode == 20 :
        
        return
    elif mode == 21 :
        TestSmoothTurns()
    elif mode == 22 :
        CalibrateTurnThreshold()
    elif mode == 23 :
        TestSmoothTurns(True)
    else :
        #TODO some sort of error as this should be unreachable
        return

def BLERecieve(data): #This function runs when we get data from bluetooth
        print("RX", data)
        BLECommands.append(data) #Push the recieved data from bluetooth to an array so we can look at it later / when we want to
        

BLEP.on_write(BLERecieve)

'''Process the BLE commands list and execute what we find #todo for now we will only process the first item send via BlueTooth'''
def ProcessBLE() -> None : 
    c = ''
    digits = -99
    secondDigits = -99
    thirdDigits = -99
    splits = []
       
    try :
        print(BLECommands)
        cmd = BLECommands.pop()

        # we get unwanted string data from the BLE console so it has to be removed before we start. Probably a cleaner way to do this.
        cmd = cmd.decode() 
        
        cmd = cmd.strip()
        cmd = cmd.lower()        
        splits = cmd.split()  

        splitCount = len(splits)

        print("CMD Split count =", splitCount)              

        if(splitCount == 4) :            
            c = splits[0]
            digits = splits[1]
            secondDigits = splits[2]
            thirdDigits = splits[3]
        elif(splitCount == 3) :            
            c = splits[0]
            digits = splits[1]
            secondDigits = splits[2]
        elif(splitCount == 2) :
            c = splits[0]
            digits = splits[1]
        else :            
            c = splits[0]             
    except :
        writeDebug("Exception occured trying to decode BLE data")
    
    if(c == '?') :
        help()
    elif(c == 'f') :
        RunFunction(int(digits))
    elif(c == 'pd') :
        writeDebug("FWD_KP value [", cr.FWD_KP / cr.PD_SCALE, "]    FWD_KD value [", cr.FWD_KD / cr.PD_SCALE, "]")
        writeDebug("ROT_KP value [", cr.ROT_KP / cr.PD_SCALE, "]    ROT_KD value [", cr.ROT_KD / cr.PD_SCALE, "]")
        writeDebug("STEERING_KP value [", cr.STEERING_KP / cr.PD_SCALE, "]    STEERING_KD value [", cr.STEERING_KD / cr.PD_SCALE, "]")
    elif(c == 'fp') :
        writeDebug("Updating FWD_KP value to [", digits, "]")
        cr.FWD_KP = float(digits) * cr.PD_SCALE    
        cr.SaveConfig()
        writeDebug("Value have been saved to config. Reboot to use")              
    elif(c == 'fd') :
        writeDebug("Updating FWD_KD value to [", digits, "]")
        cr.FWD_KD = float(digits)  * cr.PD_SCALE      
        cr.SaveConfig()
        writeDebug("Value have been saved to config. Reboot to use")              
    elif(c == 'rp'):
        writeDebug("Updating ROT_KP value to [", digits, "]")
        cr.ROT_KP = float(digits) * cr.PD_SCALE         
        cr.SaveConfig()
        writeDebug("Value have been saved to config. Reboot to use") 
    elif(c == 'rd'):
        writeDebug("Updating ROT_KD value to [", digits, "]")
        cr.ROT_KD = float(digits) * cr.PD_SCALE         
        cr.SaveConfig()
        writeDebug("Value have been saved to config. Reboot to use")              
    elif(c == 'sd') :
        writeDebug("Updating STEERING_KD value to [", digits, "]")
        cr.STEERING_KD = float(digits) * cr.PD_SCALE         
        cr.SaveConfig()
        writeDebug("Value have been saved to config. Reboot to use")              
    elif(c == 'sp') :
        writeDebug("Updating STEERING_KP value to [", digits, "]")
        cr.STEERING_KP = float(digits) * cr.PD_SCALE       
        cr.SaveConfig()
        writeDebug("Value have been saved to config. Reboot to use")
    elif(c == 's'):
        StreamSensorData()
    elif(c == 'sspd') :            
        cr.SEARCH_SPEED = int(digits)
        cr.SaveConfig()
        writeDebug("Search Speed has been updated")
        writeDebug("Value have been saved to config. Reboot to use")
    elif(c == 'fspd') :    
        cr.FAST_RUN_SPEED = int(digits)
        cr.SaveConfig()
        writeDebug("Fast Speed has been updated")
        writeDebug("Value have been saved to config. Reboot to use")
    elif(c == 'tspd') :    
        cr.SEARCH_TURN_SPEED = int(digits)
        cr.SaveConfig()
        writeDebug("Turn Speed has been updated")
        writeDebug("Value have been saved to config. Reboot to use")
    elif(c == 'run') :
        TestDrive(int(digits), int(secondDigits), int(thirdDigits))
    elif(c == 'rs') :
        TestDrive(int(digits), int(secondDigits), int(thirdDigits),False) # Run with no sensor
    elif(c == 'runcells') :
        TestDriveXCells(int(digits), int(secondDigits), int(thirdDigits))
    elif(c == 'mrun') :
        TestMotorVoltages(int(digits), int(secondDigits))
    elif(c == 'vrun') :
        TestMotorInputVsSpeed(int(digits), volts=int(secondDigits)) # Run motors at a fixed voltage for a set distance
    elif(c == 'fwd') :
        TestMoveFoward(int(digits))
    elif(c == 'spin') :    
        TestSpin(int(digits))
    else :
        writeDebug("Invalid command")

    return

def help() -> None : 
    writeDebug("?   : this text" \
    "\nfwd   : Move forward X mm" \
    "\nspin  : Turn X degrees" \
    "\nS     : show sensor readings")
    
    writeDebug("\nPD  : show PD Numbers" \
    "\nFP n   : Update FWD KP value with given number" \
    "\nFD n   : Update FWP KD value with given number" \
    "\nRP n   : Update ROT KP value with given number" \
    "\nRD n   : Update ROT KD value with given number" \
    "\nSP n   : Update Steering KP value with given number" \
    "\nSD n   : Update Steering KD value with given number")
    
    writeDebug(" ")

    writeDebug("fsdp  : Set Fast Speed" \
    "\nsspd  : Set Search Speed" \
    "\ntspd  : Set Turn Speed")

    writeDebug("run   : Go at x cells @ Y speed & Z Acc. stop center" \
    "\nrs   : Go at x cells @ Y speed & Z Acc. stop center with no steer" \
    "\nmrun x y : run x dist @ Y speed. Report voltages") 

    writeDebug("F n : Run user function n" \
    "\n       1 = Test Hand start" \
    "\n       2 = Test Maze Search" \
    "\n       3 = TestSpin180Right" \
    "\n       4 = TestSpin180Left" \
    "\n       5 = TestSpinRight" \
    "\n       6 = TestSpinLeft" \
    "\n       7 = TestMoveFoward 300mm" \
    "\n       8 = Stop At Center")

    writeDebug("       9 = LogFrontSensors" \
    "\n       10 = Calibrate Front" \
    "\n       11 = Calibrate Angle" \
    "\n       12 = Calibrate Front Ref" \
    "\n       13 = Print Constants" \
    "\n       14 = Search Classic Maze" \
    "\n       15 = Sensor Snapshot")   
    writeDebug("       16 = " \
    "\n       17 = Take 1 sensor read" \
    "\n       18 = Reset Stored Maze" \
    "\n       19 = Never ending move forward" \
    "\n       20 = Wall Follow" \
    "\n       21 = Smooth Turn Test" \
    "\n       22 = Config Turn Threshold" \
    "\n       23 = Smooth Turn Test FAST" \
    "\nEnd")   

    return

def ReadyLight()  -> None : 
    toggle_led(cm.MEZ_LED_LEFT)    
    sleep_ms(200)
    toggle_led(cm.MEZ_LED_LEFT)
    sleep_ms(200)
    toggle_led(cm.MEZ_LED_LEFT)
    sleep_ms(250)
    toggle_led(cm.MEZ_LED_LEFT)

# Local menu settings: raw left-wheel encoder counts, separate from drive tuning.
# 12 counts * 50 gearbox ratio / 8 = 75 counts per nominal eighth revolution.
# Check the actual feel on the robot. Increase the threshold for slower selection.
MENU_COUNTS_PER_STEP = 75
MENU_ENCODER_DIRECTION = 1  # Change to -1 if forward rotation selects backwards.
MENU_RESET_TIMEOUT_MS = 5000


def ShowMenu(selection, running=False):
    # Sensor LEDs read left to right as binary bits 4, 2, 1.
    cm.SENSOR_LED_LEFT.value(bool(selection & 4))
    cm.SENSOR_LED_CENTRE.value(bool(selection & 2))
    cm.SENSOR_LED_RIGHT.value(bool(selection & 1))
    cm.MEZ_LED_LEFT.value(not running)
    cm.MEZ_LED_RIGHT.value(running)


def UpdateMenuSelection(selection, movement):
    # Signed counts accumulate across polls. Reversing cancels partial movement.
    while movement >= MENU_COUNTS_PER_STEP:
        selection = min(7, selection + 1)
        movement -= MENU_COUNTS_PER_STEP
    while movement <= -MENU_COUNTS_PER_STEP:
        selection = max(0, selection - 1)
        movement += MENU_COUNTS_PER_STEP
    # Do not build up movement beyond either end of the menu.
    if (selection == 0 and movement < 0) or (selection == 7 and movement > 0):
        movement = 0
    return selection, movement


def ConfirmMenuReset():
    writeDebug("Reset maze: click function button again within 5 seconds; turn left wheel to cancel")
    start = utime.ticks_ms()
    wheel_start = encoders.LeftPulses
    while utime.ticks_diff(utime.ticks_ms(), start) < MENU_RESET_TIMEOUT_MS:
        if encoders.LeftPulses != wheel_start:
            break
        cm.MEZ_LED_RIGHT.value((utime.ticks_diff(utime.ticks_ms(), start) // 250) % 2)
        if button_pressed():
            sleep_ms(30)
            if button_pressed():
                wait_for_button_release()
                sleep_ms(30)
                return (encoders.LeftPulses == wheel_start and
                        utime.ticks_diff(utime.ticks_ms(), start) < MENU_RESET_TIMEOUT_MS)
        sleep_ms(50)
    return False


def RunMenuFunction(selection):
    # Add a RunFunction(BLE_function_number) call in any unused branch below.
    # For a new action, also add its branch in RunFunction and update README.md.
    if selection == 0:
        return  # Idle.
    elif selection == 1:
        RunFunction(2)  # Test maze search; retains the sensor hand-start.
    elif selection == 2:
        RunFunction(14)  # Classic maze search; retains the sensor hand-start.
    elif selection == 3:
        ShowMenu(3)
        if ConfirmMenuReset():
            ShowMenu(0, running=True)
            RunFunction(18, maze_reset_confirmed=True)
        else:
            writeDebug("Maze reset cancelled")
    elif selection == 4:
        pass  # Add the function call for menu slot 4 here.
    elif selection == 5:
        pass  # Add the function call for menu slot 5 here.
    elif selection == 6:
        pass  # Add the function call for menu slot 6 here.
    elif selection == 7:
        pass  # Add the function call for menu slot 7 here.


def main() -> None :
    sleep_ms(100)
    if(not button_pressed()):
        Setup()
        gc.enable()
        #gc.collect()               
                
        help()
        ReadyLight()
        motion.disable_drive()
        motors.stop()
        selection = 0
        movement = 0
        previous_count = encoders.LeftPulses
        ShowMenu(selection)

        while True:
            count = encoders.LeftPulses
            movement += (count - previous_count) * MENU_ENCODER_DIRECTION
            previous_count = count
            selection, movement = UpdateMenuSelection(selection, movement)
            ShowMenu(selection)

            # Poll only while idle. Use a debounced click, then execute on release.
            clicked = False
            if button_pressed():
                sleep_ms(30)
                if button_pressed():
                    wait_for_button_release()
                    sleep_ms(30)
                    clicked = True

            if BLECommands or clicked:
                ShowMenu(0, running=True)
                try:
                    if BLECommands:
                        ProcessBLE()  # BLE takes priority if both inputs arrive.
                    else:
                        RunMenuFunction(selection)
                finally:
                    # Stop outputs before returning to wheel selection, even on error.
                    motion.disable_drive()
                    motors.stop()
                    wait_for_button_release()
                    sleep_ms(30)
                    selection = 0
                    movement = 0
                    # Discard wheel movement and encoder resets during the function.
                    previous_count = encoders.LeftPulses
                    ShowMenu(selection)
            sleep_ms(50)

    else :
        writeDebug("Mouse not running due to button press at startup OR USB Power is connected")

if __name__ == "__main__":    
    
    try :
        main()
    except Exception :
        print(Exception)
