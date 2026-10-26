# a place for most of the custom classes that are used
import array

class SensorChannel :

    def __init__(self) -> None :
        self.raw = 0
        self.value = 0
        self.ambient = 0

class SteeringModes :
    STEER_NORMAL = 0
    STEER_LEFT_WALL = 1
    STEER_RIGHT_WALL = 2
    STEERING_OFF = 3

class ScaleType :
    MM = 0
    DEG = 1

class MazeView :
    PLAIN = 0
    COSTS = 1
    DIRS = 2

class States :
    IDLE = 0
    ACCELERATING = 1
    BRAKING = 2
    FINISHED = 3

class StartStates : 
    NO_START = 0
    LEFT_START = 1
    RIGHT_START = 2

class TurnType :
    SS90L = 0 # Search speed 90 degree left turn
    SS90R = 1 # Search speed 90 degree right turn
    SS90LF = 2 # Fast search speed 90 degree left turn
    SS90RF = 3 # Fast search speed 90 degree right turn

class Heading : 
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3
    HEADING_COUNT = 4
    BLOCKED = 99

def right_from(heading : int) -> int :
    return ((heading + 1) % Heading.HEADING_COUNT)


def left_from(heading : int) -> int :
    return ((heading + Heading.HEADING_COUNT - 1) % Heading.HEADING_COUNT)

def behind_from(heading : int) -> int :
    return ((heading + 2) % Heading.HEADING_COUNT)


'''
 * Directions are relative to the robot's orientation. They do not refer to
 * any particular heading and are just used to make code more readable where
 * decisions are made about which way to turn.
 '''

class Direction :
    AHEAD = 0
    RIGHT = 1
    BACK = 2
    LEFT = 3
    DIRECTION_COUNT = 4
 
class DirectionShortHane :
    F = 0
    R = 1
    B = 2
    L = 3
    DIRECTION_COUNT = 4
 

''' * Structure definitions used in the software. Declared here for lack of a
 * better place because the structure is needed by the robot configs.
 *
 * Robot specific instances and values are in the robot config file.
  speed;    // mm/s    - constant forward speed during turn
  accelleration; // mm/s/s - maximum accelleration during turn
  entry_offset; // mm  - distance from cell center to turn start
  exit_offset;  // mm  - distance from cell center to turn end
  angle;    // deg     - total turn angle
  omega;    // deg/s   - maximum angular velocity
  alpha;    // deg/s/s - angular acceleration
  trigger;  //         - front sensor value at start of turn
 *
 * TODO: redefine these in terms of offsets from cell centre
 * TODO: replace trigger value with sensor identifier
''' 
class TurnParameters :
    def __init__(self, speed : int, accelleration : int, entry_offset : int, exit_offset : int, angle : int, omega : int, alpha : int, trigger : int) -> None :
        self.speed = speed
        self.accelleration = accelleration
        self.entry_offset = entry_offset
        self.exit_offset = exit_offset
        self.angle = angle
        self.omega = omega
        self.alpha = alpha
        self.trigger = trigger

'''
 * Location stores a position in the maze as an (x,y) coordinate pair(zero index). This is
 * probably a more intuitive representation than storing just a single index
 * into and array. The method takes a little more code to manage than historical
 * methods but they were devised for processors with very little program memory.
 *
 *
 * Locations have a number of supporting operations collected into the struct so
 * that you don't have to keep re-writing the same bits of code.
 '''
class Location :
    #TODO this class has a big reliance on the overall maze size. Need to find a way to have this set once for each run rather than keep passing it around from the maze object
    #TODO type hinting fails for methods that return an instance of theselves. 
 
    def __init__(self, x: int=0, y: int=0, mazeWidth : int=16, mazeHeight : int=16) :
        self.x = x
        self.y = y
        self.mazeWidth = mazeWidth
        self.mazeHeight = mazeHeight

    def is_in_maze(self) -> bool :
        return self.x < self.mazeWidth and self.y < self.mazeHeight
    
    
    

    def operatorEqual(self, obj) -> bool :
         return self.x == obj.x and self.y == obj.y
    

    def operatorNotEqual(self, obj) -> bool :
         return self.x != obj.x or self.y != obj.y
    

    # these operators prevent the user from exceeding the bounds of the maze
    # by wrapping to the opposite edge
    def north(self) : #-> Location :
        return Location(self.x, (self.y + 1) % self.mazeHeight)   

    def east(self) : #-> Location :
        return Location((self.x + 1) % self.mazeWidth, self.y)
    

    def south(self) : #-> Location :
        return Location(self.x, (self.y + self.mazeHeight - 1) % self.mazeHeight)
    

    def west(self) : #-> Location :
        return Location((self.x + self.mazeWidth - 1) % self.mazeWidth, self.y)
    

    def neighbour(self, heading : int)  -> 'Location'  : #-> Location :
        
        if heading == Heading.NORTH :
            return self.north()
            
        elif heading == Heading.EAST:
            return self.east()
            
        elif heading == Heading.SOUTH:
            return self.south()
            
        elif heading == Heading.WEST:
            return self.west()
            
        else :
            return Location()  # TODO this is actually an error and should be handled
        
    def toArray(self)  :
        data = array.array('i', (0 for _ in range(4)))
        data[0] = self.x
        data[1] = self.y
        data[2] = self.mazeWidth
        data[3] = self.mazeHeight

        return data            

class WallState :
    EXIT = 0    # a wall that has been seen and confirmed absent
    WALL = 1    # a wall that has been seen and confirmed present
    UNKNOWN = 2 # a wall that has not yet been seen
    VIRTUAL = 3 # a wall that has not yet been seen

# class WallInfo : #probably not needed anymore
#     def __init__(self):
#         self.north = WallState.UNKNOWN
#         self.east = WallState.UNKNOWN
#         self.south = WallState.UNKNOWN
#         self.west = WallState.UNKNOWN            