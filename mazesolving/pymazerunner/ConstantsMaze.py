# Maze specific constants

''' 
 * For the classic micromouse contest, the maze is 16x16 cells. That is
 * convenient because a single byte can be used to store the cost associated
 * with each cell when performing a simple flood.
 *
 * If you extend the code to cope with a half-size maze of up to 32x32 cells
 * you will need to make changes to the way the cost is stored.
 *
 * A more sophisticated method for flooding the maze might also require larger
 * values for cost and changes will be needed. the AVR has very limited RAM
 * so the simplest option is used here.
'''
MAZE_WIDTH = 16
MAZE_HEIGHT = 16
MAZE_CELL_COUNT = (MAZE_WIDTH * MAZE_HEIGHT)
MAX_COST = (MAZE_CELL_COUNT - 1)



WALL_ABSENT = 0
WALL_PRESENT = 1
WALL_UNKNOWN = 2
WALL_VIRTUAL = 3
WALL_MASK = 3

ALL_UNKNOWN = 0b10101010

CLOSED_MAZE_MASK = 3
OPEN_MAZE_MASK = 1

TURN_COST = 2

# ways we can represent the maze as a string
VIEW_PLAIN = 0
VIEW_COSTS = 1
VIEW_DIRS = 2

"""
 * Directions are absolute and are not relative to any particular heading
 * these are maze things
"""
DIR_NORTH = 0
DIR_EAST = 1
DIR_SOUTH = 2
DIR_WEST = 3
DIR_COUNT = 4
DIR_BLOCKED = -1

# maze specific settings

FULL_CELL = 180
HALF_CELL = 90


blankMazeFileName = "blankMaze.txt"


mazeSaveFileName = "CurrentMaze.txt"