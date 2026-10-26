# maze.py
# Maze code for MicroPython
# Copyright (c) 2024 Peter Harrison
# Contributions from Paul Busby and David Hannaford
# Released under the MIT License (https://opensource.org/licenses/MIT)


#################################################################################
# this bit of stuff lets us use the performance-enhancing features of MicroPython
# if they are available. With it the flood is twice as fast.
# It is Python magic and you do not have to understand it to use it
try:
    import micropython
except ImportError:
    # Define a mock 'micropython' module with a no-op 'native' decorator
    import types
    micropython = types.SimpleNamespace(native=lambda f: f)
################################################################################

from ConstantsClasses import *

WALL_ABSENT = 0
WALL_PRESENT = 1
WALL_UNKNOWN = 2
WALL_VIRTUAL = 3
WALL_MASK = 3

ALL_UNKNOWN = 0b10101010

CLOSED_MAZE_MASK = 3
OPEN_MAZE_MASK = 1

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

DIR_AHEAD = 0
DIR_RIGHT = 1
DIR_BACK = 2
DIR_LEFT = 3
DIR_COUNT = 4

class Maze:    
    def __init__(self, size=16):
        self.size = size
        self._cell_xy_buf = [0, 0]  # preallocated reusable x, y
        self.MAX_COST = self.size * self.size
        self.cost = [self.MAX_COST for _ in range(self.size * self.size)]
        self.walls = [ALL_UNKNOWN for _ in range(self.size * self.size)]
        self.mask = OPEN_MAZE_MASK
        self.goal_cell = self.cell_id(8, 7)
        self.home_cell = self.cell_id(0, 0)
        self.init_walls()

    def __str__(self) -> str:
        return self.get_maze_string(VIEW_PLAIN)

    # TODO: add a __repr__ method that will prodiuce a Python declaration?

    def cell_id(self, x, y):
        return y + x * self.size

    def cell_xy(self, cell):
        '''This call is somewhat GC safe
        Not thread-safe and do not use the values returned after calling cell_xy() again, since it's overwritten
        '''
        self._cell_xy_buf[0] = cell // 16
        self._cell_xy_buf[1] = cell % 16
        return self._cell_xy_buf
        #return (cell // self.size, cell % self.size)
    
    def cell_x(self, cell_id):
        '''This call is completely GC safe'''
        return cell_id // self.size

    def cell_y(self, cell_id):
        '''This call is completely GC safe'''
        return cell_id % self.size


    def set_goal(self, cell):
        self.goal_cell = cell

    def init_walls(self):
        """
        Initialise the maze walls to all unknown
        Set the borders as walls
        Mark the start cell walls
        Does not set the goal area
        """
        self.walls = [ALL_UNKNOWN for _ in range(self.size * self.size)]
        for cell in range(self.size):
            self.set_wall(self.cell_id(cell, 0), DIR_SOUTH, WALL_PRESENT)
            self.set_wall(self.cell_id(cell, self.size-1),
                          DIR_NORTH, WALL_PRESENT)
            self.set_wall(self.cell_id(0, cell), DIR_WEST,  WALL_PRESENT)
            self.set_wall(self.cell_id(self.size-1, cell),
                          DIR_EAST, WALL_PRESENT)
        self.set_wall(0, DIR_EAST, WALL_PRESENT)
        self.set_wall(0, DIR_NORTH, WALL_ABSENT)

    def init_walls_from_string(self, lines):
        """
        Read a maze stored as a list of lines of text. The format is
        the same as the output of get_string and the mazefiles available at
        https://github.com/micromouseonline/mazefiles
        """
        ROW_DIVISOR = 2
        COL_DIVISOR = 4
        self.size = max(len(lines) // ROW_DIVISOR, len(lines[0]) // COL_DIVISOR)
        self.walls = [ALL_UNKNOWN for _ in range(self.size * self.size)]
        cell_y = self.size - 1
        for i, line in enumerate(lines):
            if i >= ROW_DIVISOR * self.size:
                continue
            line = line.rstrip()  # remove \n

            if i % ROW_DIVISOR == 0:  # north walls
                cell_x = 0
                for j in range(2, len(line), COL_DIVISOR):
                    c = line[j]
                    if c == '-' :
                        wall_state = WALL_PRESENT
                    elif c == '.' :
                        wall_state = WALL_UNKNOWN
                    else :
                       wall_state = WALL_ABSENT
                    self.update_wall(self.cell_id(cell_x, cell_y), DIR_NORTH, wall_state)
                    cell_x += 1

            else:  # west walls
                cell_x = 0
                for j in range(0, len(line), COL_DIVISOR):
                    c = line[j]
                    if c == '|' :
                        wall_state = WALL_PRESENT
                    elif c == ':' :
                        wall_state = WALL_UNKNOWN
                    else :
                       wall_state = WALL_ABSENT
                    self.update_wall(self.cell_id(cell_x, cell_y), DIR_WEST, wall_state)
                    cell_x += 1
                wall_state = WALL_PRESENT if line[-1] == '|' else WALL_ABSENT
                self.update_wall(self.cell_id(self.size - 1, cell_y), DIR_EAST, wall_state)

            cell_y -= i % ROW_DIVISOR

        # special case for the last line
        cell_y = 0
        cell_x = 0
        for j in range(2, len(line), COL_DIVISOR):
            c = line[j]
            wall_state = WALL_PRESENT if c == '-' else WALL_ABSENT
            self.set_wall(self.cell_id(cell_x, cell_y), DIR_SOUTH, wall_state)
            cell_x += 1
        return

    def set_wall(self, cell, direction, state):
        """
        Unconditionally set the state of a single wall in the maze
        This would normally only get used when initialising the maze - either
        as a blank maze or when copying a pre-defined maze
        """
        if cell < 0 or cell >= self.size * self.size:
            return
        x = self.cell_x(cell) 
        y = self.cell_y(cell)
        wall = state << direction * 2
        mask = ~(WALL_MASK << direction * 2)
        self.walls[cell] &= mask
        self.walls[cell] |= wall
        if direction == DIR_NORTH and y < self.size - 1:
            next = self.neighbour(cell, DIR_NORTH)
            self.walls[next] &= ~(WALL_MASK << DIR_SOUTH * 2)
            self.walls[next] |= (state << DIR_SOUTH * 2)
        elif direction == DIR_EAST and x < self.size - 1:
            next = self.neighbour(cell, DIR_EAST)
            self.walls[next] &= ~(WALL_MASK << DIR_WEST * 2)
            self.walls[next] |= (state << DIR_WEST * 2)
        elif direction == DIR_SOUTH and y > 0:
            next = self.neighbour(cell, DIR_SOUTH)
            self.walls[next] &= ~(WALL_MASK << DIR_NORTH * 2)
            self.walls[next] |= (state << DIR_NORTH * 2)
        elif direction == DIR_WEST and x > 0:
            next = self.neighbour(cell, DIR_WEST)
            self.walls[next] &= ~(WALL_MASK << DIR_EAST * 2)
            self.walls[next] |= (state << DIR_EAST * 2)

    def update_wall(self, cell, direction, state):
        """
        Use this to update the maze while exploring. 
        It will ensure that wall state can only be changed once.
        If you absolutely, positively have to set the wall state, use set_wall.
        """
        if cell < 0 or cell >= self.size * self.size:
            return
        this_wall = (self.walls[cell] >> direction * 2) & WALL_MASK
        if this_wall != WALL_UNKNOWN:
            return
        self.set_wall(cell, direction, state)

    def set_mask(self, mask):
        """
        The mask determines the view of the maze
        The default is OPEN_MAZE_MASK which assumes unknown walls are absent
        The CLOSED_MAZE_MASK assumes unknown walls are present

        By using these two masks, you can view the maze as 'open' or 'closed'
        Flood using the 'open' view while exploring to find a way to the goal
        Flood using the 'closed' view to find a guaranteed safe route to the goal
        """
        self.mask = mask

    def cell_has_wall(self, cell, direction) -> bool:
        """
        Returns True if there is a wall in the given cell and direction
        Uses the current mask to to view the maze as 'closed' or 'open'
        The 'closed' maze assumes unknown walls are present
        The 'open' maze assumes unknown walls are absent
        """
        wall = self.walls[cell] >> direction * 2
        return wall & self.mask == WALL_PRESENT
        

    def cell_has_exit(self, cell, direction):
        """
        Returns True if there is an exit in the given cell and direction
        Uses the current mask to to view the maze as 'closed' or 'open'
        The 'closed' maze assumes unknown walls are present
        The 'open' maze assumes unknown walls are absent
        """
        wall = self.walls[cell] >> direction * 2
        return wall & self.mask == WALL_ABSENT

    def cell_is_visited(self, cell):
        """
        Returns True if the cell has been visited
        A cell is considered visited if all of the walls are known.
        You do not actually have to go there, just see all the walls
        """
        return self.walls[cell] & ALL_UNKNOWN == 0

    def neighbour(self, cell : int, direction : int) -> int:
        """
        Calculate and return the offset of a neighbouring cell.
        The maze is assumed to be square and the returned value will
        wrap around the edges of the maze
        There is no error checking for the direction
        """
        neighbour = cell
        if direction == DIR_NORTH:
            neighbour = cell + 1
        elif direction == DIR_EAST:
            neighbour = cell + self.size
        elif direction == DIR_SOUTH:
            neighbour = cell + self.size*self.size - 1
        elif direction == DIR_WEST:
            neighbour = cell + self.size * self.size - self.size
        return neighbour % (self.size * self.size)


    def GetWallsFromHeading(self, heading : int, location : int) -> tuple[bool,bool,bool] :
    
        frontWallFound = self.cell_has_wall(location,heading)
        leftWallFound = False
        rightWallFound = False
        
        if(heading == Heading.NORTH) :
            leftWallFound = self.cell_has_wall(location,Heading.WEST)
            rightWallFound = self.cell_has_wall(location,Heading.EAST)
        elif (heading == Heading.EAST) :
            leftWallFound = self.cell_has_wall(location,Heading.NORTH)
            rightWallFound = self.cell_has_wall(location,Heading.SOUTH)        
        elif (heading == Heading.SOUTH) :
            leftWallFound = self.cell_has_wall(location,Heading.EAST)
            rightWallFound = self.cell_has_wall(location,Heading.WEST)
        elif (heading == Heading.WEST) :
            leftWallFound = self.cell_has_wall(location,Heading.SOUTH)
            rightWallFound = self.cell_has_wall(location,Heading.NORTH)

        return leftWallFound, frontWallFound, rightWallFound

    def direction_to_smallest(self, cell :int, start_direction: int =DIR_NORTH) -> int:
        """
        Return the direction of the smallest neighbouring cell.
        Testing is done in the order ahead, left, right, back.
        Provide a start direction if desired 
        """
        dir = start_direction
        cost = self.MAX_COST + 1
        if self.cell_has_exit(cell, dir):
            ahead_cost = self.cost[self.neighbour(cell, dir)]
            if ahead_cost < cost:
                cost = ahead_cost
        left_dir = ((dir + DIR_COUNT - 1) % DIR_COUNT)
        if self.cell_has_exit(cell, left_dir):
            left_cost = self.cost[self.neighbour(cell, left_dir)]
            if left_cost < cost:
                dir = left_dir
                cost = left_cost
        right_dir = (dir + 1) % DIR_COUNT
        if self.cell_has_exit(cell, right_dir):
            right_cost = self.cost[self.neighbour(cell, right_dir)]
            if right_cost < cost:
                dir = right_dir
                cost = right_cost
        back_dir = (dir + 2) % DIR_COUNT
        if self.cell_has_exit(cell, back_dir):
            back_cost = self.cost[self.neighbour(cell, back_dir)]
            if back_cost < cost:
                dir = back_dir
                cost = back_cost

        return dir

    def get_maze_string(self, view=VIEW_PLAIN) -> str:
        """
        Print a visual representation of the maze
        If view == VIEW_COSTS, print the cost of each cell
        """
        old_mask = self.mask
        self.mask = CLOSED_MAZE_MASK
        str = ""
        for y in range(self.size - 1, -1, -1):
            line = "o"
            for x in range(self.size):
                if self.cell_has_exit(self.cell_id(x, y), DIR_NORTH):
                    line += "   o"
                elif self.cell_has_wall(self.cell_id(x, y), DIR_NORTH):
                    line += "---o"
                else:
                    line += " . o"
            str += line + "\n"
            line = ""
            for x in range(self.size):
                cell = self.cell_id(x, y)
                if self.cell_has_exit(cell, DIR_WEST):
                    line += " "
                elif self.cell_has_wall(cell, DIR_WEST):
                    line += "|"
                else:
                    line += ":"
                # now the cell space
                if cell == 0 and view == VIEW_PLAIN:
                    line += "   "
                elif view == VIEW_COSTS and self.cost[cell] is not None:
                    line += f"{self.cost[cell]:>3}"
                else:
                    line += "   "
            if self.cell_has_exit(self.cell_id(self.size-1, y), DIR_EAST):
                line += " "
            else:
                line += "|"  # Rightmost boundary
            str += line + "\n"
        line = "o"
        for x in range(self.size):
            if self.cell_has_exit(self.cell_id(x, 0), DIR_SOUTH):
                line += "   o"
            else:
                line += "---o"
        # print(line)
        str += line + "\n"
        self.mask = old_mask
        return str
    
    def get_maze_string_w_location(self, location = 0) -> str:
        """
        Print a visual representation of the maze with a marker showing the mouse position
        """
        old_mask = self.mask
        self.mask = CLOSED_MAZE_MASK
        str = ""
        for y in range(self.size - 1, -1, -1):
            line = "o"
            for x in range(self.size):
                if self.cell_has_exit(self.cell_id(x, y), DIR_NORTH):
                    line += "   o"
                elif self.cell_has_wall(self.cell_id(x, y), DIR_NORTH):
                    line += "---o"
                else:
                    line += " . o"
            str += line + "\n"
            line = ""
            for x in range(self.size):
                cell = self.cell_id(x, y)
                if self.cell_has_exit(cell, DIR_WEST):
                    line += " "
                elif self.cell_has_wall(cell, DIR_WEST):
                    line += "|"
                else:
                    line += ":"
                # now the cell space
                if cell == location :
                    line += " X "
                else:
                    line += "   "
            if self.cell_has_exit(self.cell_id(self.size-1, y), DIR_EAST):
                line += " "
            else:
                line += "|"  # Rightmost boundary
            str += line + "\n"
        line = "o"
        for x in range(self.size):
            if self.cell_has_exit(self.cell_id(x, 0), DIR_SOUTH):
                line += "   o"
            else:
                line += "---o"
        # print(line)
        str += line + "\n"
        self.mask = old_mask
        return str
    
    def get_maze_string_w_route(self) -> str:
        """
        Print a visual representation of the maze with a marker showing the mouse route
        """

        # Follow the existing flood costs; no motion route is generated.
        # The caller must flood towards goal_cell using the desired maze view.
        if self.cost[self.goal_cell] != 0 or self.cost[self.home_cell] == self.MAX_COST:
            raise ValueError("Flood towards a reachable goal before displaying the route")
        route = {}
        cell = self.home_cell
        heading = DIR_NORTH
        while cell != self.goal_cell:
            next_cell = None
            for direction in (heading, (heading - 1) % DIR_COUNT,
                              (heading + 1) % DIR_COUNT, (heading + 2) % DIR_COUNT):
                neighbour = self.neighbour(cell, direction)
                if (self.cell_has_exit(cell, direction) and
                        self.cost[neighbour] == self.cost[cell] - 1):
                    route[cell] = direction
                    next_cell = neighbour
                    heading = direction
                    break
            if next_cell is None:
                raise ValueError("Flood costs do not describe a route to the goal")
            cell = next_cell

        dir_chars = "^>v<* "

        old_mask = self.mask
        self.mask = CLOSED_MAZE_MASK
        str = ""
        for y in range(self.size - 1, -1, -1):
            line = "o"
            for x in range(self.size):
                if self.cell_has_exit(self.cell_id(x, y), DIR_NORTH):
                    line += "   o"
                elif self.cell_has_wall(self.cell_id(x, y), DIR_NORTH):
                    line += "---o"
                else:
                    line += " . o"
            str += line + "\n"
            line = ""
            for x in range(self.size):
                cell = self.cell_id(x, y)
                if self.cell_has_exit(cell, DIR_WEST):
                    line += " "
                elif self.cell_has_wall(cell, DIR_WEST):
                    line += "|"
                else:
                    line += ":"
                # now the cell space                
                if cell in route:
                    line += f" {dir_chars[route[cell]]} "
                else:
                    line += "   "
            if self.cell_has_exit(self.cell_id(self.size-1, y), DIR_EAST):
                line += " "
            else:
                line += "|"  # Rightmost boundary
            str += line + "\n"
        line = "o"
        for x in range(self.size):
            if self.cell_has_exit(self.cell_id(x, 0), DIR_SOUTH):
                line += "   o"
            else:
                line += "---o"
        # print(line)
        str += line + "\n"
        self.mask = old_mask
        return str


    @micropython.native
    def flood(self, target_cell : int):
        """
        Fill cost with the minimum number of cell moves to target_cell.
        Each move costs one; turns have no additional cost.
        Uses the current mask: open permits unknown exits, closed permits
        only known exits. flood_for_search() temporarily selects the open view.

        For better performance, the flood method makes use of the intimate knowledge it has
        about the maze. 

        For example, calculating the neighbor does not use the built-in method because it
        is expensive and too general purpose. Instead, a short, inline version is used which
        relies on the maze edges having known walls.

        This is not ideal in terms of maintenance but speed is king.
        """
        MASK = WALL_MASK & self.mask
        NORTH_MASK = MASK << DIR_NORTH * 2
        EAST_MASK = MASK << DIR_EAST * 2
        SOUTH_MASK = MASK << DIR_SOUTH * 2
        WEST_MASK = MASK << DIR_WEST * 2
        MAX_COST = self.MAX_COST

        self.cost = [MAX_COST for _ in range(self.size * self.size)]
        self.cost[target_cell] = 0
        head = 0
        tail = 0
        queue = [0 for _ in range(self.size * self.size)]
        queue[tail] = target_cell
        tail += 1
        while head < tail:
            here = queue[head]
            head += 1
            walls_here = self.walls[here]
            next_cost = self.cost[here] + 1

            if walls_here & NORTH_MASK == 0:
                neighbour = here + 1
                if self.cost[neighbour] == MAX_COST:
                    self.cost[neighbour] = next_cost
                    queue[tail] = neighbour
                    tail += 1

            if walls_here & EAST_MASK == 0:
                neighbour = here + self.size
                if self.cost[neighbour] == MAX_COST:
                    self.cost[neighbour] = next_cost
                    queue[tail] = neighbour
                    tail += 1

            if walls_here & SOUTH_MASK == 0:
                neighbour = here - 1
                if self.cost[neighbour] == MAX_COST:
                    self.cost[neighbour] = next_cost
                    queue[tail] = neighbour
                    tail += 1

            if walls_here & WEST_MASK == 0:
                neighbour = here - self.size
                if self.cost[neighbour] == MAX_COST:
                    self.cost[neighbour] = next_cost
                    queue[tail] = neighbour
                    tail += 1

        return self.cost[0]


    def flood_for_search(self, target_cell):
        mask = self.mask
        self.mask = OPEN_MAZE_MASK
        cost = self.flood(target_cell)
        self.mask = mask
        return cost


    def speed_run_possible(self) -> bool:
        """
        Return True when a reachable shortest path is fully known.
        Compare ordinary open and closed floods from the goal to home.
        Restore the caller's mask; costs remain from the closed flood.
        """
        mask = self.mask
        try:
            self.mask = OPEN_MAZE_MASK
            search_cost = self.flood(self.goal_cell)
            self.mask = CLOSED_MAZE_MASK
            known_cost = self.flood(self.goal_cell)
            return known_cost < self.MAX_COST and search_cost == known_cost
        finally:
            self.mask = mask
    
   
    def SaveMazeDateToFile(self, filename : str) -> None:
        """
        Save the maze to a txt file."""
        mazeString = self.get_maze_string(VIEW_PLAIN)
        with open(filename, 'w') as f:
            f.write(mazeString)
    

    #TODO currenlty no processing is used for maze files that may contain the start and fininsh cell
    def LoadMazeFromFile(self, filename : str) -> None :
        """
        Load a maze from a txt file. 
        NOTE: This does not work with files that contain markers for the start and finish cell
        """
        with open(filename, 'r') as f:            
            lines = f.readlines()
            self.init_walls_from_string(lines)


if __name__ == "__main__":
    """
    Here is some code that will run the flood multiple times and display
    the time taken. None of this is needed by the mouse 
    """

    import time
    import os
    import sys

    def millis():
        if sys.implementation.name == 'micropython':
            # MicroPython
            return time.ticks_ms()
        else:
            # Desktop Python
            return int(round(time.time() * 1000))

    def iterations():
        if sys.implementation.name == 'micropython':
            return 1
        else:
            return 1000

    # Example usage
    maze = Maze()
    # set up a standard maze that is mostly empty
    maze.init_walls()
    maze.update_wall(maze.cell_id(7, 7), DIR_WEST, WALL_PRESENT)
    maze.update_wall(maze.cell_id(7, 7), DIR_SOUTH, WALL_PRESENT)
    maze.update_wall(maze.cell_id(8, 7), DIR_SOUTH, WALL_PRESENT)
    maze.update_wall(maze.cell_id(8, 7), DIR_EAST, WALL_PRESENT)
    maze.update_wall(maze.cell_id(8, 8), DIR_EAST, WALL_ABSENT)
    maze.update_wall(maze.cell_id(8, 8), DIR_NORTH, WALL_PRESENT)
    maze.update_wall(maze.cell_id(7, 8), DIR_NORTH, WALL_PRESENT)
    maze.update_wall(maze.cell_id(7, 8), DIR_WEST, WALL_PRESENT)
    # check that a closed off area does not get flooded
    maze.update_wall(maze.cell_id(4, 4), DIR_NORTH, WALL_PRESENT)
    maze.update_wall(maze.cell_id(4, 4), DIR_EAST, WALL_PRESENT)
    maze.update_wall(maze.cell_id(4, 4), DIR_SOUTH, WALL_PRESENT)
    maze.update_wall(maze.cell_id(4, 4), DIR_WEST, WALL_PRESENT)
    # now do some performance runs
    start_time = millis()
    target = maze.cell_id(7, 7)
    for _ in range(iterations()):
        distances = maze.flood_for_search(target)
    end_time = millis()
    t = end_time - start_time
    maze_str = maze.get_maze_string(VIEW_COSTS)
    print(maze_str)
    assert maze.cost[0] == 20, "Unexpected search flood distance"
    assert maze.cost[maze.cell_id(4, 4)] == maze.MAX_COST, "Enclosed cell is reachable"
    print("Search flood checks passed")
    print(f"{sys.implementation.name} - maze: Execution Time for {iterations()} iterations: {t:} milliseconds")

