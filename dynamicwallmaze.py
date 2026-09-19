#region VEXcode Generated Robot Configuration
import math
import random
from vexcode_vr import *

# Brain should be defined by default
brain=Brain()

drivetrain = Drivetrain("drivetrain", 0)
pen = Pen("pen", 8)
pen.set_pen_width(THIN)
left_bumper = Bumper("leftBumper", 2)
right_bumper = Bumper("rightBumper", 3)
front_eye = EyeSensor("frontEye", 4)
down_eye = EyeSensor("downEye", 5)
front_distance = Distance("frontdistance", 6)
distance = front_distance
magnet = Electromagnet("magnet", 7)
location = Location("location", 9)

#endregion VEXcode Generated Robot Configuration
# ------------------------------------------
# 
# 	Project:      VEXcode Project
#	Author:       VEX
#	Created:
#	Description:  VEXcode VR Python Project
# 
# ------------------------------------------

from enum import IntEnum
from typing import List

ROWS = 8
COLS = 8

TILE_SIZE = 250
WALL_THICKNESS = 37

class GridStatus(IntEnum):
    BLANK_TILE = 0
    BLANK_WALL = 1
    VISITED_TILE = 2
    WALL = 3
    WALL_INTERSECTION = 4

class Compass(IntEnum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3 

class Direction(IntEnum):
    FORWARD = 0
    RIGHT = 1
    BACK = 2
    LEFT = 3

grid = [[GridStatus.BLANK_TILE for _ in range(COLS * 2 - 1)] for _ in range(ROWS * 2 - 1)]
# monitor_variable("grid")
visit_stack = []

# Converts location sensor x/y mm to x/y tile
def location_to_tile(x_mm: int, y_mm: int) -> (int, int):
    # if x_mm % 125 != 0: raise Exception("Off Grid in X")
    # if y_mm % 125 != 0: raise Exception("Off Grid in Y")
    tile_x = (x_mm + 875) / TILE_SIZE
    tile_y = 7 - (y_mm + 875) / TILE_SIZE
    # things appear to be off the 250 tile size increment occasionally
    return int(tile_x + 0.5), int(tile_y + 0.5)

# Given a tile location returns the grid location
# Tiles are located on every other grid location (in x and y) starting at 0, 0
def tile_to_grid(tile_x: int, tile_y: int) -> (int, int):
    grid_x = tile_x * 2
    grid_y = tile_y * 2
    return int(grid_x), int(grid_y)

def compass_to_tile(tile_x, tile_y, new_x, new_y):
    if new_x > tile_x: return Compass.EAST
    if new_x < tile_x: return Compass.WEST
    if new_y > tile_y: return Compass.SOUTH
    if new_y < tile_y: return Compass.NORTH
    raise Exception("invalid tile")

# Given a tile location and a compass heading returns the wall grid location
# Walls are located on every other grid location (in x and y) offset by 1 (perimeter walls excluded)
# E.g. in x, wall locations on grid are at 1, 3, 5, etc.
# In y, wall locations on grid are at 1, 3, 5
# Wall intersections are when both x and y are odd, e.g. 1,3 would be a wall intersection. This is
#  upper-left from tile at grid 2,4 (or actual tile 1,2)
def tile_to_wallgrid(tile_x: int, tile_y: int, robot_heading: Compass) -> (int, int):
    grid_x = tile_x * 2
    grid_y = tile_y * 2
    if robot_heading == Compass.NORTH:
        if (grid_y <= 0): raise Exception("ERROR: INVALID PERIMETER WALL")
        grid_y -= 1
    elif robot_heading == Compass.EAST:
        if (grid_x >= COLS * 2 - 1): raise Exception("ERROR: INVALID PERIMETER WALL")
        grid_x += 1
    elif robot_heading == Compass.SOUTH:
        if (grid_y >= ROWS * 2 - 1): raise Exception("ERROR: INVALID PERIMETER WALL")
        grid_y += 1
    elif robot_heading == Compass.WEST:
        if (grid_x <= 0): raise Exception("ERROR: INVALID PERIMETER WALL")
        grid_x -= 1
    return int(grid_x), int(grid_y)

# Insert blank walls, intersections and tiles into grid
def initialize_grid():
    for y in range(ROWS * 2 - 1):
        for x in range(COLS * 2 - 1):
            if y % 2: # horizontal walls
                if x % 2: # vertical walls
                    # wall intersection
                    grid[y][x] = GridStatus.WALL_INTERSECTION
                else:
                    # blank vertical wall
                    grid[y][x] = GridStatus.BLANK_WALL
            else: # horizontal tiles
                if x % 2: # vertical walls
                    grid[y][x] = GridStatus.BLANK_WALL
                else: # blank tile
                    grid[y][x] = GridStatus.BLANK_TILE

def print_grid():
    brain.print("-----------------\n")
    for y in range(ROWS * 2 - 1):
        line = "|"
        for x in range(COLS * 2 - 1):
            if (y % 2 == 0) and (x % 2 == 0):
                if (int(x / 2), int(y / 2)) in visit_stack:
                    line += "x"
                elif grid[y][x] == GridStatus.BLANK_TILE:
                    line += " "
                elif grid[y][x] == GridStatus.VISITED_TILE:
                    line += "."
            elif grid[y][x] == GridStatus.BLANK_WALL:
                line += " "
            elif grid[y][x] == GridStatus.WALL_INTERSECTION:
                line += "+"
            elif grid[y][x] == GridStatus.WALL:
                if y % 2:
                    line += "-"
                if x % 2:
                    line += "|"
        brain.print("{}|\n".format(line))
    brain.print("-----------------\n")
    brain.print("\n")

# Converts heading into a compass
def heading_to_compass(robot_heading: int) -> Compass:
    robot_heading = int(robot_heading % 360)
    if robot_heading < 0 or robot_heading >= 360: raise Exception("Invalid Heading")
    return Compass(robot_heading / 90)

def direction_to_heading(robot_heading, direction):
    heading = (robot_heading + int(direction) * 90) % 360
    return heading

def recenter():
    brain.print("RECENTERING\n")
    drivetrain.drive_for(FORWARD, 25, MM)
    drivetrain.turn_for(LEFT, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 5, MM)
    drivetrain.turn_for(RIGHT, 90, DEGREES)

    loc_x, loc_y = location.position(X, MM), location.position(Y, MM)
    tile_x, tile_y = location_to_tile(loc_x, loc_y)

    brain.print("location = {}, {}, tile location = {}, {}\n".format(loc_x, loc_y, tile_x, tile_y))

    brain.print("NORTH: distance = {}, heading = {}\n".format(front_distance.get_distance(MM), location.position_angle(DEGREES)))

    drivetrain.turn_for(LEFT, 90, DEGREES)
    brain.print("WEST: distance = {}, heading = {}\n".format(front_distance.get_distance(MM), location.position_angle(DEGREES)))

    drivetrain.turn_for(LEFT, 90, DEGREES)
    brain.print("SOUTH: distance = {}, heading = {}\n".format(front_distance.get_distance(MM), location.position_angle(DEGREES)))

    drivetrain.turn_for(LEFT, 90, DEGREES)
    brain.print("EAST: distance = {}, heading = {}\n".format(front_distance.get_distance(MM), location.position_angle(DEGREES)))

    drivetrain.turn_for(LEFT, 90, DEGREES)

def set_visited(x: int, y: int):
    grid[y * 2][x * 2] = GridStatus.VISITED_TILE

def check_visited(x: int, y: int, compass: Compass) -> bool:
    if compass == Compass.NORTH:
        if y <= 0: return True
        return grid[(y - 1) * 2][x * 2] == GridStatus.VISITED_TILE
    if compass == Compass.SOUTH:
        if y >= ROWS - 1: return True
        return grid[(y + 1) * 2][x * 2] == GridStatus.VISITED_TILE
    if compass == Compass.WEST:
        if x <= 0: return True
        return grid[y * 2][(x - 1) * 2] == GridStatus.VISITED_TILE
    if compass == Compass.EAST:
        if x >= COLS - 1: return True
        return grid[y * 2][(x + 1) * 2] == GridStatus.VISITED_TILE

def check_wall(x: int, y: int, compass: Compass) -> bool:
    if compass == Compass.NORTH:
        if y <= 0: return True
        return grid[y * 2 - 1][x * 2] == GridStatus.WALL
    if compass == Compass.SOUTH:
        if y >= ROWS - 1: return True
        return grid[y * 2 + 1][x * 2] == GridStatus.WALL
    if compass == Compass.WEST:
        if x <= 0: return True
        return grid[y * 2][x * 2 - 1] == GridStatus.WALL
    if compass == Compass.EAST:
        if x >= COLS - 1: return True
        return grid[y * 2][x * 2 + 1] == GridStatus.WALL

def tiles_from_distance(wall_distance: int) -> int:
    # wall_distance -= WALL_THICKNESS
    # wall distance seems to vary
    tile_count = int(wall_distance / TILE_SIZE)
    # if tile_count * TILE_SIZE != wall_distance: raise Exception("Unexpected distance reading")
    return tile_count

# Inserts wall given distance sensor reading
# No walls inserted at perimeter
def process_view(tile_x: int, tile_y: int, compass: Compass, tiles_visible: int):
    # brain.print("process {} {} {} {}\n".format(tile_x, tile_y, compass, tiles_visible))
    if compass == Compass.NORTH:
        far_y = tile_y - tiles_visible
        if far_y > 0:
            grid[far_y * 2- 1][tile_x * 2] = GridStatus.WALL
    elif compass == Compass.SOUTH:
        far_y = tile_y + tiles_visible
        if far_y < ROWS - 1:
            grid[far_y * 2 + 1][tile_x * 2] = GridStatus.WALL
    elif compass == Compass.WEST:
        far_x = tile_x - tiles_visible
        if far_x > 0:
            grid[tile_y * 2][far_x * 2 - 1] = GridStatus.WALL
    elif compass == Compass.EAST:
        far_x = tile_x + tiles_visible
        if far_x < COLS - 1:
            grid[tile_y * 2][far_x * 2 + 1] = GridStatus.WALL

# Returns a list of valid moves (NESW) from current location
def get_valid_moves(x: int, y: int) -> List[Compass]:
    # Check north
    moves = []
    if not check_wall(x, y, Compass.NORTH) and not check_visited(x, y, Compass.NORTH): moves.append(Compass.NORTH) 
    if not check_wall(x, y, Compass.EAST) and not check_visited(x, y, Compass.EAST): moves.append(Compass.EAST) 
    if not check_wall(x, y, Compass.SOUTH) and not check_visited(x, y, Compass.SOUTH): moves.append(Compass.SOUTH) 
    if not check_wall(x, y, Compass.WEST) and not check_visited(x, y, Compass.WEST): moves.append(Compass.WEST)
    return moves

def turns_needed(x, y, robot_heading):
    turns = []
    left_compass = heading_to_compass(direction_to_heading(robot_heading, Direction.LEFT))
    if not check_wall(x, y, left_compass) and not check_visited(x, y, left_compass): turns.append(Direction.LEFT) 
    right_compass = heading_to_compass(direction_to_heading(robot_heading, Direction.RIGHT))
    if not check_wall(x, y, right_compass) and not check_visited(x, y, right_compass): turns.append(Direction.RIGHT) 
    return turns

# Add project code in "main"
def main():
    brain.clear()
    recenter()
    initialize_grid()
    print_grid()

    done = False
    backtrack = False   
    goal_found = False 

    while not done:

        brain.clear()
        print_grid()

        # for step in visit_stack:
        #     brain.print("{} {}\n".format(step[0], step[1]))

        brain.print("{} {} {} {}\n".format(location.position(X, MM), location.position(Y, MM), location.position_angle(DEGREES), front_distance.get_distance(MM)))
        tile_x, tile_y = location_to_tile(location.position(X, MM), location.position(Y, MM))
        tile_x = int(tile_x)
        tile_y = int(tile_y)
        if down_eye.detect(RED):
            goal_found = True
            set_visited(tile_x, tile_y)
            visit_stack.append((tile_x, tile_y))

        if not backtrack and not goal_found:
            set_visited(tile_x, tile_y)
            visit_stack.append((tile_x, tile_y))

            compass = heading_to_compass(location.position_angle(DEGREES))
            tiles_visible = tiles_from_distance(front_distance.get_distance(MM))
            process_view(tile_x, tile_y, compass, tiles_visible)

            turns = turns_needed(tile_x, tile_y, location.position_angle(DEGREES))
            brain.print("{}\n".format(turns))

            for turn in turns:
                if turn == Direction.LEFT:
                    drivetrain.turn_for(LEFT, 90, DEGREES)
                    compass = heading_to_compass(location.position_angle(DEGREES))
                    tiles_visible = tiles_from_distance(front_distance.get_distance(MM))
                    process_view(tile_x, tile_y, compass, tiles_visible)
                elif turn == Direction.RIGHT:
                    add90 = 1 if len(turns) == 2 else 0
                    drivetrain.turn_for(RIGHT, int(90 + add90 * 90), DEGREES)
                    compass = heading_to_compass(location.position_angle(DEGREES))
                    tiles_visible = tiles_from_distance(front_distance.get_distance(MM))
                    process_view(tile_x, tile_y, compass, tiles_visible)

        if not goal_found:
            moves = get_valid_moves(tile_x, tile_y)
            brain.print("{}\n".format(moves))

        if goal_found or len(moves) == 0:
            if len(visit_stack) <= 1:
                done = True
                break
            visit_stack.pop() # remove current location
            last_x, last_y = visit_stack[-1]
            print("backtrack: {} {}\n".format(last_x, last_y))
            new_heading = int(compass_to_tile(tile_x, tile_y, last_x, last_y)) * 90
            drivetrain.turn_to_heading(new_heading, DEGREES)
            drivetrain.drive_for(FORWARD, TILE_SIZE, MM)
            backtrack = True
        else:
            move = moves[0]
            new_heading = int(move) * 90
            drivetrain.turn_to_heading(new_heading, DEGREES)
            drivetrain.drive_for(FORWARD, TILE_SIZE, MM)
            backtrack = False


    # drivetrain.turn_for(LEFT, 90, DEGREES)
    # compass = heading_to_compass(location.position_angle(DEGREES))
    # tiles_visible = tiles_from_distance(front_distance.get_distance(MM))
    # process_view(tile_x, tile_y, compass, tiles_visible)

    # drivetrain.turn_for(LEFT, 90, DEGREES)
    # compass = heading_to_compass(location.position_angle(DEGREES))
    # tiles_visible = tiles_from_distance(front_distance.get_distance(MM))
    # process_view(tile_x, tile_y, compass, tiles_visible)



# VR threads — Do not delete
vr_thread(main)

'''
312 -> 1
62 -> 0
812 -> 3

0 - 1 = 250
0 - 3 = 750

y tiles = -900, -650, -400, -150, 100, 350, 600, 850
distances = 812, , , 62

wall offset north = 62

right:
32 -> 0
282 -> 1

left
42 -> 0
292 -> 1
1042 -> 4

x tiles = -870, -620, -370, -120, 130, 380, 630, 880

bottom left = 130 - 1000 = -870, -900
bottom right = 130 + 750 = 880, -900
top left = -870, 850
top right = 880, 850

top left: x = -870, y = 850

initialize: drive north 25, drive west 5. now starts: x = 125, y = -875

each axis = -875, -625, -375, -125, 125, 375, 625, 875
'''
