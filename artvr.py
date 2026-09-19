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

# new stuff
from math import sqrt, atan2, cos, sin, radians, degrees, pi

drivetrain.set_drive_velocity(100, PERCENT)
drivetrain.set_turn_velocity(100, PERCENT)

brain.clear()

# X/Y circle, parameter based
def point_on_circle(radius, number_of_segments, n):
    t = radians(n * 360 / number_of_segments)
    x = radius * cos(t)
    y = radius * sin(t)
    brain.print("xy = ", x, y)
    brain.new_line()
    return (x, y)

# X/Y elephant, parameter based
def point_on_elephant(radius, number_of_segments, n):
    t = radians(n * 360 / number_of_segments)
    y = radius * (- 60 * cos(t) + 30 * sin(t) - 8 * sin(2 * t) + 10 * sin(3 * t))
    x = radius * (50 * sin(t) + 18 * sin(2 * t) - 12 * cos(3 * t) + 14 * cos(5 * t))
    return (x, y)

# heading and distance to new X/Y from old X/Y
def directions_to_point(x1, y1, x2, y2):
    distance = sqrt((x2 - x1) ** 2 + ((y2 - y1) ** 2))
    heading = degrees(atan2(y2 - y1, x2 - x1))
    brain.print("dist, dir = ", distance, heading)
    brain.new_line()
    return (heading, distance)

# Circle using only turns and distance
def directions_on_circle(radius, number_of_segments):
    # simple tangent following
    turn = 360 / number_of_segments
    distance = 2 * pi * radius / number_of_segments
    return (turn, distance)

# Star using only turns and distance
def directions_on_star(turn, distance):
    # nothing useful done here
    return (turn, distance)

# Add project code in "main"
def main():
    pen.move(DOWN)

    # draw elephant or circle using X/Y method
    # need to keep track of previous X/Y
    pen.set_pen_color(RED)
    object_points = 32
    object_size = 6
    object_center = (100, 100)
    for i in range(object_points + 1):
        # x, y = point_on_circle(200, 16, i)
        x, y = point_on_elephant(object_size, object_points, i)
        new_x = x + object_center[0]
        new_y = y + object_center[1]
        if (i > 0):
            heading, distance = directions_to_point(old_x, old_y, new_x, new_y)
            drivetrain.turn_to_heading(heading, DEGREES)
            drivetrain.drive_for(FORWARD, distance, MM)
        old_x = new_x
        old_y = new_y

    # draw circle using turn/drive
    pen.set_pen_color(BLUE)
    circle_points = 16
    circle_size = 200
    for i in range(circle_points):
        turn, distance = directions_on_circle(circle_size, circle_points)
        drivetrain.turn_for(RIGHT, turn, DEGREES)
        drivetrain.drive_for(FORWARD, distance, MM)

    # draw star using turn/drive (while loop instead of for)
    count = 0
    while count <= 16:

        # == is comparison "is the same as"
        # < is comparison "is less than"
        # % is remainder function
        if count % 4 == 0:
            pen.set_pen_color(BLACK)
        elif count % 4 == 1:
            pen.set_pen_color(BLUE)
        elif count % 4 == 2:
            pen.set_pen_color(GREEN)
        elif count % 4 == 3:
            pen.set_pen_color(RED)

        turn, drive = directions_on_star(160, 750)
        drivetrain.turn_for(RIGHT, turn, DEGREES)
        drivetrain.drive_for(FORWARD, drive, MM)
        count = count + 1

    brain.print("done!")

# VR threads — Do not delete
vr_thread(main)
