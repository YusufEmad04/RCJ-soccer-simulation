import math

from functions import *




def get_direction(angle):
    if angle >= 345 or angle <= 15:
        return 0
    elif angle >= 165 and angle <= 195:
        return 2
    elif (angle > 15 and angle <= 180):  # or (angle > 195 and angle <= 270):
        print("right")
        return -1
    else:
        print("left")
        return 1


def move(direction, quad):
    if direction == 0:
        left_speed = -5 * quad
        right_speed = -5 * quad
        print("Fwd")
    elif direction == 2:
        left_speed = 5
        right_speed = 5
        print("back")
    else:
        left_speed = direction * 2
        right_speed = direction * -2
    return left_speed, right_speed


def move_pos(robot_pos, orientation, coord):
    angle, quad = get_coord_angle(robot_pos, orientation, coord)
    dir = get_direction(angle)
    speeds = move(dir, quad)
    return speeds
