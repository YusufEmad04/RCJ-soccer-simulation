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


def get_dist(robot_pos, coord):
    dist = math.sqrt((coord[0]-robot_pos[0]) ** 2 + (coord[1]-robot_pos[1]) ** 2)
    return dist

def dir_of_move(heading, dir):
    """
    Function to get angle needed to rotate ball to specific direction assuming ball is in front
    """
    if dir - heading < -180:
        return 360 - (heading - dir)
    elif dir - heading > 180:
        return -360 + (dir - heading)
    else:
        return dir - heading


def move_dir(robot: RCJSoccerRobot, robot_pos, heading, coord, distance, ball_speed, dir):
    ball_speed_new = ball_speed[0] * 10 / 2.56
    print(ball_speed_new)
    angle = get_coord_angle(robot_pos, heading, coord)
    robot_vel = 0
    # TO match the speed of the ball
    if ball_speed < 0.5:
        robot_vel = -3
    elif ball_speed < 10:
        robot_vel = -ball_speed
    else:
        robot_vel = 0

    if 0 <= angle <= 180:

        # checking if coordinate is in front or behind
        if 0 <= angle <= 90:

            ratio = 1 - (angle / 90)

        else:
            ratio = (90 - angle) / 90

        # set each wheel's speed (left is at maximum, right is according to ratio)
        robot.left_motor.setVelocity(-10)
        robot.right_motor.setVelocity((-10) * ratio)

        # checking coordinate is on left
    elif -180 <= angle < 0:

        # checking if coordinate is in front or behind
        if -90 <= angle < 0:

            ratio = 1 + (angle / 90)

        else:
            ratio = (angle + 90) / 90

        # set each wheel's speed (right is at maximum, left is according to ratio)
        robot.left_motor.setVelocity((-10) * ratio)
        robot.right_motor.setVelocity(-10)

def move_Fwd(robot: RCJSoccerRobot, robot_pos, heading, coord, ball_speed):
    # if ball_speed < 0.5:
    #     robot.left_motor.setVelocity(-3)
    #     robot.right_motor.setVelocity(-3)
    # elif ball_speed < 10:
    #     robot.left_motor.setVelocity(-ball_speed)
    #     robot.right_motor.setVelocity(-ball_speed)
    # else:
    #     robot.left_motor.setVelocity(0)
    #     robot.right_motor.setVelocity(0)
    """Just move forward"""
    dist = get_dist(robot_pos,coord)
    if dist <= 0.08:
        robot.left_motor.setVelocity(1.5)
        robot.right_motor.setVelocity(1.5)
    else:
        robot.left_motor.setVelocity(10)
        robot.right_motor.setVelocity(10)

def predict_ball_pos(robot: RCJSoccerRobot, t):
    if robot.ball_pos_arr:
        speed = get_ball_speed(robot)

        hyp = t * speed[0]
        dist_x = hyp * math.cos(speed[1] * math.pi / 180) / 100
        dist_y = hyp * math.sin(speed[1] * math.pi / 180) / 100
        new_x = speed[2][0] + dist_x
        new_y = speed[2][1] + dist_y
        if new_x >= 0.75:
            new_x -= 0.75
        elif new_x <= 0.75:
            new_x += 0.75
        if new_y >= 0.65:
            new_y -= 0.65
        elif new_y >= 0.65:
            new_y -= 0.65
        return new_x, new_y
