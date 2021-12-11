import math


def get_angle(directions):
    LR = directions[0]
    FB = directions[2]
    angle = -math.atan2(LR, FB)*180/math.pi
    return angle


def get_ball_distance(r):
    power = float(r) ** (-0.507)
    return 1.0146 * power


def get_ball_position(heading, ball_dist, ball_angle, robot_pos):
    robot_x = robot_pos[0]
    robot_y = robot_pos[1]

    dist_x = ball_dist * math.cos((heading + ball_angle) * math.pi / 180)
    dist_y = ball_dist * math.sin((heading + ball_angle) * math.pi / 180)
    ball_pos_x = robot_x + dist_x
    ball_pos_y = robot_y + dist_y

    return ball_pos_x,ball_pos_y
