import math
import time

from rcj_soccer_robot import RCJSoccerRobot


def get_ball_angle(directions):
    # get angle using atan2 (vectors)
    lr = directions[0]
    fb = directions[2]
    angle = -math.atan2(lr, fb) * 180 / math.pi
    return angle


def get_ball_distance(r):
    # from excel
    power = float(r) ** (-0.507)
    return 1.0146 * power


def get_coord_position(heading, coord_dist, coord_angle, robot_pos):
    robot_x = robot_pos[0]
    robot_y = robot_pos[1]

    dist_x = coord_dist * math.cos(
        (heading + coord_angle) * math.pi / 180)  # getting x-axis distance from robot to ball
    dist_y = coord_dist * math.sin(
        (heading + coord_angle) * math.pi / 180)  # getting y-axis distance from robot to ball
    coord_pos_x = robot_x + dist_x  # getting x-axis coordinates of ball
    coord_pos_y = robot_y + dist_y  # getting x-axis coordinates of ball

    return coord_pos_x, coord_pos_y


def get_coord_angle(robot_pos, heading, coord):
    x = robot_pos[0] - coord[0]  # getting x-axis distance of coordinate from robot
    y = robot_pos[1] - coord[1]  # getting y-axis distance of coordinate from robot

    # to avoid division by zero
    if x == 0:
        x = 0.00000000001

    a = (math.atan(y / x) * 180 / math.pi)  # getting tan value

    # checking sides signs to get angle (without heading)
    if a > 0:
        if y > 0:
            angle = a - 180
        else:
            angle = a
    else:
        if y < 0:
            angle = a + 180
        else:
            angle = a

    # getting angle after getting heading
    if angle - heading < -180:
        return 360 - (heading - angle)
    elif angle - heading > 180:
        return -360 + (angle - heading)
    else:
        return angle - heading


def receive_data(robot: RCJSoccerRobot):
    team_data = []  # to store other robots' data

    # Get data from compass
    heading = robot.get_compass_heading()

    # Get GPS coordinates of the robot
    robot_pos = robot.get_gps_coordinates()

    add_to_arr(robot.robot_pos_arr, robot_pos)

    # data from the supervisor (supervisor receiver)
    data = robot.get_new_data()

    # get ultrasonic values
    ultrasonic = robot.get_sonar_values()

    # get coordinates of readings
    ultrasonic_front = get_coord_position(heading, get_ultrasonic_dist(ultrasonic["front"]), 0, robot_pos)
    ultrasonic_back = get_coord_position(heading, get_ultrasonic_dist(ultrasonic["back"]), 180, robot_pos)
    ultrasonic_right = get_coord_position(heading, get_ultrasonic_dist(ultrasonic["right"]), 90, robot_pos)
    ultrasonic_left = get_coord_position(heading, get_ultrasonic_dist(ultrasonic["left"]), -90, robot_pos)

    ultrasonic_val = {
        "front": (ultrasonic_front[0], ultrasonic_front[1]),
        "back": (ultrasonic_back[0], ultrasonic_back[1]),
        "right": (ultrasonic_right[0], ultrasonic_right[1]),
        "left": (ultrasonic_left[0], ultrasonic_left[1]),
    }

    # check if there is data from (team receiver)
    # while loop to empty queue
    while robot.is_new_team_data():
        # data from the team receiver (team receiver)
        team_data.append(robot.get_new_team_data())
        team_data.append(robot.get_new_team_data())

    r = [None] * 3

    # store each robot in its index inside the array
    for i in team_data:
        r[i["robot_id"] - 1] = i

    team_d = {
        "B1": r[0],
        "B2": r[1],
        "B3": r[2]
    }

    robot.heading = heading
    robot.ultrasonic_data = ultrasonic_val
    robot.team_data = team_d

    d = {
        "heading": heading,
        "robot position": robot_pos,
        "supervisor data": data,
        "ultrasonic": ultrasonic_val,
        "team data": team_d

    }

    return d


def print_data(data, k=None):
    if k:
        print(data[k])
    else:
        # dict data with their keys
        for key in data.keys():
            print("{}:  {}".format(key, data[key]))

        print("\n--------------------\n")


def receive_ball_data(robot: RCJSoccerRobot):
    heading = robot.heading
    robot_pos = robot.robot_pos_arr[-1]

    ball_data = robot.get_new_ball_data()

    # getting all ball data
    robot_ball_angle = get_ball_angle(ball_data["direction"])
    ball_distance = get_ball_distance(ball_data["strength"])
    ball_pos = get_coord_position(heading, ball_distance, robot_ball_angle, robot_pos)

    add_to_arr(robot.ball_pos_arr, ball_pos)

    d = {
        "robot ball angle": robot_ball_angle,
        "ball distance": ball_distance,
        "ball position": ball_pos
    }

    return d


def get_team_ball_data(robot: RCJSoccerRobot):

    ball_pos = []

    # getting ball position from other robots
    for robot_id in robot.team_data:
        if robot.team_data[robot_id]:
            if robot.team_data[robot_id]["see the ball"]:
                ball_pos.append(robot.team_data[robot_id]["ball_pos"])

    # checking if other robots see the ball
    if ball_pos:
        add_to_arr(robot.ball_pos_arr, ball_pos[0])
    else:
        clear_ball_data(robot)


def move_to_point(robot: RCJSoccerRobot, robot_pos, heading, coord, forward=True):
    angle = get_coord_angle(robot_pos, heading, coord)

    if not forward:

        if 0 <= angle <= 180:
            angle -= 180
        else:
            angle += 180

    # checking coordinate is on right
    if 0 <= angle <= 180:

        # checking if coordinate is in front or behind
        if 0 <= angle <= 90:

            ratio = 1 - (angle / 90)

        else:
            ratio = (90 - angle) / 90

        # set each wheel's speed (left is at maximum, right is according to ratio)
        if forward:
            robot.left_motor.setVelocity(-10)
            robot.right_motor.setVelocity((-10) * ratio)
        else:
            robot.right_motor.setVelocity(10)
            robot.left_motor.setVelocity(10 * ratio)

    # checking coordinate is on left
    elif -180 <= angle < 0:

        # checking if coordinate is in front or behind
        if -90 <= angle < 0:

            ratio = 1 + (angle / 90)

        else:
            ratio = (angle + 90) / 90

        # set each wheel's speed (right is at maximum, left is according to ratio)
        if forward:
            robot.left_motor.setVelocity((-10) * ratio)
            robot.right_motor.setVelocity(-10)
        else:
            robot.right_motor.setVelocity(10 * ratio)
            robot.left_motor.setVelocity(10)


def move_to_point2(robot: RCJSoccerRobot, robot_pos, heading, coord, forward=True):
    # adjust robot heading then move towards point

    angle = get_coord_angle(robot_pos, heading, coord)

    if forward:
        if -10 <= angle <= 10:
            robot.left_motor.setVelocity(-10)
            robot.right_motor.setVelocity(-10)
        elif 0 <= angle <= 180:
            robot.left_motor.setVelocity(-10)
            robot.right_motor.setVelocity(10)
        elif -180 <= angle < 0:
            robot.left_motor.setVelocity(10)
            robot.right_motor.setVelocity(-10)
    else:
        if (170 <= angle <= 180) or (-180 <= angle <= -170):
            robot.left_motor.setVelocity(10)
            robot.right_motor.setVelocity(10)
        elif 0 <= angle <= 180:
            robot.left_motor.setVelocity(10)
            robot.right_motor.setVelocity(-10)
        elif -180 <= angle < 0:
            robot.left_motor.setVelocity(-10)
            robot.right_motor.setVelocity(10)


def adjust_heading(robot: RCJSoccerRobot, robot_pos, heading, obj):
    angle = get_coord_angle(robot_pos, heading, obj)

    if 5 <= angle <= 180:
        robot.left_motor.setVelocity(-10)
        robot.right_motor.setVelocity(10)
    elif -180 <= angle <= -5:
        robot.left_motor.setVelocity(10)
        robot.right_motor.setVelocity(-10)
    else:
        robot.left_motor.setVelocity(0)
        robot.right_motor.setVelocity(0)


def add_to_arr(arr, data):
    arr.append(data)
    if len(arr) >= 4:
        arr.pop(0)


def get_speed(t, pos):
    y = (pos[-1][1] - pos[0][1]) ** 2
    x = (pos[-1][0] - pos[0][0]) ** 2
    dist = math.sqrt(y + x)
    time = t[-1] - t[0]
    if time == 0:
        return 0

    return (100 * dist / time), get_coord_angle(pos[0], 0, pos[-1]), pos[-1]


def get_robot_speed(robot: RCJSoccerRobot):
    return get_speed(robot.time_steps_arr, robot.robot_pos_arr)


def increment_step(robot: RCJSoccerRobot):
    robot.time_step += 0.5
    add_to_arr(robot.time_steps_arr, robot.time_step)


def clear_ball_data(robot: RCJSoccerRobot):
    robot.ball_pos_arr = []


def get_ball_speed(robot: RCJSoccerRobot):
    if robot.ball_pos_arr:
        return get_speed(robot.time_steps_arr, robot.ball_pos_arr)


def get_ball_speed_old(robot: RCJSoccerRobot, this_robot=True):
    # check if ball position came from this robot
    if this_robot:
        return get_speed(robot.time_steps_arr, robot.ball_pos_arr)
    else:
        ball_pos = []

        # getting ball position from other robots
        for robot_id in robot.team_data:
            if robot.team_data[robot_id]:
                if robot.team_data[robot_id]["see the ball"]:
                    ball_pos.append(robot.team_data[robot_id]["ball_pos"])

        # checking if other robots see the ball
        if ball_pos:
            add_to_arr(robot.ball_pos_arr, ball_pos[0])
            return get_speed(robot.time_steps_arr, robot.ball_pos_arr)

        # clearing ball positions array (speed won't be correct)
        else:
            clear_ball_data(robot)
            return None


def get_ultrasonic_dist(r):
    return (0.001 * r) - 0.0118


def defend(robot: RCJSoccerRobot, heading):
    if check_ball_status(robot) == 1:
        print("shoot")
        move_to_point(robot, robot.robot_pos_arr[-1], robot.heading, robot.ball_pos_arr[-1])
    elif check_ball_status(robot) == 2:
        print(".")
    elif check_ball_status(robot) == 3 or 4 or 1:

        defend_strategy_1(robot, heading)
        # defend_strategy_3(robot,see_ball,heading,team_data)

    elif check_ball_status(robot) == 4:

        robot.right_motor.setVelocity(-10)
        robot.left_motor.setVelocity(-10)


def check_ball_status(robot: RCJSoccerRobot):
    """
    return conditions:
    ball is being shot - (1)
    ball with other team in our half - (2)
    ball with us - (3)
    no ball data - (4)
    """

    conditions = [1, 2, 3, 4]

    ball_speed = get_ball_speed(robot)

    # check if there is ball data
    if not ball_speed:
        return conditions[3]

    ball_angle = ball_speed[1]
    ball_pos = ball_speed[2]

    # angle from goal to two sides of the goal
    boundary_angles = [
        get_coord_angle(ball_pos, 0, (0.72, 0.35)),
        get_coord_angle(ball_pos, 0, (0.72, -0.35))
    ]

    # check if ball is being shot towards the goal
    if min(boundary_angles) <= ball_angle <= max(boundary_angles) and (ball_speed[0] > 2):
        return conditions[0]
    else:
        return conditions[2]


def check_strategy(robot: RCJSoccerRobot, robot_pos, ball_pos):
    pass


def defend_strategy_1(robot: RCJSoccerRobot, heading):
    # print("backward   {}", format(robot.moving_backward))
    # print("forward   {}", format(robot.moving_forward))

    inside = (0.7, 0)
    outside = (0.45, 0)

    if (not robot.moving_forward) and (not robot.moving_backward):

        # check if robot arrived to inside
        if (inside[0] - 0.05 <= robot.robot_pos_arr[-1][0] <= inside[0] + 0.05) and (
                inside[1] - 0.05 <= robot.robot_pos_arr[-1][1] <= inside[1] + 0.05):
            # get robot to move to y

            if (-170 <= heading <= 0) or (0 < heading <= 170):
                move_to_point2(robot, robot.robot_pos_arr[-1], heading, outside)
            else:
                robot.moving_backward = False
                robot.moving_forward = True
                robot.start_time = time.time()

        # move to y
        move_to_point(robot, robot.robot_pos_arr[-1], heading, inside)

    elif robot.moving_forward:

        if (outside[0] - 0.075 <= robot.robot_pos_arr[-1][0] <= outside[0] + 0.025) and (
                outside[1] - 0.05 <= robot.robot_pos_arr[-1][1] <= outside[1] + 0.05):
            # get robot to move to y
            robot.moving_backward = True
            robot.moving_forward = False
            robot.start_time = time.time()

        # move to y

        # robot.right_motor.setVelocity(0)
        # robot.left_motor.setVelocity(0)

        if robot.ball_pos_arr:
            adjust_heading(robot, robot.robot_pos_arr[-1], heading, robot.ball_pos_arr[-1])
        else:
            robot.right_motor.setVelocity(0)
            robot.left_motor.setVelocity(0)

        if time.time() - robot.start_time > 1:
            move_to_point2(robot, robot.robot_pos_arr[-1], heading, outside)

    elif robot.moving_backward:

        if (inside[0] - 0.05 <= robot.robot_pos_arr[-1][0] <= inside[0] + 0.05) and (
                inside[1] - 0.05 <= robot.robot_pos_arr[-1][1] <= inside[1] + 0.05):
            # get robot to move to y
            robot.moving_backward = False
            robot.moving_forward = True
            robot.start_time = time.time()

        # robot.right_motor.setVelocity(0)
        # robot.left_motor.setVelocity(0)

        if robot.ball_pos_arr:
            adjust_heading(robot, robot.robot_pos_arr[-1], heading, robot.ball_pos_arr[-1])
        else:
            robot.right_motor.setVelocity(0)
            robot.left_motor.setVelocity(0)

        # move to y
        if time.time() - robot.start_time > 2:
            move_to_point2(robot, robot.robot_pos_arr[-1], heading, inside, False)


def defend_strategy_3(robot: RCJSoccerRobot, see_ball, heading, team_data=None):
    # left of goal
    point_x = (0.6, 0.25)
    # outside penalty area
    point_y = (0.425, 0)
    # right of goal
    point_z = (0.6, -0.25)

    if robot.moving_to_x:

        # check if robot arrived to point x
        if (point_x[0] - 0.1 <= robot.robot_pos_arr[-1][0] <= point_x[0] + 0.05) and (
                point_x[1] - 0.1 <= robot.robot_pos_arr[-1][1] <= point_x[1] + 0.1):
            # get robot to move to y
            robot.moving_to_x = False
            robot.moving_to_y = True

        # move to x
        move_to_point(robot, robot.robot_pos_arr[-1], heading, point_x)

    elif robot.moving_to_y:

        # check if robot arrived to point y
        if (point_y[0] - 0.1 <= robot.robot_pos_arr[-1][0] <= point_y[0] + 0.75) and (
                point_y[1] - 0.1 <= robot.robot_pos_arr[-1][1] <= point_y[1] + 0.1):
            # get robot to move to y
            robot.moving_to_z = True
            robot.moving_to_y = False

        # move to y
        move_to_point(robot, robot.robot_pos_arr[-1], heading, point_y)

    elif robot.moving_to_z:

        # check if robot arrived to point y
        if (point_z[0] - 0.1 <= robot.robot_pos_arr[-1][0] <= point_z[0] + 0.05) and (
                point_z[1] - 0.1 <= robot.robot_pos_arr[-1][1] <= point_z[1] + 0.1):
            # get robot to move to y
            robot.moving_to_z = False
            robot.moving_to_x = True

        # move to y
        move_to_point(robot, robot.robot_pos_arr[-1], heading, point_z)


def predict_ball_pos(robot: RCJSoccerRobot, t):

    if robot.ball_pos_arr:
        speed = get_ball_speed(robot)
        print(speed, "Speed")

        hyp = t * speed[0]
        dist_x = hyp * math.cos(speed[1] * math.pi / 180) / 100
        dist_y = hyp * math.sin(speed[1] * math.pi / 180) / 100

        return speed[2][0] + dist_x, speed[2][1] + dist_y
