import math
from rcj_soccer_robot import RCJSoccerRobot
import time


def get_ball_angle(directions):
    # get angle using atan2 (vectors)
    lr = directions[1]
    fb = directions[0]
    angle = -math.atan2(lr, fb) * 180 / math.pi
    return angle


def get_ball_distance(r):
    # from excel
    power = float(r) ** (-0.504)
    return 1.0059 * power


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


def get_dist(coord1, coord2):
    return math.sqrt((coord2[1] - coord1[1]) ** 2 + (coord2[0] - coord1[0]) ** 2)


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
    add_to_arr(robot.dist_arr, get_dist(ball_pos, robot.robot_pos_arr[-1]))

    check_ball_dir_robot(robot)

    if len(robot.ball_pos_arr) > 1:
        robot.last_ball_pos = robot.ball_pos_arr[1]

    check_for_real_speeds(robot, get_ball_speed(robot)[0])

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
        add_to_arr(robot.dist_arr, get_dist(ball_pos[0], robot.robot_pos_arr[-1]))

        check_ball_dir_robot(robot)

        if len(robot.ball_pos_arr) > 1:
            robot.last_ball_pos = robot.ball_pos_arr[1]
        check_for_real_speeds(robot, get_ball_speed(robot)[0])
    else:
        clear_ball_data(robot)


def send_team_data(robot: RCJSoccerRobot, see_ball=True):
    if see_ball:
        ball_pos = robot.ball_pos_arr[-1]
        robot.send_data_to_team(robot.player_id, robot.robot_pos_arr[-1], ball_pos, True)
    else:
        ball_pos = [-2, -2]
        robot.send_data_to_team(robot.player_id, robot.robot_pos_arr[-1], ball_pos, False)


def move_to_point_3(robot: RCJSoccerRobot, coord, forward=True):
    robot_pos = robot.robot_pos_arr[-1]
    heading = robot.heading

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
            robot.left_motor.setVelocity(10 * ratio)
            robot.right_motor.setVelocity(10)
            # robot.left_motor.setVelocity(-10)
            # robot.right_motor.setVelocity(-10 * ratio)
        else:
            robot.left_motor.setVelocity(-10)
            robot.right_motor.setVelocity(-10 * ratio)
            # robot.right_motor.setVelocity(10)
            # robot.left_motor.setVelocity(10 * ratio)

    # checking coordinate is on left
    elif -180 <= angle < 0:

        # checking if coordinate is in front or behind
        if -90 <= angle < 0:

            ratio = 1 + (angle / 90)

        else:
            ratio = (angle + 90) / 90

        # set each wheel's speed (right is at maximum, left is according to ratio)
        if forward:
            robot.left_motor.setVelocity(10)
            robot.right_motor.setVelocity(10 * ratio)
            # robot.left_motor.setVelocity(-10 * ratio)
            # robot.right_motor.setVelocity(-10)
        else:
            robot.left_motor.setVelocity(-10 * ratio)
            robot.right_motor.setVelocity(-10)
            # robot.right_motor.setVelocity(10 * ratio)
            # robot.left_motor.setVelocity(10)


def move_to_point2(robot: RCJSoccerRobot, coord, forward=True, s=10):
    robot_pos = robot.robot_pos_arr[-1]
    heading = robot.heading

    # adjust robot heading then move towards point

    angle = get_coord_angle(robot_pos, heading, coord)

    if forward:
        if -10 <= angle <= 10:
            robot.left_motor.setVelocity(s)
            robot.right_motor.setVelocity(s)
            # robot.left_motor.setVelocity(-10)
            # robot.right_motor.setVelocity(-10)
        elif 0 <= angle <= 180:
            robot.left_motor.setVelocity(-s)
            robot.right_motor.setVelocity(s)
        elif -180 <= angle < 0:
            robot.left_motor.setVelocity(s)
            robot.right_motor.setVelocity(-s)
    else:
        if (170 <= angle <= 180) or (-180 <= angle <= -170):
            robot.left_motor.setVelocity(-s)
            robot.right_motor.setVelocity(-s)
            # robot.left_motor.setVelocity(10)
            # robot.right_motor.setVelocity(10)
        elif 0 <= angle <= 180:
            robot.left_motor.setVelocity(s)
            robot.right_motor.setVelocity(-s)
        elif -180 <= angle < 0:
            robot.left_motor.setVelocity(-s)
            robot.right_motor.setVelocity(s)


def move_to_point(robot: RCJSoccerRobot, coord, forward=True, s=10):
    robot_pos = robot.robot_pos_arr[-1]
    heading = robot.heading

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
            speed = (2 * s) * ratio - s

        else:
            speed = -s

        # set each wheel's speed (left is at maximum, right is according to ratio)
        if forward:
            robot.left_motor.setVelocity(speed)
            robot.right_motor.setVelocity(s)
            # robot.left_motor.setVelocity(-10)
            # robot.right_motor.setVelocity(-10 * ratio)
        else:
            robot.left_motor.setVelocity(-s)
            robot.right_motor.setVelocity(-speed)
            # robot.right_motor.setVelocity(10)
            # robot.left_motor.setVelocity(10 * ratio)

    # checking coordinate is on left
    elif -180 <= angle < 0:

        # checking if coordinate is in front or behind
        if -90 <= angle < 0:

            ratio = 1 + (angle / 90)
            speed = (2 * s) * ratio - s

        else:
            speed = -s

        # set each wheel's speed (right is at maximum, left is according to ratio)
        if forward:
            robot.left_motor.setVelocity(s)
            robot.right_motor.setVelocity(speed)
            # robot.left_motor.setVelocity(-10 * ratio)
            # robot.right_motor.setVelocity(-10)
        else:
            robot.left_motor.setVelocity(-speed)
            robot.right_motor.setVelocity(-s)
            # robot.right_motor.setVelocity(10 * ratio)
            # robot.left_motor.setVelocity(10)


def move_to_direction(robot: RCJSoccerRobot, direction, forward=True):
    if forward:
        pass
    else:
        pass


def adjust_heading(robot: RCJSoccerRobot, obj):
    robot_pos = robot.robot_pos_arr[-1]
    heading = robot.heading

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


def adjust_heading_to_angle(robot: RCJSoccerRobot, angle):
    heading = robot.heading
    if heading > angle:

        if heading - angle >= 350:
            return True
        elif max(abs(heading), abs(angle)) - min(abs(heading), abs(angle)) < 10:
            return True

        if heading - angle > 180:
            robot.right_motor.setVelocity(10)
            robot.left_motor.setVelocity(-10)
        else:
            robot.right_motor.setVelocity(-10)
            robot.left_motor.setVelocity(10)
    else:

        if heading - angle <= -350:
            return True
        elif max(abs(heading), abs(angle)) - min(abs(heading), abs(angle)) < 10:
            return True

        if heading - angle < -180:
            robot.right_motor.setVelocity(-10)
            robot.left_motor.setVelocity(10)
        else:
            robot.right_motor.setVelocity(10)
            robot.left_motor.setVelocity(-10)


def add_to_arr(arr, data):
    arr.append(data)
    if len(arr) >= 4:
        arr.pop(0)


def get_speed(t, pos):
    y = (pos[-1][1] - pos[0][1]) ** 2
    x = (pos[-1][0] - pos[0][0]) ** 2
    dist = math.sqrt(y + x)
    t = t[-1] - t[0]
    if time == 0:
        return 0

    return (100 * dist / t), get_coord_angle(pos[0], 0, pos[-1]), pos[-1]


def get_robot_speed(robot: RCJSoccerRobot):
    return get_speed(robot.time_steps_arr, robot.robot_pos_arr)


def increment_step(robot: RCJSoccerRobot):
    robot.time_step += 0.5
    add_to_arr(robot.time_steps_arr, robot.time_step)
    check_for_relocation_data(robot)


def clear_ball_data(robot: RCJSoccerRobot):
    robot.ball_pos_arr = []
    robot.dist_arr = []


def get_ball_speed(robot: RCJSoccerRobot):
    if robot.ball_pos_arr:
        return get_speed(robot.time_steps_arr, robot.ball_pos_arr)


def get_relative_ball_speed(robot: RCJSoccerRobot):
    if robot.dist_arr:
        total_dist = robot.dist_arr[-1] - robot.dist_arr[0]
        total_time = robot.time_steps_arr[-1] - robot.time_steps_arr[0]

        if total_time == 0:
            return 0

        return total_dist * 100 / total_time


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
    return (0.001 * r) - 0.0091


def intercept_ball(robot: RCJSoccerRobot):
    if robot.ball_pos_arr:
        if get_ball_speed(robot)[0] > 2:
            print(get_ball_speed(robot)[0])
            if robot.flags["intercepting ball"][0]:
                defend_strategy_2(robot)
            else:

                print("going to intercept")
                defend_strategy_2(robot, False)
        else:
            robot.flags["intercepting ball"][0] = False
            robot.right_motor.setVelocity(0)
            robot.left_motor.setVelocity(0)
    else:
        robot.flags["intercepting ball"][0] = False
        robot.right_motor.setVelocity(0)
        robot.left_motor.setVelocity(0)


def defend(robot: RCJSoccerRobot):
    # if check_ball_status(robot) == 1:
    #     print("shoot")
    #     predicted_pos = predict_ball_pos(robot, 14)
    #     move_to_point(robot, predicted_pos)
    # elif check_ball_status(robot) == 2:
    #     print(".")
    # elif check_ball_status(robot) == 3 or 4:
    #
    #     defend_strategy_1(robot)
    #     # defend_strategy_3(robot,see_ball,heading,team_data)
    #
    # elif check_ball_status(robot) == 4:
    #
    #     robot.right_motor.setVelocity(-10)
    #     robot.left_motor.setVelocity(-10)

    check_strategy(robot)


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

    # check if ball is being shot towards the goal and in our half
    if (min(boundary_angles) <= ball_angle <= max(boundary_angles)) and (ball_speed[0] > 2) and (ball_pos[0] > 0):
        return conditions[0]
    else:
        return conditions[2]


def check_strategy(robot: RCJSoccerRobot):
    # check if robot is currently intercepting the ball

    if robot.flags["intercepting ball"][0]:
        # if robot was already intercepting the ball
        # print("intercepting at {},   dir {}".format(robot.ball_intercept_pos, robot.ball_intercept_direction))

        # check which strategy the robot was intercepting from
        strategies = [defend_strategy_2, None, defend_strategy_4]

        # call the strategy
        strategies[robot.flags["intercepting ball"][1] - 2](robot)
    else:

        # check if ball status
        if (check_ball_status(robot) == 4) or (check_ball_status(robot) == 3):

            # go forward and backward
            # print("normal")
            defend_strategy_1(robot)
        elif check_ball_status(robot) == 1:

            # robot should intercept the ball

            pos = predict_ball_pos(robot, 18)

            if (robot.robot_pos_arr[-1][0] - 0.1 <= robot.ball_pos_arr[-1][0]) and robot.ball_pos_arr[-1][0] > 0.6:
                # go to either sides of the goal
                defend_strategy_4(robot, False)
            elif pos[0] > 0.7:
                defend_strategy_5(robot, pos, False)
            else:

                # intercept the ball
                print("going to intercept")
                defend_strategy_2(robot, False)


def defend_strategy_1(robot: RCJSoccerRobot):
    # print("backward   {}", format(robot.moving_backward))
    # print("forward   {}", format(robot.moving_forward))

    heading = robot.heading

    inside = (0.7, 0)
    outside = (0.45, 0)

    if (not robot.flags["moving forward"]) and (not robot.flags["moving backward"]):

        # check if robot arrived to inside
        if (inside[0] - 0.05 <= robot.robot_pos_arr[-1][0] <= inside[0] + 0.05) and (
                inside[1] - 0.05 <= robot.robot_pos_arr[-1][1] <= inside[1] + 0.05):
            # get robot to move to y

            if (-170 <= heading <= 0) or (0 < heading <= 170):
                move_to_point2(robot, outside)
            else:
                robot.flags["moving backward"] = False
                robot.flags["moving forward"] = True
                robot.start_time = time.time()

        # move to y
        move_to_point(robot, inside)

    elif robot.flags["moving forward"]:

        if (outside[0] - 0.075 <= robot.robot_pos_arr[-1][0] <= outside[0] + 0.025) and (
                outside[1] - 0.05 <= robot.robot_pos_arr[-1][1] <= outside[1] + 0.05):
            # get robot to move to y
            robot.flags["moving backward"] = True
            robot.flags["moving forward"] = False
            robot.start_time = time.time()

        # move to y

        # robot.right_motor.setVelocity(0)
        # robot.left_motor.setVelocity(0)

        if robot.ball_pos_arr:
            adjust_heading(robot, robot.ball_pos_arr[-1])
        else:
            robot.right_motor.setVelocity(0)
            robot.left_motor.setVelocity(0)

        if time.time() - robot.start_time > 1:
            move_to_point2(robot, outside)

    elif robot.flags["moving backward"]:

        if (inside[0] - 0.05 <= robot.robot_pos_arr[-1][0] <= inside[0] + 0.05) and (
                inside[1] - 0.05 <= robot.robot_pos_arr[-1][1] <= inside[1] + 0.05):
            # get robot to move to y
            robot.flags["moving backward"] = False
            robot.flags["moving forward"] = True
            robot.start_time = time.time()

        # robot.right_motor.setVelocity(0)
        # robot.left_motor.setVelocity(0)

        if robot.ball_pos_arr:
            adjust_heading(robot, robot.ball_pos_arr[-1])
        else:
            robot.right_motor.setVelocity(0)
            robot.left_motor.setVelocity(0)

        # move to y
        if time.time() - robot.start_time > 2:
            move_to_point2(robot, inside, False)


def defend_strategy_2(robot: RCJSoccerRobot, was_intercepting=True):
    # check if function is called while robot is intercepting (no need to calculate)

    if not was_intercepting:
        robot.flags["intercepting ball"][0] = True
        robot.flags["intercepting ball"][1] = 2
        # predicted_pos = predict_ball_pos(robot, 18)
        predicted_pos = predict_optimal_pos(robot, False)

        robot.ball_intercept_pos = predicted_pos
        robot.ball_intercept_direction = get_ball_speed(robot)[1]
        robot.initial_ball_pos = get_ball_speed(robot)[2]
    else:

        pos = robot.ball_intercept_pos

        # check if robot arrived to intercept position

        if (pos[0] - 0.04 <= robot.robot_pos_arr[-1][0] <= pos[0] + 0.04) and (
                pos[1] - 0.04 <= robot.robot_pos_arr[-1][1] <= pos[1] + 0.04):
            # print("\n__stop\n")
            if robot.ball_pos_arr:
                # robot moves towards the ball
                move_to_point(robot, robot.ball_pos_arr[-1])
            else:
                robot.right_motor.setVelocity(0)
                robot.left_motor.setVelocity(0)
        else:
            move_to_point(robot, robot.ball_intercept_pos)
            # move_to_point(robot, robot.ball_intercept_pos)

        # check if ball has changed its direction

        if not robot.ball_pos_arr:
            move_to_point(robot, robot.ball_intercept_pos)
            # move_to_point(robot, robot.ball_intercept_pos)

        # check if ball has changed its direction and position or not
        elif ((get_ball_speed(robot)[1] > robot.ball_intercept_direction + 40) or (
                get_ball_speed(robot)[1] < robot.ball_intercept_direction - 40)) and (
                get_dist(robot.ball_pos_arr[-1], robot.ball_intercept_pos) > 1):
            robot.flags["intercepting ball"][0] = False
            robot.flags["intercepting ball"][1] = 0
            print("\ncanceled {}\n".format(get_ball_speed(robot)[1]))


def defend_strategy_3(robot: RCJSoccerRobot, see_ball, heading, team_data=None):
    # left of goal
    point_x = (0.6, 0.25)
    # outside penalty area
    point_y = (0.425, 0)
    # right of goal
    point_z = (0.6, -0.25)

    if robot.flags["moving to x"]:

        # check if robot arrived to point x
        if (point_x[0] - 0.1 <= robot.robot_pos_arr[-1][0] <= point_x[0] + 0.05) and (
                point_x[1] - 0.1 <= robot.robot_pos_arr[-1][1] <= point_x[1] + 0.1):
            # get robot to move to y
            robot.flags["moving to x"] = False
            robot.flags["moving to y"] = True

        # move to x
        move_to_point(robot, point_x)

    elif robot.flags["moving to y"]:

        # check if robot arrived to point y
        if (point_y[0] - 0.1 <= robot.robot_pos_arr[-1][0] <= point_y[0] + 0.75) and (
                point_y[1] - 0.1 <= robot.robot_pos_arr[-1][1] <= point_y[1] + 0.1):
            # get robot to move to y
            robot.flags["moving to z"] = True
            robot.flags["moving to y"] = False

        # move to y
        move_to_point(robot, point_y)

    elif robot.flags["moving to z"]:

        # check if robot arrived to point y
        if (point_z[0] - 0.1 <= robot.robot_pos_arr[-1][0] <= point_z[0] + 0.05) and (
                point_z[1] - 0.1 <= robot.robot_pos_arr[-1][1] <= point_z[1] + 0.1):
            # get robot to move to y
            robot.flags["moving to z"] = False
            robot.flags["moving to x"] = True

        # move to y
        move_to_point(robot, point_z)


def defend_strategy_4(robot: RCJSoccerRobot, was_intercepting=True):
    goal_tip_l = (0.73, -1.3)
    goal_tip_r = (0.73, 1.3)

    if not was_intercepting:

        robot.flags["intercepting ball"][0] = True
        robot.flags["intercepting ball"][1] = 4
        robot.ball_intercept_direction = get_ball_speed(robot)[1]

        # getting necessary data
        heading = robot.heading
        angle = get_coord_angle(robot.robot_pos_arr[-1], 0, robot.ball_pos_arr[-1])
        direction = "front"

        if angle > 15:
            direction = "right"
        elif angle < -15:
            direction = "left"

        if direction == "left":
            print("\n going to left goal tip")
            goal_angle = get_coord_angle(robot.robot_pos_arr[-1], heading, goal_tip_l)
            if goal_angle < 0:
                move_to_point(robot, goal_tip_l)
                robot.ball_intercept_pos = goal_tip_l
                robot.flags["strategy 4 data"]["forward"] = True
                robot.flags["strategy 4 data"]["function"] = 1
            else:
                move_to_point2(robot, goal_tip_l, False)
                robot.ball_intercept_pos = goal_tip_l
                robot.flags["strategy 4 data"]["forward"] = False
                robot.flags["strategy 4 data"]["function"] = 2

        elif direction == "right":
            print("\n going to right goal tip")
            goal_angle = get_coord_angle(robot.robot_pos_arr[-1], heading, goal_tip_r)
            if goal_angle > 0:

                move_to_point(robot, goal_tip_r)
                robot.ball_intercept_pos = goal_tip_r
                robot.flags["strategy 4 data"]["forward"] = True
                robot.flags["strategy 4 data"]["function"] = 1
            else:

                move_to_point2(robot, goal_tip_r, False)
                robot.ball_intercept_pos = goal_tip_r
                robot.flags["strategy 4 data"]["forward"] = False
                robot.flags["strategy 4 data"]["function"] = 2

        elif direction == "front":
            robot.flags["intercepting ball"][0] = False
            robot.flags["intercepting ball"][1] = 0

    else:

        def arrived(point):
            if (point[0] - 0.025 <= robot.robot_pos_arr[-1][0] <= point[0] + 0.025) and (
                    point[1] - 0.025 <= robot.robot_pos_arr[-1][1] <= point[1] + 0.025):
                return True

        if arrived(robot.ball_intercept_pos):
            robot.right_motor.setVelocity(0)
            robot.left_motor.setVelocity(0)
            # robot.intercepting_ball[0] = False
            # robot.intercepting_ball[1] = 0
        else:
            if robot.flags["strategy 4 data"]["forward"]:
                if robot.flags["strategy 4 data"]["function"] == 1:
                    move_to_point(robot, robot.ball_intercept_pos)
                else:
                    move_to_point2(robot, robot.ball_intercept_pos)
            else:
                if robot.flags["strategy 4 data"]["function"] == 2:
                    move_to_point(robot, robot.ball_intercept_pos, False)
                else:
                    move_to_point2(robot, robot.ball_intercept_pos, False)

        if not robot.ball_pos_arr:
            move_to_point(robot, robot.ball_intercept_pos)

        elif (get_ball_speed(robot)[1] > robot.ball_intercept_direction + 60) or (
                get_ball_speed(robot)[1] < robot.ball_intercept_direction - 60):
            robot.flags["intercepting ball"][0] = False
            robot.flags["intercepting ball"][1] = 0
            print("\ncanceled {}\n".format(get_ball_speed(robot)[1]))

        print("\n___riskkk___")


def defend_strategy_5(robot: RCJSoccerRobot, predicted_pos=None, was_intercepting=True):
    robot.ball_intercept_direction = get_ball_speed(robot)[1]

    gradient = math.tan(get_coord_angle(robot.ball_pos_arr[-1], 0, predicted_pos) * math.pi / 180)

    constant = predicted_pos[1] - (gradient * predicted_pos[0])

    y = 0.74 * gradient + constant

    if gradient > 0:
        y -= 0.03
    else:
        y += 0.03

    if y > 0.145:
        y = 0.145
    elif y < -0.145:
        y = -0.145

    point = (0.74, y)

    robot.ball_intercept_pos = point

    if (get_ball_speed(robot)[1] > robot.ball_intercept_direction + 60) or (
            get_ball_speed(robot)[1] < robot.ball_intercept_direction - 60):
        robot.flags["intercepting ball"][0] = False
        robot.flags["intercepting ball"][1] = 0
        print("\ncanceled {}\n".format(get_ball_speed(robot)[1]))

    def arrived(point):
        if (point[0] - 0.025 <= robot.robot_pos_arr[-1][0] <= point[0] + 0.025) and (
                point[1] - 0.025 <= robot.robot_pos_arr[-1][1] <= point[1] + 0.025):
            return True

    if arrived(robot.ball_intercept_pos):
        move_to_point(robot, robot.ball_pos_arr[-1], False)
        # adjust_heading(robot, robot.ball_pos_arr[-1])

    if (0 <= robot.heading <= 90) or -90 <= robot.heading < 0:
        move_to_point2(robot, robot.ball_intercept_pos)
    else:
        move_to_point2(robot, robot.ball_intercept_pos, False)


def predict_ball_pos(robot: RCJSoccerRobot, t):
    # get distance moved on hyp side then get new x and y

    if robot.ball_pos_arr:
        speed = get_ball_speed(robot)

        hyp = t * speed[0]
        dist_x = hyp * math.cos(speed[1] * math.pi / 180) / 100
        dist_y = hyp * math.sin(speed[1] * math.pi / 180) / 100

        direction = speed[1]
        gradient = math.tan(direction * math.pi / 180)
        constant = speed[2][1] - (gradient * speed[2][0])

        # right or down wall
        if 0 <= direction < 90:
            right_interception = [0.729, 0.729 * gradient + constant]
            down_interception = [(0.629 - constant) / gradient, 0.629]

            x_corner = (1.317 - constant) / (1 + gradient)
            y_corner = gradient * x_corner + constant

            corner_intercept = [x_corner, y_corner]

            dist_right = get_dist(speed[2], right_interception)
            dist_down = get_dist(speed[2], down_interception)
            dist_corner = get_dist(speed[2], corner_intercept)

            shortest_dist = min(dist_corner, dist_right, dist_down)

            if shortest_dist == dist_corner:
                pass
            elif shortest_dist == dist_right:
                pass
            elif shortest_dist == dist_down:
                pass

        # down or left wall
        elif 90 <= direction <= 180:
            left_interception = [-0.729, -0.729 * gradient + constant]
            down_interception = [(0.629 - constant) / gradient, 0.629]

            x_corner = (constant - 1.3179) / (1 - gradient)
            y_corner = gradient * x_corner + constant

            corner_intercept = [x_corner, y_corner]

            dist_left = get_dist(speed[2], left_interception)
            dist_corner = get_dist(speed[2], corner_intercept)
            dist_down = get_dist(speed[2], down_interception)

            shortest_dist = min(dist_corner, dist_left, dist_down)

            if shortest_dist == dist_corner:
                pass
            elif shortest_dist == dist_left:
                pass
            elif shortest_dist == dist_down:
                pass

        # up or right wall
        elif -90 <= direction < 0:
            right_interception = [0.729, 0.729 * gradient + constant]
            up_interception = [(-0.629 - constant) / gradient, -0.629]

            x_corner = (-constant - 1.3179) / (1 - gradient)
            y_corner = gradient * x_corner + constant

            corner_intercept = [x_corner, y_corner]

            dist_right = get_dist(speed[2], right_interception)
            dist_up = get_dist(speed[2], up_interception)
            dist_corner = get_dist(speed[2], corner_intercept)

            shortest_dist = min(dist_corner, dist_right, dist_up)

            if shortest_dist == dist_corner:
                pass
            elif shortest_dist == dist_up:
                pass
            elif shortest_dist == dist_right:
                pass

        # left or up wall
        elif -180 <= direction < -90:
            left_interception = [-0.729, -0.729 * gradient + constant]
            up_interception = [(-0.629 - constant) / gradient, -0.629]

            x_corner = -(1.317 + constant) / (1 + gradient)
            y_corner = gradient * x_corner + constant

            corner_intercept = [x_corner, y_corner]

            dist_left = get_dist(speed[2], left_interception)
            dist_up = get_dist(speed[2], up_interception)
            dist_corner = get_dist(speed[2], corner_intercept)

            shortest_dist = min(dist_corner, dist_left, dist_up)

            if shortest_dist == dist_corner:
                pass
            elif shortest_dist == dist_left:
                pass
            elif shortest_dist == dist_up:
                pass

        return speed[2][0] + dist_x, speed[2][1] + dist_y


def predict_ball_time(robot: RCJSoccerRobot, dist):
    # divide distance by time

    if robot.ball_pos_arr:
        speed = get_ball_speed(robot)
        if speed[0] == 0:
            return 0

        return dist / (speed[0] / 100)


def predict_optimal_pos(robot: RCJSoccerRobot, defence=True):
    # get speed, direction and position of ball
    speed = get_ball_speed(robot)

    # start position
    b1 = speed[2]

    # end position (after 45 time steps)
    b2 = predict_ball_pos(robot, 45)

    # check if x or y coordinate is out of field boundaries
    if b2[0] > 0.7:
        b2 = [0.7, b2[1]]

    if b2[0] < -0.7:
        b2 = [-0.7, b2[1]]

    if b2[1] > 0.59:
        b2 = [b2[0], 0.59]

    if b2[1] < -0.59:
        b2 = [b2[0], -0.59]

    print("b2 : {}".format(b2))

    # get distance between start and end position
    dist = get_dist(b2, b1)
    pos = (0, 0)
    ball_time = 0
    robot_time = (0, 0, 0)
    final_robot_time = 0

    # divide distance into equal parts with length 0.05
    for i in range(int(dist / 0.05)):
        hyp = i * 0.05
        dist_x = hyp * math.cos(speed[1] * math.pi / 180)
        dist_y = hyp * math.sin(speed[1] * math.pi / 180)

        # try each position ( from b1 to b2)
        pos = (speed[2][0] + dist_x, speed[2][1] + dist_y)

        # get robot and ball time to point
        ball_time = predict_ball_time(robot, hyp)
        robot_time = predict_robot_time(robot.robot_pos_arr[-1], robot.heading, pos, 0)

        # get ball angle when robot arrives to point
        robot_ball_angle = get_coord_angle(robot_time[2], robot_time[1], robot.ball_pos_arr[-1])
        # estimate turing time for robot to adjust heading towards the ball
        turn_time = (abs(robot_ball_angle) / 360) * 14

        # final time take by the robot = robot time to point + turning time + 4.5 time step
        final_robot_time = robot_time[0] + turn_time + 4.5

        # check if robot time is less than or equal ball time (optimal point)
        if final_robot_time <= ball_time:
            print("optimal r: {}, b: {}".format(robot_time[0], ball_time))
            print("turn time: {}, b: {}, r: {}".format(turn_time, speed[1], robot_time[1]))
            return pos

    print("r: {}, b: {}".format(robot_time[0], ball_time))
    return pos


def diff_steer(robot_pos, heading, left_speed, right_speed, t):
    x = robot_pos[0]
    y = robot_pos[1]
    h = heading
    if h < 0:
        h = 360 + h
    h = h * math.pi / 180

    # get turning speed for motors
    vr = right_speed * 0.0255 / 10
    vl = left_speed * 0.0255 / 10

    if vl == vr:
        hyp = vr
        x_change = math.cos(heading * math.pi / 180) * hyp
        y_change = math.sin(heading * math.pi / 180) * hyp
        return x + x_change, y + y_change, heading, hyp * 100

    # get x and y changes (radius and angle as functions of time)
    x_change = + ((0.09 * (vl + vr)) / (2 * (vl - vr))) * (math.sin(((t * (vl - vr)) / 0.09) + h) - math.sin(h))
    y_change = - ((0.09 * (vl + vr)) / (2 * (vl - vr))) * (math.cos(((t * (vl - vr)) / 0.09) + h) - math.cos(h))

    # heading after arrival
    angle = (((t * (vl - vr)) / 0.09) + h) * 180 / math.pi

    if angle > 180:
        angle = -360 + angle

    # getting arc length
    dist = get_dist(robot_pos, (x + x_change, y + y_change))
    radius = ((0.09 * (vl + vr)) / (2 * (vl - vr)))

    if radius == 0:
        return x + x_change, y + y_change, angle, 0

    a = math.acos(((dist ** 2) - (2 * (radius ** 2))) / -(2 * (radius ** 2)))

    # getting speed
    sp = (radius * a) * 100 / t

    return x + x_change, y + y_change, angle, sp


def predict_robot_time(robot_pos, heading, coord, t):
    rp = robot_pos
    h = heading
    t = t

    # check if predicted robot position is within the range of the coordinate
    while not ((coord[0] - 0.03 <= rp[0] <= coord[0] + 0.03) and (
            coord[1] - 0.03 <= rp[1] <= coord[1] + 0.03)):
        t += 1

        angle = get_coord_angle(rp, h, coord)

        # get each motor speed
        if 0 <= angle <= 180:

            # checking if coordinate is in front or behind
            if 0 <= angle <= 90:

                ratio = 1 - (angle / 90)
                speed = 20 * ratio - 10

            else:
                speed = -10

            # recursively call the function using the predicted robot position and heading after 1 time step
            predicted_robot_pos = diff_steer(rp, h, 10, speed, 1)

            rp = predicted_robot_pos[0], predicted_robot_pos[1]
            h = predicted_robot_pos[2]

        # checking coordinate is on left
        elif -180 <= angle < 0:

            # checking if coordinate is in front or behind
            if -90 <= angle < 0:

                ratio = 1 + (angle / 90)
                speed = 20 * ratio - 10

            else:
                speed = -10

            # recursively call the function using the predicted robot position and heading
            predicted_robot_pos = diff_steer(rp, h, speed, 10, 1)

            rp = predicted_robot_pos[0], predicted_robot_pos[1]
            h = predicted_robot_pos[2]

            if t > 100:
                return False

    return t, h, rp


def shoot(robot: RCJSoccerRobot):
    if robot.stuck:
        if robot.ball_pos_arr and robot.relocated:
            ball_pos = get_ball_speed(robot)[2]
            if ball_pos[1] > 0:
                goal_pos = [-0.729, -0.075]
            else:
                goal_pos = [-0.729, 0.075]
            goal_angle = get_coord_angle(ball_pos, 0, goal_pos)
            gradient = math.tan(goal_angle * math.pi / 180)
            constant = ball_pos[1] - (ball_pos[0] * gradient)

            pos = [ball_pos[0] + 0.15, (ball_pos[0] + 0.15) * gradient + constant]

            if (pos[0] - 0.04 <= robot.robot_pos_arr[-1][0] <= pos[0] + 0.04) and (
                    pos[1] - 0.04 <= robot.robot_pos_arr[-1][1] <= pos[1] + 0.04):
                robot.arrived_to_shoot = True

            if robot.arrived_to_shoot:
                move_to_point2(robot, goal_pos)

            else:
                move_to_point(robot, pos)
        else:

            pos = [0.1, 0]
            goal_pos = [-0.729, 0]

            if (pos[0] - 0.04 <= robot.robot_pos_arr[-1][0] <= pos[0] + 0.04) and (
                    pos[1] - 0.04 <= robot.robot_pos_arr[-1][1] <= pos[1] + 0.04):
                robot.arrived = True

            if robot.arrived:
                # if robot.ball_pos_arr:
                #     robot.right_motor.setVelocity(0)
                #     robot.left_motor.setVelocity(0)
                # else:
                #
                #     move_to_point2(robot, goal_pos, s=3)

                robot.right_motor.setVelocity(0)
                robot.left_motor.setVelocity(0)

            else:
                move_to_point(robot, pos)


def check_for_relocation_data(robot: RCJSoccerRobot):
    speed = (0, 0, 0)
    if robot.ball_pos_arr:
        speed = get_ball_speed(robot)
    if speed[0] <= 1 or (not robot.ball_pos_arr):
        if not robot.stuck:
            robot.stuck = True
            robot.timer = time.time()
    elif speed[0] > 3 and robot.real_speed:
        robot.stuck = False
        robot.timer = 0
        robot.arrived = False
        robot.arrived_to_shoot = False
        robot.relocated = False
        robot.ready_for_relocation = False

    check_for_relocation(robot)


def check_for_relocation(robot: RCJSoccerRobot):
    if robot.stuck and robot.time_step > 8:
        if not robot.ball_pos_arr:
            robot.ready_for_relocation = True
        else:
            if robot.ready_for_relocation:
                if get_ball_speed(robot)[0] == 0 or (abs(robot.last_ball_pos[0] - robot.ball_pos_arr[-1][0]) > 0.1) or (
                        abs(robot.last_ball_pos[1] - robot.ball_pos_arr[-1][1]) > 0.1):
                    robot.relocated = True


def check_for_real_speeds(robot: RCJSoccerRobot, speed):
    if robot.temp_speeds:
        if abs(robot.temp_speeds[-1] - speed) > 1:
            robot.temp_speeds = []
            robot.real_speed = False
        else:
            robot.temp_speeds.append(speed)

        if len(robot.temp_speeds) >= 5:
            robot.real_speed = True
        else:
            robot.real_speed = False
    else:
        robot.temp_speeds.append(speed)


def handle_ball(robot: RCJSoccerRobot):
    robot_ball_dist = get_dist(robot.robot_pos_arr[-1], robot.ball_pos_arr[-1])
    ball_speed = get_ball_speed(robot)[0]

    if robot_ball_dist > 0.08:
        # robot.right_motor.setVelocity(10)
        # robot.left_motor.setVelocity(10)
        move_to_point(robot, robot.ball_pos_arr[-1])

    else:
        if ball_speed >= 2.55:
            # robot.right_motor.setVelocity(10)
            # robot.left_motor.setVelocity(10)
            move_to_point(robot, robot.ball_pos_arr[-1])
        elif ball_speed >= 1.7:
            motors_speed = ball_speed * 10 / 2.55
            # robot.right_motor.setVelocity(motors_speed)
            # robot.left_motor.setVelocity(motors_speed)
            move_to_point(robot, robot.ball_pos_arr[-1], s=motors_speed)
        else:
            # robot.right_motor.setVelocity(6)
            # robot.left_motor.setVelocity(6)
            move_to_point(robot, robot.ball_pos_arr[-1], s=6)

    # if robot_ball_dist < 0.085:
    #     robot.right_motor.setVelocity(5.75)
    #     robot.left_motor.setVelocity(5.75)
    # else:
    #     motors_speed = ball_speed * 10 / 2.55
    #     robot.right_motor.setVelocity(motors_speed)
    #     robot.left_motor.setVelocity(motors_speed)


def check_ball_dir_robot(robot: RCJSoccerRobot):
    if len(robot.ball_pos_arr) > 2:
        first_pos = robot.ball_pos_arr[0]
        last_pos = robot.ball_pos_arr[-1]

        if get_dist(robot.robot_pos_arr[-1], last_pos) < get_dist(robot.robot_pos_arr[-1], first_pos):
            robot.flags["ball getting closer"] = True
        else:
            robot.flags["ball getting closer"] = False
