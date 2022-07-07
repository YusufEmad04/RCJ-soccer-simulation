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

    check_for_real_robot_speed(robot, get_robot_speed(robot)[0])

    # data from the supervisor (supervisor receiver)
    data = robot.get_new_data()
    robot.flags["waiting_for_kickoff"] = data["waiting_for_kickoff"]

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

    robot.ultrasonic_arr = sorted([(i, get_dist(ultrasonic_val[i], robot.robot_pos_arr[-1])) for i in ultrasonic_val],
                                  key=lambda x: x[1])

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

    if team_d["B1"]:
        if robot.name[1] == "2":
            robot.roles[1] = team_d["B1"]["robot 2 role"]
        elif robot.name[1] == "3":
            robot.roles[2] = team_d["B1"]["robot 3 role"]

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
    adjust_robot_stuck_timer(robot)
    adjust_robot_penalty_time(robot)

    return d


def stop_all(robot: RCJSoccerRobot):
    robot.set_left_vel(0)
    robot.set_right_vel(0)


def print_data(data, k=None):
    if k:
        print(data[k])
    else:
        # dict data with their keys
        for key in data.keys():
            print("{}:  {}".format(key, data[key]))

        print("\n--------------------\n")


def calculate_score(robot: RCJSoccerRobot):
    if robot.ball_pos_arr:
        if abs(robot.ball_pos_arr[-1][1]) <= 0.2 and not (robot.flags["goal_registered"]):
            if robot.ball_pos_arr[-1][0] >= 0.75:
                robot.flags["goal_registered"] = True
                robot.enemy_goals += 1
            elif robot.ball_pos_arr[-1][0] <= -0.75:
                robot.flags["goal_registered"] = True
                robot.my_goals += 1
    else:
        robot.flags["goal_registered"] = True
        robot.my_goals += 1


def receive_ball_data(robot: RCJSoccerRobot):
    heading = robot.heading
    robot_pos = robot.robot_pos_arr[-1]

    ball_data = robot.get_new_ball_data()
    # if robot.player_id == 1:
    #     print_data(ball_data)
    # if robot.ball_pos_arr:
    #     print("pos: {}".format(get_ball_speed(robot)[2]))

    # getting all ball data
    robot_ball_angle = get_ball_angle(ball_data["direction"])
    ball_distance = get_ball_distance(ball_data["strength"])
    ball_pos = get_coord_position(heading, ball_distance, robot_ball_angle, robot_pos)

    add_to_arr(robot.ball_pos_arr, ball_pos)
    add_to_arr(robot.dist_arr, get_dist(ball_pos, robot.robot_pos_arr[-1]))

    check_ball_dir_robot(robot)

    if len(robot.ball_pos_arr) > 1:
        robot.last_ball_pos = robot.ball_pos_arr[1]

    check_for_real_ball_speed(robot, get_ball_speed(robot)[0])

    d = {
        "robot ball angle": robot_ball_angle,
        "ball distance": ball_distance,
        "ball position": ball_pos
    }

    check_for_relocation_data(robot)

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
        check_for_real_ball_speed(robot, get_ball_speed(robot)[0])
    else:
        clear_ball_data(robot)

    check_for_relocation_data(robot)


def send_team_data(robot: RCJSoccerRobot, see_ball=True):
    if robot.name[1] == "1":
        robots_roles = [robot.roles[1], robot.roles[2]]
    else:
        robots_roles = [-1, -1]

    if see_ball:
        ball_pos = robot.ball_pos_arr[-1]
        robot.send_data_to_team(robot.player_id, robot.robot_pos_arr[-1], ball_pos, True,
                                robot.predicted_intercept_time,
                                *robots_roles, robot.flags["defense_signal"])
    else:
        ball_pos = [-2, -2]
        robot.send_data_to_team(robot.player_id, robot.robot_pos_arr[-1], ball_pos, False,
                                robot.predicted_intercept_time,
                                *robots_roles, robot.flags["defense_signal"])


def move_to_point2(robot: RCJSoccerRobot, coord, forward=True, s=10):
    robot_pos = robot.robot_pos_arr[-1]
    heading = robot.heading

    # adjust robot heading then move towards point

    angle = get_coord_angle(robot_pos, heading, coord)

    if forward:
        if -10 <= angle <= 10:
            robot.set_left_vel(s)
            robot.set_right_vel(s)
            # robot.set_left_vel(-10)
            # robot.set_right_vel(-10)
        elif 0 <= angle <= 180:
            robot.set_left_vel(-s)
            robot.set_right_vel(s)
        elif -180 <= angle < 0:
            robot.set_left_vel(s)
            robot.set_right_vel(-s)
    else:
        if (170 <= angle <= 180) or (-180 <= angle <= -170):
            robot.set_left_vel(-s)
            robot.set_right_vel(-s)
            # robot.set_left_vel(10)
            # robot.set_right_vel(10)
        elif 0 <= angle <= 180:
            robot.set_left_vel(s)
            robot.set_right_vel(-s)
        elif -180 <= angle < 0:
            robot.set_left_vel(-s)
            robot.set_right_vel(s)


def move_to_point(robot: RCJSoccerRobot, coord, forward=True, s=10):
    robot_pos = robot.robot_pos_arr[-1]
    heading = robot.heading

    angle = get_coord_angle(robot_pos, heading, coord)

    if abs(angle) > 90:
        forward = False
    else:
        forward = True

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
        if (coord[0] - 0.04 <= robot.robot_pos_arr[-1][0] <= coord[0] + 0.04) and (
                coord[1] - 0.04 <= robot.robot_pos_arr[-1][1] <= coord[1] + 0.04):
            stop_all(robot)
        else:
            if forward:
                robot.set_left_vel(speed)
                robot.set_right_vel(s)
                # robot.set_left_vel(-10)
                # robot.set_right_vel(-10 * ratio)
            else:
                robot.set_left_vel(-s)
                robot.set_right_vel(-speed)
                # robot.set_right_vel(10)
                # robot.set_left_vel(10 * ratio)

    # checking coordinate is on left
    elif -180 <= angle < 0:

        # checking if coordinate is in front or behind
        if -90 <= angle < 0:

            ratio = 1 + (angle / 90)
            speed = (2 * s) * ratio - s

        else:
            speed = -s

        # set each wheel's speed (right is at maximum, left is according to ratio)
        if (coord[0] - 0.04 <= robot.robot_pos_arr[-1][0] <= coord[0] + 0.04) and (
                coord[1] - 0.04 <= robot.robot_pos_arr[-1][1] <= coord[1] + 0.04):
            stop_all(robot)
        else:
            if forward:
                robot.set_left_vel(s)
                robot.set_right_vel(speed)
                # robot.set_left_vel(-10 * ratio)
                # robot.set_right_vel(-10)
            else:
                robot.set_left_vel(-speed)
                robot.set_right_vel(-s)
                # robot.set_right_vel(10 * ratio)
                # robot.set_left_vel(10)


def adjust_heading(robot: RCJSoccerRobot, obj):
    robot_pos = robot.robot_pos_arr[-1]
    heading = robot.heading

    angle = get_coord_angle(robot_pos, heading, obj)

    if 5 <= angle <= 180:
        robot.set_left_vel(-10)
        robot.set_right_vel(10)
    elif -180 <= angle <= -5:
        robot.set_left_vel(10)
        robot.set_right_vel(-10)
    else:
        robot.set_left_vel(0)
        robot.set_right_vel(0)


def adjust_heading_to_angle(robot: RCJSoccerRobot, wanted_angle, s=10):
    heading = robot.heading

    angle = robot.heading - wanted_angle
    if abs(angle) >= 90:
        if wanted_angle > 0:
            wanted_angle -= 180
        else:
            wanted_angle += 180

    if heading > wanted_angle:

        if heading - wanted_angle >= 350:
            robot.set_left_vel(0)
            robot.set_right_vel(0)
            return True
        elif heading - wanted_angle < 10:
            robot.set_left_vel(0)
            robot.set_right_vel(0)
            return True

        if heading - wanted_angle > 180:
            robot.set_right_vel(s)
            robot.set_left_vel(-s)
        else:
            robot.set_right_vel(-s)
            robot.set_left_vel(s)
    else:

        if heading - wanted_angle <= -350:
            robot.set_left_vel(0)
            robot.set_right_vel(0)
            return True
        elif heading - wanted_angle > -10:
            robot.set_left_vel(0)
            robot.set_right_vel(0)
            return True

        if heading - wanted_angle < -180:
            robot.set_right_vel(-s)
            robot.set_left_vel(s)
        else:
            robot.set_right_vel(s)
            robot.set_left_vel(-s)


def add_to_arr(arr, data):
    arr.append(data)
    if len(arr) >= 4:
        arr.pop(0)


def get_speed(t, pos):
    y = (pos[-1][1] - pos[0][1]) ** 2
    x = (pos[-1][0] - pos[0][0]) ** 2
    dist = math.sqrt(y + x)
    t = t[-1] - t[0]
    if t == 0:
        return 0, get_coord_angle(pos[0], 0, pos[-1]), pos[-1]

    return (100 * dist / t), get_coord_angle(pos[0], 0, pos[-1]), pos[-1]


def get_robot_speed(robot: RCJSoccerRobot):
    return get_speed(robot.time_steps_arr, robot.robot_pos_arr)


def increment_step(robot: RCJSoccerRobot):
    robot.time_step += 0.5
    add_to_arr(robot.time_steps_arr, robot.time_step)


def clear_ball_data(robot: RCJSoccerRobot):
    if robot.ball_pos_arr:
        robot.previous_ball_pos = robot.ball_pos_arr[-1]
    robot.ball_pos_arr = []
    robot.dist_arr = []


def get_ball_speed(robot: RCJSoccerRobot):
    if robot.ball_pos_arr:
        return get_speed(robot.time_steps_arr, robot.ball_pos_arr)


def get_relative_ball_speed(robot: RCJSoccerRobot):
    ball_data = get_ball_speed(robot)
    robot_speed = diff_steer(robot.robot_pos_arr[-1], robot.heading, robot.left_wheel_vel, robot.right_wheel_vel, 1)[3]

    ball_y, ball_x = ball_data[0] * math.sin(ball_data[1] * math.pi / 180), ball_data[0] * math.cos(
        ball_data[1] * math.pi / 180)
    robot_y, robot_x = robot_speed * math.sin(robot.heading * math.pi / 180), robot_speed * math.cos(
        robot.heading * math.pi / 180)

    final_y, final_x = robot_y - ball_y, robot_x - ball_x

    speed = math.sqrt(
        (final_y ** 2) + (final_x ** 2)
    )

    if robot.flags["ball getting closer"]:
        return speed * -1
    else:
        return speed


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
            robot.set_right_vel(0)
            robot.set_left_vel(0)
    else:
        robot.flags["intercepting ball"][0] = False
        robot.set_right_vel(0)
        robot.set_left_vel(0)


def check_ball_status(robot: RCJSoccerRobot):
    """
    return conditions:
    ball is being shot at us - (1)
    ball is being shot at enemy - (2)
    corner - (3)
    defence - (4)
    attack - (5)
    no ball data - (6)
    """

    conditions = [1, 2, 3, 4, 5, 6]

    ball_speed = get_ball_speed(robot)

    # check if there is ball data
    if not ball_speed:
        add_to_arr(robot.ball_status_arr, conditions[5])
        return conditions[5]

    ball_angle = ball_speed[1]
    if ball_angle > 0:
        ball_angle2 = ball_angle - 180
    else:
        ball_angle2 = ball_angle + 180

    ball_pos = ball_speed[2]

    # angle from goal to two sides of the goal
    boundary_angles = [
        get_coord_angle(ball_pos, 0, (0.72, 0.35)),
        get_coord_angle(ball_pos, 0, (0.72, -0.35))
    ]

    boundary_angles2 = [
        get_coord_angle(ball_pos, 180, (-0.72, 0.35)),
        get_coord_angle(ball_pos, 180, (-0.72, -0.35))
    ]

    # check if ball is being shot towards the goal and in our half
    if (abs(ball_angle) < 75) and (ball_speed[0] > 1.75) and (ball_pos[0] > 0):
        add_to_arr(robot.ball_status_arr, conditions[0])
    elif (min(boundary_angles2) <= ball_angle2 <= max(boundary_angles2)) and (ball_speed[0] > 1.75) and (
            ball_pos[0] < 0):
        add_to_arr(robot.ball_status_arr, conditions[1])
    elif ball_pos[0] > 0.45 and abs(ball_pos[1]) > 0.20:
        add_to_arr(robot.ball_status_arr, conditions[2])
    elif ball_pos[0] > 0:
        add_to_arr(robot.ball_status_arr, conditions[3])
    else:
        add_to_arr(robot.ball_status_arr, conditions[4])


def get_real_ball_status(robot: RCJSoccerRobot):
    check_ball_status(robot)

    if len(robot.ball_status_arr) > 2:
        if (robot.ball_status_arr[0] == robot.ball_status_arr[1] == robot.ball_status_arr[2]) or robot.ball_status_arr[
            -1] == 6:
            if robot.ball_status != robot.ball_status_arr[-1]:
                reset(robot)
            robot.ball_status = robot.ball_status_arr[-1]
    else:
        robot.ball_status = robot.ball_status_arr[-1]


def reset(robot: RCJSoccerRobot):
    # Interceptor
    robot.predicted_intercept_time = -1
    robot.flags["predicted"] = False
    # Mimic
    robot.flags["adjusted heading"] = False
    # Corner
    robot.flags["arrived at corner"] = False


def go_to_corner(robot: RCJSoccerRobot):
    ball_data = get_ball_speed(robot)
    ball_pos = ball_data[2]
    rb_dist = get_dist(robot.robot_pos_arr[-1], ball_pos)
    # print()
    if rb_dist <= 0.09 and robot.flags["arrived at corner"]:
        if abs(get_coord_angle(robot.robot_pos_arr[-1], robot.heading, ball_pos)) < 90:
            move_to_point(robot, ball_pos)
        else:
            move_to_point(robot, ball_pos, False)
    elif ball_pos[1] > 0:
        corner = [robot.goal[0][0], robot.goal[0][1] + 0.08]
        if get_dist(robot.robot_pos_arr[-1], corner) > 1:
            robot.flags["arrived at corner"] = False

        if (corner[0] - 0.04 <= robot.robot_pos_arr[-1][0] <= corner[0] + 0.04) and (
                corner[1] - 0.04 <= robot.robot_pos_arr[-1][1] <= corner[1] + 0.04):
            robot.flags["arrived at corner"] = True
            adjust_heading(robot, ball_pos)
        else:
            angle = get_coord_angle(robot.robot_pos_arr[-1], robot.heading, corner)
            if abs(angle) > 90:
                move_to_point(robot, corner, False)
            else:
                move_to_point(robot, corner)

    else:
        corner = robot.goal[2]
        if get_dist(robot.robot_pos_arr[-1], corner) > 1:
            robot.flags["arrived at corner"] = False

        if (corner[0] - 0.04 <= robot.robot_pos_arr[-1][0] <= corner[0] + 0.04) and (
                corner[1] - 0.04 <= robot.robot_pos_arr[-1][1] <= corner[1] + 0.04):
            adjust_heading(robot, ball_pos)
        else:
            angle = get_coord_angle(robot.robot_pos_arr[-1], robot.heading, corner)
            if abs(angle) > 90:
                move_to_point(robot, corner, False)
            else:
                move_to_point(robot, corner)


def go_to_corner2(robot: RCJSoccerRobot):
    ball_data = get_ball_speed(robot)
    ball_pos = ball_data[2]
    rb_dist = get_dist(robot.robot_pos_arr[-1], ball_pos)
    print("Distance: {}".format(rb_dist))

    if rb_dist <= 0.09 and robot.flags["arrived at corner"]:
        if abs(get_coord_angle(robot.robot_pos_arr[-1], robot.heading, ball_pos)) < 90:
            move_to_point(robot, ball_pos)
        else:
            move_to_point(robot, ball_pos, False)
    elif ball_pos[1] > 0:
        corner = [0.75, 0.15]
        if get_dist(robot.robot_pos_arr[-1], corner) > 1:
            robot.flags["arrived at corner"] = False

        if (corner[0] - 0.04 <= robot.robot_pos_arr[-1][0] <= corner[0] + 0.04) and (
                corner[1] - 0.04 <= robot.robot_pos_arr[-1][1] <= corner[1] + 0.04):
            robot.flags["arrived at corner"] = True
            adjust_heading_to_angle(robot, 135, 5)
        else:
            angle = get_coord_angle(robot.robot_pos_arr[-1], robot.heading, corner)
            if abs(angle) > 90:
                move_to_point(robot, corner, False)
            else:
                move_to_point(robot, corner)

    else:
        corner = [0.75, -0.15]
        if get_dist(robot.robot_pos_arr[-1], corner) > 1:
            robot.flags["arrived at corner"] = False

        if (corner[0] - 0.04 <= robot.robot_pos_arr[-1][0] <= corner[0] + 0.04) and (
                corner[1] - 0.04 <= robot.robot_pos_arr[-1][1] <= corner[1] + 0.04):
            robot.flags["arrived at corner"] = True
            adjust_heading_to_angle(robot, 45, 5)
        else:
            angle = get_coord_angle(robot.robot_pos_arr[-1], robot.heading, corner)
            if abs(angle) > 90:
                move_to_point(robot, corner, False)
            else:
                move_to_point(robot, corner)


def check_strategy(robot: RCJSoccerRobot):
    # check if robot is currently intercepting the ball
    if robot.player_id == 3 and robot.time_step < 7 and robot.ball_pos_arr:
        move_to_point(robot, robot.ball_pos_arr[-1])
    else:
        assign_role(robot)
        role = robot.roles[robot.player_id - 1]
        if get_dist(robot.robot_pos_arr[0], robot.robot_pos_arr[-1]) > 1 or robot.flags["waiting_for_kickoff"] or robot.time_step < 3:
            if robot.flags["waiting_for_kickoff"]:
                robot.time_step = 0
            if robot.flags["waiting_for_kickoff"] and not (robot.flags["goal_registered"]):
                calculate_score(robot)
            print("stop")
            robot.set_left_vel(0)
            robot.set_right_vel(0)
        else:
            robot.flags["goal_registered"] = False
            if not (robot.flags["robot is stuck"] and (15 >= time.time() - robot.stuck_timer >= 10)):
                if not ((robot.flags["robot in penalty area"] and 15 >= time.time() - robot.penalty_area_timer >= 10) or
                        (not (robot.flags["robot in penalty area"]) and 3 >= time.time() - robot.outside_timer >= 0)) or robot.flags["ball is close"]:
                    robot.flags["defense_signal"] = False
                    if role == 1:
                        if robot.flags["intercepting ball"][0]:
                            defend_strategy_2(robot)
                        else:
                            defend_strategy_2(robot, False)
                    elif role == 2:
                        if not (robot.ball_status == 6):
                            defense(robot)
                    elif role == 3:
                        if robot.ball_pos_arr:
                            ready_to_defend(robot)
                    elif role == 4:
                        if robot.ball_pos_arr:
                            move_to_point(robot, robot.ball_pos_arr[-1])
                    elif role == 5:
                        if robot.ball_pos_arr:
                            mimic(robot)
                        else:
                            robot.set_left_vel(0)
                            robot.set_right_vel(0)
                    elif role == 6:
                        move_to_point(robot, [0.71, 0.1])
                    elif role == 7:
                        move_to_point(robot, [0.71, -0.1])
                    elif role == 8:
                        shoot(robot)
                    elif role == 9:
                        move_to_point(robot, (-0.3, 0.3))
                    elif role == 10:
                        if robot.ball_pos_arr:
                            receive_pass(robot)
                    else:
                        if robot.previous_ball_pos[0] >= 0:
                            x_sign = 1
                        else:
                            x_sign = -1
                        if robot.previous_ball_pos[1] >= 0:
                            y_sign = 1
                        else:
                            y_sign = -1
                        move_to_point(robot, (0.2 * x_sign, 0.2 * y_sign))

                else:
                    print("In penalty for long")
                    robot.flags["defense_signal"] = True
                    if robot.ball_pos_arr:
                        if not get_dist(robot.robot_pos_arr[-1], robot.ball_pos_arr[-1]) < 0.07:
                            robot.flags["ball is close"] = False
                            print("mimic {}".format(robot.player_id))
                            mimic(robot, 0.43)
                        else:
                            robot.flags["ball is close"] = True
                    else:
                        if robot.robot_pos_arr[-1][1] > 0:
                            move_to_point(robot, (0.43, 0.1))
                        else:
                            move_to_point(robot, (0.43, -0.1))
            else:
                print("robot is stuck")
                if robot.ball_pos_arr:
                    print("moving to ball {}".format(robot.player_id))
                    move_to_point(robot, robot.ball_pos_arr[-1])
                else:
                    print("moving to furthest coord {}".format(robot.player_id))
                    move_to_point(robot, robot.ultrasonic_data[robot.ultrasonic_arr[-1][0]])


def defense(robot: RCJSoccerRobot):
    if abs(robot.ball_pos_arr[-1][1]) > 0.15 and robot.ball_pos_arr[-1][0] > 0.4:
        go_to_corner2(robot)
    else:
        defend_mimic_at_goal(robot)


def receive_pass(robot: RCJSoccerRobot):
    if get_dist(robot.robot_pos_arr[-1], robot.ball_pos_arr[-1]) >= 0.13:
        if abs(robot.ball_pos_arr[-1][1]) >= 0.2:
            if robot.ball_pos_arr[-1][1] >= 0:
                sign = 1
                coord = [-0.66, -0.1]
            else:
                sign = -1
                coord = [-0.66, 0.1]

            if (coord[0] - 0.04 <= robot.robot_pos_arr[-1][0] <= coord[0] + 0.04) and (
                    coord[1] - 0.04 <= robot.robot_pos_arr[-1][1] <= coord[1] + 0.04):
                if adjust_heading_to_angle(robot, sign * 135):
                    stop_all(robot)
            else:
                move_to_point(robot, coord)
        elif robot.ball_pos_arr[-1][0] >= -0.56:
            # coord = [robot.ball_pos_arr[-1][0] + 0.1, robot.ball_pos_arr[-1][1] * -1]
            coord = [robot.ball_pos_arr[-1][0], robot.ball_pos_arr[-1][1] * -1]
            move_to_point(robot, coord)
        else:
            stop_all(robot)
    else:
        move_to_point(robot, robot.ball_pos_arr[-1])
        # stop_all(robot)

def ready_to_defend(robot: RCJSoccerRobot):
    if robot.flags["ready_to_push"] and get_dist(robot.robot_pos_arr[-1], robot.ball_pos_arr[-1]) <= 0.09:
        move_to_point(robot, robot.ball_pos_arr[-1])
    else:
        robot.flags["ready_to_push"] = False
        if robot.ball_pos_arr[-1][0] > 0.55 and abs(robot.ball_pos_arr[-1][1]) < 0.35:
            if robot.robot_pos_arr[-1][1] >= 0 or robot.flags["go_to_down_corner"]:
                coord = (0.7, 0.43)
                sign = -1
                robot.flags["go_to_down_corner"] = True
            else:
                coord = (0.7, -0.43)
                sign = 1
                robot.flags["go_to_down_corner"] = False
            if (coord[0] - 0.04 <= robot.robot_pos_arr[-1][0] <= coord[0] + 0.04) and (
                    coord[1] - 0.04 <= robot.robot_pos_arr[-1][1] <= coord[1] + 0.04):
                if adjust_heading_to_angle(robot, sign * 120):
                    robot.flags["ready_to_push"] = True
            else:
                move_to_point(robot, coord)
        else:
            robot.flags["go_to_down_corner"] = False
            move_to_point(robot, robot.ball_pos_arr[-1])


def defend_mimic_at_goal(robot: RCJSoccerRobot):
    # if robot.flags["robot is stuck"] and (15 >= time.time() - robot.stuck_timer >= 10):
    #     move_to_point(robot, robot.ball_pos_arr[-1])
    # else:

    # if get_dist(robot.robot_pos_arr[-1], [0.725, 0]) > 0.1 and abs(robot.robot_pos_arr[-1][0] - 0.725) > 0.1:
    #     robot.flags["adjusted heading"] = False
    #     move_to_point(robot, [0.725, 0])

    if abs(robot.ball_pos_arr[-1][1]) > 0.19:
        if robot.ball_pos_arr[-1][1] > 0:
            ball_y = 0.19
        else:
            ball_y = -0.19
    else:
        ball_y = robot.ball_pos_arr[-1][1]

    if abs(robot.robot_pos_arr[-1][0] - 0.66) > 0.1:
        robot.flags["adjusted heading"] = False
        goal_coord = [0.66, ball_y]
        if abs(get_coord_angle(robot.robot_pos_arr[-1], robot.heading, goal_coord)) > 90:
            move_to_point(robot, goal_coord, False)
        else:
            move_to_point(robot, goal_coord)
    else:

        if robot.flags["adjusted heading"]:

            coord = [0.66, ball_y]

            if (coord[0] - 0.04 <= robot.robot_pos_arr[-1][0] <= coord[0] + 0.04) and (
                    coord[1] - 0.04 <= robot.robot_pos_arr[-1][1] <= coord[1] + 0.04):
                robot.set_left_vel(0)
                robot.set_right_vel(0)
            else:
                ball_angle = get_coord_angle(robot.robot_pos_arr[-1], robot.heading, robot.ball_pos_arr[-1])
                if abs(ball_angle) > 90:
                    move_to_point(robot, coord, forward=False)

                else:

                    move_to_point(robot, coord)


        else:
            if robot.heading > 0:
                if adjust_heading_to_angle(robot, 90, s=7):
                    robot.flags["adjusted heading"] = True
            else:
                if adjust_heading_to_angle(robot, -90, s=7):
                    robot.flags["adjusted heading"] = True


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

        # robot.set_right_vel(0)
        # robot.set_left_vel(0)

        if robot.ball_pos_arr:
            adjust_heading(robot, robot.ball_pos_arr[-1])
        else:
            robot.set_right_vel(0)
            robot.set_left_vel(0)

        if time.time() - robot.start_time > 10:
            move_to_point2(robot, outside)

    elif robot.flags["moving backward"]:

        if (inside[0] - 0.05 <= robot.robot_pos_arr[-1][0] <= inside[0] + 0.05) and (
                inside[1] - 0.05 <= robot.robot_pos_arr[-1][1] <= inside[1] + 0.05):
            # get robot to move to y
            robot.flags["moving backward"] = False
            robot.flags["moving forward"] = True
            robot.start_time = time.time()

        # robot.set_right_vel(0)
        # robot.set_left_vel(0)

        if robot.ball_pos_arr:
            adjust_heading(robot, robot.ball_pos_arr[-1])
        else:
            robot.set_right_vel(0)
            robot.set_left_vel(0)

        # move to y
        if time.time() - robot.start_time > 2:
            move_to_point2(robot, inside, False)


def defend_strategy_2(robot: RCJSoccerRobot, was_intercepting=True):
    # check if function is called while robot is intercepting (no need to calculate)

    if not was_intercepting:
        robot.flags["intercepting ball"][0] = True
        robot.flags["intercepting ball"][1] = 2
        # predicted_pos = predict_ball_pos(robot, 18)
        prediction = predict_optimal_pos(robot, False)
        predicted_pos = prediction[0]

        print(prediction[2])

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
                robot.set_right_vel(0)
                robot.set_left_vel(0)
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
            robot.set_right_vel(0)
            robot.set_left_vel(0)
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
        if gradient == 0:
            gradient = 0.00000000000001
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
    print("robot {} is predicting".format(robot.player_id))
    # get speed, direction and position of ball
    speed = get_ball_speed(robot)

    # start position
    b1 = speed[2]

    # end position (after 45 time steps)
    b2 = predict_ball_pos(robot, 45)

    # print("b1: {}, b2: {}".format(b1, b2))

    # check if x or y coordinate is out of field boundaries
    if b2[0] > 0.7:
        b2 = [0.7, b2[1]]

    if b2[0] < -0.7:
        b2 = [-0.7, b2[1]]

    if b2[1] > 0.59:
        b2 = [b2[0], 0.59]

    if b2[1] < -0.59:
        b2 = [b2[0], -0.59]

    # print("b2 : {}".format(b2))

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
            # print("optimal r: {}, b: {}".format(robot_time[0], ball_time))
            # print("turn time: {}, b: {}, r: {}".format(turn_time, speed[1], robot_time[1]))
            return pos, final_robot_time, True

    # print("r: {}, b: {}".format(robot_time[0], ball_time))

    if speed[0] < 0.3:
        pos = b1
        robot_time = predict_robot_time(robot.robot_pos_arr[-1], robot.heading, pos, 0)
        robot_ball_angle = get_coord_angle(robot_time[2], robot_time[1], robot.ball_pos_arr[-1])
        turn_time = (abs(robot_ball_angle) / 360) * 14
        final_robot_time = robot_time[0] + turn_time + 4.5
        return pos, final_robot_time, True

    return pos, final_robot_time, False


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
                #     robot.set_right_vel(0)
                #     robot.set_left_vel(0)
                # else:
                #
                #     move_to_point2(robot, goal_pos, s=3)

                robot.set_right_vel(0)
                robot.set_left_vel(0)

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
    elif speed[0] > 3 and robot.flags["real ball speed"]:
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


def check_for_real_ball_speed(robot: RCJSoccerRobot, speed):
    if robot.temp_ball_speeds:
        if abs(robot.temp_ball_speeds[-1] - speed) > 1:
            robot.temp_ball_speeds = []
            robot.flags["real ball speed"] = False
        else:
            robot.temp_ball_speeds.append(speed)

        if len(robot.temp_ball_speeds) >= 5:
            robot.flags["real ball speed"] = True
            robot.temp_ball_speeds.pop(0)
        else:
            robot.flags["real ball speed"] = False
    else:
        robot.temp_ball_speeds.append(speed)


def check_for_real_robot_speed(robot: RCJSoccerRobot, speed):
    if robot.temp_robot_speeds:
        if abs(robot.temp_robot_speeds[-1] - speed) > 1:
            robot.temp_robot_speeds = []
            robot.flags["real robot speed"] = False
        else:
            robot.temp_robot_speeds.append(speed)

        if len(robot.temp_robot_speeds) >= 5:
            robot.flags["real robot speed"] = True
            robot.temp_robot_speeds.pop(0)
        else:
            robot.flags["real robot speed"] = False
    else:
        robot.temp_robot_speeds.append(speed)


def avoid_object(robot: RCJSoccerRobot, coord=(0, 0), ball=True):
    if ball:
        ball_data = get_ball_speed(robot)
        coord = ball_data[2]
        angle = ball_data[1]
    else:
        angle = 0
    angle_1 = (angle + 90) * math.pi / 180
    angle_2 = (angle - 90) * math.pi / 180
    pos_1 = (coord[0] + 0.1 * math.cos(angle_1), coord[1] + 0.1 * math.sin(angle_1))
    pos_2 = (coord[0] + 0.1 * math.cos(angle_2), coord[1] + 0.1 * math.sin(angle_2))
    if robot.robot_pos_arr[-1][1] >= 0:
        move_to_point(robot, pos_1)
    else:
        move_to_point(robot, pos_2)


def handle(robot: RCJSoccerRobot):
    ball_data = get_ball_speed(robot)
    dist = get_dist(ball_data[2], robot.robot_pos_arr[-1])
    robot_goal = get_coord_angle(robot.robot_pos_arr[-1], robot.heading, [-0.729, 0])
    robot_ball = get_coord_angle(robot.robot_pos_arr[-1], robot.heading, robot.ball_pos_arr[-1])
    predicted_dist = get_dist(robot.ball_predicted_pos, robot.robot_pos_arr[-1])

    diff = robot_goal - robot_ball

    if dist > 0.16:
        move_to_point(robot, robot.ball_pos_arr[-1])
    else:
        if not robot.flags["predicted"]:
            robot.ball_predicted_pos = predict_ball_pos(robot, 10)
            robot.flags["predicted"] = True
            robot.pos_test = robot.robot_pos_arr[-1]
        if predicted_dist > 0.16:
            move_to_point(robot, robot.ball_predicted_pos)
        else:
            if robot.flags["shooting from left"]:
                if robot.time_step - robot.shoot_start_time <= 3.5:
                    robot.set_right_vel(4.5)
                    robot.set_left_vel(10)
                elif robot.time_step - robot.shoot_start_time <= 10:
                    robot.set_right_vel(10)
                    robot.set_left_vel(3)
                elif robot.time_step - robot.shoot_start_time <= 13:
                    move_to_point(robot, robot.ball_pos_arr[-1])
                else:
                    robot.flags["shooting from left"] = False
                    robot.shoot_start_time = 0
                    robot.flags["predicted"] = False
                    print("current: {}, \nprevious: {}, \ndist: {}".format(
                        robot.robot_pos_arr[-1],
                        robot.pos_test,
                        get_dist(robot.robot_pos_arr[-1], robot.pos_test)
                    ))
            elif robot.flags["shooting from right"]:
                if robot.time_step - robot.shoot_start_time <= 3.5:
                    robot.set_right_vel(10)
                    robot.set_left_vel(4.5)
                elif robot.time_step - robot.shoot_start_time <= 10:
                    robot.set_right_vel(3)
                    robot.set_left_vel(10)
                elif robot.time_step - robot.shoot_start_time <= 13:
                    move_to_point(robot, robot.ball_pos_arr[-1])
                else:
                    robot.flags["shooting from right"] = False
                    robot.shoot_start_time = 0
                    robot.flags["predicted"] = False
                    print("current: {}, \nprevious: {}, \ndist: {}".format(
                        robot.robot_pos_arr[-1],
                        robot.pos_test,
                        get_dist(robot.robot_pos_arr[-1], robot.pos_test)
                    ))
            else:

                if abs(robot_ball) > 15:
                    move_to_point(robot, robot.ball_pos_arr[-1])
                else:
                    if robot.flags["shooting from left"]:
                        robot.flags["shooting from left"] = True
                        robot.shoot_start_time = robot.time_step
                    elif robot.flags["shooting from right"]:
                        robot.flags["shooting from right"] = True
                        robot.shoot_start_time = robot.time_step
                    else:
                        if robot_goal > 0:
                            robot.flags["shooting from left"] = True
                            robot.shoot_start_time = robot.time_step
                        else:
                            robot.flags["shooting from right"] = True
                            robot.shoot_start_time = robot.time_step


def check_ball_dir_robot(robot: RCJSoccerRobot):
    if len(robot.ball_pos_arr) > 2:
        first_pos = robot.ball_pos_arr[0]
        last_pos = robot.ball_pos_arr[-1]

        if get_dist(robot.robot_pos_arr[-1], last_pos) < get_dist(robot.robot_pos_arr[-1], first_pos):
            robot.flags["ball getting closer"] = True
        else:
            robot.flags["ball getting closer"] = False


def adjust_robot_penalty_time(robot: RCJSoccerRobot):
    if robot.robot_pos_arr:
        robot_pos = robot.robot_pos_arr[-1]
        if robot.robot_pos_arr[-1][0] > 0.47 and abs(robot_pos[1]) < 0.35:
            if not robot.flags["robot in penalty area"]:
                if time.time() - robot.outside_timer > 2:
                    robot.penalty_area_timer = time.time()
                    robot.outside_timer = time.time()
                robot.flags["robot in penalty area"] = True
        else:
            if robot.flags["robot in penalty area"]:
                robot.outside_timer = time.time()
                robot.flags["robot in penalty area"] = False
            if time.time() - robot.penalty_area_timer > 4:
                robot.penalty_area_timer = time.time()


def adjust_robot_stuck_timer(robot: RCJSoccerRobot):
    if robot.robot_pos_arr:
        if get_robot_speed(robot)[0] < 1 and robot.flags["real robot speed"]:
            if not robot.flags["robot is stuck"]:
                robot.stuck_timer = time.time()
                robot.flags["robot is stuck"] = True
                robot.stuck_pos = robot.robot_pos_arr[-1]
        else:
            if not ((robot.stuck_pos[0] - 0.1 <= robot.robot_pos_arr[-1][0] <= robot.stuck_pos[0] + 0.1) and (
                    robot.stuck_pos[1] - 0.1 <= robot.robot_pos_arr[-1][1] <= robot.stuck_pos[1] + 0.1)):
                robot.stuck_timer = 0
                robot.flags["robot is stuck"] = False
                robot.stuck_pos = [0, 0]


def mimic(robot: RCJSoccerRobot, x=0.52):
    # if robot.flags["robot is stuck"] and (15 >= time.time() - robot.stuck_timer >= 10):
    #     move_to_point(robot, robot.ball_pos_arr[-1])
    # else:

    if abs(robot.robot_pos_arr[-1][0] - x) > 0.1:
        robot.flags["adjusted heading"] = False

    if abs(robot.robot_pos_arr[-1][0] - x) > 0.05:
        move_to_point(robot, [x, robot.ball_pos_arr[-1][1]])
    else:

        if robot.flags["adjusted heading"]:
            coord = [x, robot.ball_pos_arr[-1][1]]

            if (coord[0] - 0.04 <= robot.robot_pos_arr[-1][0] <= coord[0] + 0.04) and (
                    coord[1] - 0.04 <= robot.robot_pos_arr[-1][1] <= coord[1] + 0.04):
                robot.set_left_vel(0)
                robot.set_right_vel(0)
            else:
                ball_angle = get_coord_angle(robot.robot_pos_arr[-1], robot.heading, robot.ball_pos_arr[-1])
                if abs(ball_angle) > 90:
                    move_to_point(robot, coord, forward=False)
                else:
                    move_to_point(robot, coord)

        else:
            if robot.heading > 0:
                if adjust_heading_to_angle(robot, 90, s=7):
                    robot.flags["adjusted heading"] = True
            else:
                if adjust_heading_to_angle(robot, -90, s=7):
                    robot.flags["adjusted heading"] = True


def mimic_with_distance(robot: RCJSoccerRobot, x=0.4):
    # if robot.flags["robot is stuck"] and (15 >= time.time() - robot.stuck_timer >= 10):
    #     move_to_point(robot, robot.ball_pos_arr[-1])
    # else:

    if abs(robot.robot_pos_arr[-1][0] - x) > 0.1:
        robot.flags["adjusted heading"] = False

    if abs(robot.robot_pos_arr[-1][0] - x) > 0.05:
        if robot.ball_pos_arr[-1][1] < 0:
            move_to_point(robot, [x, robot.ball_pos_arr[-1][1] + 0.15])
        else:
            move_to_point(robot, [x, robot.ball_pos_arr[-1][1] - 0.15])
    else:

        if robot.flags["adjusted heading"]:
            if robot.ball_pos_arr[-1][1] > 0:
                coord = [x, robot.ball_pos_arr[-1][1] - 0.15]
            else:
                coord = [x, robot.ball_pos_arr[-1][1] + 0.15]

            if (coord[0] - 0.04 <= robot.robot_pos_arr[-1][0] <= coord[0] + 0.04) and (
                    coord[1] - 0.04 <= robot.robot_pos_arr[-1][1] <= coord[1] + 0.04):
                robot.set_left_vel(0)
                robot.set_right_vel(0)
            else:
                ball_angle = get_coord_angle(robot.robot_pos_arr[-1], robot.heading, robot.ball_pos_arr[-1])
                if abs(ball_angle) > 90:
                    move_to_point(robot, coord, forward=False)
                else:
                    move_to_point(robot, coord)

        else:
            if robot.heading > 0:
                if adjust_heading_to_angle(robot, 90, s=7):
                    robot.flags["adjusted heading"] = True
            else:
                if adjust_heading_to_angle(robot, -90, s=7):
                    robot.flags["adjusted heading"] = True


def assign_role(robot: RCJSoccerRobot):
    get_real_ball_status(robot)
    ball_status = robot.ball_status
    if robot.player_id == 1:
        print("ball status: {}".format(ball_status))

        if robot.stuck and robot.player_id == 1 and time.time() - robot.timer > 6:
            if not (8 in robot.roles):
                if robot.ball_pos_arr:
                    ball_dist_arr = [("1", get_dist(robot.robot_pos_arr[-1], robot.ball_pos_arr[-1]))]
                    for name in robot.team_data:
                        if robot.team_data[name]:
                            ball_dist_arr.append(
                                (name[1], get_dist(robot.team_data[name]["robot_pos"], robot.ball_pos_arr[-1])))
                    ball_dist_arr.sort(key=lambda x: x[1])

                    robot.roles[int(ball_dist_arr[-1][0]) - 1] = 8
                else:
                    relocation_point = [0.1, 0]
                    relocation_point_dist_arr = [("1", get_dist(robot.robot_pos_arr[-1], relocation_point))]
                    for name in robot.team_data:
                        if robot.team_data[name]:
                            relocation_point_dist_arr.append(
                                (name[1], get_dist(robot.team_data[name]["robot_pos"], relocation_point)))
                    relocation_point_dist_arr.sort(key=lambda x: x[1])

                    robot.roles[int(relocation_point_dist_arr[-1][0]) - 1] = 8
        else:
            if ball_status in [1, 3, 4]:
                if robot.player_id == 1:
                    robots_arr = [1, 2, 3]
                    # Nearest to goal defends
                    robot_id = 0
                    if 2 in robot.roles:
                        robot_id = robot.roles.index(2) + 1
                    goal_dist_arr = []
                    for i in robots_arr:
                        if i == robot_id and ((
                                robot.robot_pos_arr[-1][0] > 0.47 and abs(robot.robot_pos_arr[-1][1]) < 0.35)
                                              or ball_status == 1):
                            # print("remove #Same goalie")
                            robots_arr.remove(i)
                        else:
                            if i != 1:
                                pos = robot.team_data["B" + str(i)]["robot_pos"]
                                goal_dist_arr.append(
                                    (i, get_dist(pos, robot.goal[1]))
                                )
                            else:
                                goal_dist_arr.append(
                                    (i, get_dist(robot.robot_pos_arr[-1], robot.goal[1]))
                                )

                    goal_dist_arr.sort(key=lambda x: x[1])

                    if robot_id == 0:
                        allowed_robots = []
                        if not robot.flags["defense_signal"]:
                            pos = robot.robot_pos_arr[-1]
                            if 1 in robots_arr:
                                allowed_robots.append((1, get_dist(pos, robot.goal[1])))

                        for name in robot.team_data:
                            if robot.team_data[name]:
                                if not robot.team_data[name]["defense_signal"]:
                                    pos = robot.team_data[name]["robot_pos"]
                                    if int(name[1]) in robots_arr:
                                        allowed_robots.append((int(name[1]), get_dist(pos, robot.goal[1])))

                        if allowed_robots:
                            allowed_robots.sort(key=lambda x: x[1])
                            robot.roles[allowed_robots[0][0] - 1] = 2
                            # print("remove #New goalie")
                            robots_arr.remove(allowed_robots[0][0])
                            for i in goal_dist_arr:
                                if i[0] == allowed_robots[0][0]:
                                    goal_dist_arr.remove(i)
                        else:
                            robot.roles[goal_dist_arr[0][0] - 1] = 3
                            # print("remove #goalie to be 3")
                            robots_arr.remove(goal_dist_arr[0][0])
                            goal_dist_arr.pop(0)

                    # if int(goal_dist_arr[0][0]) == 1:
                    #     defense_signal = robot.flags["defense_signal"]
                    # else:
                    #     defense_signal = robot.team_data["B" + str(goal_dist_arr[0][0])]
                    #
                    # if not defense_signal:
                    #     robot.roles[int(goal_dist_arr[0][0]) - 1] = 2
                    #     robots_arr.pop(int(goal_dist_arr[0][0]) - 1)
                    # elif robot.roles[int(goal_dist_arr[1][0]) - 1] == 3:
                    #     robot.roles[int(goal_dist_arr[1][0]) - 1] = 2
                    #     robots_arr.pop(int(goal_dist_arr[1][0]
                    # else:
                    #     pass
                    if ball_status == 1:
                        if not robot.flags["predicted"]:
                            predicted_time = predict_optimal_pos(robot)
                            if predicted_time[2]:
                                robot.predicted_intercept_time = predicted_time[1]
                            else:
                                robot.predicted_intercept_time = -1
                            robot.flags["predicted"] = True
                        else:
                            if robot.player_id == 1:
                                optimal_time_arr = []
                                # print("speed: {}, \nangle: {}".format(*get_ball_speed(robot)[:2]))
                                if 1 in robots_arr:
                                    optimal_time_arr = [("1", robot.predicted_intercept_time)]
                                for name in robot.team_data:
                                    if robot.team_data[name]:
                                        if robot.team_data[name]["predicted intercept time"] != -1 and (
                                                int(name[1]) in robots_arr):
                                            optimal_time_arr.append(
                                                (name[1], robot.team_data[name]["predicted intercept time"]))
                                # check if one of the robots can intercept the ball
                                if len(optimal_time_arr) <= 1:
                                    robot.roles[goal_dist_arr[0][0] - 1] = 3
                                    # print("remove #corner Intercept")
                                    robots_arr.remove(goal_dist_arr[0][0])
                                    goal_dist_arr.pop(0)
                                elif optimal_time_arr[0][1] == -1:
                                    robot.roles[goal_dist_arr[0][0] - 1] = 3
                                    # print("remove #corner Intercept")
                                    robots_arr.remove(goal_dist_arr[0][0])
                                    goal_dist_arr.pop(0)
                                else:
                                    optimal_time_arr.sort(key=lambda x: x[1])
                                    optimal_time_arr = list(filter(lambda x: x[1] > 0, optimal_time_arr))
                                    # Assign interceptor role to lowest time and pop him from array
                                    if optimal_time_arr[0][0] == "1":
                                        robot.roles[0] = 1
                                        # robots_arr.pop(0)
                                        # print("remove #intercept")
                                        robots_arr.remove(1)
                                    elif optimal_time_arr[0][0] == "2":
                                        robot.roles[1] = 1
                                        # robots_arr.pop(1)
                                        # print("remove #intercept")
                                        robots_arr.remove(2)
                                    elif optimal_time_arr[0][0] == "3":
                                        robot.roles[2] = 1
                                        # robots_arr.pop(2)
                                        # print("remove #intercept")
                                        robots_arr.remove(3)
                    elif ball_status in [3, 4]:
                        if robot.ball_pos_arr[-1][1] > 0:
                            corner = robot.goal[0]
                        else:
                            corner = robot.goal[2]

                        corner_dist_arr = []
                        for i in robots_arr:
                            if i != 1:
                                pos = robot.team_data["B" + str(i)]["robot_pos"]
                                corner_dist_arr.append(
                                    (i, get_dist(pos, corner))
                                )
                            else:
                                corner_dist_arr.append(
                                    (i, get_dist(robot.robot_pos_arr[-1], corner))
                                )

                        corner_dist_arr.sort(key=lambda x: x[1])
                        robot.roles[int(corner_dist_arr[0][0]) - 1] = 3
                        # print("remove #corner")
                        robots_arr.remove(int(corner_dist_arr[0][0]))
                    if 9 not in robot.roles:
                        robot.roles[int(robots_arr[0]) - 1] = 9
                    print("robots arr: {}".format(robots_arr))

            elif ball_status in [2, 5]:

                if robot.player_id == 1:

                    robots_arr = [1, 2, 3]

                    if 5 not in robot.roles:

                        # Assign closest 2 to follow and 3rd to mimic

                        ball_dist_arr = [("1", get_dist(robot.robot_pos_arr[-1], robot.ball_pos_arr[-1]))]

                        for name in robot.team_data:

                            if robot.team_data[name]:
                                ball_dist_arr.append(

                                    (name[1], get_dist(robot.team_data[name]["robot_pos"], robot.ball_pos_arr[-1])))

                        ball_dist_arr.sort(key=lambda x: x[1])

                        robot.roles[int(ball_dist_arr[2][0]) - 1] = 5

                        robot.roles[int(ball_dist_arr[1][0]) - 1] = 10

                        robot.roles[int(ball_dist_arr[0][0]) - 1] = 4
            elif ball_status == 6:
                allowed_roles = [-1, 6, 7]
                if not (robot.roles[robot.player_id - 1] in allowed_roles):
                    robot.roles[robot.player_id - 1] = -1

                if robot.player_id == 1 and robot.time_step > 2:
                    robots_arr = [1, 2, 3]
                    # Nearest to goal defends
                    up_corner_dist_arr = []
                    for i in robots_arr:
                        if i != 1:
                            pos = robot.team_data["B" + str(i)]["robot_pos"]
                            up_corner_dist_arr.append(
                                (i, get_dist(pos, robot.goal[0]))
                            )
                        else:
                            up_corner_dist_arr.append(
                                (i, get_dist(robot.robot_pos_arr[-1], robot.goal[0]))
                            )

                    up_corner_dist_arr.sort(key=lambda x: x[1])

                    robot.roles[int(up_corner_dist_arr[0][0]) - 1] = 6
                    robots_arr.pop(int(up_corner_dist_arr[0][0]) - 1)

                    down_corner_dist_arr = []
                    for i in robots_arr:
                        if i != 1:
                            pos = robot.team_data["B" + str(i)]["robot_pos"]
                            down_corner_dist_arr.append(
                                (i, get_dist(pos, robot.goal[2]))
                            )
                        else:
                            down_corner_dist_arr.append(
                                (i, get_dist(robot.robot_pos_arr[-1], robot.goal[2]))
                            )

                    down_corner_dist_arr.sort(key=lambda x: x[1])

                    robot.roles[int(down_corner_dist_arr[0][0]) - 1] = 7
                    robot.roles[int(down_corner_dist_arr[1][0]) - 1] = -1
