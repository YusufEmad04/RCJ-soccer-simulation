import math
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

    dist_x = coord_dist * math.cos((heading + coord_angle) * math.pi / 180)  # getting x-axis distance from robot to ball
    dist_y = coord_dist * math.sin((heading + coord_angle) * math.pi / 180)  # getting y-axis distance from robot to ball
    ball_pos_x = robot_x + dist_x  # getting x-axis coordinates of ball
    ball_pos_y = robot_y + dist_y  # getting x-axis coordinates of ball

    return ball_pos_x, ball_pos_y


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

    d = {
        "heading": heading,
        "robot position": robot_pos,
        "supervisor data": data,
        "ultrasonic": {
            "front": (robot_pos[0] + ultrasonic_front[0], robot_pos[1] + ultrasonic_front[1]),
            "back": (robot_pos[0] + ultrasonic_back[0], robot_pos[1] + ultrasonic_back[1]),
            "right": (robot_pos[0] + ultrasonic_right[0], robot_pos[1] + ultrasonic_right[1]),
            "left": (robot_pos[0] + ultrasonic_left[0], robot_pos[1] + ultrasonic_left[1]),
        },
        "team data": {
            "B1": r[0],
            "B2": r[1],
            "B3": r[2]
        }

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


def receive_ball_data(robot: RCJSoccerRobot, heading, robot_pos):
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


def move_to_point(robot: RCJSoccerRobot, robot_pos, heading, coord):
    angle = get_coord_angle(robot_pos, heading, coord)

    # checking coordinate is on right
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

    return 100 * dist / time


def get_robot_speed(robot: RCJSoccerRobot):
    return get_speed(robot.time_steps_arr, robot.robot_pos_arr)


def increment_step(robot: RCJSoccerRobot):
    robot.time_step += 0.5
    add_to_arr(robot.time_steps_arr, robot.time_step)


def clear_ball_data(robot: RCJSoccerRobot):
    robot.ball_pos_arr = []


def get_ball_speed(robot: RCJSoccerRobot, this_robot, team_ball_data=None):
    # check if ball position came from this robot
    if this_robot:
        return get_speed(robot.time_steps_arr, robot.ball_pos_arr)
    else:
        ball_pos = []

        # getting ball position from other robots
        for robot_id in team_ball_data:
            if team_ball_data[robot_id]:
                if team_ball_data[robot_id]["see the ball"]:
                    ball_pos.append(team_ball_data[robot_id]["ball_pos"])

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
