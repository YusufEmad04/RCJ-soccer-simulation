import math


def get_coord_angle(robot_pos, heading, coord):
    # x = coord[0]
    # y = coord[1]
    # robot_angle: float = math.radians(orientation)
    # # Get the angle between the robot and the ball
    # angle = math.atan2(
    #     y - robot_pos[0],
    #     x - robot_pos[1],
    # )
    # # set 0 at east side of field
    # angle += 0.5 * math.pi
    # # Make angle between 180 and -180
    # if angle < - math.pi:
    #     angle = (2 * math.pi) + angle
    # if angle > math.pi:
    #     angle = angle - (2 * math.pi)
    # # Make clockwise +ve
    # angle = angle * -1
    # # Get angle robot needs to rotate
    # final_angle = math.degrees(angle - robot_angle)
    # # Swap faces to measure angle from blue
    # final_angle += 180
    # # Make angle between 180 and -180
    # if final_angle > 180:
    #     final_angle -= 360
    # if final_angle < -180:
    #     final_angle += 360
    # return final_angle
    x = robot_pos[0] - coord[0]
    y = robot_pos[1] - coord[1]

    if x == 0:
        x = 0.00000000001

    a = (math.atan(y / x)*180/math.pi) #getting tan value

    #checking sides signs to get angle (without heading)
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


    #getting angle after getting heading
    if angle - heading < -180:
        return 360 - (heading-angle)
    elif angle - heading > 180:
        return -360 + (angle-heading)
    else:
        return angle - heading



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
    angle, quad = get_g_angle(robot_pos, orientation, coord)
    dir = get_direction(angle)
    speeds = move(dir, quad)
    return speeds
