import math

def get_g_angle (robot_pos,orientation,coord):
    x= coord[0]
    y= coord[1]
    robot_angle: float = orientation
    if robot_pos[0] < 0 or robot_pos[1] <0:
        quad = -1
    else :
        quad = 1
    # Get the angle between the robot and the ball
    angle = math.atan2(
       y - robot_pos[0],
       x - robot_pos[1],
    )

    if angle < 0:
        angle = 2 * math.pi + angle

    if robot_angle < 0:
        robot_angle = 2 * math.pi + robot_angle

    final_angle = math.degrees(angle + robot_angle)

    final_angle -= 90
    if final_angle > 360:
        final_angle -= 360
    if final_angle < 0 :
        final_angle += 360

    print( "Final angle:",final_angle)
    print("Angle:", angle)
    robot_angle = (robot_angle*180)/math.pi
    print("robot angle",robot_angle)
    return final_angle,quad


'''def get_direction(angle):
    if angle >= 345 or angle <= 15:
        return 0
    elif angle >= 165 and angle <= 195:
        return 2
    elif (angle > 15 and angle <=180) : #or (angle > 195 and angle <= 270):
        print("right")
        return -1
    else :
        print("left")
        return 1
'''
def get_direction(ball_angle: float) -> int:
    """Get direction to navigate robot to face the ball

    Args:
        ball_angle (float): Angle between the ball and the robot

    Returns:
        int: 0 = forward, -1 = right, 1 = left
    """
    if ball_angle >= 350 or ball_angle <= 10:
        return 0
    return -1 if ball_angle < 180 else 1


def move (direction,quad):
    if direction == 0 :
        left_speed = -5 * quad
        right_speed = -5 * quad
        print("Fwd")
    elif direction == 2 :
        left_speed = 5
        right_speed = 5
        print("back")
    else :
        left_speed = direction * 2
        right_speed = direction * -2
    return left_speed,right_speed

def move_pos (robot_pos,orientation,coord):
    angle,quad = get_g_angle(robot_pos,orientation,coord)
    dir = get_direction(angle)
    speeds= move(dir,quad)
    return speeds