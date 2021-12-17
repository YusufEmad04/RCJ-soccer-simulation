import math

def get_g_angle (robot_pos,orientation,coord):
    x= coord[0]
    y= coord[1]
    robot_angle: float = math.radians(orientation)
    # Get the angle between the robot and the ball
    angle = math.atan2(
       y - robot_pos[0],
       x - robot_pos[1],
    )
    angle += 0.5 * math.pi

    if angle < - math.pi :
        angle = (2 * math.pi) + angle
    if angle > math.pi:
        angle = angle - (2 * math.pi)

    angle = angle * -1
    final_angle = math.degrees(angle - robot_angle)
    final_angle += 180

    if final_angle > 180:
        final_angle -= 360
    if final_angle < -180 :
        final_angle += 360
    print( "Final angle:",final_angle)
    print("Angle:", math.degrees(angle))
    print("robot angle",math.degrees(robot_angle))
    return final_angle


def get_direction(angle):
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