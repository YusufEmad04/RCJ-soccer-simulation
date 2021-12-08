import math

def get_angle(directions):
    LR = directions[0]
    FB = directions[2]
    angle = -math.atan2(LR, FB)*180/math.pi
    return angle if angle > 0 else 360 + angle
