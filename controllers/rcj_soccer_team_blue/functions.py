import math

def get_angle(directions):
    LR = directions[0]
    FB = directions[2]
    return -math.atan2(LR, FB)*180/math.pi
