import math

def get_angle(directions):
    LR = directions[0]
    FB = directions[2]
    angle = -math.atan2(LR, FB)*180/math.pi
    return angle


def get_distance(r):
    power = float(r) ** (-0.507)
    return 1.0146 * power



