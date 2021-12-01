def get_angle(directions):
    LR = directions[0]
    FB = directions[2]

    LR_angle = LR * 90

    if LR > 0:
        return 360 - LR_angle if FB >= 0 else 180 + LR_angle
    else:
        return -LR_angle if FB >= 0 else 180 + LR_angle
