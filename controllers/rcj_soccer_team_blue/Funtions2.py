def get_direction(angle):
    if angle >= 345 or angle <= 15:
        return 0
    
    return -1 if angle < 180 else 1