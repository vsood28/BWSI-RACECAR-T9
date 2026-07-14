import sys
import numpy as np
from enum import Enum

sys.path.insert(1, "../library")
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

DESIRED_WALL_DIST = 120.0 
WALL_THRESHOLD = 500.0  
FRONT_CRITICAL = 60.0     
FRONT_CLEAR = 90.0       

STEER_KP = -0.008         

class State(Enum):
    FORWARD = 0
    REVERSE = 1

state = State.FORWARD
backup_dir = 1.0

def get_lidar_distance(scan_data, angle_start, angle_end):
    points = []
    for angle in range(angle_start, angle_end):
        dist = scan_data[angle % 360]
        if 0.0 < dist < 1000.0:
            points.append(dist)
    return np.median(points) if points else 999.0

def start():
    global state
    state = State.FORWARD
    rc.drive.stop()


def update():
    global state, backup_dir

    scan_data = rc.lidar.get_samples()
    if scan_data is None:
        return
    front_dist = get_lidar_distance(scan_data, -10, 10)
    left_dist = get_lidar_distance(scan_data, 260, 280)
    right_dist = get_lidar_distance(scan_data, 80, 100)

    if state == State.FORWARD:
        if front_dist < FRONT_CRITICAL:
            state = State.REVERSE
            backup_dir = 0.9 if (left_dist < right_dist) else -0.9
            
    elif state == State.REVERSE:
        if front_dist > FRONT_CLEAR:
            state = State.FORWARD

    if state == State.REVERSE:
        rc.drive.set_speed_angle(-0.25, backup_dir)
        return

    left_valid = left_dist < WALL_THRESHOLD
    right_valid = right_dist < WALL_THRESHOLD
    
    error = 0.0

    if left_valid and right_valid:
        error = right_dist - left_dist

    elif left_valid:
        error = (DESIRED_WALL_DIST - left_dist) * 2.0

    elif right_valid:
        error = (right_dist - DESIRED_WALL_DIST) * 2.0

    else:
        error = 0.0

    angle = rc_utils.clamp(error * STEER_KP, -1.0, 1.0)

    speed = 0.42 * (1.0 - 0.45 * abs(angle))
    speed = rc_utils.clamp(speed, 0.15, 0.45)

    rc.drive.set_speed_angle(speed, angle)

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()