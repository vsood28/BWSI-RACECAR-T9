import time
import sys
import math
sys.path.insert(1, '../../library')

import racecar_core
import racecar_utils as rc_utils
import time

start_time = None

rc = racecar_core.create_racecar()


KD = 0
KP = 0.005

global angle
angle = 0.0

global error
error = 0.0

def start():
    global start_time

    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(0.4)

    start_time = time.time()

    rc.set_update_slow_time(0.5)

global lastError
lastError = error
def update():
    global angle, error, speed
    global lastError
    error = get_error(rc.lidar.get_samples())
    dt = rc.get_delta_time()
    angle = KP * error + KD * ((error - lastError) / dt)
    lastError = error
    angle = rc_utils.clamp(angle, -1, 1)
    speed = 0.5
    rc.drive.set_speed_angle(speed, angle)
    
def get_error(scan):
    n = len(scan)
    left = 0
    right = 0
    left_idx = 0
    right_idx = 0
    for i in range(-n//4, 0):
        dist = scan[i]
        if dist > left:
            left = dist
            left_idx = i
    for i in range(0, n //4):
        dist = scan[i]
        if dist > right:
            right = dist
            right_idx = i
    lw, rw = weight(left, right)
    mid_idx = right_idx * rw + left_idx * lw
    if mid_idx > 90:
        mid_idx -= 360
    return mid_idx        
                    

def weight(l, r):
    return l / (l + r), r / (l + r)

def update_slow():
    global start_time
    global angle, speed
    global error

    elapsed = time.time() - start_time

    print(f"Elapsed: {elapsed}")
    print(f"car Angle: {angle}")
    print(f"Error: {error}")


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()