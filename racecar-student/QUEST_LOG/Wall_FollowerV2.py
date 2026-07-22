from ftg_func import angle_to
from ftg_func import largest_gap

import sys
import math

sys.path.insert(1, '../../library')

import racecar_core
import racecar_utils as rc_utils
import time

start_time = None

rc = racecar_core.create_racecar()

global lastError
lastError = 0

KP = 0.00185
KD = 0.02

def start():
    global lastError
    global start_time

    lastError = 0
    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(1)

    start_time = time.time()



global angle
angle = 0.0
global error
error = 0.0
def update():
    global lastError, angle, error

    error = angle_to(largest_gap(rc.lidar))

    dt = rc.get_delta_time()

    angle = KP * error + KD * ((error - lastError) / dt)
   
    lastError = error
    angle = rc_utils.clamp(angle, -1, 1)
    rc.drive.set_max_speed(1)
    rc.drive.set_speed_angle(1, angle)


def update_slow():
    global start_time
    global angle
    global error
    elapsed = time.time() - start_time
    print(f"Elapsed: {elapsed}")
    print(f"Angle: {angle}")
    print(f"Error: {error}")


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
