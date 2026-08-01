# traded for code from team 1. essentially a furthest point follower

MAX_SPEED = 1.0

angle = 0.0
speed = MAX_SPEED
last_error = 0.0

kP = 1.1
kD = 0.02

import sys
import time

import numpy as np

#TODO: add the method for zereos and just use static rays

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar(isSimulation=False)
time.sleep(2)

MIN_ZEROES = 5

def findMaxW(scan, minA, maxA, window):
    length = scan.shape[0]

    originA = minA
    minA %= 360
    maxA %= 360

    start = int(minA * length / 360)
    end = int(maxA * length / 360)

    if start > end:
        sScan = np.append(scan[start:], scan[:end + 1])
    else:
        sScan = scan[start:end + 1]

    wSize = int(window * length / 360)
    sums = np.convolve(sScan, np.ones(wSize), "valid")
    center = int(sums.argmax()) + wSize // 2

    return originA + center * 360 / length

def start():
    rc.drive.stop()
    rc.set_update_slow_time(0.1)
    rc.drive.set_max_speed(MAX_SPEED)

def get_zeroes(scan):
    num = 0
    for i in range(len(scan)):
        if scan[i] == 0:
            num += 1
    return num        

def update():
    global speed, angle, last_error

    scan = rc.lidar.get_samples()

    #if get_zeroes(scan) >= MIN_ZEROES:
        #reg_follow()

    raw_gap = findMaxW(scan, -90, 90, 30)

    error = raw_gap / 180
    dt = rc.get_delta_time()
    d_error = (error - last_error) / dt if dt > 0 else 0.0
    last_error = error
    angle = float(np.clip(error * kP + d_error * kD, -1.0, 1.0))
    speed = 1

    rc.drive.set_speed_angle(speed, rc_utils.clamp(angle, -1.0, 1.0))
    print(f"raw raw_gap: {raw_gap}, angle: {angle},  speed: {speed}")

def reg_follow():
    pass


rc.set_start_update(start, update)
rc.go()