MAX_SPEED = 1.0

angle = 0.0
speed = MAX_SPEED
last_error = 0.0

kP = 1.1
kD = 0.02


lastError = 0

import sys
import time
import math

import numpy as np

#calculate slopes for straight segments.
#TODO: add the method for zereos and just use static rays


sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar(isSimulation=False)
time.sleep(2)

MIN_ZEROES = 5
def tocart(dist, index):
    angle = math.radians(index / 1080)
    return (dist * math.cos(angle), dist * math.sin(angle))

def is_straight(scan):
    n = len(scan)
    left1 = scan[n // 4]
    left2 = scan[(n // 4) - 170]
    right1 = scan[(3*n//4)]
    right2 = scan[(3*n//4) + 170]
    left1p = tocart(left1, n // 4)
    left2p = tocart(left2, (n//4) - 170)
    right1p = tocart(right1, (3*n//4))
    right2p = tocart(right2, (3*n//4) + 170)
    leftdiff = (left2p[0] - left1p[0])
    rightdiff = (right2p[0] - right1p[0])
    if abs(leftdiff) <= 5 and abs(rightdiff) <= 5:
        return True
    return False


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
 

def update():
    global speed, angle, last_error
    #markers = rc_utils.get_ar_markers()
    scan = rc.lidar.get_samples()
    dt = rc.get_delta_time()
    scan = process(scan)
    raw_gap = findMaxW(scan, -90, 90, 30)
    error = raw_gap / 180
    d_error = (error - last_error) / dt if dt > 0 else 0.0
    last_error = error
    angle = float(np.clip(error * kP + d_error * kD, -1.0, 1.0))
    print(f"raw raw_gap: {raw_gap}, angle: {angle},  speed: {speed}")
    if abs(angle) > 0.1:
        speed = -0.6 * abs(angle) + 1   
    else:
        speed = 1
    str = is_straight(scan)
    if str:
        print("Straight")    
    rc.drive.set_speed_angle(rc_utils.clamp(speed, -1,1), rc_utils.clamp(angle - 0.05, -1.0, 1.0))    

def process(scan):
    n = len(scan)
    scan = scan.copy()
    for i in range(-n//4, n//4):
        if scan[i] == 0:
            scan[i] = 1000
    return scan        


rc.set_start_update(start, update)
rc.go()