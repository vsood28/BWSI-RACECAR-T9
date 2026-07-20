import sys
import math

sys.path.insert(1, '../../library')

import racecar_core
import racecar_utils as rc_utils
import WFC
import importlib
import csv
import time

log_file = None
log_writer = None
start_time = None

rc = racecar_core.create_racecar()

global lastError
lastError = 0

def start():
    global lastError
    global log_file, log_writer, start_time

    lastError = 0
    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(1)

    start_time = time.time()

    log_file = open("wall_follow_log.csv", "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow(["time", "error", "angle", "proportional", "derivative"])


global angle
angle = 0.0
speed = 0.8 #constant for speed to easily adjust
def update():
    global kP, kD, lastError, angle

    importlib.reload(WFC) #import variables from a config file so that they can be changed without restarting the program
    scan = rc.lidar.get_samples()

    #implement normalization condition.
    #get left and right lidar scans with average distances
    right_dist = rc_utils.get_lidar_average_distance(scan, 60, 20)
    left_dist = rc_utils.get_lidar_average_distance(scan, 300, 20)

    error = (left_dist - right_dist) # error is how far we are from center

    dt = rc.get_delta_time()

    angle = WFC.KP * error + WFC.KD * (error - lastError) / dt #normal pd controller for angle
    print(f"Proportional term: {WFC.KP * error},   Derivative term: {WFC.KD * (error - lastError) / dt}")
   

    elapsed = time.time() - start_time
    log_writer.writerow([elapsed, error, angle, error * WFC.KP, ((error - lastError) / dt) * WFC.KD])
    lastError = error
    angle = rc_utils.clamp(angle, -1, 1) #clamp angle so we don't return errors
    rc.drive.set_max_speed(1)
    rc.drive.set_speed_angle(speed, -angle)


def update_slow():
    global angle
    global lastError
    global kP
    print(f"Last Error: {lastError}") #print statements for debugging
    print(f"Angle: {angle}")


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    try:
        rc.go()
    finally:
        if log_file is not None:
            log_file.close()