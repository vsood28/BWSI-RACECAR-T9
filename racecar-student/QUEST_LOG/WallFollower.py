import sys
import math

sys.path.insert(1, '../../library')

import racecar_core
import racecar_utils as rc_utils
import WFC
import importlib

rc = racecar_core.create_racecar()

global lastError
lastError = 0

def start():
    global lastError

    lastError = 0
    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(1)

global angle
angle = 0.0
def update():
    global kP, kD, lastError, angle

    importlib.reload(WFC)
    scan = rc.lidar.get_samples()

    #implement normalization condition.
    #get left and right lidar scans with average distances
    right_dist = rc_utils.get_lidar_average_distance(scan, 50, 10)
    left_dist = rc_utils.get_lidar_average_distance(scan, 310, 10)

    error = (left_dist - right_dist) / (left_dist + right_dist) 

    dt = rc.get_delta_time()

    angle = WFC.KP * error + WFC.KD * (error - lastError) / dt
    #print("Proportional")
    #print(WFC.KP * error)
    #print("Derivative")
    #print(WFC.KD * ((error - lastError) / dt))
    lastError = error
    angle = rc_utils.clamp(angle, -1, 1)
    rc.telemetry.declare_variables("Angle", "Error")
    rc.telemetry.record(angle, error)
    rc.drive.set_max_speed(1)
    rc.drive.set_speed_angle(0.9, -angle)


def update_slow():
    global angle
    global lastError
    global kP
    print(f"Last Error: {lastError}") #print statements for debugging
    print(f"Angle: {angle}")


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
    rc.telemetry.visualize()