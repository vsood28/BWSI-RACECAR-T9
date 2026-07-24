# Imports

import sys

sys.path.insert(1, '../../library')
import racecar_core 
import racecar_utils as rc_utils


# Global variables

rc = racecar_core.create_racecar()

LOOKAHEAD_DIST = 500
speed = 1

# Functions
def start():    
    rc.drive.set_speed_angle(0.0, 0.0)
    rc.drive.set_max_speed(1)
    rc.set_update_slow_time(0.5)

def update():
    scan = rc.lidar.get_samples() # 1080 points
    num = rc.lidar.get_num_samples() # 1080
    largest = 0
    len = 0
    largest_len = 0
    startpos = 0
    center = 0

    for i in range (-num//4, num//4): # front half of the lidar scan
        if scan[i] == 0:
            scan[i] = LOOKAHEAD_DIST

        if scan[i] <= LOOKAHEAD_DIST and scan[i] >= largest: # average distance of every x points into _ and then nyoom
            startpos = i - len
            len = len + 1
            if len > largest_len:
                largest_len = len

        center = (startpos + (startpos + largest_len)) / 2

    angle = center / 270.0
    angle = rc_utils.clamp(angle, -1, 1)
    rc.drive.set_speed_angle(speed, angle)
    print("center: ", center, "angle: ", angle)

def setAngle(distance):
    global angle

    if distance is not None:
        setpoint = 0
        kp = -5
        error = setpoint - distance
        angle = kp * error

        angle = rc_utils.clamp(angle, -1, 1)

def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
    