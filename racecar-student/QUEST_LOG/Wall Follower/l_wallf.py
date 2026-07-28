<<<<<<< HEAD
# Imports
import sys

sys.path.insert(1, '../../library')
import racecar_core 
import racecar_utils as rc_utils


# Global variables

rc = racecar_core.create_racecar()

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

    for i in range (-num//4, 0): # 270 to 360 degrees of the lidar scan
        if scan[i] == 0:
            scan[i] = LOOKAHEAD_DIST

        if scan[i] <= LOOKAHEAD_DIST and scan[i] >= largest: # average distance of every x points into _ and then nyoom
            startpos = i - len
            len = len + 1
            if len > largest_len:
                largest_len = len

        center = (startpos + (startpos + largest_len)) / 2

    for i in range (0, num//4): # 0 to 90 degrees of the lidar scan

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
=======
# Imports           
import sys

sys.path.insert(1, '../../library')
import racecar_core 
import racecar_utils as rc_utils

import numpy as np # maybe unnecessary?

# Global variables

rc = racecar_core.create_racecar()

BIG = 99999
WINDOW_LEN = 12 # 3 degrees 

speed = 1
angle = 0

largest_l = 0
window_l = []

largest_r = 0
window_r = []

# Functions
def start():    
    rc.drive.set_speed_angle(0.0, 0.0)
    rc.drive.set_max_speed(1)
    rc.set_update_slow_time(0.5)

def update():
    updateLargest()

    angle = rc_utils.clamp(angle, -1, 1)
    rc.drive.set_speed_angle(speed, angle)
    print("center: ", center, "angle: ", angle)

def updateLargest():
    global largest_l, largest_r
    global window_l, window_r

    scan = rc.lidar.get_samples() # 1080 points
    num = rc.lidar.get_num_samples() # 1080

    temp = []
    for i in range (-num//4, -num//4 + WINDOW_LEN):
        temp.append(i)
    window_l.extend(temp)

    temp.clear()
    for i in range (0, WINDOW_LEN):
        temp.append(i)
    window_r.extend(temp)

    for i in range (-num//4 + WINDOW_LEN + 1, 0): # 270 to 360 degrees of the lidar scan
            scan_l = scan
            if scan_l[i] == 0:
                scan_l[i] = BIG
    
            if 
            # i acc dont know how to implement this uhhhhhhhhhhhhhh

             if start > end:
                sScan = np.append(scan[start:], scan[:end + 1])
            else:
                sScan = scan[start:end + 1]

            if scan[i] <= LOOKAHEAD_DIST and scan[i] >= largest: # average distance of every x points into _ and then nyoom
                startpos = i - len
                len = len + 1
                if len > largest_len:
                    largest_len = len
    
            center = (startpos + (startpos + largest_len)) / 2
    
    for i in range (WINDOW_LEN + 1, num//4 - WINDOW_LEN): # 0 to 90 degrees of the lidar scan


        


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
>>>>>>> 9ec6a8e576bcf7fe4fb77c366c1e96e97f2b8022
    