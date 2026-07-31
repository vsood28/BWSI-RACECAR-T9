# Imports           
import sys

sys.path.insert(1, '../../library')
import racecar_core 
import racecar_utils as rc_utils

import numpy as np # maybe unnecessary?

# Global variables

rc = racecar_core.create_racecar()

BIG = 99999
WINDOW_LEN = 16 # 8 degrees in each direction

speed = 1
angle = 0

largest_l = 0
index_l = 270
scan_l = []
# window_l = []

largest_r = 0
index_r = 0
scan_r = []
# window_r = []

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
    global index_l, index_r
    global scan_l, scan_r
    # global window_l, window_r

    scan = rc.lidar.get_samples() # 1080 points
    num = rc.lidar.get_num_samples() # 1080

    temp = []
    for i in range (-num//4, -num//4 + WINDOW_LEN): # add 270 - 273 degrees to the window (12 values)
        if scan[i] == 0:
            scan_l[i] = BIG

        temp.append(scan[i])
    window_l.extend(temp) 

    temp.clear()
    for i in range (0, WINDOW_LEN): # add 0 - 3 degrees to the window (12 values)
        if scan[i] == 0:
            scan_r[i] = BIG

        temp.append(scan[i])
    window_r.extend(temp)
    # temp should get garbage collected

    for i in range (-num//4 + WINDOW_LEN + 1, 0): # 270 to 360 degrees of the lidar scan
            scan_l = scan # this is probably inefficient
            if scan_l[i] == 0:
                scan_l[i] = BIG

            val = rc_utils.get_lidar_average_distance(scan_l, i, WINDOW_LEN)
            
            if val > largest_l:
                
    
            center = (startpos + (startpos + largest_len)) / 2
    
    for i in range (WINDOW_LEN + 1, num//4 - WINDOW_LEN): # 0 to 90 degrees of the lidar scan
        pass


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
    