# Imports           
import sys

sys.path.insert(1, '../../library')
import racecar_core 
import racecar_utils as rc_utils

import numpy as np # maybe unnecessary?

# Global variables

rc = racecar_core.create_racecar()

# BIG = 99999
WINDOW_LEN = 10 # 5 degrees in each direction

speed = 0
angle = 0

largest_l = 0
index_l = 360

largest_r = 0
index_r = 0

weight = 200
kP = 0.05

# Functions
def start():    
    rc.drive.set_speed_angle(0.0, 0.0)
    rc.drive.set_max_speed(1)
    rc.set_update_slow_time(0.5)

def update():
    updateLargest()

    rc.drive.set_speed_angle(speed, angle)
    print("==========================================================================")
    print("left_angle: ", left_angle, "left_angle_weight: ", left_angle_weight)
    print("right_angle: ", right_angle, "right_angle_weight: ", right_angle_weight)
    print("speed: ", speed, "angle: ", angle)

def updateLargest():
    global left_angle, right_angle
    global left_angle_weight, right_angle_weight
    global largest_l, largest_r
    global index_l, index_r
    global speed, angle

    scan = rc.lidar.get_samples() # 1080 points
    num = rc.lidar.get_num_samples() # 1080

    largest_l = 0
    index_l = 360

    largest_r = 0
    index_r = 0

    scan_l = scan[270*3:360*3]
    scan_r = scan[0:90*3]

    for i in range (270, 360): # 270 to 360 degrees of the lidar scan
        l_val = rc_utils.get_lidar_average_distance(scan_l, i, WINDOW_LEN)
            
        if l_val > largest_l:
            largest_l = l_val   
            index_l = i # e.g. 280
    
    for i in range (0, 90): # 0 to 90 degrees of the lidar scan
        r_val = rc_utils.get_lidar_average_distance(scan_r, i, WINDOW_LEN) 
                    
        if r_val > largest_r:
            largest_r = r_val
            index_r = i # e.g. 45

    left_angle = (360 - index_l) * -1 # - from 0
    right_angle = index_r

    # add weights together: minimum 0.5 and max 1.0 for an individual weight
    if largest_r > largest_l:
        right_angle_weight = (largest_r - largest_l) / weight
        if right_angle_weight > 0.5:
            right_angle_weight = 0.5
        left_angle_weight = right_angle_weight * -1
    else:
        left_angle_weight = (largest_l - largest_r) / weight
        if left_angle_weight > 0.5:
            left_angle_weight = 0.5
        right_angle_weight = left_angle_weight * -1

    left_angle_weight += 0.5
    right_angle_weight += 0.5

    error = ((left_angle * left_angle_weight) + (right_angle * right_angle_weight)) / 2
    angle = error * kP

    if abs(angle) > 0.15:
        speed = -0.4 * abs(angle) + 1
    else:
        speed = 1

    speed = rc_utils.clamp(speed, -1, 1)
    angle = rc_utils.clamp(angle, -1, 1)

def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
    