# attempt at eats implementation

# Imports           
import sys

sys.path.insert(1, '../../library')
import racecar_core 
import racecar_utils as rc_utils

import numpy as np # maybe unnecessary?

# Global variables

rc = racecar_core.create_racecar()

# BIG = 99999
WINDOW_LEN = 10 # looking 5 degrees in each direction

speed = 0
angle = 0

largest_l = 0 # distance of the largest distance found on left side
index_l = 360 # ^ its index

largest_r = 0
index_r = 0

weight = 200 # raise number to lower weights
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
    # i think some of these globals are unnecessary but my code wasn't working without them
    global left_angle, right_angle
    global left_angle_weight, right_angle_weight
    global largest_l, largest_r
    global index_l, index_r
    global speed, angle

    scan = rc.lidar.get_samples() # 1080 points
    num = rc.lidar.get_num_samples() # 1080

    largest_l = 0 
    index_l = 360 

    largest_r = 0 # same as above
    index_r = 0

    scan_l = scan[270*3:360*3] # 270 to 360 deg
    scan_r = scan[0:90*3] # 0 to 90 deg

    # was going to set up a solution for zeros using like shifting arrays 
    # but then i couldnt figure it out so they got ignored

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

    left_angle = (360 - index_l) * -1 # making negative from zero instead of positive
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

    left_angle_weight += 0.5 # setting baseline to 0.0 so even low values are considered reasonably
    right_angle_weight += 0.5

    error = ((left_angle * left_angle_weight) + (right_angle * right_angle_weight)) / 2 
    # 2 is changable number to scale down error
    # weighted average of left and right angles * weights
    # icl this is gpted
    angle = error * kP

    if abs(angle) > 0.15:
        speed = -0.4 * abs(angle) + 1 # 0.4 needs to be tuned but this is a speed controller 
        # (graph inverse |x| and then move it up 1 and smooth it out by multiplying by a fraction)
    else:
        speed = 1

    speed = rc_utils.clamp(speed, -1, 1)
    angle = rc_utils.clamp(angle, -1, 1)

def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
    