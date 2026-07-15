"""
luke code
wall follower, numbers need to be tuned for irl
"""

########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
speed = 0
angle = 0

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed, angle
    print("start button pressed")
        
    speed = 1
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.2)

def update(): 
    global left_dist, right_dist
    global angle, speed

    scan = rc.lidar.get_samples()

    right_dist = rc_utils.get_lidar_average_distance(scan, 70, 10)
    left_dist = rc_utils.get_lidar_average_distance(scan, 290, 10)

    if left_dist == 0 and right_dist == 0:
        angle = 0
    else:
        dist = right_dist - left_dist
        setAngle(dist)
    
    rc.drive.set_speed_angle(speed, angle)

def setAngle(distance):
    global angle

    if distance is not None and distance != 0:
        setpoint = 0
        kp = -0.003
        error = setpoint - distance
        angle = kp * error

        angle = rc_utils.clamp(angle, -1, 1)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    print("speed:", speed, "angle", angle, "left", left_dist, "right", right_dist)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
