import sys
import math
import time
import filters

sys.path.insert(1, '../../library')

import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

old_time = 0.0
velocity_x = 0.0
velocity_y = 0.0
velocity_z = 0.0

filter = filters.KalmanFilter(0.1, 0) # change to 1 if needed

def start():
    print("start button pressed")
    rc.drive.stop()
    rc.drive.set_speed_angle(0, 0)

def update():
    # global old_time # ?

    new_time = time.time()
    
    if old_time == 0.0:
        old_time = new_time
        return
    
    dt = new_time - old_time
    old_time = new_time

    accel = rc.physics.get_linear_acceleration()
    velocity_y = velocity_y + accel[1] * dt

    smoothed = filter.update(velocity_y)
    print(smoothed)

def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()