import sys
import math


sys.path.insert(1, "../../library")


import racecar_core
import racecar_utils as rc_utils
import SLAMICP
import WFCFICP


rc = racecar_core.create_racecar()


ALPHA = 0.5


times = 0.0
last_filtered = 0.0


last_heading = 0.0


global angle
angle = 0.0
global lastError
lastError = 0




def start():
    global lastError
    lastError = 0
    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(0.2)
    rc.set_update_slow_time(0.5)

STOP = 100000
def update():
    global times
    global last_filtered, last_heading

    dt = rc.get_delta_time()
    times += dt

    scan = rc.lidar.get_samples()
    SLAMICP.submit_scan(scan) 
    x, y, h = SLAMICP.get_pose()    

    w = rc.physics.get_angular_velocity()
    gyrohead = last_heading + w[2] * dt

    h1 = ALPHA * math.sin(gyrohead) + (1 - ALPHA) * math.sin(h)
    h2 = ALPHA * math.cos(gyrohead) + (1 - ALPHA) * math.cos(h)
    heading = math.atan2(h1, h2)

    last_heading = heading

    print(f"X: {x}, Y: {y}, Yaw: {math.degrees(heading)}")

    if times <= STOP:
        global kP, kD, lastError, angle

        right_dist = rc_utils.get_lidar_average_distance(scan, 50, 20)
        left_dist = rc_utils.get_lidar_average_distance(scan, 310, 20)

        error = (left_dist - right_dist)

        angle = WFCFICP.KP * error + WFCFICP.KD * (error - lastError) / dt
        lastError = error
        angle = rc_utils.clamp(angle, -1, 1)
        rc.drive.set_speed_angle(0.8, -angle)


def update_slow():
    pass




if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()



