from cs_func import find_cones, target_point, angle_to_target, min_scan, closest_cone
import time

import sys
import math #Weighted ALgorithimic Farthest Opening LOokahead and Wayfinding 

sys.path.insert(1, '../../library')

import racecar_core
import racecar_utils as rc_utils
import time

start_time = None

rc = racecar_core.create_racecar()

class PID: #pid class very simple
    def __init__(self, kP=0,kI=0,kD=0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.prev_error = 0
        self.cum_i_val = 0
        self.prev_tick_called = 0

    def reset(self): #reset 
        self.prev_error = 0
        self.cum_i_val = 0
        self.prev_tick_called = 0

    def tick(self, setpoint, val, reset=False):
        if reset:
            self.reset()

        error = val - setpoint #def error
        dt = time.perf_counter() - self.prev_tick_called #dt

        p = self.kP * error #p
        self.cum_i_val += self.kI * error * dt #i
        d = self.kD * (error - self.prev_error) / dt #d

        self.prev_error = error
        self.prev_tick_called = time.perf_counter()

        return p + self.cum_i_val + d  #p + i + d

angle = 0.0

error = 0.0

prev_close = None
side = True

KP = 1.3
KD = 0.0

KPS = 0.002
KDS = 0

CONE_JUMP = 50

steering_pid = PID(kP=KP, kD=KD) #steering kp
speed_pid = PID(kP=KPS, kD=KDS) #speed kp

def start():
    global start_time

    steering_pid.reset()

    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(1)

    start_time = time.time()

    rc.set_update_slow_time(0.33)

window = 10

def update():
    global angle, error, speed, side, prev_close

    cn = find_cones(rc.lidar, 12, CONE_JUMP)

    closest = closest_cone(cn)

    smp = rc.lidar.get_samples()
    n = rc.lidar.get_num_samples()

    r = min_scan(smp, n//4, window)

    l = min_scan(smp, -n//4, window)

    print(f"{l}, {r}")
    #
    if (side and l != 0 and l < 60) or (not side and r != 0 and r < 60)    :
        side = not side
        print("flip")

    target = target_point(closest, side, 40)

    error = angle_to_target(target) #lookahead target

    s = rc.lidar.get_samples()

    i, d = rc_utils.get_lidar_closest_point(s, (-120, 120))

    
    angle = steering_pid.tick(0, error) #tar angel from steering angle

    angle = rc_utils.clamp(angle, -1, 1) #clamp

    speed_error = min(abs(math.pi/2 - error), abs(-math.pi/2 - error)) #redefine it to be inversely related to angle needed to turn, greater angle (closer to 90* is slower)

    speed = speed_pid.tick(0, speed_error) #speed pid

    speed = rc_utils.clamp(speed, 0.1, 1) #dont stop (believin')

    rc.drive.set_speed_angle(speed, angle) #set

import matplotlib.pyplot as plt

def display_pts(pts, f_name="pts"):
    plt.figure()
    x = [pt.x for pt in pts]
    y = [pt.y for pt in pts]

    plt.scatter(x, y, 0.4)
    plt.savefig(f"{f_name}.png")

def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()