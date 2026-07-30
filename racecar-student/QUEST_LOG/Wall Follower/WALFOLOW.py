from ftg_func import angle_to 
from ftg_func import tar_ang, farthest_gap
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

global angle
angle = 0.0

global error
error = 0.0

KP = 1.3
KD = 0.0

KPS = 0.002
KDS = 0

SPEED_BASELINE = 100 #speed baseline for beginning speed control

steering_pid = PID(kP=KP, kD=KD) #steering kp
speed_pid = PID(kP=KPS, kD=KDS) #speed kp

def start():
    global start_time

    steering_pid.reset()

    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(1)

    start_time = time.time()

    rc.set_update_slow_time(0.33)

tmp = None #var for display

def update():
    global angle, error, speed, tmp

    lg = farthest_gap(rc.lidar) #get farthest gap
    tmp = lg
    error = tar_ang(rc.lidar.get_samples(), rc.lidar.get_num_samples(), lg) #error using tar ang function given frathest gap and lidar

    angle = steering_pid.tick(0, error) #tar angel from steering angle

    angle = rc_utils.clamp(angle, -1, 1) #clamp

    smp = rc.lidar.get_samples()

    l = smp[0]

    #l  = sum(smp[-2:] + smp[:2] - smp[0]) / 3

    if l == 0: #actually inf
        l = 99999999

    speed = speed_pid.tick(SPEED_BASELINE, l) #speed pid
    speed = rc_utils.clamp(speed, 0.1, 1) #dont stop (believin')


    #rc.drive.set_speed_angle(speed, angle) #set


def update_slow():
    global start_time
    global angle, speed
    global error

    elapsed = time.time() - start_time

    print(f"car Angle: {angle}, speed: {speed} Error: {error * 180/math.pi}, lg:{tmp}") #print


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()