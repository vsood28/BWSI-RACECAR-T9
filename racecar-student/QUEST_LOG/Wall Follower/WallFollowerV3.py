from ftg_func import angle_to
from ftg_func import largest_gap
import time

import sys
import math

sys.path.insert(1, '../../library')

import racecar_core
import racecar_utils as rc_utils
import time

start_time = None

rc = racecar_core.create_racecar()

class PID:
    def __init__(self, kP=0,kI=0,kD=0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.prev_error = 0
        self.cum_i_val = 0
        self.prev_tick_called = 0

    def reset(self):
        self.prev_error = 0
        self.cum_i_val = 0
        self.prev_tick_called = 0

    def tick(self, setpoint, val, reset=False):
        if reset:
            self.reset()

        error = setpoint - val
        dt = time.perf_counter() - self.prev_tick_called

        p = self.kP * error
        self.cum_i_val += self.kI * error * dt
        d = self.kD * (error - self.prev_error) / dt

        self.prev_error = error
        self.prev_tick_called = time.perf_counter()

        return p + self.cum_i_val + d

global angle
angle = 0.0

global error
error = 0.0

KP = 0.1
KD = 0.0

KPS = 0.002
KDS = 0

SPEED_BASELINE = 300

ZERO_THRESHOLD = 7

steering_pid = PID(kP=KP, kD=KD)
speed_pid = PID(kP=KPS, kD=KDS)

def start():
    global start_time

    steering_pid.reset()

    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(1)

    start_time = time.time()

    rc.set_update_slow_time(0.5)


def update():
    global angle, error, speed

    index = largest_ray(rc.lidar.get_samples())

    n = rc.lidar.get_num_samples()

    error = index * 360 / n
    if error > 180:
        error -= 360
    #CHECK ANGLE SIGN
    angle = steering_pid.tick(0, error)

    angle = rc_utils.clamp(angle, -1, 1)

    l = rc.lidar.get_samples()[0]

    speed = speed_pid.tick(SPEED_BASELINE, l)
    speed = rc_utils.clamp(speed, 0.1, 1) #dont stop (believin')
    rc.drive.set_max_speed(1)
    rc.drive.set_speed_angle(speed, angle)
    

def largest_ray(scan):
    n = len(scan)

    largest_dist = -1
    best_idx = 0

    for i in range(n // 4):
        dist = scan[i]
        if dist == 0:
            dist = math.inf

        if dist > largest_dist:
            largest_dist = dist
            best_idx = i

    for i in range(3 * n // 4, n):
        dist = scan[i]
        if dist == 0:
            dist = math.inf

        if dist > largest_dist:
            largest_dist = dist
            best_idx = i

    return best_idx


def update_slow():
    global start_time
    global angle, speed
    global error

    elapsed = time.time() - start_time

    print(f"Elapsed: {elapsed}")
    print(f"car Angle: {angle}, speed: {speed}")
    print(f"Error: {error}")


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()