from cs_func import find_cones, target_point, angle_to_target, min_scan, closest_cone
import time
import sys
import math  # Weighted Algorithmic Farthest Opening Lookahead and Wayfinding
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()


class PID:  # simple PID
    def __init__(self, kP=0, kI=0, kD=0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.prev_error = 0
        self.cum_i_val = 0
        self.prev_tick_called = time.perf_counter()

    def reset(self):
        self.prev_error = 0
        self.cum_i_val = 0
        self.prev_tick_called = time.perf_counter()  # FIX: seed with real time, not 0

    def tick(self, setpoint, val, reset=False):
        if reset:
            self.reset()
        now = time.perf_counter()
        dt = now - self.prev_tick_called
        if dt <= 0:                      # FIX: guard against div-by-zero / huge first dt
            dt = 1e-3
        error = val - setpoint
        p = self.kP * error
        self.cum_i_val += self.kI * error * dt
        d = self.kD * (error - self.prev_error) / dt
        self.prev_error = error
        self.prev_tick_called = now
        return p + self.cum_i_val + d



DETECT_RANGE = 25
CONE_JUMP    = 80   # c6m gap that splits one cone cluster from the next

# --- The two biggest fixes ---
OFFSET       = 25        
FLIP_THRESH  = OFFSET * 1.8   # cm: flip side once the cone we're passing is abeam,
                              #   i.e. its side-scan distance drops below this.


KP = 0.8  # 0.9             
KI = 0.0
KD = 0.08            

# speed
MAX_SPEED   = 0.1
MIN_SPEED   = 0.05
TURN_SLOW   = 0.03 # 0..1, how much hard steering cuts speed
SEARCH_SPEED = 0.05    # creep forward when no cone is visible


FLIP_COOLDOWN = 1      #   was 1 s minimum between flips
REARM_FACTOR  = 1.8      # side scan must clear FLIP_THRESH * this before another flip
WINDOW        = 12       #  12                                                         min_scan window width
# ------------------------------------------------------------

steering_pid = PID(kP=KP, kI=KI, kD=KD)

# module state
angle = 0.0
error = 0.0
side  = True             # which side we currently pass a cone on
armed = True             # ready to flip? (set once we've cleared the last cone)
last_flip_time = 0.0
start_time = None


def start():
    global start_time, side, armed, last_flip_time
    steering_pid.reset()
    side = True
    armed = True
    last_flip_time = 0.0
    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(1)
    start_time = time.time()
    rc.set_update_slow_time(0.33)


def update():
    global angle, error, side, armed, last_flip_time

    now = time.time()
    cones   = find_cones(rc.lidar, DETECT_RANGE, CONE_JUMP)
    closest = closest_cone(cones)

    # --- No cone in view: creep straight so we don't crash / chase a stale target ---
    if closest is None:
        rc.drive.set_speed_angle(SEARCH_SPEED, 0.0)
        return

    smp = rc.lidar.get_samples()
    n   = rc.lidar.get_num_samples()
    r   = min_scan(smp,  n // 4, WINDOW)   # ~90 deg to the right
    l   = min_scan(smp, -n // 4, WINDOW)   # ~90 deg to the left

   
    watched = l if side else r
    if watched != 0 and watched > FLIP_THRESH * REARM_FACTOR:
        armed = True                                   # cone is well clear -> re-arm
    if (armed and watched != 0 and watched < FLIP_THRESH
            and (now - last_flip_time) > FLIP_COOLDOWN):
        side = not side
        armed = False
        last_flip_time = now
        print("flip")

    # --- Steer toward an offset point beside the cone ---
    target = target_point(closest, side, OFFSET)   # aim OFFSET cm to the side of it
    error  = angle_to_target(target)
    angle  = steering_pid.tick(0, error)
    angle  = rc_utils.clamp(angle, -1, 1)

   
    speed = MAX_SPEED - TURN_SLOW * abs(angle) * (MAX_SPEED - MIN_SPEED)
    speed = rc_utils.clamp(speed, MIN_SPEED, MAX_SPEED)

    rc.drive.set_speed_angle(speed, angle)


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