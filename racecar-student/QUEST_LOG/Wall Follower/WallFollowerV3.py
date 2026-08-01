import time
import sys
import math
sys.path.insert(1, '../../library')

import racecar_core
import racecar_utils as rc_utils
import time

start_time = None

rc = racecar_core.create_racecar()

global angle
angle = 0.0

global error
error = 0.0

KP = 0.01
KD = 0.0

KPS = 0.002
KDS = 0

ZERO_THRESHOLD = 5

def start():
    global start_time

    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(1)

    rc.set_update_slow_time(0.5)


def update():
    global angle, error, speed
    error = follow_gap(rc.lidar)
    #check angle sign
    angle = 0

    angle = rc_utils.clamp(angle, -1, 1)

    speed = rc_utils.clamp(speed, 0.1, 1)
    rc.drive.set_speed_angle(speed, angle)
    
def follow_gap(lidar):
    num_zeroes = 0
    scan = list(lidar.get_samples())
    n = lidar.get_num_samples()
    for i in range(-n//4, n//4):
        if scan[i] == 0:
            num_zeroes += 1
    if num_zeroes < ZERO_THRESHOLD:
        print("Following Largest Ray")
        return largest_ray(scan)
    else:
        angle = largest_window(scan)
        print("Following Gap")
        print(f"Target Angle: {angle}")
        return angle
        
    


def largest_ray(scan):
    n = len(scan)

    largest_dist = -1
    best_idx = 0

    for i in range(-n//4, n//4):
        dist = scan[i]
        if dist > largest_dist:
            largest_dist = dist
            best_idx = i
        
    print(f"Largest Dist: {largest_dist}")
    angle = best_idx * 360 / n
    if angle > 180:
        angle -= 360
    print(f"Target Angle: {angle}")    
    return angle    

def largest_window(scan):
    n = len(scan)
    win = None
    largest_win = [0, 0]
    for i in range(-n//4, n//4):
        if scan[i] == 0: #if zeros
            if win is None: #if no window
                win = [i, i] #open
            else:
                win[1] = i #udpate cur window end
        else: #if not zero
            if win is not None: #if has window
                if win[1] - win[0] >= largest_win[1] - largest_win[0]: #if winodw greater
                    largest_win = win #update largest
                win = None
    if win is not None:
        if win[1] - win[0] >= largest_win[1] - largest_win[0]:
            largest_win = win            
    print(f"Largest Win: {largest_win}")            
    best_idx = (largest_win[0] + largest_win[1]) // 2 #index center
    angle = best_idx * 360 / n
    if angle > 180:
        angle -= 360
    return angle#to angle

def update_slow():
    global start_time
    global angle, speed
    global error

    elapsed = time.time() - start_time

    print(f"Elapsed: {elapsed}")
    print(f"car Angle: {angle}")
    print(f"Error: {error}")


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()