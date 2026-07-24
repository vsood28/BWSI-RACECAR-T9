import sys
import cv2 as cv
import numpy as np

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import LFC
import csv
import time

log_file = None
log_writer = None
start_time = None

rc = racecar_core.create_racecar()

global error
error = 0.0

global lastError
lastError = error

speed = 0.0
angle = 0.0
last_angle = angle


def start():
    global speed
    global angle
    global log_file, log_writer, start_time
    speed = 0
    angle = 0

    start_time = time.time()

    #logging the data to be displayed in the csv filed and graphed
    log_file = open("line_follow_log.csv", "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow(["time", "error", "angle", "proportional", "derivative"])

    

    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.5)
    rc.drive.set_max_speed(1)

global flag
flag = False
def update_contour():
    image = rc.camera.get_color_image()
    if image is None:
        print("No image")
        return
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, LFC.BLUE[0], LFC.BLUE[1])
    return mask

def update():
    global speed
    global angle
    global last_angle
    global error
    global lastError

    mask = update_contour()
    row = mask[450]
    xs = np.where(row > 0)[0]
    error = 0
    if len(xs):
        target_x = int(xs.mean())
        error = target_x - (rc.camera.get_width() // 2)
    else:
        error = lastError

    dt = rc.get_delta_time()
    angle = (LFC.KP * error) + LFC.KD * ((error - lastError) / dt)
    elapsed = time.time() - start_time
    log_writer.writerow([elapsed, error, angle, LFC.KP * error, LFC.KD * ((error - lastError) / dt)])
    angle = rc_utils.clamp(angle, -1, 1)

    lastError = error
    speed = 0.8
    rc.drive.set_speed_angle(speed, angle)
    last_angle = angle


def update_slow():
    global speed
    global angle
    global maxc
    global start_time
    print(f"Speed {speed}")
    print(f"Angle {angle}")
    print(f"Time: {time.time() - start_time}")



if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()