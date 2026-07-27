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

#Develop separate strategy - pure pursuit?

rc = racecar_core.create_racecar()
CROP = ((180, 0), (rc.camera.get_height(), rc.camera.get_width()))


# 1920 by 1080

LOOKAHEAD_Y = 220

global error
error = 0.0

global lastError
lastError = error

speed = 0.0
angle = 0.0
last_angle = angle


import hashlib

global last_frame_hash
last_frame_hash = None

def update_path():
    global last_frame_hash

    image = rc.camera.get_color_image()
    if image is None:
        return
    image = rc_utils.crop(image, CROP[0], CROP[1])
    frame_hash = hashlib.md5(image.tobytes()).digest()
    if frame_hash == last_frame_hash:
        return
    last_frame_hash = frame_hash

    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    blue_mask = cv.inRange(hsv, LFC.BLUE[0], LFC.BLUE[1])
    ys, xs = np.where(blue_mask > 0)
    if len(xs) < 500:
        return None
    coeffs = np.polyfit(ys, xs, 2)
    lookahead_x = np.polyval(coeffs, LOOKAHEAD_Y)
    #rc.display.show_color_image(image)
    return lookahead_x

def start():
    global speed
    global angle
    global log_file, log_writer, start_time
    speed = 0
    angle = 0
    start_time = time.time()
    log_file = open("line_follow_log.csv", "w", newline="")
    log_writer = csv.writer(log_file)
    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.5)
    rc.drive.set_max_speed(0.4)
def pid(p, d, sp):
    error = (sp - LFC.CAMERA_OFFSET) - (rc.camera.get_width() // 2)
    dt = rc.get_delta_time()
    angle = (p * error) + d * ((error - lastError) / dt)
    return angle

def update():
    global speed
    global angle
    global last_angle
    global maxc
    global error
    global contour_center
    global lastError
    global log_writer
    sp = update_path()

    if sp is not None:
        angle = pid(LFC.KP, LFC.KD, sp)
        elapsed = time.time() - start_time
        log_writer.writerow([elapsed, error, angle])
        angle = rc_utils.clamp(angle, -1, 1)
        log_writer.writerow(["time", "error", "angle"])
    else:
        angle = last_angle

    lastError = error
    speed = 0.25
    rc.drive.set_speed_angle(speed, angle)
    last_angle = angle


def update_slow():
    global speed
    global angle
    global maxc
    global start_time
    print_params(speed, angle, time, start_time, maxc)

def print_params(speed, angle, time, start_time, maxc):
    print(f"Speed {speed}")
    print(f"Angle {angle}")
    print(f"Time: {time.time() - start_time}")
    if maxc is not None: 
        print(f"Contour Area: {cv.contourArea(maxc)}")

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()