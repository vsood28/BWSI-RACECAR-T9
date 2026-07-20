import sys
import csv
import time

import cv2 as cv

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import LFC

rc = racecar_core.create_racecar()

MIN_CONTOUR_AREA = 3000
CROP = ((180, 0), (rc.camera.get_height(), rc.camera.get_width()))

SPEED = 0.8   
# ------------------------------ state -------------------------------------
log_file = None
log_writer = None
start_time = None

maxc = None
contour_center = None
contour_area = 0
error = 0.0
lastError = 0.0
angle = 0.0
last_angle = 0.0
speed = 0.0


def update_contour():
    global maxc, contour_center, contour_area

    image = rc.camera.get_color_image()
    if image is None:
        contour_center, contour_area, maxc = None, 0, None
        return

    image = rc_utils.crop(image, CROP[0], CROP[1])
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    blue_mask = cv.inRange(hsv, LFC.BLUE[0], LFC.BLUE[1])
    contours, _ = cv.findContours(blue_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    largest = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
    if largest is not None:
        contour_center = rc_utils.get_contour_center(largest)
        contour_area = cv.contourArea(largest)
        maxc = largest
        cv.drawContours(image, [largest], -1, (255, 0, 0), 3)
    else:
        contour_center, contour_area, maxc = None, 0, None

    # rc.display.show_color_image(image)   # uncomment to see the mask; adds camera lag


def start():
    global speed, angle, lastError, last_angle
    global log_file, log_writer, start_time

    speed = 0
    angle = 0
    lastError = 0.0
    last_angle = 0.0
    start_time = time.time()

    log_file = open("line_follow_log.csv", "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow(["time", "error", "angle_raw", "angle_cmd", "p", "d", "found"])

    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.5)
    rc.drive.set_max_speed(1)


def update():
    global speed, angle, last_angle, error, lastError

    update_contour()

    if contour_center is not None:
        error = (contour_center[1] - LFC.CAMERA_OFFSET) - (rc.camera.get_width() // 2)
        dt = rc.get_delta_time()
        deriv = (error - lastError) / dt if dt > 0 else 0.0
        p_term = LFC.Kp * error
        d_term = LFC.Kd * deriv
        angle_raw = p_term + d_term          # before clamp -> shows saturation
        angle = rc_utils.clamp(angle_raw, -1, 1)
        lastError = error
        found = 1
    else:
        angle = last_angle
        p_term, d_term, angle_raw = 0.0, 0.0, last_angle
        found = 0

    log_writer.writerow([time.time() - start_time, error, angle_raw, angle, p_term, d_term, found])
    log_file.flush()

    speed = SPEED
    rc.drive.set_speed_angle(speed, angle)
    last_angle = angle


def update_slow():
   
    print("Speed {:.2f}   Angle {:+.2f}   Error {:+.1f}   Time {:.1f}".format(
        speed, angle, error, time.time() - start_time))
    if maxc is not None:
        print("Contour area: {}".format(int(cv.contourArea(maxc))))
    else:
        print("No line found")


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()