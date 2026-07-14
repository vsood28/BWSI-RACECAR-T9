import sys

import cv2 as cv

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils


import untitled

rc = racecar_core.create_racecar()

MIN_CONTOUR_AREA = 500

_TOP = rc.camera.get_height() // 3
CROP = ((_TOP, 0), (rc.camera.get_height(), rc.camera.get_width()))


KP = 0.009
KD = 0.001

MAX_SPEED = 0.4  # speed on straights
MIN_SPEED = 0.2   # speed on the sharpest turns

# ------------------------------ State -------------------------------------
contour_center = None
contour_area = 0
last_error = 0.0
last_angle = 0.0


def update_contour():
    """Find the largest blue contour and store its center + area."""
    global contour_center, contour_area

    image = rc.camera.get_color_image()
    if image is None:
        contour_center, contour_area = None, 0
        return

    image = rc_utils.crop(image, CROP[0], CROP[1])
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, untitled.BLUE[0], untitled.BLUE[1])

    contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    largest = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

    if largest is not None:
        contour_center = rc_utils.get_contour_center(largest)
        contour_area = cv.contourArea(largest)
    else:
        contour_center, contour_area = None, 0


def start():
    global last_error, last_angle
    last_error = 0.0
    last_angle = 0.0
    rc.drive.set_max_speed(MAX_SPEED)
    rc.drive.set_speed_angle(0, 0)
    rc.set_update_slow_time(0.5)


def update():
    global last_error, last_angle

    update_contour()

    if contour_center is not None:
        width = rc.camera.get_width()
        # Normalize the line's horizontal position to [-1, 1]; 0 = centered.
        error = rc_utils.remap_range(contour_center[1], 0, width, -1.0, 1.0)

        dt = rc.get_delta_time()
        derivative = (error - last_error) / dt if dt > 0 else 0.0

        angle = KP * error + KD * derivative
        angle = rc_utils.clamp(angle, -1.0, 1.0)

        last_error = error
        last_angle = angle
    else:
        # Line lost: keep curving the way we last were so we swing back toward it.
        angle = last_angle

    # Ease off the throttle on sharp turns, run fast on straights.
    speed = rc_utils.remap_range(abs(angle), 0.0, 1.0, MAX_SPEED, MIN_SPEED)
    speed = rc_utils.clamp(speed, MIN_SPEED, MAX_SPEED)

    rc.drive.set_speed_angle(speed, angle)

    rc.telemetry.declare_variables("Speed", "Angle", "Error")
    rc.telemetry.record(speed, angle, last_error)


def update_slow():
    if rc.camera.get_color_image() is None:
        print("X" * 10 + " (No image) " + "X" * 10)
    elif contour_center is None:
        print("No line found")
    else:
        print("Line x = {}, area = {}".format(contour_center[1], int(contour_area)))


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()