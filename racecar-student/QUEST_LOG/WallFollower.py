import sys

import cv2 as cv

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

# ----------------------------- Tunables -----------------------------------
# HSV range for the blue line.
LOWER_HSV = (94, 75, 130)
UPPER_HSV = (103, 115, 215)

MIN_CONTOUR_AREA = 4000

# Crop band: only look at the lower two-thirds of the frame.
# Lower _TOP = looks further ahead (earlier reaction), higher = more stable.
_TOP = rc.camera.get_height() // 3
CROP = ((_TOP, 0), (rc.camera.get_height(), rc.camera.get_width()))

# PD gains. Error is normalized to [-1, 1], so these are on that scale.
KP = 0.6
KD = 0.1

SPEED = 0.5   # constant drive speed

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
    mask = cv.inRange(hsv, LOWER_HSV, UPPER_HSV)

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

    rc.drive.set_max_speed(SPEED)
    rc.drive.set_speed_angle(0, 0)
    rc.set_update_slow_time(0.5)

    # Declare once here -- repeated calls in update() have no effect.
    rc.telemetry.declare_variables("Angle", "Error", "P", "D")


def update():
    global last_error, last_angle

    update_contour()

    if contour_center is not None:
        width = rc.camera.get_width()
        # Normalize the line's horizontal position to [-1, 1]; 0 = centered.
        error = rc_utils.remap_range(contour_center[1], 0, width, -1.0, 1.0)

        dt = rc.get_delta_time()
        derivative = (error - last_error) / dt if dt > 0 else 0.0

        p_term = KP * error
        d_term = KD * derivative

        angle = rc_utils.clamp(p_term + d_term, -1.0, 1.0)

        last_error = error
        last_angle = angle
    else:
        # Line lost: hold the last steering command so we curve back toward it.
        angle = last_angle
        p_term = 0.0
        d_term = 0.0

    rc.drive.set_speed_angle(SPEED, angle)

    # Logging P and D separately makes the exit graph actually useful for tuning.
    rc.telemetry.record(angle, last_error, p_term, d_term)


def update_slow():
    if rc.camera.get_color_image() is None:
        print("X" * 10 + " (No image) " + "X" * 10)
    elif contour_center is None:
        print("No line found")
    else:
        print(
            "Line x = {}, area = {}, error = {:+.3f}".format(
                contour_center[1], int(contour_area), last_error
            )
        )


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()