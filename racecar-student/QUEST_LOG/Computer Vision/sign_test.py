"""
sign_detection.py
Color-based STOP / GO sign detection for the MIT RACECAR.
RED sign -> stop, GREEN sign -> go. No text/OCR yet.
"""

import sys
import cv2
import numpy as np

# Adjust this path if your library lives somewhere else.
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()


# HSV ranges (OpenCV: H 0-180, S/V 0-255). Red wraps the hue axis,
# so it needs two ranges.
RED_LOW  = ((0,   120, 100), (10,  255, 255))
RED_HIGH = ((170, 120, 100), (180, 255, 255))
GREEN    = ((40,   60,  60), (80,  255, 255))

# Bigger => sign must be closer/larger before the car reacts.
MIN_CONTOUR_AREA = 800

# Consecutive frames a sign must be seen before acting on it.
CONFIRM_FRAMES = 4

DRIVE_SPEED = 0.5
DRIVE_ANGLE = 0.0

SHOW_DEBUG = True

# ======================================================================
#  STATE
# ======================================================================
red_streak = 0
green_streak = 0
current_state = "NONE"   # "STOP", "GO", or "NONE"


# ======================================================================
#  HELPERS
# ======================================================================
def get_color_masks(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    red_mask = cv2.inRange(hsv, RED_LOW[0], RED_LOW[1]) | \
               cv2.inRange(hsv, RED_HIGH[0], RED_HIGH[1])
    green_mask = cv2.inRange(hsv, GREEN[0], GREEN[1])

    kernel = np.ones((5, 5), np.uint8)
    for m in (red_mask, green_mask):
        cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel, dst=m)
        cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel, dst=m)

    return red_mask, green_mask


def find_sign(mask):
    contours = rc_utils.find_contours(mask)
    largest = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
    if largest is None:
        return None

    return {
        "area": rc_utils.get_contour_area(largest),
        "center": rc_utils.get_contour_center(largest),  # (row, col)
        "bbox": cv2.boundingRect(largest),               # (x, y, w, h)
        "contour": largest,
    }


def draw_debug(image, red_sign, green_sign):
    if red_sign is not None:
        x, y, w, h = red_sign["bbox"]
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 3)
        cv2.putText(image, "STOP", (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    if green_sign is not None:
        x, y, w, h = green_sign["bbox"]
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.putText(image, "GO", (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    cv2.putText(image, f"STATE: {current_state}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    rc.display.show_color_image(image)


# ======================================================================
#  RACECAR CALLBACKS
# ======================================================================
def start():
    global red_streak, green_streak, current_state
    red_streak = 0
    green_streak = 0
    current_state = "NONE"

    rc.drive.set_speed_angle(0, 0)
    rc.set_update_slow_time(0.5)
    print(">> Sign detection ready. RED = stop, GREEN = go.")


def update():
    global red_streak, green_streak, current_state

    image = rc.camera.get_color_image()
    if image is None:
        return

    red_mask, green_mask = get_color_masks(image)
    red_sign = find_sign(red_mask)
    green_sign = find_sign(green_mask)

    red_streak = red_streak + 1 if red_sign is not None else 0
    green_streak = green_streak + 1 if green_sign is not None else 0

    # STOP wins ties, for safety.
    if red_streak >= CONFIRM_FRAMES:
        current_state = "STOP"
    elif green_streak >= CONFIRM_FRAMES:
        current_state = "GO"
    elif red_streak == 0 and green_streak == 0:
        current_state = "NONE"
    # else: not confirmed yet -> hold previous state

    if current_state == "STOP":
        rc.drive.set_speed_angle(0, 0)
    elif current_state == "GO":
        rc.drive.set_speed_angle(DRIVE_SPEED, DRIVE_ANGLE)
    else:
        # No sign in view. Default: hold still.
        rc.drive.set_speed_angle(0, 0)

    if rc.controller.is_down(rc.controller.Button.B):
        current_state = "NONE"
        rc.drive.set_speed_angle(0, 0)

    if SHOW_DEBUG:
        draw_debug(image, red_sign, green_sign)


def update_slow():
    print(f"state={current_state:5s}  red_streak={red_streak}  "
          f"green_streak={green_streak}")


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()