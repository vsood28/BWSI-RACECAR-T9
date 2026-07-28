import sys
import time
import cv2 as cv

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import LFC

rc = racecar_core.create_racecar()

# ── Tune these ────────────────────────────────────────────────────────────────
MIN_CONTOUR_AREA = 3000
SPEED          = 0.50
RECOVERY_SPEED = 0.20

# ── State ─────────────────────────────────────────────────────────────────────
CROP         = None
contour_center = None
contour_area   = 0
error          = 0.0
lastError      = 0.0
angle          = 0.0
last_angle     = 0.0
speed          = 0.0
was_tracking   = False
start_time     = None


def update_contour():
    global contour_center, contour_area

    image = rc.camera.get_color_image()
    if image is None:
        contour_center, contour_area = None, 0
        return

    cropped  = rc_utils.crop(image, CROP[0], CROP[1])
    mask     = cv.inRange(cv.cvtColor(cropped, cv.COLOR_BGR2HSV),
                          LFC.BLUE[0], LFC.BLUE[1])
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    largest = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

    if largest is not None:
        contour_center = rc_utils.get_contour_center(largest)
        contour_area   = cv.contourArea(largest)
    else:
        contour_center, contour_area = None, 0


def start():
    global speed, angle, error, lastError, last_angle, was_tracking, start_time, CROP

    CROP = ((100, 0), (rc.camera.get_height(), rc.camera.get_width()))

    speed, angle  = 0.0, 0.0
    error         = 0.0
    lastError     = 0.0
    last_angle    = 0.0
    was_tracking  = False
    start_time    = time.time()

    rc.drive.set_speed_angle(0, 0)
    rc.set_update_slow_time(0.5)
    rc.drive.set_max_speed(1)


def update():
    global speed, angle, last_angle, error, lastError, was_tracking

    dt = rc.get_delta_time()
    update_contour()

    if contour_center is not None:
        if not was_tracking:
            lastError = (contour_center[1] - LFC.CAMERA_OFFSET) - (rc.camera.get_width() // 2)
        was_tracking = True
        error  = (contour_center[1] - LFC.CAMERA_OFFSET) - (rc.camera.get_width() // 2)
        p      = LFC.Kp * error
        d      = LFC.Kd * ((error - lastError) / dt if dt > 0 else 0.0)
        angle  = rc_utils.clamp(p + d, -1.0, 1.0)
        lastError = error
        speed  = SPEED
    else:
        was_tracking = False
        angle = last_angle
        speed = RECOVERY_SPEED

    rc.drive.set_speed_angle(speed, angle)
    last_angle = angle


def update_slow():
    t = f"{time.time() - start_time:.1f}s"
    if was_tracking:
        print(f"[{t}] TRACKING  spd={speed:.2f}  ang={angle:+.2f}  err={error:+.0f}  area={int(contour_area)}")
    else:
        print(f"[{t}] SEARCHING spd={speed:.2f}  ang={angle:+.2f}")


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()