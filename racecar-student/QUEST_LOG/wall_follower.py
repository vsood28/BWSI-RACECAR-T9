import sys

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

MAX_SPEED_SCALE = 0.50
CRUISE_SPEED    = 0.65
TIGHT_SPEED     = 0.32

RIGHT_ANGLE  = 90
LEFT_ANGLE   = 270
FRONT_DIAG   = 45
FRONT_WINDOW = 14
SIDE_WINDOW  = 8

CENTER_KP      = 0.014
CENTER_KD      = 0.007
DIAG_KP        = 0.012
WALL_TARGET_CM = 55.0
WALL_KP        = 0.014
VALID_MIN_CM   = 2.0
SEE_WALL_CM    = 250.0

GAP_TIGHT_CM = 120.0
GAP_WIDE_CM  = 300.0

prev_center_err = 0.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def side_distance(scan, angle, window=SIDE_WINDOW):
    d = rc_utils.get_lidar_average_distance(scan, angle % 360, window)
    if d is None or d <= VALID_MIN_CM:
        return None
    return d


def wall_seen(dist):
    return dist is not None and dist < SEE_WALL_CM


def speed_from_gap(left, right, base_speed):
    if left is None or right is None:
        return base_speed
    gap = left + right
    if gap <= GAP_TIGHT_CM:
        return TIGHT_SPEED
    if gap >= GAP_WIDE_CM:
        return base_speed
    return rc_utils.remap_range(gap, GAP_TIGHT_CM, GAP_WIDE_CM,
                                TIGHT_SPEED, base_speed, True)


def drive_lidar_corridor(scan, base_speed):
    global prev_center_err

    left  = side_distance(scan, LEFT_ANGLE)
    right = side_distance(scan, RIGHT_ANGLE)
    fr    = side_distance(scan, FRONT_DIAG)
    fl    = side_distance(scan, 360 - FRONT_DIAG)

    l_ok = wall_seen(left)
    r_ok = wall_seen(right)

    diag = 0.0
    if fr is not None and fl is not None:
        diag = DIAG_KP * (fr - fl)

    if l_ok and r_ok:
        err   = right - left
        d_err = err - prev_center_err
        prev_center_err = err
        angle = CENTER_KP * err + CENTER_KD * d_err + diag
        speed = speed_from_gap(left, right, base_speed)

    elif r_ok:
        err   = right - WALL_TARGET_CM
        angle = WALL_KP * err + diag
        speed = base_speed
        prev_center_err = 0.0

    elif l_ok:
        err   = left - WALL_TARGET_CM
        angle = -WALL_KP * err + diag
        speed = base_speed
        prev_center_err = 0.0

    else:
        angle = diag
        speed = base_speed
        prev_center_err = 0.0

    return speed, clamp(angle, -1.0, 1.0)


def start():
    global prev_center_err
    prev_center_err = 0.0
    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(MAX_SPEED_SCALE)


def update():
    scan = rc.lidar.get_samples()
    speed, angle = drive_lidar_corridor(scan, CRUISE_SPEED)
    rc.drive.set_speed_angle(speed, angle)


def update_slow():
    scan = rc.lidar.get_samples()
    left  = side_distance(scan, LEFT_ANGLE)
    right = side_distance(scan, RIGHT_ANGLE)
    front = side_distance(scan, 0, FRONT_WINDOW)

    def fmt(x):
        return f"{x:6.1f}" if x is not None else "  ----"

    print(f"L:{fmt(left)} F:{fmt(front)} R:{fmt(right)}")


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()