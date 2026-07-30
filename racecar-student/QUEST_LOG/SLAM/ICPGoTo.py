import sys
import math

sys.path.insert(1, "../../library")

import racecar_core
import racecar_utils as rc_utils
import SLAMICP

rc = racecar_core.create_racecar()

KP = 1.0
KD = 0.15

GOAL = (10.0, 3.0)

last_error = 0.0


def start():
    rc.drive.set_max_speed(1.0)
    rc.set_update_slow_time(0.5)


def update():
    global last_error

    SLAMICP.update(rc.lidar.get_samples())

    x, y, heading = SLAMICP.get_pose()

    dx = GOAL[0] - x
    dy = GOAL[1] - y
    goal_heading = math.atan2(dy, dx)

    error = goal_heading - heading
    error = math.atan2(math.sin(error), math.cos(error))

    dt = max(rc.get_delta_time(), 1e-6)

    derivative = (error - last_error) / dt

    angle = KP * error + KD * derivative
    angle = rc_utils.clamp(angle, -1.0, 1.0)

    last_error = error


    speed = 0.4

    rc.drive.set_speed_angle(speed, angle)

    print(
        f"pos=({x:.2f}, {y:.2f}) "
        f"head={math.degrees(heading):.1f}° "
        f"goal={math.degrees(goal_heading):.1f}° "
        f"err={math.degrees(error):.1f}° "
        f"ang={angle:.2f}"
    )


def update_slow():
    pass


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()