import sys

sys.path.insert(1, '../../library')

import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

def start():
    print("start button pressed")
    rc.drive.stop()
    rc.drive.set_speed_angle(0, 0)

def update():
    print("angular velocity & acceleration:")
    print(rc.physics.get_angular_velocity())
    print(rc.physics.get_linear_acceleration())

def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()