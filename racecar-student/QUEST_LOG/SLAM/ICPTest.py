import sys

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import SLAMICP

rc = racecar_core.create_racecar()
global speed 
speed = 0
global angle
angle = 0

def start():
    global speed
    global angle
    speed = 0
    angle = 0

    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.5)
    rc.drive.set_max_speed(1)

global pose
pose = [0,0,0]
def update():
    global speed
    global angle
    delta = SLAMICP.update(rc.lidar.get_samples())
    for i in range(3):
        pose[i] += delta[i]
    if rc.controller.get_trigger(rc.controller.Trigger.RIGHT) > 0:
        speed = 1
    elif rc.controller.get_trigger(rc.controller.Trigger.LEFT) > 0:
        speed = -1
    else:
        speed = 0

    (x, y) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    if x > 0.5:
        angle = 1
    elif x < -0.5:
        angle = -1
    else:
        angle = 0




    


def update_slow():
    pass



if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()