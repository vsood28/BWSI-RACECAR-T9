import sys


sys.path.insert(1, "../../library")


import racecar_core
import SLAMICP


rc = racecar_core.create_racecar()


pose = [0.0, 0.0, 0.0]


speed = 0
angle = 0




def start():
    rc.drive.set_max_speed(1.0)
    rc.set_update_slow_time(0.5)




def update():
    global speed, angle, pose


    delta = SLAMICP.update(rc.lidar.get_samples())


    pose[0] += delta[0]
    pose[1] += delta[1]
    pose[2] += delta[2]


    if rc.controller.get_trigger(rc.controller.Trigger.RIGHT) > 0:
        speed = 1
    elif rc.controller.get_trigger(rc.controller.Trigger.LEFT) > 0:
        speed = -1
    else:
        speed = 0


    x, _ = rc.controller.get_joystick(rc.controller.Joystick.LEFT)


    if x > 0.5:
        angle = 1
    elif x < -0.5:
        angle = -1
    else:
        angle = 0


    rc.drive.set_speed_angle(speed, angle)




def update_slow():
    pass




if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
