import sys

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import PathFollower

rc = racecar_core.create_racecar()
global speed 
speed = 0
global angle
angle = 0
global follower
follower = PathFollower(0.1,0.1,[0,0,0],rc)

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
    follower.update()
    global pose
    pose = follower.get_pose()
    print(f"X: {pose[0]}")
    print(f"Y: {pose[1]}")
    print(f"Theta: {pose[2]}")
    
    


def update_slow():
    pass



if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()