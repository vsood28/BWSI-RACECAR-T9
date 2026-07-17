import sys

sys.path.insert(1, '../../library')
import racecar_core

rc = racecar_core.create_racecar()

ORANGE = (50, 255, 0)


def start():
   

        n = rc.led.get_num_pixels()
        print("LED strip found: {} pixels".format(n))

        rc.led.set_pixel(25, (10, 0, 0))

        print("Pixels now:", rc.led.get_pixels()[:4])

    
def update():
    pass


if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()