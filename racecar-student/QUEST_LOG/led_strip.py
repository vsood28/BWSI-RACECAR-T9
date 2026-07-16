from matplotlib.pyplot import rc

import led_strip
import sys
import numpy as np

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import time

def start(): # initializes values
   n  = rc.led.get_num_leds()
   for i in range(n):
       rc.led.set_color(i, (0, 0, 0))


def update(): 
    rc.led.set_color(0, (255, 100, 0))


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()