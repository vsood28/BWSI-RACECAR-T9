
import sys
import cv2 as cv
import numpy as np

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import time

rc = racecar_core.create_racecar()

global kP
global maxc
global perp
perp = False
maxc = None
kP = -0.0038
MIN_CONTOUR_AREA = 20

CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

BLUE = ((90, 50, 50), (120, 255, 255))  
GREEN  = ((50, 150, 50), (85, 255, 255))  
RED = ((0, 150, 50), (10, 255, 255)) 
COLOR_PRIORITY = (RED, GREEN, BLUE)

t = 3
speed = 0.0
angle = 0.0
last_angle = angle
contour_center = None
contour_area = 0
contour_center_filtered = None

SMOOTHING_FACTOR = 0.3
MAX_ANGLE_CHANGE = 0.3

def update_contour():
    global maxc
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
        return

    image = rc_utils.crop(image, (180, 0), (rc.camera.get_height(), rc.camera.get_width()))
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    red_mask = cv.inRange(hsv, RED[0], RED[1])
    blue_mask = cv.inRange(hsv, BLUE[0], BLUE[1])
    green_mask = cv.inRange(hsv, GREEN[0], GREEN[1])

    red_contours, _ = cv.findContours(red_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    blue_contours, _ = cv.findContours(blue_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv.findContours(green_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    def get_largest(contours):
        max_c = None
        max_area = 0
        for c in contours:
            area = cv.contourArea(c)
            if area > max_area and area > MIN_CONTOUR_AREA:
                max_area = area
                max_c = c
        return max_c, max_area

    redmax, redarea = get_largest(red_contours)
    greenmax, greenarea = get_largest(green_contours)
    bluemax, bluearea = get_largest(blue_contours)

    if redmax is not None:
        contour_center = rc_utils.get_contour_center(redmax)
        contour_area = redarea
        maxc = redmax
    elif greenmax is not None:
        contour_center = rc_utils.get_contour_center(greenmax)
        contour_area = greenarea
        maxc = greenmax
    elif bluemax is not None:
        contour_center = rc_utils.get_contour_center(bluemax)
        contour_area = bluearea
        maxc = bluemax
    else:
        contour_center = None
        contour_area = 0
        maxc = None

    cv.drawContours(image, red_contours, 0, (0,255,0))
    rc.display.show_color_image(image)

global st
st = 0
def start():
    global speed
    global angle
    global contour_center_filtered
    speed = 0
    angle = 0
    contour_center_filtered = None
    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.5)



def update():

    global speed
    global st
    global kP
    global angle
    global contour_center_filtered
    global last_angle
    global perp
    global maxc
    last_angle = angle

    update_contour()
    last_angle = angle  

    if contour_center is not None:
        if contour_center_filtered is None:
            contour_center_filtered = contour_center[1]
        else:
            contour_center_filtered = (
                SMOOTHING_FACTOR * contour_center[1] +
                (1 - SMOOTHING_FACTOR) * contour_center_filtered
            )

        error = (rc.camera.get_width() // 2) - contour_center_filtered
        new_angle = kP * error
        delta = new_angle - angle
        if delta > MAX_ANGLE_CHANGE:
            delta = MAX_ANGLE_CHANGE
        elif delta < -MAX_ANGLE_CHANGE:
            delta = -MAX_ANGLE_CHANGE
        angle += delta
        angle = rc_utils.clamp(angle, -1, 1)
    else:
        speed = 0.7
    rc.drive.set_speed_angle(speed, angle)

def update_slow():
    global maxc
    global st

    if rc.camera.get_color_image() is None:
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))
        else:
            s = ["-"] * 32
            idx = int(contour_center[1] / 20)
            if idx >= 32:
                idx = 31
            s[idx] = "|"
            print("".join(s) + " : area = " + str(contour_area))



if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()