import sys
import cv2 as cv
import numpy as np

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import time

rc = racecar_core.create_racecar()

global kP
global maxc # max contour area of blue mask
maxc = None
kP = 0.005
MIN_CONTOUR_AREA = 200

# check the crop and hsv values
CROP = ((240, 0), (rc.camera.get_height(), rc.camera.get_width()))
BLUE = ((85, 100, 150), (105, 255, 255))

speed = 0.0
angle = 0.0
last_angle = angle
contour_center = None
contour_area = 0
contour_center_filtered = None


def update_contour():
    global maxc
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
        return

    image = rc_utils.crop(image, CROP[0], CROP[1])
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    blue_mask = cv.inRange(hsv, BLUE[0], BLUE[1])
 
    blue_contours, _ = cv.findContours(blue_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    # precondition: called
    # postconditon: returns the largest contour from the set it is called with
    def get_largest(contours):
        max_c = None
        max_area = 0
        for c in contours:
            area = cv.contourArea(c)
            if area > max_area and area > MIN_CONTOUR_AREA:
                max_area = area
                max_c = c
        return max_c, max_area

    bluemax, bluearea = get_largest(blue_contours)

    if bluemax is not None: # sets the maximum contour and it's area if it exists
        contour_center = rc_utils.get_contour_center(bluemax)
        contour_area = bluearea
        maxc = bluemax 
    else:
        contour_center = None
        contour_area = 0
        maxc = None
    cv.drawContours(image, blue_contours, -1, (255,0,0), 3)   
    #rc.display.show_color_image(image)


def start(): # initializes values
    global speed
    global angle
    global contour_center_filtered
    speed = 0
    angle = 0
    contour_center_filtered = None
    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.5)


global error
error = 0.0

def update(): 
    # calls update_contour and sets the angle of the car depending on where the line is
    # also changes the angle of the car so it makes less jerky turns and is smooth
    global speed
    global kP
    global angle
    global contour_center_filtered
    global last_angle
    global perp
    global maxc
    global error 
    global contour_center

    update_contour()

    if contour_center is not None:
        contour_center_filtered = contour_center[1]
        error = contour_center_filtered - (rc.camera.get_width() // 2) - 77 # tune offset
        angle = (kP * error) - 0.05
        angle = rc_utils.clamp(angle, -1, 1)
    else:
        angle = last_angle 
    rc.drive.set_speed_angle(0.3, angle)
    last_angle = angle

def update_slow():
    global maxc
    global angle
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
    global error
    print(error)
    print(angle)



if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()