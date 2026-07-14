import sys
import cv2 as cv
import numpy as np

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import importlib
import untitled


rc = racecar_core.create_racecar()

global maxc # max contour area of blue mask
maxc = None
MIN_CONTOUR_AREA = 4000 # tune

# check the crop and hsv values
CROP = ((240, 0), (rc.camera.get_height(), rc.camera.get_width()))

global error
error = 0.0

global lastError
lastError = error

global filteredError
filteredError = 0.0

global lastFilteredError
lastFilteredError = 0.0

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
    hsv = cv.GaussianBlur(hsv, (5,5), 0)
    blue_mask = cv.inRange(hsv, untitled.BLUE[0], untitled.BLUE[1])
    blue_mask = cv.morphologyEx(blue_mask, cv.MORPH_OPEN, np.ones((5,5), np.uint8))
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


def start():
    global speed
    global angle
    global contour_center_filtered
    global filteredError
    global lastFilteredError
    speed = 0
    angle = 0
    contour_center_filtered = None
    filteredError = 0.0
    lastFilteredError = 0.0

    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.5)
    rc.drive.set_max_speed(1)


def update():
    global speed
    global angle
    global contour_center_filtered
    global last_angle
    global maxc
    global error
    global contour_center
    global lastError
    global filteredError
    global lastFilteredError
    importlib.reload(untitled)
    update_contour()

    if contour_center is not None:
        contour_center_filtered = contour_center[1]
        error = (contour_center_filtered - untitled.CAMERA_OFFSET) - (rc.camera.get_width() // 2)
        filteredError = 0.8 * filteredError + 0.2 * error

        angle = (untitled.KP * filteredError) - untitled.ANGLE_OFFSET

        print("P")
        print(untitled.KP * filteredError)
        angle = rc_utils.clamp(angle, -1, 1)
    else:
        angle = last_angle

    lastFilteredError = filteredError
    lastError = error
    rc.drive.set_speed_angle(0.7, angle)
    rc.telemetry.declare_variables("Speed", "Angle", "Error")
    rc.telemetry.record(speed, angle, error)
    last_angle = angle


def update_slow():
    global maxc
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