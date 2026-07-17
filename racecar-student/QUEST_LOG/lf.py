import sys
import cv2 as cv
import numpy as np

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import LFC
import csv
import time

log_file = None
log_writer = None
start_time = None


rc = racecar_core.create_racecar()

global maxc # max contour area of blue mask
maxc = None
MIN_CONTOUR_AREA = 3000 # tune the minimum to pick up less reflections

# check the crop and hsv values (change crop here)
CROP = ((100, 100), (rc.camera.get_height(), rc.camera.get_width()))

global error
error = 0.0

global lastError
lastError = error

speed = 0.0
angle = 0.0
last_angle = angle
contour_center = None
contour_area = 0


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
    blue_mask = cv.inRange(hsv, LFC.BLUE[0], LFC.BLUE[1])
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
    global log_file, log_writer, start_time
    speed = 0
    angle = 0

    start_time = time.time()

    log_file = open("line_follow_log.csv", "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow(["time", "error", "angle"])


    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.5)
    rc.drive.set_max_speed(1)


def update():
    global speed
    global angle
    global last_angle
    global maxc
    global error
    global contour_center
    global lastError
    update_contour() #offloaded the image processing into a helper function to reduce strain on the pi (faster camera polling)

    if contour_center is not None:
        error = (contour_center[1] - LFC.CAMERA_OFFSET) - (rc.camera.get_width() // 2)
        dt = rc.get_delta_time()
        angle = (LFC.KP * error) + LFC.KD * ((error - lastError) / dt)
        elapsed = time.time() - start_time
        log_writer.writerow([elapsed, error, angle])
        angle = rc_utils.clamp(angle, -1, 1)
    else:
        angle = last_angle

    lastError = error
    speed = 0.85
    speed *= (1 - abs(error * LFC.SPD_KP))
    rc.drive.set_speed_angle(speed, angle)
    last_angle = angle


def update_slow():
    global speed
    global angle
    global maxc
    global start_time
    print(f"Speed {speed}") #Print speed and angle in update slow to reduce strain on pi
    print(f"Angle {angle}")
    print(f"Time: {time.time() - start_time}") #Print time in update slow for better readability during debugging



if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()