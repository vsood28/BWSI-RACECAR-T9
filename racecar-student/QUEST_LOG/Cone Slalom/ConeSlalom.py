import sys

sys.path.insert(0, '../../library')

import racecar_core
import cv2 as cv
import racecar_utils as rc_utils
import CSC
import importlib

#To tune - Trigger Area, next cone area, hardcode turn time, HSV values, p gain.

rc = racecar_core.create_racecar()     

MAX_TURN_BACK_SECONDS = 2.5 
DRIVE_SPEED = 1
P_GAIN = 0.9    

COLOR_RED = 'red'
COLOR_BLUE = 'blue'
STATE_ALIGN = 'ALIGN'
STATE_TURN = 'TURN'
STATE_TURN_BACK = 'TURN_BACK'

state = STATE_ALIGN
state_timer = 0.0

target_color = None  
turn_direction = None   

def get_cone_info(image_hsv, hsv_ranges):
    mask = None
    for lower, upper in hsv_ranges:
        m = cv.inRange(image_hsv, lower, upper)
        mask = m if mask is None else cv.bitwise_or(mask, m)

    contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    best_area = CSC.MIN_CONTOUR_AREA
    best_contour = None
    for c in contours:
        area = cv.contourArea(c)
        if area > best_area:
            best_area = area
            best_contour = c

    frame_width = image_hsv.shape[1]

    if best_contour is None:
        return 0, None, frame_width

    center = rc_utils.get_contour_center(best_contour)
    center_col = center[1] if center is not None else None
    return best_area, center_col, frame_width
global startTime
startTime = 0
def start():
    global state, state_timer, target_color, turn_direction, startTime, trials
    state = STATE_ALIGN
    state_timer = 0.0
    target_color = None
    turn_direction = None
    startTime = 0
    rc.drive.set_max_speed(0.5)
    rc.set_update_slow_time(0.5)
    rc.drive.stop()
        
def update():
    global state, state_timer, target_color, turn_direction, startTime
    importlib.reload(CSC)
    image = rc.camera.get_color_image()
    image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    delta_time = rc.get_delta_time()

    red_area, red_center, frame_width = get_cone_info(image_hsv, CSC.RED_RANGES)
    blue_area, blue_center, _ = get_cone_info(image_hsv, [CSC.BLUE])

    state_timer += delta_time

    if state == STATE_ALIGN:
        if target_color is None:
            if red_area >= CSC.MIN_CONTOUR_AREA or blue_area >= CSC.MIN_CONTOUR_AREA:
                target_color = COLOR_RED if red_area >= blue_area else COLOR_BLUE

        if target_color == COLOR_RED:
            area, center = red_area, red_center
        elif target_color == COLOR_BLUE:
            area, center = blue_area, blue_center
        else:
            area, center = 0, None


        if area >= CSC.MIN_CONTOUR_AREA and center is not None:
            error = (center - frame_width / 2) / (frame_width / 2)  # -1 (left) .. 1 (right)
            angle = max(-1.0, min(1.0, P_GAIN * error))
            rc.drive.set_speed_angle(DRIVE_SPEED, angle)

            if area >= CSC.TRIGGER_AREA:
                turn_direction = 'right' if target_color == COLOR_RED else 'left'
                state = STATE_TURN
                state_timer = 0.0
        else:
            rc.drive.set_speed_angle(DRIVE_SPEED, 0.0)

    elif state == STATE_TURN:
        angle = 1.0 if turn_direction == 'right' else -1.0
        rc.drive.set_speed_angle(DRIVE_SPEED, angle)

        if state_timer >= CSC.TURN_SECONDS:
            state = STATE_TURN_BACK
            state_timer = 0.0

    elif state == STATE_TURN_BACK:
        angle = -1.0 if turn_direction == 'right' else 1.0
        rc.drive.set_speed_angle(DRIVE_SPEED, angle)
        next_color = COLOR_BLUE if target_color == COLOR_RED else COLOR_RED
        next_area = blue_area if next_color == COLOR_BLUE else red_area

        next_cone_seen = next_area >= CSC.NEXT_CONE_AREA
        timed_out = state_timer >= MAX_TURN_BACK_SECONDS

        if next_cone_seen or timed_out:
            target_color = next_color if next_cone_seen else None
            state = STATE_ALIGN
            state_timer = 0.0

def update_slow():
    global state, turn_direction
    print(f"State: {state}")
    print(f"Turn Direction {turn_direction}")


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()