import sys
import numpy as np
from enum import Enum

sys.path.insert(1, "../library")
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

# ==========================================
# HYPER-RESPONSIVE TUNING PARAMETERS
# ==========================================
DESIRED_WALL_DIST = 120.0 # Safe tracking distance when following a single wall (cm)
WALL_THRESHOLD = 500.0    # Massive threshold to detect super-wide corridors (cm)
FRONT_CRITICAL = 60.0     # Trigger reverse if a wall is closer than this (cm)
FRONT_CLEAR = 90.0        # Safe distance to switch back to forward drive (cm)

# Steering Proportional Gain (Higher = faster corrections, lower = smoother)
STEER_KP = 0.008         

class State(Enum):
    FORWARD = 0
    REVERSE = 1

# ==========================================
# STATE VARIABLES
# ==========================================
state = State.FORWARD
backup_dir = 1.0

# ==========================================
# LIDAR SENSOR HELPER
# ==========================================
def get_lidar_distance(scan_data, angle_start, angle_end):
    """Returns the median distance in a window to filter out noise/posts."""
    points = []
    for angle in range(angle_start, angle_end):
        dist = scan_data[angle % 360]
        if 0.0 < dist < 1000.0:
            points.append(dist)
    return np.median(points) if points else 999.0

# ==========================================
# START
# ==========================================
def start():
    global state
    state = State.FORWARD
    rc.drive.stop()
    print(">> CONTINUOUS RESPONSTIVE WALL FOLLOWER ACTIVE")

# ==========================================
# UPDATE
# ==========================================
def update():
    global state, backup_dir

    scan_data = rc.lidar.get_samples()
    if scan_data is None:
        return

    # 1. Grab straightforward directional readings
    front_dist = get_lidar_distance(scan_data, -10, 10)
    left_dist = get_lidar_distance(scan_data, 260, 280)
    right_dist = get_lidar_distance(scan_data, 80, 100)

    # 2. State Machine (Emergency Backup Decisions)
    if state == State.FORWARD:
        if front_dist < FRONT_CRITICAL:
            state = State.REVERSE
            # Decide reverse turn: if closer to the left, turn wheels right to swing nose right
            backup_dir = 0.9 if (left_dist < right_dist) else -0.9
            
    elif state == State.REVERSE:
        if front_dist > FRONT_CLEAR:
            state = State.FORWARD

    # 3. Execution Control Logic
    if state == State.REVERSE:
        rc.drive.set_speed_angle(-0.25, backup_dir)
        return

    # Forward State Logic (Continuous Proportional Control)
    left_valid = left_dist < WALL_THRESHOLD
    right_valid = right_dist < WALL_THRESHOLD
    
    error = 0.0

    # Scenario A: Both walls are visible -> Always keep equal distance (No matter how wide or narrow!)
    if left_valid and right_valid:
        error = right_dist - left_dist

    # Scenario B: Only Left wall is visible -> Track only left wall
    elif left_valid:
        error = (DESIRED_WALL_DIST - left_dist) * 2.0

    # Scenario C: Only Right wall is visible -> Track only right wall
    elif right_valid:
        error = (right_dist - DESIRED_WALL_DIST) * 2.0

    # Scenario D: Open space (no walls found)
    else:
        error = 0.0

    # Calculate precise steering angle based on real-time distance error
    angle = rc_utils.clamp(error * STEER_KP, -1.0, 1.0)

    # Cautious forward speed: slow down dynamically when turning sharply
    speed = 0.42 * (1.0 - 0.45 * abs(angle))
    speed = rc_utils.clamp(speed, 0.15, 0.45)

    rc.drive.set_speed_angle(speed, angle)

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()