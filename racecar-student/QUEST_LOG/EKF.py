"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-outreach-labs

File Name: template.py << [Modify with your own file name!]

Title: [PLACEHOLDER] << [Modify with your own title]

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: [PLACEHOLDER] << [Write the purpose of the script here]

Expected Outcome: [PLACEHOLDER] << [Write what you expect will happen when you run
the script.]
"""

########################################################################################
# Imports
########################################################################################
import sys

sys.path.insert(0, '../library')

from email.mime import image
import racecar_core
import math
import random
import time
import cv2 as cv
import numpy as np
from scipy.spatial import KDTree
# import racecar_utils as rc_utils  # uncomment when you need helpers from racecar_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

INF = float('inf')
FOV_CIRCLE_RATIO = 100/720 #50 POINTS TO EITHER SIDE
FOV = FOV_CIRCLE_RATIO * math.pi*2 #25 degrees either side, 50 lidar points per side (720 total, 50/720 = 25/360)
HALF_FOV = FOV/2
TURN_RADIUS = 80

#linear relation between lidar index (aka angle) and x position on camera
def lidar_ray_to_camera_x(lidar, i, camera):
    n = lidar.get_num_samples()
    mx = camera.get_width()

    l_fov = FOV_CIRCLE_RATIO * n # fraction of circle in fov * total number pts in circle

    if i > n/2:
        i = i - n
    
    if i > 50 or i < -50:
        return None #off screen
    
    return (i + l_fov/2) * mx/l_fov

def find_color_contours(image, lower_bound, upper_bound, min_area=30):
    if image is None:
        return []

    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower_bound, upper_bound)

    contours, _ = cv.findContours(
        mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
    )

    results = []

    for contour in contours:
        area = cv.contourArea(contour)
        if area >= min_area:
            center = rc_utils.get_contour_center(contour)
            results.append((contour, center, area))

    return results


def lidar_to_positions(lidar):
    n = lidar.get_num_samples()
    
    pts = []

    for i in range(n):
        pt = lidar_ray_to_position(lidar, i)
        if pt is None:
            continue

        pts.append(pt)

    return pts

def lidar_ray_to_position(lidar, i):
    smp = lidar.get_samples()
    n = lidar.get_num_samples()

    d = smp[i]
    if d == 0: #inf dist
        return None

    raw_angle = i/n * 2 * math.pi
    cart_angle = math.pi/2 - raw_angle #angle in cartesian with car as origin

    x = math.cos(cart_angle) * d
    y = math.sin(cart_angle) * d

    return Position(x, y)

def lidar_ray_to_angle(lidar, i):
    n = lidar.get_num_samples()

    raw_angle = i/n * 2 * math.pi
    cart_angle = math.pi/2 - raw_angle #angle in cartesian with car as origin
    if cart_angle < 0:
        cart_angle += math.pi * 2

    return cart_angle

def angle_to_lidar_ray(angle, lidar):
    n = lidar.get_num_samples()

    # Undo the cartesian transform
    raw_angle = (math.pi/2 - angle) % (2 * math.pi)

    # Convert back to a lidar sample index
    i = raw_angle / (2 * math.pi) * n

    return int(round(i)) % n

def lidar_ray_within_range(lidar, i, left, right):
    n = lidar.get_num_samples()

    raw_angle = i / n * 2 * math.pi
    cart_angle = math.pi / 2 - raw_angle
    if cart_angle < 0:
        cart_angle += 2 * math.pi

    # Normalize bounds
    left %= 2 * math.pi
    right %= 2 * math.pi

    if right <= left: # Check if cart_angle is on the CCW arc from right to left
        return right <= cart_angle <= left
    else: # Arc wraps around 2π
        return cart_angle >= right or cart_angle <= left

class PID:
    def __init__(self, kP=0,kI=0,kD=0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.prev_error = 0
        self.cum_i_val = 0
        self.prev_tick_called = 0

    def reset(self):
        self.prev_error = 0
        self.cum_i_val = 0
        self.prev_tick_called = 0

    def tick(self, setpoint, val, reset=False):
        if reset:
            self.reset()

        error = setpoint - val
        dt = time.perf_counter() - self.prev_tick_called

        p = self.kP * error
        self.cum_i_val += self.kI * error * dt
        d = self.kD * (self.prev_error - error) / dt

        self.prev_error = error
        self.prev_tick_called = time.perf_counter()

        return p + self.cum_i_val + d

class Position:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def to_pose(self):
        return Pose(self.x, self.y, 0)

    def to_array(self):
        return np.array([self.x, self.y])
    
    def to_absolute(self, reference_pose):
        """Convert this local position into the global frame defined by reference_pose."""
        c = math.cos(reference_pose.dir)
        s = math.sin(reference_pose.dir)

        x = reference_pose.x + self.x * c - self.y * s,
        y = reference_pose.y + self.x * s + self.y * c,

        return Position(x, y)

    def dot(self, other):
        return self.x * other.x + self.y * other.y
    
    def magnitude(self):
        return math.sqrt(self.x*self.x + self.y*self.y)
    
    def round_to_nearest(self, i):
        self.x = self.x//i * i
        self.y = self.y//i * i

    def distance(self, other):
        return (self - other).magnitude()

    # Position + Position
    def __add__(self, other):
        if isinstance(other, Position):
            return Position(self.x + other.x, self.y + other.y)
        return NotImplemented

    # Position - Position
    def __sub__(self, other):
        if isinstance(other, Position):
            return Position(self.x - other.x, self.y - other.y)
        return NotImplemented

    # Position * scalar
    def __mul__(self, scalar):
        if isinstance(scalar, (int, float)):
            return Position(self.x * scalar, self.y * scalar)
        return NotImplemented

    # scalar * Position
    __rmul__ = __mul__

    # Position / scalar
    def __truediv__(self, scalar):
        if isinstance(scalar, (int, float)):
            return Position(self.x / scalar, self.y / scalar)
        return NotImplemented

    def __repr__(self):
        return f"Position(x={self.x}, y={self.y})"


class Pose:
    def __init__(self, x=0, y=0, dir=0):
        self.x = x
        self.y = y
        self.dir = dir

    def to_position(self):
        return Position(self.x, self.y)

    def to_array(self):
        return np.array([self.x, self.y, self.dir])

    # Pose + Pose
    def __add__(self, other):
        if isinstance(other, Pose):
            return Pose(
                self.x + other.x,
                self.y + other.y,
                self.dir + other.dir,
            )
        return NotImplemented

    # Pose - Pose
    def __sub__(self, other):
        if isinstance(other, Pose):
            return Pose(
                self.x - other.x,
                self.y - other.y,
                self.dir - other.dir,
            )
        return NotImplemented

    def __repr__(self):
        return f"Pose(x={self.x}, y={self.y}, dir={self.dir})"
    
class Line:
    @staticmethod
    def best_fit(*args): #assumes all points are roughly linear, no extreme outliers towards ends
        pts = []

        for arg in args:
            if isinstance(arg, list):
                for pt in arg:
                    if not isinstance(pt, Position):
                        raise TypeError("Non-point in list")
                pts.extend(arg)
            elif isinstance(arg, Position):
                pts.append(arg)
            else:
                raise TypeError("Non-point in args")

        n = len(pts)
        if n == 0:
            raise ValueError("No points provided")

        sum_x = sum(p.x for p in pts)
        sum_y = sum(p.y for p in pts)
        sum_xy = sum(p.x * p.y for p in pts)
        sum_x2 = sum(p.x * p.x for p in pts)

        denom = n * sum_x2 - sum_x * sum_x
        if denom == 0: #vertical line
            y_c = [p.y for p in pts]
            min_y = min(y_c)
            max_y = max(y_c)
            start = Position(pts[0].x, min_y)
            end = Position(pts[0].x, max_y)
            return Line(start, end)

        m = (n * sum_xy - sum_x * sum_y) / denom
        b = (sum_y - m * sum_x) / n

        #find line endpoints
        x_c = [p.x for p in pts]
        y_c = [p.y for p in pts]
        min_x = min(x_c)
        max_x = max(x_c)
        min_y = min(y_c)
        max_y = max(y_c)
        #pick greatest differnce, use as axis. (minimize impact of variance of points on axes with small difference)
        if (max_x - min_x) > (max_y - min_y):
            start = Position(min_x, m * min_x + b)
            end = Position(max_x, m * max_x + b)
        else: #y = mx + b => x = (y - b)/m
            start = Position((min_y - b) / m, min_y)
            end = Position((max_y - b) / m, max_y)

        return Line(start, end)

    @staticmethod
    def line_from_points_ransac(pts, tolerance=5, iterations=100): #line in format Line(Pos, Pos)
        if len(pts) < 2:
            raise ValueError("Need at least two points")

        def point_line_distance(p, line):
            if line is None:
                return INF
            start, end = line.start, line.end
            line_vec = end - start
            point_vec = p - start
            line_len = line_vec.magnitude()

            if line_len == 0:
                return (p - start).magnitude()

            # project point onto the line segment
            t = max(0, min(1, point_vec.dot(line_vec) / (line_len ** 2)))
            projection = start + line_vec * t

            return (p - projection).magnitude()

        best_inliers = []
        best_line = None

        for _ in range(iterations):
            p1, p2 = random.sample(pts, 2)

            # skip duplicate points
            if p1.x == p2.x and p1.y == p2.y:
                continue

            candidate = Line(p1, p2)

            inliers = [
                p for p in pts
                if point_line_distance(p, candidate) <= tolerance
            ]

            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_line = candidate

        if len(best_inliers) < 2:
            raise ValueError("Could not find a valid line")

        # refine using least-squares fit on all inliers
        try:
            best_line = Line.best_fit(best_inliers)
        except ValueError:
            x_coords = [p.x for p in best_inliers]
            avg_x = sum(x_coords) / len(x_coords) if x_coords else 0
            y_coords = [p.y for p in best_inliers]
            best_line = Line( #approximate line 
                start=Position(avg_x, y=min(y_coords)),
                end=Position(avg_x, y=max(y_coords))
            )

        return best_line, best_inliers

    def __init__(self, start=Position(), end=Position()):
        self.start = start
        self.end = end
        self.length = (end - start).magnitude()

    def is_valid(self, point, tolerance=1):
        return self.distance_to_point(point) <= tolerance
    
    def distance_to_point(self, point):
        line_vec = self.end - self.start
        point_vec = point - self.start

        if self.length == 0:
            return (point - self.start).magnitude()

        # Project onto the segment
        t = max(0, min(1, point_vec.dot(line_vec) / (self.length ** 2)))
        projection = self.start + line_vec * t

        return (point - projection).magnitude()

    
    def sum_of_squared_residuals(self, pts):
        line_vec = self.end - self.start

        if self.length == 0:
            return sum((p - self.start).magnitude() ** 2 for p in pts)

        ssr = 0.0
        for p in pts:
            point_vec = p - self.start

            # Project onto the line segment
            t = max(0, min(1, point_vec.dot(line_vec) / (self.length ** 2)))
            projection = self.start + line_vec * t

            residual = (p - projection).magnitude()
            ssr += residual ** 2

        return ssr
    
    def slope(self):
        v = (self.end - self.start)
        return v.y/v.x
    
    def __repr__(self):
        return f"Line(start={self.start}, end={self.end})"

class Arc:
    @staticmethod
    def from_three_points(p1, p2, p3):
        # compute determinant to detect collinear points
        d = 2 * (
            p1.x * (p2.y - p3.y)
            + p2.x * (p3.y - p1.y)
            + p3.x * (p1.y - p2.y)
        )

        if abs(d) < 1e-9:
            raise ValueError("Points are collinear")

        # circumcenter
        ux = (
            (p1.x**2 + p1.y**2) * (p2.y - p3.y)
            + (p2.x**2 + p2.y**2) * (p3.y - p1.y)
            + (p3.x**2 + p3.y**2) * (p1.y - p2.y)
        ) / d

        uy = (
            (p1.x**2 + p1.y**2) * (p3.x - p2.x)
            + (p2.x**2 + p2.y**2) * (p1.x - p3.x)
            + (p3.x**2 + p3.y**2) * (p2.x - p1.x)
        ) / d

        center = Position(ux, uy)
        radius = (p1 - center).magnitude()

        # angles of the three points
        a1 = math.atan2(p1.y - uy, p1.x - ux) % (2 * math.pi)
        a2 = math.atan2(p2.y - uy, p2.x - ux) % (2 * math.pi)
        a3 = math.atan2(p3.y - uy, p3.x - ux) % (2 * math.pi)

        # true if target lies on the CCW arc from start to end
        def ccw_contains(start, end, target):
            if end >= start:
                return start <= target <= end
            else:
                return target >= start or target <= end

        # choose arc whose sweep contains middle point
        if ccw_contains(a1, a3, a2):
            start_angle = a1
            end_angle = a3
        else:
            start_angle = a3
            end_angle = a1

        return Arc(
            center=center,
            radius=radius,
            start_angle=start_angle,
            end_angle=end_angle,
        )
    
    @staticmethod
    def best_fit(*args): #assumes all points are roughly circular, no extreme outliers towards ends
        pts = []

        for arg in args:
            if isinstance(arg, list):
                for pt in arg:
                    if not isinstance(pt, Position):
                        raise TypeError("Non-point in list")
                pts.extend(arg)
            elif isinstance(arg, Position):
                pts.append(arg)
            else:
                raise TypeError("Non-point in args")

        n = len(pts)
        if n < 3:
            raise ValueError("Need at least three points")

        # calculate the centroid of the points
        centroid_x = sum(p.x for p in pts) / n
        centroid_y = sum(p.y for p in pts) / n

        # calculate the average distance from the centroid to the points
        avg_radius = sum(math.sqrt((p.x - centroid_x) ** 2 + (p.y - centroid_y) ** 2) for p in pts) / n

        # calculate the start and end angles of the arc
        pts_relative = [Position(p.x - centroid_x, p.y - centroid_y) for p in pts]
        angles = [math.atan2(p.y, p.x) for p in pts_relative]
        start_angle = min(angles)
        end_angle = max(angles)

        return Arc(
            center=Position(centroid_x, centroid_y),
            radius=avg_radius,
            start_angle=start_angle,
            end_angle=end_angle
        )
    
    @staticmethod
    def arc_from_points_ransac(pts, tolerance=5, iterations=100):
        if len(pts) < 3:
            raise ValueError("Need at least three points")

        def point_arc_distance(p, arc):
            if arc is None:
                return INF
            center, radius = arc.center, arc.radius
            dist_to_center = (p - center).magnitude()
            return abs(dist_to_center - radius)

        best_inliers = []
        best_arc = None

        for _ in range(iterations):
            p1, p2, p3 = random.sample(pts, 3)

            # skip collinear points
            if (p2.y - p1.y) * (p3.x - p2.x) == (p3.y - p2.y) * (p2.x - p1.x):
                continue

            candidate = Arc.from_three_points(p1, p2, p3)

            inliers = [
                p for p in pts
                if point_arc_distance(p, candidate) <= tolerance
            ]

            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_arc = candidate

        if len(best_inliers) < 3:
            raise ValueError("Could not find a valid arc")

        # refine using least-squares fit on all inliers
        try:
            best_arc = Arc.best_fit(best_inliers)
        except ValueError:
            x_coords = [p.x for p in best_inliers]
            avg_x = sum(x_coords) / len(x_coords) if x_coords else 0
            y_coords = [p.y for p in best_inliers]
            avg_y = sum(y_coords) / len(y_coords) if y_coords else 0
            best_arc = Arc(
                center=Position(avg_x, avg_y),
                radius=0,
                start_angle=0,
                end_angle=0
            )

        return best_arc, best_inliers
    
    def sum_of_squared_residuals(self, pts):
        ssr = 0.0

        for p in pts:
            dist_to_center = (p - self.center).magnitude()
            residual = dist_to_center - self.radius
            ssr += residual ** 2

        return ssr

    
    def __init__(self, center=Position(), radius=0, start_angle=0, end_angle=0):
        self.center = center
        self.radius = radius
        self.start_angle = start_angle
        self.end_angle = end_angle
        sweep = (end_angle - start_angle) % (2 * math.pi)
        self.length = radius * sweep

    def is_valid(self, point, tolerance=1):
        dist_to_center = (point - self.center).magnitude()
        if abs(dist_to_center - self.radius) > tolerance:
            return False

        angle = math.atan2(point.y - self.center.y, point.x - self.center.x)
        angle_tolerance = math.tan(tolerance / self.radius)  # Convert linear tolerance to angular tolerance
        if self.start_angle <= self.end_angle:
            return self.start_angle - angle_tolerance <= angle <= self.end_angle + angle_tolerance
        else:
            return (angle >= self.start_angle - angle_tolerance) or (angle <= self.end_angle + angle_tolerance)

    def __repr__(self):
        return f"Arc(center={self.center}, radius={self.radius}, start_angle={self.start_angle}, end_angle={self.end_angle})"

def angular_distance(a1, a2):
    t = abs(a1 - a2)
    if t > math.pi:
        t = 2*math.pi - t
    return abs(t)

import matplotlib.pyplot as plt

def display_pts(pts, f_name="pts"):
    plt.figure()
    x = [pt.x for pt in pts]
    y = [pt.y for pt in pts]

    plt.scatter(x, y, 0.4)
    plt.savefig(f"{f_name}.png")

RGB = ['r', 'g', 'b', 'c', 'm', 'y', 'k', '#ab2345', "#9D8EFF", "#c97acd"]

def display_pts_group(pts_group):
    plt.figure()
    i = 0
    print(len(pts_group))
    for pts in pts_group:
        x = [pt.x for pt in pts]
        y = [pt.y for pt in pts]

        print(pts)
        plt.scatter(x, y, 0.4, c=RGB[i%len(RGB)])
        i += 1
        
    
    plt.savefig("pts.png")

WHEELBASE = 4 #temp value, should read this from a file when finished

#all jacobians are specific to this particular racecar/design
def state_transistion_jacobian(state_estimate, control_input, delta_t, **kwargs): #x, y, heading, steering angle | speed, angle_speed | delta_t | wheelbase
    x, y, theta, sigma = state_estimate
    v, omega = control_input
    L = kwargs.get('wheelbase', WHEELBASE)

    return np.array([ #partial derivaitves of the control model from lecture 3
        [1, 0, -v * math.sin(theta) * delta_t, 0],
        [0, 1, v * math.cos(theta) * delta_t, 0],
        [0, 0, 1, v/L * (1/np.cos(sigma)**2) * delta_t],
        [0, 0, 0, 1]
    ])

def measurement_jacobian(state_estimate, delta_t, **kwargs): #kwargs for system parameters, measurement doesnt need control input, since unrelated
    return np.array([ #since measurement = state estimation from lidar icp directly gives pose transform, 1:1 mapping for all x, y, omega, 0 for sigma since steering angle unfindable form icp alone
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0] #how measurement changes when state changes
    ])

def process_noise_jacobian(state_estimate, control_input, delta_t, **kwargs): #x, y, heading, steering angle | speed, angle_speed | delta_t | wheelbase
    x, y, theta, sigma = state_estimate #gives effect of process noise. since noise enters through controls exclusively in this model, control jacobian is not needed
    L = kwargs.get('wheelbase', WHEELBASE)

    return np.array([
        [math.cos(theta) * delta_t, 0], #derivative of control model + noise (sub v -> v + v_noise, theta -> theta + theta_noise)
        [math.sin(theta) * delta_t, 0],
        [math.tan(sigma)/L * delta_t, 0],
        [0, delta_t]
    ])

class ExtendedKalmanFilter:
    def __init__(self,
                 initial_state_estimate,
                 initial_state_covariance,
                 process_noise_covariance,
                 measurement_noise_covariance,
                 state_transistion_jacobian,
                 measurement_jacobian,
                 process_noise_jacobian): #last three are funcs

        self.state_estimate = initial_state_estimate
        self.state_estimation_covariance = initial_state_covariance
        self.process_noise_covariance = process_noise_covariance
        self.measurement_noise_covariance = measurement_noise_covariance

        self.state_transistion_jacobian = state_transistion_jacobian
        self.measurement_jacobian = measurement_jacobian
        self.process_noise_jacobian = process_noise_jacobian

        self.kalman_gain = None

        self.change_in_predicted_state = np.zeros(3) #accumulate imu steps, while waiting for lidar input

    def predict_state(self, control_input, imu_transform): #imu based, imu transform (state estimation function) evaluated in odom section
        self.change_in_predicted_state += imu_transform
        F = self.state_transistion_jacobian()

    def update_kalman_gain(self, control_input, delta_t, **kwargs):
        H = self.measurement_jacobian(self.state_estimate, control_input, delta_t, **kwargs) #get jacobian
        P = self.state_estimation_covariance
        R = self.measurement_noise_covariance

        S = H @ P @ H.T + R #innovation covariance: basically, hpht is uncertainty of state translated into uncertainty of measurement, R, is uncertainty due to sensor noise, sum is "measurement error" (not exactly, close enough)
        self.kalman_gain = P @ H.T @ np.linalg.inv(S) # pht translates from "how wrong is my measurement" (innovation, or S) to how states are affeced by how wrong the measruement is

    def update_state(self, control_input, lidar_transform):
        pass #lidar based

class KalmanFilter: #1d, for processign IMU and encoder datas
    def __init__():
        pass

class Particle:
    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

class LidarProcessingDummy: #dummy repersentation for a ros node on real bot
    def __init__(self, ekf, ):
        pass

    def lidar_measurement_callback(data):
        pass
       
class SLAM:
    def __init__(self, ekf, ):
        self.prob_occupancy_map = np.array()


########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    pass
# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    pass # Remove 'pass' and write your source code for the update() function here

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter

def avg_points(pts, sample_size=3, step_size=1, dist_tolerance=15): #step size = 1 for rolling, stepsize = samplesize for avg 
    l = len(pts)
    out = []
    
    for i in range(0, l, step_size):
        cur_pt_sum = Position(0, 0)
        cur_pt_cnt = 0
        p_pt = pts[i] #first one, always valid

        for j in range(sample_size):
            cur_pt = pts[(i + j) % l]

            if (cur_pt - p_pt).magnitude() > dist_tolerance:
                break
            cur_pt_sum += cur_pt
            cur_pt_cnt += 1
            p_pt = cur_pt #update prev
        
        p = cur_pt_sum/cur_pt_cnt
        out.append(p)
    return out

def grid_filter(pts, grid_size=5):
    out = set()
    for pt in pts:
        out.add((pt.x//grid_size * grid_size, pt.y//grid_size * grid_size))

    return [Position(p[0], p[1]) for p in out]

def agg_avg_points(pts, dist_tolerance=9): #step size = 1 for rolling, stepsize = samplesize for avg 
    l = len(pts)
    out = []

    prev_agg = None
    
    for i in range(1, l):
        if prev_agg is None:
            if (pts[i] - pts[i - 1]).magnitude() < dist_tolerance:
                prev_agg = (pts[i] + pts[i - 1]) / 2
                out.append(prev_agg) #add new aggregate point
            else:
                out.append(pts[i]) #just add normal poitn
        elif (pts[i] - prev_agg).magnitude() < dist_tolerance:
            prev_agg = (pts[i] + prev_agg) / 2 #aggregate cur and prev
            out[-1] = prev_agg #replace prev agg with cur agg
        else:
            out.append(pts[i])
            prev_agg = None #no prevagg and wihtin dstnace

    return out


def update_slow():
    pass
    
    #ft = get_local_features(lidar=rc.lidar,tolerance=15,step_size=1)
    

# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
