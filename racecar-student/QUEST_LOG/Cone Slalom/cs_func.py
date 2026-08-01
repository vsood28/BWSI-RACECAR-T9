import math
import numpy as np

def lidar_to_positions(lidar): #lidar to cartesian
    smp = lidar.get_samples()
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


class Position: #position class 
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def to_array(self):
        return np.array([self.x, self.y])
    
    def to_absolute(self, reference_pose):
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

    def normal(self):
        n = Position(-self.y, self.x)
        n /= n.magnitude()
        return n

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

def point_avg(pts): #avg of points
    out = Position(0, 0)
    for pt in pts:
        out += pt
    return out / len(pts)

def find_cones(lidar, cone_max_size=12, min_gap=50):
    pts = lidar_to_positions(lidar) #carts

    cone_pts = []

    current = []

    for pt in pts:
        if len(current) == 0 or pt.distance(current[-1]) < min_gap: #not a gap
            current.append(pt)
        else: #gap, looking at new cluster now
            if len(current) <= cone_max_size: #if cluster small enogu
                cone_pts.append(current) #yay cone

            current = [pt] #create new cluster

    out = [point_avg(c_gp) for c_gp in cone_pts] #create single point to reperesnt each point

    return out

def closest_cone(cones, ignore_behind=True):
    if len(cones) == 0: #null case
        return None
    
    closest = cones[0] #default

    for pt in cones:
        if (not ignore_behind or pt.y > 0) and pt.magnitude() < closest.magnitude(): #get closest (i mean what did you expect)
            closest = pt

    return closest 

def target_point(cone, side, target_offset=30): #get target given cone pose and target side
    if cone is None:
        return Position(0, 1) #forwards

    n = cone.normal() #normal

    if (n.x > 0 and side) or (n.x < 0 and not side): #correct side, side=true means right
        n *= target_offset #scale to target offset
    else:  
        n *= -target_offset #flip so normal faces the right side

    return cone + n #add the offset from cone to cone pose

def angle_to_target(pt): #pt toangle for error
    return math.atan2(pt.x, pt.y)

def min_scan(scan, ind, wind): #min of scan
    m = float('inf')
    n = len(scan)

    for offset in range(-wind, wind + 1):
        value = scan[(ind + offset) % n]  # circular indexing

        if value != 0 and value < m:
            m = value #if val lower, update

    return m