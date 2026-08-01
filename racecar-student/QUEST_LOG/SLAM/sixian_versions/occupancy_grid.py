from nav_msgs.msg import OccupancyGrid as ROSOccupancyGrid #comment out for unittest
import numpy as np
import math

'''
from dataclasses import dataclass, field #fake class for unit test outside of ros

@dataclass
class Header:
    frame_id: str = ""
    stamp = None


@dataclass
class Position:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Orientation:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class Pose:
    position: Position = field(default_factory=Position)
    orientation: Orientation = field(default_factory=Orientation)


@dataclass
class MapMetaData:
    resolution: float = 0.0
    width: int = 0
    height: int = 0
    origin: Pose = field(default_factory=Pose)


@dataclass
class ROSOccupancyGrid:
    header: Header = field(default_factory=Header)
    info: MapMetaData = field(default_factory=MapMetaData)
    data: list[int] = field(default_factory=list)
'''

#units of meters, radians,  and logodds

def to_absolute(point, pose): #convert point in lidar frame to world frame
    x_abs = point.x * math.cos(pose.theta) - point.y * math.sin(pose.theta) + pose.x #rotation + translation
    y_abs = point.x * math.sin(pose.theta) + point.y * math.cos(pose.theta) + pose.y
    return (x_abs, y_abs)

def bayesian_update(prev_odds, mea_odds, prior_odds): #bayesian update log odds
    return prev_odds + mea_odds - prior_odds #addition since log turns mult to add

def log_odds(prob):
    return math.log(prob/(1 - prob)) #prob (0-1) to log odds
    
def recover_probability(log_odd):
    return 1 / (1 + math.exp(-log_odd)) #log odds back to prob

def bresenham(x0, y0, x1, y1, cell_size=1): #bressenham, algo for all points between two points, used to find all cells between lidar origin and hit point (empty)
    x0 = int(math.floor(x0 / cell_size)) # world coordinates into grid cell coordinates
    y0 = int(math.floor(y0 / cell_size))
    x1 = int(math.floor(x1 / cell_size))
    y1 = int(math.floor(y1 / cell_size))

    cells = [] #output list of cells crossed by the line

    dx = abs(x1 - x0) #horiz dist between start and end cells
    dy = abs(y1 - y0) #vert dist between start and end cells

    sx = 1 if x0 < x1 else -1 #dir to move in x (+1 or -1)
    sy = 1 if y0 < y1 else -1 #dir to move in y (+1 or -1)

    err = dx - dy #error term to decide when to step in each direction

    while True:
        cells.append((x0, y0)) #add current cell to the line path

        if x0 == x1 and y0 == y1: #stop once the end cell is reached
            break

        e2 = 2 * err #double error to avoid repeated calculations

        if e2 > -dy: #if error is large enough, move in x direction
            err -= dy #update error
            x0 += sx #move to next x cell

        if e2 < dx: #if error is large enough, move in y direction
            err += dx #update error
            y0 += sy #move to next

    return cells #return all cells crossed by the line

class OccupancyGrid: #class for occupancy grid repersentation 
    def __init__(self, width, height, resolution, prior_odds, poh, pom):
        self.width = width #set arams
        self.height = height
        self.resolution = resolution
        self.prior_odds = log_odds(prior_odds)
        self.prob_occ_hit = log_odds(poh)
        self.prob_occ_miss = log_odds(pom)
        self.grid = np.full((width, height), log_odds(prior_odds), dtype=np.float64) #init grid with pr odds

    def to_ros_occupancy_grid( #function to convert internal grid to ros occgrid msg
        self,
        frame_id="map",
        origin_x=0.0,
        origin_y=0.0,
        stamp=None,
        unknown_threshold=0.1 #min differnce from prior odds required to acutally be considered to be confidently one or the other
    ):
        msg = ROSOccupancyGrid() #create

        msg.header.frame_id = frame_id #header

        if stamp is not None:
            msg.header.stamp = stamp #stamp

        msg.info.resolution = float(self.resolution) #metadata
        msg.info.width = int(self.width)
        msg.info.height = int(self.height)

        msg.info.origin.position.x = float(origin_x) #oroigin
        msg.info.origin.position.y = float(origin_y)
        msg.info.origin.position.z = 0.0

        msg.info.origin.orientation.x = 0.0 #world frame, no rot
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0 #reper as quat becuase quat better


        probabilities = 1.0 / (1.0 + np.exp(-self.grid)) #cant use other since this is using vectorization from np

        occupancy = np.rint(probabilities * 100.0).astype(np.int8) #0-1 -> 0-100 as expected by ros

        prior_probability = recover_probability(self.prior_odds) #prior

        unknown = ( #all where prob has not changed significantly (unknown threshold)
            np.abs(probabilities - prior_probability)
            < unknown_threshold
        )

        occupancy[unknown] = -1 #set all those to uknown

        msg.data = occupancy.T.flatten().tolist() #flatten, row major, to list

        return msg


    def update_grid(self, point_cloud, pose): #points
        abs_points = [to_absolute(p, pose) for p in point_cloud] #get points in world frame

        x0, y0 = pose[0], pose[1] #inital point

        empty = []
        hit = []

        for p in abs_points: #grab bressenham from lidar origin to each point, all points inbetween are emtpy (since lidar didn thit there)
            empty += bresenham(x0, y0, p[0], p[1], self.resolution)[:-1] #last point is the final point that is hit
            hit.append((int(p[0] // self.resolution), int(p[1] // self.resolution))) #add one hit (lidar hit), from world coords to grid coords

        for cell in empty: #for each empty
            x, y = cell
            self.grid[x][y] = bayesian_update(self.grid[x][y], self.prob_occ_miss, self.prior_odds) #update each empty cell, using probablity that occupied given a miss

        for cell in hit:
            x, y = cell
            self.grid[x][y] = bayesian_update(self.grid[x][y], self.prob_occ_hit, self.prior_odds) #update each hit cell, using probablity that occupied given a hit

    def world_to_grid(self, x, y): #helper 
        return int(x // self.resolution), int(y // self.resolution)
    
    def grid_to_world(self, x, y): #helper, self explanatory
        return x * self.resolution, y * self.resolution

    def get_odds(self, x, y): #get true probablity of occupancy at world coordinate
        return recover_probability(self.grid[int(x // self.resolution)][int(y // self.resolution)]) #world coord to grid coord
        