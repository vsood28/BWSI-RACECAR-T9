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

def bayesian_update(prev_odds, mea_odds, prior_odds):
    return prev_odds + mea_odds - prior_odds

def log_odds(prob):
    return math.log(prob/(1 - prob))
    
def recover_probability(log_odd):
    return 1 / (1 + math.exp(-log_odd))

def bresenham(x0, y0, x1, y1, cell_size=1):
    x0 = int(math.floor(x0 / cell_size))
    y0 = int(math.floor(y0 / cell_size))
    x1 = int(math.floor(x1 / cell_size))
    y1 = int(math.floor(y1 / cell_size))

    cells = []

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    err = dx - dy

    while True:
        cells.append((x0, y0))

        if x0 == x1 and y0 == y1:
            break

        e2 = 2 * err

        if e2 > -dy:
            err -= dy
            x0 += sx

        if e2 < dx:
            err += dx
            y0 += sy

    return cells

class OccupancyGrid:
    def __init__(self, width, height, resolution, prior_odds, poh, pom):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.prior_odds = log_odds(prior_odds)
        self.prob_occ_hit = log_odds(poh)
        self.prob_occ_miss = log_odds(pom)
        self.grid = np.full((width, height), log_odds(prior_odds), dtype=np.float64)

    def to_ros_occupancy_grid(
        self,
        frame_id="map",
        origin_x=0.0,
        origin_y=0.0,
        stamp=None,
        unknown_threshold=0.1 #min differnce from prior odds required to acutally be considered to be confidently one or the other
    ):
        """
        Convert the internal log-odds occupancy grid to a ROS 2
        nav_msgs/msg/OccupancyGrid message.

        Parameters
        ----------
        frame_id : str
            Coordinate frame of the map.

        origin_x, origin_y : float
            World coordinates of the lower-left corner of the map.

        stamp : builtin_interfaces.msg.Time, optional
            ROS timestamp. If None, the message stamp is left at zero.

        unknown_threshold : float
            Cells whose probability is close to the prior probability
            are marked as unknown (-1).
        """

        msg = ROSOccupancyGrid()

        # Header
        msg.header.frame_id = frame_id

        if stamp is not None:
            msg.header.stamp = stamp

        # Map metadata
        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.width)
        msg.info.height = int(self.height)

        # Map origin
        msg.info.origin.position.x = float(origin_x)
        msg.info.origin.position.y = float(origin_y)
        msg.info.origin.position.z = 0.0

        # Identity orientation: no rotation.
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Convert log-odds to probabilities.
        probabilities = 1.0 / (1.0 + np.exp(-self.grid)) #cant use other since this is using vectorizatio

        # Convert probabilities from [0, 1] to ROS occupancy [0, 100].
        occupancy = np.rint(probabilities * 100.0).astype(np.int8)

        # Cells that have not meaningfully changed from the prior are unknown.
        prior_probability = recover_probability(self.prior_odds)

        unknown = (
            np.abs(probabilities - prior_probability)
            < unknown_threshold
        )

        occupancy[unknown] = -1

        # ROS uses row-major order:
        # data[y * width + x]
        #
        # Internal grid shape is (width, height), so transpose it
        # to (height, width) before flattening.
        msg.data = occupancy.T.flatten().tolist()

        return msg


    def update_grid(self, point_cloud, pose):
        abs_points = [p.to_absolute(pose) for p in point_cloud]

        origin = pose.to_position()
        x0, y0 = origin.x, origin.y

        empty = []
        hit = []

        for p in abs_points:
            empty += bresenham(x0, y0, p.x, p.y, self.resolution)[:-1] #last point is the final point that is hit
            hit.append((int(p.x // self.resolution), int(p.y // self.resolution))) #from world coords to grid coords

        for cell in empty:
            x, y = cell
            self.grid[x][y] = bayesian_update(self.grid[x][y], self.prob_occ_miss, self.prior_odds) #update each empty cell 

        for cell in hit:
            x, y = cell
            self.grid[x][y] = bayesian_update(self.grid[x][y], self.prob_occ_hit, self.prior_odds) #update each hit cell

    def world_to_grid(self, x, y):
        return int(x // self.resolution), int(y // self.resolution)
    
    def grid_to_world(self, x, y):
        return x * self.resolution, y * self.resolution

    def get_odds(self, x, y):
        return recover_probability(self.grid[int(x // self.resolution)][int(y // self.resolution)]) #world coord to grid coord
        