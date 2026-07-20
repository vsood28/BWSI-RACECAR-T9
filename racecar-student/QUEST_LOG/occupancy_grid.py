import numpy as np
import math

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
        self.prior_odds = prior_odds
        self.prob_occ_hit = log_odds(poh)
        self.prob_occ_miss = log_odds(pom)
        self.grid = np.full((width, height), log_odds(prior_odds))

    def update_grid(self, point_cloud, pose):
        abs_points = [p.to_absolute(pose) for p in point_cloud]

        origin = pose.to_position()
        x0, y0 = origin.x, origin.y

        empty = []
        hit = []

        for p in abs_points:
            empty += bresenham(x0, y0, p.x, p.y, self.resolution)[:-1] #last point is the final point that is hit
            hit.append((p.x // self.resolution, p.y // self.resolution)) #from world coords to grid coords

        for cell in empty:
            x, y = cell
            self.grid[x][y] = bayesian_update(self.grid[x][y], self.prob_occ_miss, self.prior_odds)

        for cell in hit:
            x, y = cell
            self.grid[x][y] = bayesian_update(self.grid[x][y], self.prob_occ_hit, self.prior_odds)

    def get_odds(self, x, y):
        return recover_probability(self.grid[x // self.resolution][y // self.resolution]) #world coord to grid coord
        