import math
import numpy as np
import pytest

from occupancy_grid import (
    OccupancyGrid,
    bayesian_update,
    log_odds,
    recover_probability,
    bresenham,
)



# ---------------------------------------------------------------------
# Helper classes
# ---------------------------------------------------------------------

class DummyPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def to_absolute(self, pose):
        return DummyPoint(
            self.x + pose.x,
            self.y + pose.y,
        )


class DummyPose:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def to_position(self):
        return DummyPoint(self.x, self.y)


# ---------------------------------------------------------------------
# Probability utilities
# ---------------------------------------------------------------------

@pytest.mark.parametrize("p", [0.1, 0.2, 0.5, 0.8, 0.95])
def test_log_odds_inverse(p):
    """recover_probability(log_odds(p)) == p"""
    assert recover_probability(log_odds(p)) == pytest.approx(p)


def test_log_odds_half_is_zero():
    assert log_odds(0.5) == pytest.approx(0.0)


def test_bayesian_update():
    prev = log_odds(0.5)
    hit = log_odds(0.8)
    prior = log_odds(0.5)

    updated = bayesian_update(prev, hit, prior)

    assert recover_probability(updated) == pytest.approx(0.8)


# ---------------------------------------------------------------------
# Bresenham
# ---------------------------------------------------------------------

def test_bresenham_horizontal():
    cells = bresenham(0, 0, 3, 0)

    assert cells == [
        (0, 0),
        (1, 0),
        (2, 0),
        (3, 0),
    ]


def test_bresenham_vertical():
    cells = bresenham(0, 0, 0, 3)

    assert cells == [
        (0, 0),
        (0, 1),
        (0, 2),
        (0, 3),
    ]


def test_bresenham_diagonal():
    cells = bresenham(0, 0, 3, 3)

    assert cells == [
        (0, 0),
        (1, 1),
        (2, 2),
        (3, 3),
    ]


# ---------------------------------------------------------------------
# Grid initialization
# ---------------------------------------------------------------------

def test_grid_initialization():
    grid = OccupancyGrid(
        width=20,
        height=15,
        resolution=0.5,
        prior_odds=0.5,
        poh=0.8,
        pom=0.3,
    )

    assert grid.width == 20
    assert grid.height == 15
    assert grid.resolution == 0.5

    assert grid.grid.shape == (20, 15)

    assert np.all(grid.grid == 0) #0 is log odds of 0.5


# ---------------------------------------------------------------------
# Coordinate conversion
# ---------------------------------------------------------------------

def test_world_to_grid():
    grid = OccupancyGrid(10, 10, 1.0, 0.5, 0.8, 0.3)

    assert grid.world_to_grid(2.9, 4.2) == (2, 4)


def test_grid_to_world():
    grid = OccupancyGrid(10, 10, 0.5, 0.5, 0.8, 0.3)

    assert grid.grid_to_world(4, 6) == (2.0, 3.0)


# ---------------------------------------------------------------------
# Occupancy lookup
# ---------------------------------------------------------------------

def test_get_odds_prior():
    grid = OccupancyGrid(10, 10, 1.0, 0.5, 0.8, 0.3)

    assert grid.get_odds(3, 4) == pytest.approx(0.5)


# ---------------------------------------------------------------------
# Grid updates
# ---------------------------------------------------------------------

def test_single_ray_update():
    """
    Ray:
    robot -> (3,0)

    Expected:

    (0,0) free
    (1,0) free
    (2,0) free
    (3,0) occupied
    """

    grid = OccupancyGrid(
        width=10,
        height=10,
        resolution=1.0,
        prior_odds=0.5,
        poh=0.8,
        pom=0.3,
    )

    pose = DummyPose(0, 0)

    cloud = [
        DummyPoint(3, 0)
    ]

    grid.update_grid(cloud, pose)

    assert recover_probability(grid.grid[0][0]) < 0.5
    assert recover_probability(grid.grid[1][0]) < 0.5
    assert recover_probability(grid.grid[2][0]) < 0.5

    assert recover_probability(grid.grid[3][0]) > 0.5


def test_multiple_updates_strengthen_belief():
    grid = OccupancyGrid(
        10,
        10,
        1.0,
        0.5,
        0.8,
        0.3,
    )

    pose = DummyPose()

    cloud = [DummyPoint(2, 0)]

    grid.update_grid(cloud, pose)
    first = recover_probability(grid.grid[2][0])

    grid.update_grid(cloud, pose)
    second = recover_probability(grid.grid[2][0])

    assert second > first


def test_two_hit_cells():
    grid = OccupancyGrid(
        10,
        10,
        1.0,
        0.5,
        0.8,
        0.3,
    )

    pose = DummyPose()

    cloud = [
        DummyPoint(3, 0),
        DummyPoint(0, 3),
    ]

    grid.update_grid(cloud, pose)

    assert recover_probability(grid.grid[3][0]) > 0.5
    assert recover_probability(grid.grid[0][3]) > 0.5


# ---------------------------------------------------------------------
# ROS conversion
# ---------------------------------------------------------------------

def test_to_ros_occupancy_grid():
    grid = OccupancyGrid(
        5,
        5,
        1.0,
        0.5,
        0.8,
        0.3,
    )

    # Make one occupied cell
    grid.grid[2][2] = log_odds(0.9)

    print(grid.grid)

    ros = grid.to_ros_occupancy_grid()

    assert ros.info.width == 5
    assert ros.info.height == 5
    assert ros.info.resolution == pytest.approx(1.0)

    assert len(ros.data) == 25

    print(ros)

    # Cell should not be unknown
    index = 2 * 5 + 2
    assert ros.data[index] > 50


def test_unknown_cells_marked_unknown():
    grid = OccupancyGrid(
        5,
        5,
        1.0,
        0.5,
        0.8,
        0.3,
    )

    ros = grid.to_ros_occupancy_grid()

    assert all(v == -1 for v in ros.data)