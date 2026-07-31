import heapq
import math

#from nav_msgs.msg import OccupancyGrid #comment out if unit test

''' 
class OccupancyGrid:  #dummy to stop error for unit test outside of ros
    pass
    
'''

class AStarPlanner:
    """
    A* path planner for ROS 2 nav_msgs/msg/OccupancyGrid messages.

    The planner expects occupancy values in the standard ROS format:

        -1      = unknown
         0      = free
        1..100  = occupancy probability

    By default:
        - cells with occupancy >= 60 are treated as obstacles
        - unknown cells are treated as obstacles
    """

    def __init__(
        self,
        occupancy_grid: OccupancyGrid,
        obstacle_threshold: int = 60,
        allow_unknown: bool = False,
    ):
        self.map = occupancy_grid

        self.width = occupancy_grid.info.width
        self.height = occupancy_grid.info.height
        self.resolution = occupancy_grid.info.resolution

        self.origin_x = occupancy_grid.info.origin.position.x
        self.origin_y = occupancy_grid.info.origin.position.y

        self.obstacle_threshold = obstacle_threshold
        self.allow_unknown = allow_unknown

        # ROS OccupancyGrid stores data in row-major order:
        #
        # index = y * width + x
        #
        self.data = occupancy_grid.data

    @staticmethod
    def heuristic(a, b):
        """
        Euclidean-distance heuristic.
        """
        return math.hypot(
            a[0] - b[0],
            a[1] - b[1],
        )

    def world_to_grid(self, x, y):
        """
        Convert world coordinates to integer grid coordinates.

        This implementation assumes the map origin has no rotation.
        """
        grid_x = math.floor(
            (x - self.origin_x) / self.resolution
        )

        grid_y = math.floor(
            (y - self.origin_y) / self.resolution
        )

        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """
        Convert a grid cell to the world coordinates of its center.
        """
        world_x = (
            self.origin_x
            + (grid_x + 0.5) * self.resolution
        )

        world_y = (
            self.origin_y
            + (grid_y + 0.5) * self.resolution
        )

        return world_x, world_y

    def is_in_bounds(self, x, y):
        """
        Check whether a grid coordinate is inside the map.
        """
        return (
            0 <= x < self.width
            and 0 <= y < self.height
        )

    def get_occupancy(self, x, y):
        """
        Return the occupancy value for a grid cell.

        ROS OccupancyGrid data is stored as:

            data[y * width + x]
        """
        index = y * self.width + x
        return self.data[index]

    def is_traversable(self, x, y):
        """
        Determine whether a cell can be used by the planner.
        """
        if not self.is_in_bounds(x, y):
            return False

        occupancy = self.get_occupancy(x, y)

        # Unknown cells have a value of -1.
        if occupancy == -1:
            return self.allow_unknown

        return occupancy < self.obstacle_threshold

    def get_neighbors(self, current):
        """
        Return valid neighboring cells and their movement costs.

        Diagonal movement is allowed. Diagonal corner-cutting is prevented,
        so the robot cannot move diagonally between two obstacles.
        """
        x, y = current

        directions = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]

        for dx, dy in directions:
            nx = x + dx
            ny = y + dy

            if not self.is_traversable(nx, ny):
                continue

            # Prevent diagonal corner-cutting.
            if dx != 0 and dy != 0:
                if not self.is_traversable(x + dx, y):
                    continue

                if not self.is_traversable(x, y + dy):
                    continue

            step_cost = math.hypot(dx, dy)

            yield (nx, ny), step_cost

    def plan(self, start_world, goal_world):
        """
        Find a path from start_world to goal_world.

        Args:
            start_world:
                Tuple containing (x, y) in the map frame.

            goal_world:
                Tuple containing (x, y) in the map frame.

        Returns:
            A list of world-coordinate tuples:

                [
                    (x0, y0),
                    (x1, y1),
                    ...
                ]

            Returns None if no path exists.
        """
        start = self.world_to_grid(
            start_world[0],
            start_world[1],
        )

        goal = self.world_to_grid(
            goal_world[0],
            goal_world[1],
        )

        if not self.is_in_bounds(*start):
            raise ValueError(
                f"Start position {start_world} is outside the map."
            )

        if not self.is_in_bounds(*goal):
            raise ValueError(
                f"Goal position {goal_world} is outside the map."
            )

        if not self.is_traversable(*start):
            raise ValueError(
                f"Start cell {start} is occupied or unknown."
            )

        if not self.is_traversable(*goal):
            raise ValueError(
                f"Goal cell {goal} is occupied or unknown."
            )

        open_set = []

        # A counter avoids comparing grid-coordinate tuples when
        # two heap entries have the same priority.
        counter = 0

        start_f = self.heuristic(
            start,
            goal,
        )

        heapq.heappush(
            open_set,
            (
                start_f,
                counter,
                start,
            ),
        )

        came_from = {}

        g_score = {
            start: 0.0
        }

        closed = set()

        while open_set:

            _, _, current = heapq.heappop(
                open_set
            )

            if current in closed:
                continue

            if current == goal:
                return self._reconstruct_path(
                    came_from,
                    current,
                )

            closed.add(current)

            for neighbor, step_cost in self.get_neighbors(
                current
            ):

                if neighbor in closed:
                    continue

                tentative_g = (
                    g_score[current]
                    + step_cost
                )

                if tentative_g >= g_score.get(
                    neighbor,
                    float("inf"),
                ):
                    continue

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g

                f_score = (
                    tentative_g
                    + self.heuristic(
                        neighbor,
                        goal,
                    )
                )

                counter += 1

                heapq.heappush(
                    open_set,
                    (
                        f_score,
                        counter,
                        neighbor,
                    ),
                )

        return None

    def _reconstruct_path(
        self,
        came_from,
        current,
    ):
        """
        Reconstruct the grid path and convert it to world coordinates.
        """
        grid_path = [current]

        while current in came_from:
            current = came_from[current]
            grid_path.append(current)

        grid_path.reverse()

        return [
            self.grid_to_world(x, y)
            for x, y in grid_path
        ]