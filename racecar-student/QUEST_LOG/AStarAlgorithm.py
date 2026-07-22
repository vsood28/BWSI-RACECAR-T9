import heapq
import math

from occupancy_grid import recover_probability


def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def astar(occupancy_grid, start_world, goal_world, obstacle_threshold=0.6):

    start = occupancy_grid.world_to_grid(*start_world)
    goal = occupancy_grid.world_to_grid(*goal_world)

    neighbors = [
        (-1, 0), (1, 0),
        (0, -1), (0, 1),
        (-1, -1), (-1, 1),
        (1, -1), (1, 1),
    ]

    open_set = []
    heapq.heappush(open_set, (0, start))

    closed = set()

    came_from = {}

    g_score = {
        start: 0.0
    }

    while open_set:

        _, current = heapq.heappop(open_set)

        if current in closed:
            continue

        closed.add(current)

        if current == goal:

            path = []

            while current in came_from:
                path.append(current)
                current = came_from[current]

            path.append(start)
            path.reverse()

            return [
                occupancy_grid.grid_to_world(x, y)
                for x, y in path
            ]

        cx, cy = current

        for dx, dy in neighbors:

            nx = cx + dx
            ny = cy + dy

            if (
                nx < 0
                or ny < 0
                or nx >= occupancy_grid.width
                or ny >= occupancy_grid.height
            ):
                continue

            if (nx, ny) in closed:
                continue


            probability = recover_probability(
                occupancy_grid.grid[nx][ny]
            )

            if probability >= obstacle_threshold:
                continue

            step_cost = math.hypot(dx, dy)

            tentative_g = g_score[current] + step_cost

            if tentative_g < g_score.get((nx, ny), float("inf")):

                came_from[(nx, ny)] = current

                g_score[(nx, ny)] = tentative_g

                f = tentative_g + heuristic(
                    (nx, ny),
                    goal,
                )

                heapq.heappush(
                    open_set,
                    (
                        f,
                        (nx, ny),
                    ),
                )

    return None