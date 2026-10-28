"""
A* Path Planner and Path Following Module for Section 1
Implements 8-connected A* search with diagonal corner-cutting prevention,
line-of-sight path smoothing, and proportional waypoint tracking.
"""
import heapq
import math
import config

class PathPlanner:
    def __init__(self, grid):
        self.grid = grid

    def plan(self, start_world, goal_world):
        """
        Plans a smoothed path from start_world (x, y) to goal_world (x, y).
        Returns list of (x, y) waypoints in world coordinates, or [] if no path.
        """
        sgx, sgy = self.grid.world_to_grid(start_world[0], start_world[1])
        ggx, ggy = self.grid.world_to_grid(goal_world[0], goal_world[1])

        # Clamp start to bounds
        sgx = max(0, min(self.grid.width - 1, sgx))
        sgy = max(0, min(self.grid.height - 1, sgy))

        # If goal is occupied, find closest traversable cell
        if not self.grid.is_traversable(ggx, ggy):
            ggx, ggy = self._find_nearest_traversable(ggx, ggy)
            if ggx is None:
                return []

        # A* Priority Queue: (f_score, cost, (gx, gy))
        open_set = []
        heapq.heappush(open_set, (0.0, 0.0, (sgx, sgy)))

        came_from = {}
        g_score = {(sgx, sgy): 0.0}

        # 8-connectivity movements: (dx, dy, cost)
        moves = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]

        found = False
        target = (ggx, ggy)

        while open_set:
            f, cost, current = heapq.heappop(open_set)
            cx, cy = current

            if current == target:
                found = True
                break

            if cost > g_score.get(current, float('inf')):
                continue

            for dx, dy, move_cost in moves:
                nx, ny = cx + dx, cy + dy

                # Check bounds
                if not self.grid.in_bounds(nx, ny):
                    continue

                cell_val = self.grid.grid[ny][nx]
                # Hard obstacle (1) is impassable
                if cell_val == 1:
                    continue

                # Soft inflated obstacle (2) has heavy cost penalty
                extra_cost = 6.0 if cell_val == 2 else 0.0

                # Corner-cutting check for diagonal moves
                if dx != 0 and dy != 0:
                    if self.grid.grid[cy][cx + dx] == 1 or self.grid.grid[cy + dy][cx] == 1:
                        continue

                tentative_g = cost + move_cost + extra_cost
                neighbor = (nx, ny)

                if tentative_g < g_score.get(neighbor, float('inf')):
                    g_score[neighbor] = tentative_g
                    # Euclidean heuristic
                    h = math.hypot(ggx - nx, ggy - ny)
                    heapq.heappush(open_set, (tentative_g + h, tentative_g, neighbor))
                    came_from[neighbor] = current

        if not found:
            return []

        # Reconstruct path
        path_grid = []
        curr = target
        while curr in came_from:
            path_grid.append(curr)
            curr = came_from[curr]
        path_grid.append((sgx, sgy))
        path_grid.reverse()

        # Smooth path using line-of-sight
        smoothed_grid = self._smooth_path(path_grid)

        # Convert to world coordinates
        waypoints = [self.grid.grid_to_world(gx, gy) for gx, gy in smoothed_grid]
        return waypoints

    def _smooth_path(self, path):
        """Line-of-sight path pruning."""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            furthest = current_idx + 1
            for next_idx in range(len(path) - 1, current_idx + 1, -1):
                if self._line_of_sight(path[current_idx], path[next_idx]):
                    furthest = next_idx
                    break
            smoothed.append(path[furthest])
            current_idx = furthest

        return smoothed

    def _line_of_sight(self, p0, p1):
        """Bresenham check between two grid cells."""
        x0, y0 = p0
        x1, y1 = p1
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            if not self.grid.is_traversable(x, y) and (x, y) != p0 and (x, y) != p1:
                return False
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return True

    def _find_nearest_traversable(self, gx, gy):
        """BFS search for closest traversable cell if goal is in obstacle."""
        queue = [(gx, gy)]
        visited = {(gx, gy)}

        for _ in range(100): # search up to 100 cells
            if not queue:
                break
            cx, cy = queue.pop(0)
            if self.grid.is_traversable(cx, cy):
                return cx, cy

            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = cx + dx, cy + dy
                if self.grid.in_bounds(nx, ny) and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny))

        return None, None
