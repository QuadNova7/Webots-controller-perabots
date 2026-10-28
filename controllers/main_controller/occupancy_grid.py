"""
Occupancy Grid Mapping Module for Section 1
Maintains a 2.5 cm resolution 2D occupancy grid, updates cells using raycasting
from distance sensors, and applies obstacle inflation for safe navigation.
"""
import math
import config

class OccupancyGrid:
    def __init__(self):
        self.res = config.GRID_RESOLUTION
        self.width = config.GRID_WIDTH   # 98 cells along X
        self.height = config.GRID_HEIGHT # 49 cells along Y

        # 0: UNKNOWN/FREE, 1: OCCUPIED, 2: INFLATED
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.hit_counts = [[0 for _ in range(self.width)] for _ in range(self.height)]

        # Inflation radius in grid cells
        self.inflation_cells = int(math.ceil(config.OBSTACLE_INFLATION_RADIUS / self.res))

        # Pre-initialize Section 1 boundary walls
        self._init_boundary_walls()

    def _init_boundary_walls(self):
        """Mark outer boundary walls of Section 1."""
        # Bottom wall (y=0)
        for gx in range(self.width):
            self.set_occupied(gx, 0)
            self.set_occupied(gx, 1)

        # Left wall (x=0)
        for gy in range(self.height):
            self.set_occupied(0, gy)
            self.set_occupied(1, gy)

        # Right wall (x=2.44)
        for gy in range(self.height):
            self.set_occupied(self.width - 1, gy)
            self.set_occupied(self.width - 2, gy)

        # Top wall (y=1.22), leaving open the transition gap [0.10, 0.34]
        gap_gx_min = int(0.10 / self.res)
        gap_gx_max = int(0.34 / self.res)

        for gx in range(self.width):
            if not (gap_gx_min <= gx <= gap_gx_max):
                self.set_occupied(gx, self.height - 1)
                self.set_occupied(gx, self.height - 2)

    def world_to_grid(self, x, y):
        """Convert world coordinates (m) to grid coordinates (gx, gy)."""
        gx = int((x - config.S1_X_MIN) / self.res)
        gy = int((y - config.S1_Y_MIN) / self.res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Convert grid coordinates (gx, gy) to center of cell in world coordinates."""
        wx = config.S1_X_MIN + (gx + 0.5) * self.res
        wy = config.S1_Y_MIN + (gy + 0.5) * self.res
        return wx, wy

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_traversable(self, gx, gy):
        """Returns True if cell is free to enter."""
        if not self.in_bounds(gx, gy):
            return False
        return self.grid[gy][gx] == 0

    def set_occupied(self, gx, gy):
        """Mark cell as occupied and inflate neighbors."""
        if not self.in_bounds(gx, gy):
            return
        self.grid[gy][gx] = 1

        # Inflate surrounding cells
        r = self.inflation_cells
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    nx, ny = gx + dx, gy + dy
                    if self.in_bounds(nx, ny) and self.grid[ny][nx] == 0:
                        self.grid[ny][nx] = 2 # Inflated

    def update_from_sensors(self, rx, ry, ryaw, distances):
        """
        Raycast all distance sensors from robot pose into the grid.
        Clears free cells along the ray, and marks obstacles at hit points.
        """
        for name, d in distances.items():
            if name not in config.SENSOR_ANGLES:
                continue

            # Determine sensor origin relative to robot body
            if name == "ds_rear":
                ox_b, oy_b = config.SENSOR_ORIGIN_REAR
            else:
                ox_b, oy_b = config.SENSOR_ORIGIN_FRONT

            # Sensor position in world frame
            sx = rx + ox_b * math.cos(ryaw) - oy_b * math.sin(ryaw)
            sy = ry + ox_b * math.sin(ryaw) + oy_b * math.cos(ryaw)

            # Ray angle in world frame
            ray_angle = ryaw + config.SENSOR_ANGLES[name]

            # Raycast
            hit = (d < config.SENSOR_MAX_RANGE - 0.015)
            ray_dist = d if hit else config.SENSOR_MAX_RANGE

            # End point of ray
            ex = sx + ray_dist * math.cos(ray_angle)
            ey = sy + ray_dist * math.sin(ray_angle)

            # Trace ray using Bresenham
            self._trace_ray(sx, sy, ex, ey, hit)

    def _trace_ray(self, x0, y0, x1, y1, hit):
        gx0, gy0 = self.world_to_grid(x0, y0)
        gx1, gy1 = self.world_to_grid(x1, y1)

        # Bresenham's line algorithm
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy

        curr_x, curr_y = gx0, gy0

        while True:
            # Reached target cell
            if curr_x == gx1 and curr_y == gy1:
                if hit and self.in_bounds(curr_x, curr_y):
                    self.hit_counts[curr_y][curr_x] += 1
                    # Require 2 consecutive hits or close proximity to solidify obstacle
                    if self.hit_counts[curr_y][curr_x] >= 1:
                        self.set_occupied(curr_x, curr_y)
                break

            # Free cell along the ray
            if self.in_bounds(curr_x, curr_y):
                # Only clear if not firmly occupied
                if self.grid[curr_y][curr_x] == 2:
                    # Can be reclaimed if clear ray passes through periphery
                    pass

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                curr_x += sx
            if e2 < dx:
                err += dx
                curr_y += sy
