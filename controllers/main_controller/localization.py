"""
Localization and Pose Estimation Module
Fuses wheel encoder deltas with high-accuracy IMU yaw, tracking continuous (x, y, yaw),
total distance travelled, and discrete maze cell coordinates.
"""
import math
import config

class Localizer:
    def __init__(self, init_x=config.S1_START_POS[0], init_y=config.S1_START_POS[1], init_yaw=config.S1_START_YAW):
        self.x = init_x
        self.y = init_y
        self.yaw = init_yaw

        self.total_distance = 0.0
        self.last_progress_x = self.x
        self.last_progress_y = self.y
        self.last_progress_time = 0.0

    def set_pose(self, x, y, yaw=None):
        """Manually override or calibrate pose."""
        self.x = x
        self.y = y
        if yaw is not None:
            self.yaw = self._normalize_angle(yaw)
        self.last_progress_x = self.x
        self.last_progress_y = self.y

    def update(self, delta_left, delta_right, imu_yaw, current_time=0.0):
        """
        Update position using wheel distance deltas and absolute IMU yaw.
        """
        # Linear distance travelled in this step
        ds = (delta_left + delta_right) / 2.0
        self.total_distance += abs(ds)

        # Update heading directly from IMU (drift-free)
        self.yaw = self._normalize_angle(imu_yaw)

        # Position integration
        dx = ds * math.cos(self.yaw)
        dy = ds * math.sin(self.yaw)

        self.x += dx
        self.y += dy

        # Check progress for timeout watchdog
        dist_from_last = math.hypot(self.x - self.last_progress_x, self.y - self.last_progress_y)
        if dist_from_last >= config.STUCK_MIN_MOVEMENT:
            self.last_progress_x = self.x
            self.last_progress_y = self.y
            self.last_progress_time = current_time

    def get_pose(self):
        """Returns (x, y, yaw)."""
        return self.x, self.y, self.yaw

    def get_total_distance(self):
        """Returns total travel distance in meters."""
        return self.total_distance

    def time_since_progress(self, current_time):
        """Seconds since meaningful movement (> 2 cm)."""
        return current_time - self.last_progress_time

    def get_maze_cell(self):
        """
        Returns (col, row) in the Section 2 maze, clamped to [0, MAZE_COLS-1] and [0, MAZE_ROWS-1].
        """
        col = int(round((self.x - config.MAZE_ORIGIN_X) / config.MAZE_CELL_PITCH))
        row = int(round((self.y - config.MAZE_ORIGIN_Y) / config.MAZE_CELL_PITCH))
        col = max(0, min(config.MAZE_COLS - 1, col))
        row = max(0, min(config.MAZE_ROWS - 1, row))
        return col, row

    def get_cell_center(self, col, row):
        """Returns world (x, y) coordinates of the center of cell (col, row)."""
        cx = config.MAZE_ORIGIN_X + col * config.MAZE_CELL_PITCH
        cy = config.MAZE_ORIGIN_Y + row * config.MAZE_CELL_PITCH
        return cx, cy

    def get_cardinal_direction(self):
        """
        Returns closest cardinal direction: DIR_EAST (0), DIR_NORTH (1), DIR_WEST (2), DIR_SOUTH (3).
        """
        yaw = self.yaw
        if -math.pi / 4.0 <= yaw < math.pi / 4.0:
            return config.DIR_EAST
        elif math.pi / 4.0 <= yaw < 3.0 * math.pi / 4.0:
            return config.DIR_NORTH
        elif -3.0 * math.pi / 4.0 <= yaw < -math.pi / 4.0:
            return config.DIR_SOUTH
        else:
            return config.DIR_WEST

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
