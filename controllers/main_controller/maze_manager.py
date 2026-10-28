"""
Maze Manager Module for Section 2
Maintains the 9x4 maze graph, detects walls in all four cardinal directions,
manages systematic BFS-driven exploration, tracks unique colored tiles,
computes the majority color, and routes to the designated end cell (8, 3).
"""
import config

class MazeManager:
    def __init__(self):
        self.cols = config.MAZE_COLS # 9
        self.rows = config.MAZE_ROWS # 4

        # Graph walls: walls[c][r][dir] = True if wall exists, False if open, None if unknown
        # Directions: 0: East, 1: North, 2: West, 3: South
        self.walls = [[[None for _ in range(4)] for _ in range(self.rows)] for _ in range(self.cols)]

        # Pre-set outer boundary walls
        self._init_outer_walls()

        # Visited status
        self.visited = [[False for _ in range(self.rows)] for _ in range(self.cols)]

        # Unique tile locations: sets of (col, row)
        self.blue_tiles = set()
        self.green_tiles = set()

        # Exploration complete flag
        self.exploration_done = False

    def _init_outer_walls(self):
        """Set outer boundary walls of the 9x4 maze."""
        # West walls (col 0)
        for r in range(self.rows):
            self.walls[0][r][config.DIR_WEST] = True

        # East walls (col 8)
        for r in range(self.rows):
            self.walls[self.cols - 1][r][config.DIR_EAST] = True

        # North walls (row 3)
        for c in range(self.cols):
            self.walls[c][self.rows - 1][config.DIR_NORTH] = True

        # South walls (row 0): all True so robot explores strictly inside maze
        for c in range(self.cols):
            self.walls[c][0][config.DIR_SOUTH] = True

    def is_in_bounds(self, c, r):
        return 0 <= c < self.cols and 0 <= r < self.rows

    def update_walls(self, c, r, heading_dir, distances):
        """
        Updates walls around cell (c, r) from distance sensor readings.
        heading_dir: current cardinal heading (0: E, 1: N, 2: W, 3: S).
        distances: dict of sensor readings.
        """
        if not self.is_in_bounds(c, r):
            return

        self.visited[c][r] = True

        # Wall threshold from cell center:
        # Distance to wall is ~12.5 cm. A reading < 19 cm indicates a wall.
        WALL_THRESHOLD = 0.19

        # Map sensors to relative directions:
        # ds_front -> ahead (offset 0)
        # ds_left  -> left (+90 deg, offset +1)
        # ds_rear  -> behind (180 deg, offset +2)
        # ds_right -> right (-90 deg, offset +3)
        sensor_offsets = [
            ("ds_front", 0),
            ("ds_left",  1),
            ("ds_rear",  2),
            ("ds_right", 3)
        ]

        for s_name, offset in sensor_offsets:
            if s_name in distances:
                d = distances[s_name]
                has_wall = (d < WALL_THRESHOLD)
                cardinal_dir = (heading_dir + offset) % 4
                self.set_wall(c, r, cardinal_dir, has_wall)

    def set_wall(self, c, r, direction, has_wall):
        """Sets wall state symmetrically for adjacent cells."""
        if not self.is_in_bounds(c, r):
            return

        self.walls[c][r][direction] = has_wall

        # Neighbor cell
        dx, dy = config.DIR_VECTORS[direction]
        nc, nr = c + dx, r + dy
        opposite_dir = (direction + 2) % 4

        if self.is_in_bounds(nc, nr):
            self.walls[nc][nr][opposite_dir] = has_wall

    def has_wall(self, c, r, direction):
        """Returns True if wall exists, False if open or None."""
        if not self.is_in_bounds(c, r):
            return True
        return self.walls[c][r][direction] is True

    def register_tile(self, c, r, color):
        """Records unique tile color for cell (c, r)."""
        if not self.is_in_bounds(c, r):
            return

        if color == config.COLOR_BLUE:
            if (c, r) not in self.blue_tiles:
                self.blue_tiles.add((c, r))
                print(f"[COLOUR] Unique Blue tile discovered at cell ({c}, {r}) [Total Blue: {len(self.blue_tiles)}]", flush=True)
        elif color == config.COLOR_GREEN:
            if (c, r) not in self.green_tiles:
                self.green_tiles.add((c, r))
                print(f"[COLOUR] Unique Green tile discovered at cell ({c}, {r}) [Total Green: {len(self.green_tiles)}]", flush=True)

    def get_majority_decision(self):
        """
        Determines majority color.
        Returns ('BLUE' or 'GREEN', blue_count, green_count).
        """
        num_blue = len(self.blue_tiles)
        num_green = len(self.green_tiles)

        if num_green > num_blue:
            return "GREEN", num_blue, num_green
        elif num_blue > num_green:
            return "BLUE", num_blue, num_green
        else:
            return "BLUE", num_blue, num_green

    def is_dead_end(self, c, r):
        """Returns True if cell has 3 walls (dead end)."""
        if not self.is_in_bounds(c, r):
            return False
        wall_count = sum(1 for d in range(4) if self.walls[c][r][d] is True)
        return wall_count >= 3

    def find_path(self, start_c, start_r, goal_c, goal_r):
        """
        BFS shortest path on known maze graph between two cells.
        Returns list of (c, r) cells, or [] if no path.
        """
        if (start_c, start_r) == (goal_c, goal_r):
            return [(start_c, start_r)]

        queue = [(start_c, start_r)]
        came_from = {(start_c, start_r): None}

        found = False
        while queue:
            curr = queue.pop(0)
            cc, cr = curr

            if curr == (goal_c, goal_r):
                found = True
                break

            for direction in range(4):
                # Cannot cross wall
                if self.walls[cc][cr][direction] is True:
                    continue

                dx, dy = config.DIR_VECTORS[direction]
                nc, nr = cc + dx, cr + dy

                if self.is_in_bounds(nc, nr) and (nc, nr) not in came_from:
                    came_from[(nc, nr)] = curr
                    queue.append((nc, nr))

        if not found:
            return []

        # Reconstruct path
        path = []
        curr = (goal_c, goal_r)
        while curr is not None:
            path.append(curr)
            curr = came_from[curr]
        path.reverse()
        return path

    def find_next_exploration_target(self, curr_c, curr_r):
        """
        Finds BFS shortest route from (curr_c, curr_r) to the closest unvisited cell.
        Returns (target_c, target_r, path) or (None, None, []) if all reachable cells visited.
        """
        queue = [(curr_c, curr_r)]
        came_from = {(curr_c, curr_r): None}

        target = None
        while queue:
            curr = queue.pop(0)
            cc, cr = curr

            if not self.visited[cc][cr]:
                target = curr
                break

            for direction in range(4):
                # Don't cross known wall
                if self.walls[cc][cr][direction] is True:
                    continue

                dx, dy = config.DIR_VECTORS[direction]
                nc, nr = cc + dx, cr + dy

                if self.is_in_bounds(nc, nr) and (nc, nr) not in came_from:
                    came_from[(nc, nr)] = curr
                    queue.append((nc, nr))

        if target is None:
            self.exploration_done = True
            return None, None, []

        # Reconstruct path
        path = []
        curr = target
        while curr is not None:
            path.append(curr)
            curr = came_from[curr]
        path.reverse()
        return target[0], target[1], path
