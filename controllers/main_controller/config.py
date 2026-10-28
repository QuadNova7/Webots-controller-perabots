"""
PeraBots 2026 - Configuration and Constants
Single source of truth for robot parameters, arena geometry, speeds, and tuning.
"""
import math

# --- Simulation Parameters ---
TIME_STEP = 16  # milliseconds, matches world basicTimeStep
DT = TIME_STEP / 1000.0  # seconds (0.016 s)

# --- Robot Physical Dimensions ---
WHEEL_RADIUS = 0.020  # meters
WHEEL_BASE = 0.064    # meters (distance between wheels)
WHEEL_X_OFFSET = 0.012 # meters (forward offset of wheel axle from chassis center)
CHASSIS_LENGTH = 0.080 # meters
CHASSIS_WIDTH = 0.050  # meters
ROBOT_BOUNDING_RADIUS = 0.050 # meters
OBSTACLE_INFLATION_RADIUS = 0.055 # meters (allows 25 cm passages to remain easily traversable)

# --- Motion Limits ---
MAX_WHEEL_VELOCITY = 10.0 # rad/s (max rotational speed, 0.20 m/s)
MAX_LINEAR_ACCEL = 1.5    # m/s^2 (smooth acceleration to prevent slip/tilt)
MAX_ANGULAR_ACCEL = 8.0   # rad/s^2

S1_CRUISE_SPEED = 0.18    # m/s (tuned)
S1_TURN_SPEED = 2.2       # rad/s (tuned)
MAZE_CRUISE_SPEED = 0.175 # m/s (tuned)
MAZE_MOVE_SPEED = MAZE_CRUISE_SPEED # alias
MAZE_TURN_SPEED = 2.2     # rad/s (tuned)
APPROACH_SPEED = 0.08     # m/s (for docking / transitions)

# --- Sensor Layout ---
SENSOR_ORIGIN_FRONT = (0.035, 0.0) # (x, y) relative to robot body
SENSOR_ORIGIN_REAR = (-0.035, 0.0)

# Sensor names and ray angles relative to body +x (radians)
SENSOR_ANGLES = {
    "ds_front": 0.0,
    "ds_front_left": 0.44,
    "ds_front_left_wide": 0.87,
    "ds_left": math.pi / 2,        # 1.5708
    "ds_front_right": -0.44,
    "ds_front_right_wide": -0.87,
    "ds_right": -math.pi / 2,      # -1.5708
    "ds_rear": math.pi            # 3.1416
}

SENSOR_MAX_RANGE = 0.80 # meters

# --- Section 1 Arena Geometry ---
S1_X_MIN = 0.0
S1_X_MAX = 2.44
S1_Y_MIN = 0.0
S1_Y_MAX = 1.22

# Start zone: bottom-right corner [2.19, 2.44] x [0.0, 0.25]
S1_START_POS = (2.2687, 0.1598) # Official default start pose
S1_START_YAW = 2.3236           # ~133.1 degrees (facing up-left)

# Section 1 Occupancy Grid Settings
GRID_RESOLUTION = 0.025 # 2.5 cm cells (98 x 49 grid cells for 2.44 x 1.22 m)
GRID_WIDTH = int(math.ceil((S1_X_MAX - S1_X_MIN) / GRID_RESOLUTION))
GRID_HEIGHT = int(math.ceil((S1_Y_MAX - S1_Y_MIN) / GRID_RESOLUTION))

# Transition Chamber Geometry
TRANSITION_APPROACH_POS = (0.22, 1.12)
TRANSITION_CHAMBER_POS = (0.22, 1.275) # Center of transition tile
TRANSITION_CHAMBER_ENTRY_Y = 1.22
TRANSITION_CHAMBER_EXIT_Y = 1.33

# --- Section 2 Maze Geometry ---
MAZE_COLS = 9
MAZE_ROWS = 4
MAZE_CELL_PITCH = 0.25 # meters
MAZE_WALL_THICKNESS = 0.012 # meters
MAZE_CELL_CLEAR = 0.238 # meters

# Origin cell (col 0, row 0) center in world coordinates:
MAZE_ORIGIN_X = 0.22
MAZE_ORIGIN_Y = 1.455

# Designated end cell is col 8, row 3 (top-right corner of maze)
MAZE_END_COL = 8
MAZE_END_ROW = 3
MAZE_END_X = MAZE_ORIGIN_X + MAZE_END_COL * MAZE_CELL_PITCH # 2.22 m
MAZE_END_Y = MAZE_ORIGIN_Y + MAZE_END_ROW * MAZE_CELL_PITCH # 2.205 m

# Maze Cardinal Directions (headings in radians)
# NORTH = +Y (+pi/2, 1.5708)
# EAST  = +X (0.0)
# SOUTH = -Y (-pi/2, -1.5708)
# WEST  = -X (pi, 3.1416)
DIR_EAST  = 0
DIR_NORTH = 1
DIR_WEST  = 2
DIR_SOUTH = 3

DIR_VECTORS = {
    DIR_EAST:  (1, 0),
    DIR_NORTH: (0, 1),
    DIR_WEST:  (-1, 0),
    DIR_SOUTH: (0, -1)
}

DIR_HEADINGS = {
    DIR_EAST:  0.0,
    DIR_NORTH: math.pi / 2,
    DIR_WEST:  math.pi,
    DIR_SOUTH: -math.pi / 2
}

DIR_NAMES = {
    DIR_EAST:  "EAST",
    DIR_NORTH: "NORTH",
    DIR_WEST:  "WEST",
    DIR_SOUTH: "SOUTH"
}

# --- Floor Tile Colors ---
COLOR_UNKNOWN = 0
COLOR_WHITE   = 1 # Section 1 floor
COLOR_BLACK   = 2 # Section 2 floor
COLOR_RED     = 3 # Start / Chamber / End markers
COLOR_BLUE    = 4 # Section 2 target blue tile
COLOR_GREEN   = 5 # Section 2 target green tile

# --- Timeouts & Safety ---
STUCK_CHECK_PERIOD = 2.0 # seconds
STUCK_MIN_MOVEMENT = 0.02 # meters
OFFICIAL_TIMEOUT_THRESHOLD = 14.0 # seconds (official termination is 15s)
RECOVERY_MAX_ATTEMPTS = 5
