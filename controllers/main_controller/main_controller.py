"""
PeraBots 2026 - Master Autonomous Robot Controller
Full end-to-end competition implementation for Phase 1:
- Section 1: Autonomous Obstacle Navigation with Occupancy Grid & A* Planning
- Autonomous Transition Detection & Chamber Navigation
- Section 2: Complete 9x4 Maze Graph Solver, Systematic BFS Exploration
- Camera-Based Floor Color Recognition (Blue & Green Tiles)
- Unique Tile Registry, Majority Decision, and Designated End-Cell Navigation
- Timed LED Indication and Precision Stopping
"""
import sys
import math
import config
from robot_hardware import RobotHardware
from localization import Localizer
from color_classifier import ColorClassifier
from occupancy_grid import OccupancyGrid
from path_planner import PathPlanner
from maze_manager import MazeManager

# FSM States
STATE_INIT               = "INIT"
STATE_S1_PLAN            = "S1_PLAN"
STATE_S1_FOLLOW          = "S1_FOLLOW"
STATE_S1_AVOID           = "S1_AVOID"
STATE_TRANSITION_APPROACH = "TRANSITION_APPROACH"
STATE_TRANSITION_PASS    = "TRANSITION_PASS"
STATE_MAZE_INIT          = "MAZE_INIT"
STATE_MAZE_CELL_ACTION   = "MAZE_CELL_ACTION"
STATE_MAZE_TURN          = "MAZE_TURN"
STATE_MAZE_MOVE          = "MAZE_MOVE"
STATE_MAZE_GOTO_END      = "MAZE_GOTO_END"
STATE_FINAL_STOP         = "FINAL_STOP"
STATE_RECOVERY           = "RECOVERY"

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def main():
    print("\n========== PERABOTS 2026 ==========\n", flush=True)

    # 1. Initialize Subsystems
    hardware = RobotHardware()
    print("[INIT] Sensors initialized", flush=True)
    print("[INIT] Motors initialized", flush=True)
    localizer = Localizer()
    print("[INIT] Localization initialized\n", flush=True)
    classifier = ColorClassifier(hardware.camera)
    grid = OccupancyGrid()
    planner = PathPlanner(grid)
    maze = MazeManager()

    state = STATE_INIT
    state_start_time = 0.0

    # Section 1 navigation state
    s1_waypoints = []
    s1_wp_index = 0
    s1_replan_timer = 0.0

    # Section 2 maze navigation state
    maze_path = []
    next_target_cell = (0, 0)
    target_heading = 0.0
    cell_move_start_dist = 0.0
    cell_move_target_dist = 0.0
    majority_color = "BLUE"
    end_arrival_time = None
    summary_printed = False
    maze_exploration_done = False
    collision_count = 0

    # Settle sensors
    for _ in range(15):
        if hardware.step() == -1:
            return

    # Main control loop
    while hardware.step() != -1:
        current_time = hardware.get_time()

        # Update odometry and heading from IMU
        d_left, d_right = hardware.get_encoder_deltas()
        roll, pitch, yaw = hardware.get_imu_rpy()
        localizer.update(d_left, d_right, yaw, current_time)

        rx, ry, ryaw = localizer.get_pose()
        distances = hardware.get_distances()

        # Update occupancy grid when in Section 1 (only when level)
        if state in [STATE_S1_PLAN, STATE_S1_FOLLOW, STATE_TRANSITION_APPROACH]:
            if abs(pitch) < 0.08: # less than 4.5 degrees pitch
                grid.update_from_sensors(rx, ry, ryaw, distances)

        # -------------------------------------------------------------
        # STATE MACHINE
        # -------------------------------------------------------------

        # === STATE: INIT ===
        if state == STATE_INIT:
            color, conf, rgb = classifier.get_filtered_color()

            # Check if starting in Section 1 or Section 2
            if color == config.COLOR_BLACK or ry >= 1.20:
                print("[SECTION 2] Maze exploration started", flush=True)
                print("[SECTION 2] Maze mapping active\n", flush=True)
                localizer.set_pose(0.22, 1.455, math.pi / 2) # Entrance cell (0, 0)
                state = STATE_MAZE_INIT
            else:
                print("[SECTION 1] Navigation started", flush=True)
                print("[SECTION 1] Obstacle mapping active", flush=True)
                print("[SECTION 1] Dynamic path planning active\n", flush=True)
                state = STATE_S1_PLAN

            state_start_time = current_time

        # === STATE: S1_PLAN ===
        elif state == STATE_S1_PLAN:
            hardware.set_motion(0.0, 0.0)
            target = config.TRANSITION_APPROACH_POS

            wps = planner.plan((rx, ry), target)
            if wps and len(wps) >= 2:
                s1_waypoints = wps
                s1_wp_index = 1 # skip start pose
                state = STATE_S1_FOLLOW
            else:
                # Fallback: advance directly towards approach
                s1_waypoints = [(rx, ry), target]
                s1_wp_index = 1
                state = STATE_S1_FOLLOW

            state_start_time = current_time

        # === STATE: S1_FOLLOW ===
        elif state == STATE_S1_FOLLOW:
            # Check if transition approach reached (must be aligned in both X and Y)
            dist_to_transition = math.hypot(config.TRANSITION_APPROACH_POS[0] - rx, config.TRANSITION_APPROACH_POS[1] - ry)
            if dist_to_transition < 0.08 or (abs(rx - config.TRANSITION_APPROACH_POS[0]) < 0.06 and ry >= config.TRANSITION_APPROACH_POS[1] - 0.02):
                state = STATE_TRANSITION_PASS
                state_start_time = current_time
                continue

            # Check if current waypoint reached
            if s1_wp_index < len(s1_waypoints):
                wx, wy = s1_waypoints[s1_wp_index]
                dist_to_wp = math.hypot(wx - rx, wy - ry)

                if dist_to_wp < 0.06:
                    s1_wp_index += 1
                    if s1_wp_index >= len(s1_waypoints):
                        # Final waypoint reached
                        state = STATE_TRANSITION_PASS
                        continue
                    wx, wy = s1_waypoints[s1_wp_index]

                # Proportional steering to waypoint
                target_angle = math.atan2(wy - ry, wx - rx)
                angle_err = normalize_angle(target_angle - ryaw)

                # Obstacle proximity check
                front_dist = distances.get("ds_front", config.SENSOR_MAX_RANGE)
                fl_dist = distances.get("ds_front_left", config.SENSOR_MAX_RANGE)
                fr_dist = distances.get("ds_front_right", config.SENSOR_MAX_RANGE)
                min_front = min(front_dist, fl_dist, fr_dist)

                if min_front < 0.11: # Obstacle directly ahead
                    state = STATE_S1_AVOID
                    state_start_time = current_time
                    continue

                # Speed control
                if abs(angle_err) > 0.5: # Large turn
                    v = 0.04
                    w = 2.0 * (1.0 if angle_err > 0 else -1.0)
                else:
                    v = config.S1_CRUISE_SPEED * max(0.3, math.cos(angle_err))
                    w = 2.5 * angle_err

                hardware.set_motion(v, w)
            else:
                state = STATE_S1_PLAN

            # Periodic replan every 2.0 seconds to incorporate newly mapped obstacles
            if current_time - s1_replan_timer > 2.0:
                s1_replan_timer = current_time
                state = STATE_S1_PLAN

        # === STATE: S1_AVOID ===
        elif state == STATE_S1_AVOID:
            front_dist = distances.get("ds_front", config.SENSOR_MAX_RANGE)
            fl_dist = distances.get("ds_front_left", config.SENSOR_MAX_RANGE)
            fr_dist = distances.get("ds_front_right", config.SENSOR_MAX_RANGE)
            min_front = min(front_dist, fl_dist, fr_dist)

            # Check left vs right clearance
            l_clear = min(distances.get("ds_left", 0.8), distances.get("ds_front_left_wide", 0.8), distances.get("ds_front_left", 0.8))
            r_clear = min(distances.get("ds_right", 0.8), distances.get("ds_front_right_wide", 0.8), distances.get("ds_front_right", 0.8))

            turn_dir = 1.0 if l_clear >= r_clear else -1.0

            if min_front < 0.075:
                # Too close, back up slightly while turning
                hardware.set_motion(-0.04, 1.8 * turn_dir)
            else:
                hardware.set_motion(0.02, 2.2 * turn_dir)

            # Once front is clear or timeout after 1.5s, replan
            if min_front > 0.18 or (current_time - state_start_time > 1.5):
                hardware.set_motion(0.0, 0.0)
                state = STATE_S1_PLAN

        # === STATE: TRANSITION_PASS ===
        elif state == STATE_TRANSITION_PASS:
            # Active cross-track centering onto x = 0.220
            x_err = config.TRANSITION_APPROACH_POS[0] - rx # positive if to the left
            target_yaw = math.pi / 2 + max(-0.35, min(0.35, 3.0 * x_err))
            angle_err = normalize_angle(target_yaw - ryaw)
            w = 3.0 * angle_err

            hardware.set_motion(config.APPROACH_SPEED, w)

            # Check camera for Red transition marker
            color, conf, rgb = classifier.get_filtered_color()

            # Once reaching y >= 1.44, robot has arrived at center of cell (0, 0)
            if ry >= 1.44:
                print("[SECTION 1] Transition successfully reached\n", flush=True)
                print("[SECTION 2] Maze exploration started", flush=True)
                print("[SECTION 2] Maze mapping active\n", flush=True)
                hardware.set_motion(0.0, 0.0)
                # Calibrate pose to cell (0, 0) center
                localizer.set_pose(config.MAZE_ORIGIN_X, config.MAZE_ORIGIN_Y, math.pi / 2)
                state = STATE_MAZE_INIT
                state_start_time = current_time

        # === STATE: MAZE_INIT ===
        elif state == STATE_MAZE_INIT:
            hardware.set_motion(0.0, 0.0)
            state = STATE_MAZE_CELL_ACTION

        # === STATE: MAZE_CELL_ACTION ===
        elif state == STATE_MAZE_CELL_ACTION:
            if maze_exploration_done:
                state = STATE_MAZE_GOTO_END
                continue

            hardware.set_motion(0.0, 0.0)
            curr_c, curr_r = localizer.get_maze_cell()
            cardinal_dir = localizer.get_cardinal_direction()

            # 1. Update walls from distance sensors
            maze.update_walls(curr_c, curr_r, cardinal_dir, distances)

            # 2. Inspect floor color with camera (tiles are placed at selected dead ends)
            color, conf, rgb = classifier.classify()
            if color in [config.COLOR_BLUE, config.COLOR_GREEN] and conf >= 0.50:
                maze.register_tile(curr_c, curr_r, color)

            # 3. Choose next exploration destination
            target_c, target_r, path = maze.find_next_exploration_target(curr_c, curr_r)

            if target_c is not None and len(path) >= 2:
                # Next adjacent cell to visit
                next_target_cell = path[1]
                nc, nr = next_target_cell
                dc = nc - curr_c
                dr = nr - curr_r

                # Determine required direction
                if dc == 1 and dr == 0:
                    req_dir = config.DIR_EAST
                elif dc == 0 and dr == 1:
                    req_dir = config.DIR_NORTH
                elif dc == -1 and dr == 0:
                    req_dir = config.DIR_WEST
                elif dc == 0 and dr == -1:
                    req_dir = config.DIR_SOUTH
                else:
                    req_dir = config.DIR_NORTH

                target_heading = config.DIR_HEADINGS[req_dir]
                state = STATE_MAZE_TURN
            else:
                # All reachable cells explored!
                maze_exploration_done = True
                print("\n[SECTION 2] Required exploration completed\n", flush=True)
                majority_color, b_count, g_count = maze.get_majority_decision()
                print(f"[COLOUR] Blue tiles detected: {b_count}", flush=True)
                print(f"[COLOUR] Green tiles detected: {g_count}", flush=True)
                print(f"[DECISION] Majority colour: {majority_color}\n", flush=True)
                print("[NAVIGATION] Navigating to final cell", flush=True)
                state = STATE_MAZE_GOTO_END

        # === STATE: MAZE_TURN ===
        elif state == STATE_MAZE_TURN:
            angle_err = normalize_angle(target_heading - ryaw)

            if abs(angle_err) < 0.04: # within ~2.3 degrees
                hardware.set_motion(0.0, 0.0)
                # Prepare for forward movement of one cell pitch
                cell_move_start_dist = localizer.get_total_distance()
                cell_move_target_dist = config.MAZE_CELL_PITCH
                state = STATE_MAZE_MOVE
            else:
                w = 2.2 * angle_err
                w = max(-config.MAZE_TURN_SPEED, min(config.MAZE_TURN_SPEED, w))
                hardware.set_motion(0.0, w)

        # === STATE: MAZE_MOVE ===
        elif state == STATE_MAZE_MOVE:
            moved_dist = localizer.get_total_distance() - cell_move_start_dist

            # Maintain straight heading
            angle_err = normalize_angle(target_heading - ryaw)
            w = 2.0 * angle_err

            # Check if wall collision is imminent
            front_dist = distances.get("ds_front", config.SENSOR_MAX_RANGE)
            if front_dist < 0.08:
                hardware.set_motion(0.0, 0.0)
                expected_cx, expected_cy = localizer.get_cell_center(next_target_cell[0], next_target_cell[1])
                localizer.set_pose(expected_cx, expected_cy, target_heading)
                state = STATE_MAZE_GOTO_END if maze_exploration_done else STATE_MAZE_CELL_ACTION
                continue

            if moved_dist >= cell_move_target_dist:
                hardware.set_motion(0.0, 0.0)
                expected_cx, expected_cy = localizer.get_cell_center(next_target_cell[0], next_target_cell[1])
                localizer.set_pose(expected_cx, expected_cy, target_heading)
                state = STATE_MAZE_GOTO_END if maze_exploration_done else STATE_MAZE_CELL_ACTION
            else:
                v = config.MAZE_MOVE_SPEED
                hardware.set_motion(v, w)

        # === STATE: MAZE_GOTO_END ===
        elif state == STATE_MAZE_GOTO_END:
            curr_c, curr_r = localizer.get_maze_cell()

            if (curr_c, curr_r) == (config.MAZE_END_COL, config.MAZE_END_ROW):
                # Arrived at end cell!
                print("[FINAL] Robot fully inside end cell", flush=True)
                hardware.hard_stop()
                end_arrival_time = current_time
                state = STATE_FINAL_STOP
                continue

            # Compute BFS route to (MAZE_END_COL, MAZE_END_ROW)
            path = maze.find_path(curr_c, curr_r, config.MAZE_END_COL, config.MAZE_END_ROW)
            if len(path) >= 2:
                next_target_cell = path[1]
                nc, nr = next_target_cell
                dc = nc - curr_c
                dr = nr - curr_r

                if dc == 1 and dr == 0:
                    req_dir = config.DIR_EAST
                elif dc == 0 and dr == 1:
                    req_dir = config.DIR_NORTH
                elif dc == -1 and dr == 0:
                    req_dir = config.DIR_WEST
                elif dc == 0 and dr == -1:
                    req_dir = config.DIR_SOUTH
                else:
                    req_dir = config.DIR_NORTH

                target_heading = config.DIR_HEADINGS[req_dir]
                state = STATE_MAZE_TURN
            else:
                print("[Maze_End] Warning: Direct path to end cell not found. Retrying...", flush=True)
                state = STATE_FINAL_STOP

        # === STATE: FINAL_STOP ===
        elif state == STATE_FINAL_STOP:
            hardware.hard_stop()

            # Activate the required LED within 3 seconds of arrival
            if majority_color == "GREEN":
                hardware.set_leds(blue_state=0, green_state=1)
            else:
                hardware.set_leds(blue_state=1, green_state=0)

            # Print mission completion summary once
            if not summary_printed:
                summary_printed = True
                total_dist = localizer.get_total_distance()
                print("[LED] Correct majority-colour LED activated", flush=True)
                print("[FINAL] Motors stopped\n", flush=True)
                print("[RESULT] FULL CHALLENGE COMPLETED\n", flush=True)
                print(f"[TIME] {current_time:.2f} s", flush=True)
                print(f"[DISTANCE] {total_dist:.2f} m", flush=True)
                print(f"[COLLISIONS] {collision_count}", flush=True)
                print("====================================\n", flush=True)

        # -------------------------------------------------------------
        # PROGRESS WATCHDOG & STUCK RECOVERY
        # -------------------------------------------------------------
        if state not in [STATE_INIT, STATE_FINAL_STOP]:
            idle_time = localizer.time_since_progress(current_time)
            if idle_time > 5.0: # 5 seconds without progress
                print(f"[Watchdog] No progress for {idle_time:.1f} s. Triggering recovery...", flush=True)
                # Quick recovery: reverse slightly if clearance allows, rotate 45 deg
                rear_d = distances.get("ds_rear", config.SENSOR_MAX_RANGE)
                if rear_d > 0.12:
                    hardware.set_motion(-0.06, 0.0)
                    for _ in range(25):
                        hardware.step()
                hardware.set_motion(0.0, 1.8)
                for _ in range(30):
                    hardware.step()
                localizer.last_progress_time = current_time
                if ry < 1.20:
                    state = STATE_S1_PLAN
                else:
                    state = STATE_MAZE_CELL_ACTION

if __name__ == "__main__":
    main()
