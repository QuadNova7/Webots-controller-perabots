"""
Robot Hardware Interface
Wraps Webots Robot API, device initialization, sensor filtering, and smooth motor control.
"""
import math
from controller import Robot
import config

class RobotHardware:
    def __init__(self):
        self.robot = Robot()
        self.time_step = config.TIME_STEP

        # --- Distance Sensors ---
        self.distance_sensors = {}
        for name in config.SENSOR_ANGLES.keys():
            dev = self.robot.getDevice(name)
            if dev:
                dev.enable(self.time_step)
                self.distance_sensors[name] = dev
            else:
                print(f"[Hardware] Warning: Device {name} not found!")

        # --- Camera ---
        self.camera = self.robot.getDevice("camera_down")
        if self.camera:
            self.camera.enable(self.time_step)

        # --- IMU ---
        self.imu = self.robot.getDevice("imu")
        if self.imu:
            self.imu.enable(self.time_step)

        # --- Motors & Encoders ---
        self.left_motor = self.robot.getDevice("left_wheel_motor")
        self.right_motor = self.robot.getDevice("right_wheel_motor")
        self.left_sensor = self.robot.getDevice("left_wheel_sensor")
        self.right_sensor = self.robot.getDevice("right_wheel_sensor")

        self.left_sensor.enable(self.time_step)
        self.right_sensor.enable(self.time_step)

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # --- LEDs ---
        self.led_blue = self.robot.getDevice("led_blue")
        self.led_green = self.robot.getDevice("led_green")
        if self.led_blue:
            self.led_blue.set(0)
        if self.led_green:
            self.led_green.set(0)

        # --- Internal State ---
        self.prev_left_enc = None
        self.prev_right_enc = None
        self.current_v = 0.0
        self.current_w = 0.0

        # Step counter
        self.step_count = 0

    def step(self):
        """Advance simulation by one time step."""
        res = self.robot.step(self.time_step)
        if res != -1:
            self.step_count += 1
        return res

    def get_time(self):
        """Simulation time in seconds."""
        return self.robot.getTime()

    def get_distances(self):
        """
        Returns dict of sensor readings in meters.
        Readings >= SENSOR_MAX_RANGE - 0.005 are treated as clear (no obstacle).
        """
        readings = {}
        for name, dev in self.distance_sensors.items():
            val = dev.getValue()
            # clamp to max range
            if val >= config.SENSOR_MAX_RANGE - 0.005 or math.isnan(val) or val < 0.0:
                readings[name] = config.SENSOR_MAX_RANGE
            else:
                readings[name] = val
        return readings

    def get_imu_rpy(self):
        """Returns (roll, pitch, yaw) in radians. Yaw is normalized to [-pi, pi]."""
        if not self.imu:
            return 0.0, 0.0, 0.0
        rpy = self.imu.getRollPitchYaw()
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]
        # Normalize yaw
        while yaw > math.pi:
            yaw -= 2.0 * math.pi
        while yaw < -math.pi:
            yaw += 2.0 * math.pi
        return roll, pitch, yaw

    def get_encoder_deltas(self):
        """
        Returns (delta_dist_left, delta_dist_right) in meters since last call.
        """
        left_val = self.left_sensor.getValue()
        right_val = self.right_sensor.getValue()

        if math.isnan(left_val) or math.isnan(right_val):
            return 0.0, 0.0

        if self.prev_left_enc is None:
            self.prev_left_enc = left_val
            self.prev_right_enc = right_val
            return 0.0, 0.0

        d_left_rad = left_val - self.prev_left_enc
        d_right_rad = right_val - self.prev_right_enc

        self.prev_left_enc = left_val
        self.prev_right_enc = right_val

        d_left_m = d_left_rad * config.WHEEL_RADIUS
        d_right_m = d_right_rad * config.WHEEL_RADIUS

        return d_left_m, d_right_m

    def set_motion(self, v_target, w_target):
        """
        Set desired linear speed (m/s) and angular speed (rad/s) with acceleration limiting.
        """
        dt = config.DT

        # Linear acceleration limiting
        max_dv = config.MAX_LINEAR_ACCEL * dt
        dv = v_target - self.current_v
        if dv > max_dv:
            self.current_v += max_dv
        elif dv < -max_dv:
            self.current_v -= max_dv
        else:
            self.current_v = v_target

        # Angular acceleration limiting
        max_dw = config.MAX_ANGULAR_ACCEL * dt
        dw = w_target - self.current_w
        if dw > max_dw:
            self.current_w += max_dw
        elif dw < -max_dw:
            self.current_w -= max_dw
        else:
            self.current_w = w_target

        # Convert (v, w) to wheel linear speeds
        v_l = self.current_v - self.current_w * (config.WHEEL_BASE / 2.0)
        v_r = self.current_v + self.current_w * (config.WHEEL_BASE / 2.0)

        # Convert to rotational speeds (rad/s)
        omega_l = v_l / config.WHEEL_RADIUS
        omega_r = v_r / config.WHEEL_RADIUS

        # Clamp to max motor velocity
        max_v = config.MAX_WHEEL_VELOCITY
        if abs(omega_l) > max_v or abs(omega_r) > max_v:
            scale = max_v / max(abs(omega_l), abs(omega_r))
            omega_l *= scale
            omega_r *= scale

        self.left_motor.setVelocity(omega_l)
        self.right_motor.setVelocity(omega_r)

    def stop(self):
        """Smoothly bring motors to zero speed."""
        self.set_motion(0.0, 0.0)

    def hard_stop(self):
        """Immediately command zero velocity."""
        self.current_v = 0.0
        self.current_w = 0.0
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def set_leds(self, blue_state, green_state):
        """Set blue and green LEDs (0 for off, 1 for on)."""
        if self.led_blue:
            self.led_blue.set(1 if blue_state else 0)
        if self.led_green:
            self.led_green.set(1 if green_state else 0)

    def get_camera_image(self):
        """Returns camera image buffer or None."""
        if self.camera:
            return self.camera.getImage()
        return None
