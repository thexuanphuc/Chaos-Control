import numpy as np
import time

class Controller:
    def __init__(self):
        """
        Base controller class.
        """
        pass

    def compute_control(self, robot_state, predefined_path):
        """
        Compute control commands for the robot. Must be implemented by subclasses.
        :param robot_state: Current state of the robot (x, y, theta).
        :param predefined_path: The target path as an array of (x, y) points.
        :return: Control commands (format depends on subclass) and potentially error/debug info.
        """
        raise NotImplementedError("This method should be implemented by subclasses.")

class LyapunovEnergyBasedController(Controller):
    def __init__(self, dt, k_forward=1.0, k_theta=2.0, k_lateral_gain_factor=1.0, predefined_path=None,
                 wheel_width=0.5, wheel_radius=0.1, v_ref=1.0, omega_max=np.pi):
        """
        Lyapunov-based controller for path tracking for a differential drive robot.

        :param dt: Simulation time step.
        :param k_forward: Gain for forward error correction.
        :param k_theta: Gain for orientation error correction.
        :param k_lateral_gain_factor: Factor scaling the lateral error correction term (often related to k_theta).
        :param predefined_path: The target path numpy array (N, 2).
        :param wheel_width: Distance between wheels.
        :param wheel_radius: Radius of wheels.
        :param v_ref: Reference forward velocity along the path.
        :param omega_max: Maximum allowable angular velocity for the robot.
        """
        super().__init__()
        self.dt = dt
        self.kf = k_forward
        self.ktt = k_theta
        self.k_lat_factor = k_lateral_gain_factor # Gain factor for lateral correction term
        self.predefined_path = predefined_path
        self.wheel_width = wheel_width
        self.wheel_radius = wheel_radius
        self.v_ref = v_ref  # Use a constant reference speed
        self.omega_max = omega_max
        self.closest_index = 0
        self.previous_cmd = (0.0, 0.0) # (omega_left, omega_right)
        self.last_computation_time = time.time() - dt # Ensure first computation runs

    def _safe_sinc(self, x):
        """ Calculates sin(x)/x, handling x=0. """
        if np.abs(x) < 1e-6: # Small threshold near zero
            # Use Taylor expansion: sin(x)/x = (x - x^3/6 + ...)/x = 1 - x^2/6 + ...
            return 1.0 - x**2 / 6.0
        else:
            return np.sin(x) / x

    def compute_control(self, robot_state, predefined_path: np.ndarray):
        """
        Compute wheel angular velocities using a Lyapunov-based control law.

        :param robot_state: Current state of the robot (x, y, theta).
        :param predefined_path: The target path as an array of (x, y) points.
        :return: Tuple (omega_left, omega_right, errors, V), where errors is (err_fwd, err_lat, err_theta)
                 and V is the Lyapunov energy. Returns previous command if dt hasn't passed.
        """
        current_time = time.time()
        # Optional: Check if enough time has passed since last computation
        # if (current_time - self.last_computation_time) < self.dt:
        #      # This time check might be too aggressive if simulation runs faster than real-time
        #      # For pure simulation, we usually compute every step.
        #      # For real robot interface, this can be useful.
        #      # Let's compute every step for simulation:
        #      # return self.previous_cmd[0], self.previous_cmd[1], self.previous_errors, self.previous_V
        #      pass # Compute every step

        x, y, theta = robot_state

        # --- Path Following Logic ---
        # 1. Find the closest point on the path (using a limited search window for efficiency)
        search_radius = 10 # Search +/- points around the last closest index
        start_index = max(0, self.closest_index - search_radius)
        end_index = min(len(predefined_path), self.closest_index + search_radius + 1) # +1 for slicing
        path_segment = predefined_path[start_index:end_index]
        current_pos = np.array([x, y])
        distances = np.linalg.norm(path_segment - current_pos, axis=1)

        relative_closest_index = np.argmin(distances)
        self.closest_index = start_index + relative_closest_index

        # 2. Determine the target point (lookahead point)
        lookahead_index = min(self.closest_index + 1, len(predefined_path) - 1)
        target_point = predefined_path[lookahead_index]
        x_d, y_d = target_point

        # Check for path completion
        if self.closest_index >= len(predefined_path) - 2: # If close to the end
             target_point = predefined_path[-1] # Target the last point
             x_d, y_d = target_point
             dist_to_end = np.linalg.norm(current_pos - target_point)
             if dist_to_end < 0.1: # Close enough to the end goal
                 print("Reached end of path.")
                 self.previous_cmd = (0.0, 0.0)
                 self.previous_errors = (0.0, 0.0, 0.0)
                 self.previous_V = 0.0
                 return 0.0, 0.0, (0.0, 0.0, 0.0), 0.0 # Stop

        # 3. Calculate desired orientation (theta_d) and reference angular velocity (omega_ref)
        # Use two points ahead to estimate path tangent/curvature more smoothly
        next_index = min(self.closest_index + 1, len(predefined_path) - 1)
        next_next_index = min(self.closest_index + 2, len(predefined_path) - 1)

        if next_index == next_next_index: # At the very end, point towards the last point
            dx = predefined_path[next_index][0] - x
            dy = predefined_path[next_index][1] - y
            theta_path_tangent = np.arctan2(dy, dx)
            omega_ref = 0.0 # No curvature at the end
        else:
            # Vector from current target point to next point
            vec1 = predefined_path[next_index] - predefined_path[self.closest_index]
            # Vector from next point to the point after that
            vec2 = predefined_path[next_next_index] - predefined_path[next_index]
            # Desired orientation is angle of the segment the robot is approaching
            theta_path_tangent = np.arctan2(vec1[1], vec1[0])

            # Estimate curvature/required turning rate (omega_ref)
            # Change in angle between consecutive segments / estimated distance covered
            angle1 = np.arctan2(vec1[1], vec1[0])
            angle2 = np.arctan2(vec2[1], vec2[0])
            delta_theta_path = (angle2 - angle1 + np.pi) % (2 * np.pi) - np.pi # Normalize angle diff
            dist1 = np.linalg.norm(vec1)
            dist2 = np.linalg.norm(vec2)
            avg_dist = (dist1 + dist2) / 2.0 + 1e-6 # Average segment length
            # Reference angular velocity = v_ref * curvature = v_ref * (delta_theta / delta_s)
            omega_ref = self.v_ref * (delta_theta_path / avg_dist)

        # Angle error to the *target point* (used for error coordinates)
        error_x = x_d - x
        error_y = y_d - y
        angle_to_target = np.arctan2(error_y, error_x)

        # --- Error Calculation in Robot Frame ---
        rho = np.sqrt(error_x**2 + error_y**2) # Distance error (magnitude)
        # Angle between robot heading and vector to target point
        alpha = (angle_to_target - theta + np.pi) % (2 * np.pi) - np.pi
        # Angle between path tangent at target and robot heading
        beta = (theta_path_tangent - theta + np.pi) % (2 * np.pi) - np.pi

        # Errors used in Lyapunov-like control laws:
        # error_forward: Often related to distance 'rho', projected. Here use direct x-error in robot frame.
        # error_lateral: Often related to angle 'alpha' or y-error in robot frame.
        # error_theta: Often related to angle 'beta' or combined angles.

        # Transform world errors (error_x, error_y) to robot frame errors
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        error_forward = error_x * cos_t + error_y * sin_t
        error_lateral = -error_x * sin_t + error_y * cos_t
        # Use beta as the primary orientation error (error relative to path tangent)
        error_theta = beta

        # Ensure error_theta is in [-pi, pi] (already handled by beta calculation)

        # --- Lyapunov Function (Example) ---
        # V = 0.5 * error_forward^2 + 0.5 * error_lateral^2 + 0.5 * (1/gamma) * error_theta^2
        # We don't explicitly need gamma if we use gains kf, k_lat, ktt directly.
        # Let's just calculate a representative energy value based on squared errors.
        # Use ktt to scale the theta error contribution relative to positional errors.
        V = 0.5 * (error_forward**2 + error_lateral**2 + abs(self.ktt) * error_theta**2) # Example energy

        # --- Control Law Computation ---
        # Based on common Lyapunov derivations for posture regulation / path following:
        # v = v_ref * cos(error_theta) + kf * error_forward  (Adjust speed based on fwd error and alignment)
        # w = omega_ref + k_lat * v_ref * sinc(error_theta) * error_lateral + ktt * error_theta (Correct lateral and orientation errors)

        forward_velocity = self.v_ref * np.cos(error_theta) + self.kf * error_forward

        # Use safe sinc for the lateral error term. Note np.sinc(x) = sin(pi*x)/(pi*x)
        # We need sin(error_theta)/error_theta, so use sin(error_theta) / error_theta directly with safe division.
        sinc_term = self._safe_sinc(error_theta)
        # Add k_lat_factor to tune lateral correction independently if needed
        k_lat = self.ktt * self.k_lat_factor # Link lateral gain to theta gain
        angular_velocity = omega_ref + k_lat * self.v_ref * sinc_term * error_lateral + self.ktt * error_theta

        # Limit the velocities (robot physical limits)
        # Note: We don't explicitly limit v here, but rely on omega limits and diff-drive conversion
        angular_velocity = np.clip(angular_velocity, -self.omega_max, self.omega_max)

        # --- Convert Robot Velocities to Wheel Velocities ---
        # v = (v_r + v_l) / 2 => v_r + v_l = 2v
        # w = (v_r - v_l) / W => v_r - v_l = w * W
        # Adding eq: 2*v_r = 2v + w*W => v_r = v + w*W/2
        # Subtracting eq: 2*v_l = 2v - w*W => v_l = v - w*W/2
        v_right_wheel = forward_velocity + (angular_velocity * self.wheel_width / 2)
        v_left_wheel = forward_velocity - (angular_velocity * self.wheel_width / 2)

        # Convert linear wheel velocities to angular wheel velocities
        omega_right = v_right_wheel / self.wheel_radius
        omega_left = v_left_wheel / self.wheel_radius

        # --- Store state for next step/return ---
        self.previous_cmd = (omega_left, omega_right)
        self.last_computation_time = current_time
        errors = (error_forward, error_lateral, error_theta)
        self.previous_errors = errors # Store for potential use if dt check is enabled
        self.previous_V = V

        return omega_left, omega_right, errors, V