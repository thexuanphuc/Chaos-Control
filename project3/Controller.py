import numpy as np
import time # Not strictly used in this file but often kept for consistency

class Controller:
    """
    Base class for controllers.
    Provides a common interface for different control strategies.
    All controllers should inherit from this class and implement compute_control.
    """
    def __init__(self):
        """ Initializes the base controller. """
        pass

    def compute_control(self, robot_state, predefined_path):
        """
        Computes the control commands based on the robot's state and a predefined path.
        This method MUST be implemented by concrete subclasses.

        :param robot_state: A tuple, list, or array representing the current state of the robot.
                            Typically (x, y, theta, v1, omega).
        :param predefined_path: A numpy array of shape (N, 2) representing the target path (x,y coordinates).
        :return: Control commands. The nature of these commands (e.g., desired velocities, torques)
                 depends on the specific controller implementation.
        :raises NotImplementedError: If the method is not overridden in a subclass.
        """
        raise NotImplementedError("The compute_control method must be implemented by subclasses.")

    def _safe_sinc(self, x):
        """
        Calculates sin(x)/x, handling the singularity at x=0 using Taylor expansion.
        This function is often used in kinematic control laws for nonholonomic robots
        to ensure numerical stability when an error term (like orientation error) is near zero.

        For small x, sin(x) approx x - x^3/3! + x^5/5! - ...
        So, sin(x)/x approx 1 - x^2/6 + x^4/120 - ...

        :param x: Input value (typically an angle in radians).
        :return: The value of sin(x)/x.
        """
        if np.abs(x) < 1e-6:  # Threshold for using Taylor expansion to avoid division by zero or precision loss
            return 1.0 - x**2 / 6.0 # Second-order Taylor expansion
        else:
            return np.sin(x) / x

# --- LyapunovKinematicController ---
class LyapunovKinematicController(Controller):
    """
    Lyapunov-based kinematic controller (Outer Loop in Backstepping).
    This controller computes desired linear velocity (v1_d) and angular velocity (omega_d)
    to steer a robot along a predefined path. It assumes these velocities can be perfectly tracked
    by a lower-level dynamic controller. Its design is often inspired by Lyapunov stability analysis
    to ensure convergence of path following errors.
    """
    def __init__(self, k_forward=1.0, k_theta=2.0, k_lateral_gain_factor=1.0,
                 v_ref=1.0, omega_max=np.pi, lookahead_dist=0.5):
        """
        Initializes the LyapunovKinematicController.

        :param k_forward: Gain for correcting error in the forward direction of the robot (kf).
                          Influences how quickly the robot reduces distance error along its heading.
        :param k_theta: Gain for correcting orientation error (k_theta or ktt).
                        Also scales the lateral error correction component.
        :param k_lateral_gain_factor: Additional factor to scale the lateral error correction term.
                                     Allows independent tuning of the robot's aggressiveness in
                                     steering towards the path versus correcting its heading.
        :param v_ref: Reference forward velocity when on the path and aligned (v_r).
        :param omega_max: Maximum allowable commanded angular velocity (saturation limit).
        :param lookahead_dist: Lookahead distance to select the target point on the path.
                               A larger lookahead can smooth the approach but might cut corners.
        """
        super().__init__()
        self.kf = k_forward
        self.ktt = k_theta
        self.k_lat_factor = k_lateral_gain_factor
        self.v_ref = v_ref
        self.omega_max = omega_max
        
        if lookahead_dist <= 0:
             print("Warning: Kinematic controller lookahead_dist should be positive. Setting to a small default (0.1).")
             self.lookahead_dist = 0.1
        else:
            self.lookahead_dist = lookahead_dist

        self.closest_index = 0 # Memoization for the index of the closest path point to speed up search.
        self.prev_v1d = 0.0    # Stores the previously commanded v1_d for external derivative calculation.
        self.prev_omegad = 0.0 # Stores the previously commanded omega_d.
        self.finished_flag = False # Flag indicating if the robot has reached the end of the path.

    def find_target_point(self, robot_pos_tuple, path_coords):
        """
        Finds a suitable target point on the path based on lookahead distance
        and calculates the tangent (orientation) of the path at that target point.

        :param robot_pos_tuple: Current position of the robot (x, y) as a tuple or list.
        :param path_coords: The predefined path as a numpy array of shape (N, 2).
        :return: A tuple (target_point, theta_path_tangent).
                 target_point: (x_d, y_d) coordinates of the lookahead point.
                 theta_path_tangent: Orientation (angle in radians) of the path at the target point.
                 Returns (None, None) if no valid path or target can be found.
        """
        current_pos_np = np.array(robot_pos_tuple) # Ensure numpy array for vector operations
        
        # Calculate squared distances to all path points for finding the closest point
        # Using squared distances is slightly more efficient as it avoids sqrt,
        # but for finding the minimum, the actual distance or squared distance gives the same index.
        # distances_sq_to_path = np.sum((path_coords - current_pos_np)**2, axis=1) # Alternative

        distances_to_path = np.linalg.norm(path_coords - current_pos_np, axis=1)

        # Search for the closest point in a limited window around the previous closest_index for efficiency.
        search_radius = 20 # Number of points to search forward and backward from self.closest_index.
        start_idx = max(0, self.closest_index - search_radius)
        end_idx = min(len(path_coords), self.closest_index + search_radius + 1)
        
        path_segment_for_closest = path_coords[start_idx:end_idx]

        if path_segment_for_closest.shape[0] == 0: # Path is very short or empty.
             if len(path_coords) > 0: # If path exists, search the whole path.
                 self.closest_index = np.argmin(distances_to_path)
             else: # No path available.
                 return None, None 
        else: # Search within the segment.
             relative_closest_idx = np.argmin(np.linalg.norm(path_segment_for_closest - current_pos_np, axis=1))
             self.closest_index = start_idx + relative_closest_idx

        # Find the lookahead point: search forward from the closest_index until
        # a point is found that is at least lookahead_dist away from the robot's current position.
        target_idx = self.closest_index
        found_lookahead = False
        for i in range(self.closest_index, len(path_coords)):
            dist_to_candidate_point = np.linalg.norm(path_coords[i] - current_pos_np)
            if dist_to_candidate_point >= self.lookahead_dist:
                target_idx = i
                found_lookahead = True
                break
        if not found_lookahead: # If no such point is found (e.g., robot is near the end of the path)
            target_idx = len(path_coords) - 1 # Target the last point of the path.

        target_point_coords = path_coords[target_idx]

        # Check for path completion: if the current target is the last point on the path
        # and the robot is close to this last point, set the finished_flag.
        dist_to_final_path_point = np.linalg.norm(path_coords[-1] - current_pos_np)
        # Threshold for being "close enough" to the end. Should be less than lookahead_dist.
        finish_threshold = self.lookahead_dist * 0.3 
        if target_idx == len(path_coords) - 1 and dist_to_final_path_point < finish_threshold:
            self.finished_flag = True

        # Calculate the tangent (orientation) of the path at the target_point.
        # This is approximated using the vector between the target_point and the preceding point.
        if target_idx > 0: # If target is not the first point.
            path_segment_vector = path_coords[target_idx] - path_coords[target_idx - 1]
        elif len(path_coords) > 1: # If target is the first point, use the segment to the next point.
            path_segment_vector = path_coords[1] - path_coords[0]
        else: # Path has only one point.
            path_segment_vector = np.array([1.0, 0.0]) # Assume a default horizontal tangent.

        # Handle cases where path_segment_vector might be zero (e.g., duplicate points in path definition).
        path_segment_norm = np.linalg.norm(path_segment_vector)
        if path_segment_norm < 1e-6: # If vector is (near) zero length.
            # Try using segment *after* target point if possible and not at the very end.
            if target_idx + 1 < len(path_coords):
                 path_segment_vector = path_coords[target_idx+1] - path_coords[target_idx]
                 path_segment_norm = np.linalg.norm(path_segment_vector)
            
            if path_segment_norm < 1e-6: # If still zero, default to horizontal.
                path_segment_vector = np.array([1.0, 0.0])

        theta_path_tangent_at_target = np.arctan2(path_segment_vector[1], path_segment_vector[0])

        return target_point_coords, theta_path_tangent_at_target

    def compute_desired_velocities(self, robot_state, predefined_path):
        """
        Computes desired kinematic velocities (v1_d, omega_d) for path following.
        This is the primary method called by an external agent (e.g., a dynamic controller or simulation).

        :param robot_state: Current state of the robot (x, y, theta, v1, omega).
        :param predefined_path: Target path as a numpy array (N, 2).
        :return: Tuple (v1_d, omega_d, errors, finished_flag)
                 v1_d: Desired linear velocity (m/s).
                 omega_d: Desired angular velocity (rad/s).
                 errors: Tuple (error_forward, error_lateral, error_theta).
                 finished_flag: Boolean, true if the end of the path is considered reached.
        """
        if self.finished_flag: # If path completion was previously signaled.
             return 0.0, 0.0, (0.0, 0.0, 0.0), True

        x_robot, y_robot, theta_robot, _, _ = robot_state # Unpack current robot pose.
        current_robot_pos = (x_robot, y_robot)

        if predefined_path is None or len(predefined_path) < 2: # Path needs at least 2 points for tangent.
            print("Kinematic Controller: Path is invalid (None or < 2 points).")
            self.finished_flag = True
            return 0.0, 0.0, (0.0, 0.0, 0.0), True

        # Find the target point on the path and its tangent.
        target_point, theta_path_tangent = self.find_target_point(current_robot_pos, predefined_path)

        # The find_target_point method might set self.finished_flag. Check again.
        if self.finished_flag:
             self.prev_v1d = 0.0 # Ensure desired velocities are zero for derivative calculation.
             self.prev_omegad = 0.0
             return 0.0, 0.0, (0.0, 0.0, 0.0), True

        if target_point is None: # Safeguard if target finding failed.
            print("Kinematic Controller: Failed to find a valid target point.")
            self.finished_flag = True
            return 0.0, 0.0, (0.0, 0.0, 0.0), True

        x_target, y_target = target_point

        # --- Error Calculation ---
        # Calculate errors in the world frame.
        error_x_world = x_target - x_robot
        error_y_world = y_target - y_robot

        # Transform world-frame errors to the robot's local frame.
        cos_theta_robot = np.cos(theta_robot)
        sin_theta_robot = np.sin(theta_robot)
        
        # error_forward: Error along the robot's current heading.
        error_forward = error_x_world * cos_theta_robot + error_y_world * sin_theta_robot
        # error_lateral: Error perpendicular to the robot's current heading (to the left is positive).
        error_lateral = -error_x_world * sin_theta_robot + error_y_world * cos_theta_robot
        
        # Orientation error: Difference between the path's tangent orientation and the robot's current orientation.
        error_theta = theta_path_tangent - theta_robot
        # Normalize the orientation error to the range [-pi, pi].
        error_theta = (error_theta + np.pi) % (2 * np.pi) - np.pi

        # --- Kinematic Control Law (Lyapunov-inspired structure) ---
        # Desired linear velocity (v1_d):
        # Consists of a term to match reference speed along path (v_ref * cos(error_theta))
        # and a term to reduce forward error (self.kf * error_forward).
        v1_d = self.v_ref * np.cos(error_theta) + self.kf * error_forward

        # Desired angular velocity (omega_d):
        # Combines a term to correct lateral error (proportional to v_ref * sinc(error_theta) * error_lateral)
        # and a term to correct orientation error (proportional to error_theta).
        # The sinc(error_theta) term is crucial for nonholonomic robots, ensuring smooth steering.
        effective_lateral_gain = self.ktt * self.k_lat_factor
        omega_d = effective_lateral_gain * self.v_ref * self._safe_sinc(error_theta) * error_lateral \
                  + self.ktt * error_theta

        # Apply saturation to the commanded angular velocity.
        omega_d = np.clip(omega_d, -self.omega_max, self.omega_max)
        
        # Optional: Saturate linear velocity (e.g., to prevent excessive speeds or allow reverse).
        # max_abs_linear_vel = 1.5 * self.v_ref
        # v1_d = np.clip(v1_d, -max_abs_linear_vel, max_abs_linear_vel) # Example if reversing is allowed.
        # v1_d = np.clip(v1_d, 0, max_abs_linear_vel) # If only forward motion.

        # Store the computed desired velocities for use by an outer loop (e.g., for derivative calculation).
        self.prev_v1d = v1_d
        self.prev_omegad = omega_d

        current_errors = (error_forward, error_lateral, error_theta)
        return v1_d, omega_d, current_errors, self.finished_flag


# --- Adaptive Dynamic Controller ---
class AdaptiveDynamicController(Controller):
    """
    Adaptive dynamic controller (Inner Loop in Backstepping).
    This controller computes motor torques to make the robot's actual velocities (v1, omega)
    track the desired velocities (v1_d, omega_d) provided by a kinematic controller.
    It uses an adaptive law to estimate the robot's dynamic parameters (mass 'm' and inertia 'I').
    The control structure is based on adaptive backstepping principles, often involving:
    1. A feedforward term based on estimated parameters and desired accelerations.
    2. A feedback term to correct velocity tracking errors.
    3. A robustifying term to handle uncertainties and disturbances.
    4. An adaptation law to update parameter estimates.
    """
    def __init__(self, dt, wheel_radius, wheel_width,
                 kinematic_controller: LyapunovKinematicController,
                 initial_p_hat=np.array([1.0, 0.1]), 
                 gamma_p=np.diag([1.0, 0.1]),       
                 kd=np.diag([10.0, 5.0]),           
                 disturbance_bound=0.5,             
                 use_robust_term=True,
                 min_params = np.array([0.1, 0.01]), 
                 max_params = np.array([100.0, 50.0]) 
                 ):
        """
        Initializes the AdaptiveDynamicController.

        :param dt: Simulation time step (seconds). Critical for numerical differentiation and integration.
        :param wheel_radius: Radius of the robot's wheels (r) in meters.
        :param wheel_width: Distance between the centers of the two driven wheels (W) in meters.
        :param kinematic_controller: An instance of a kinematic controller (e.g., LyapunovKinematicController)
                                     that provides the desired velocities (v1_d, omega_d).
        :param initial_p_hat: Initial estimate for the parameter vector p_hat = [m_hat, I_hat]^T.
        :param gamma_p: Adaptation gain matrix (typically diagonal, positive definite) for p_hat.
                        Controls the speed of parameter adaptation.
        :param kd: Feedback gain matrix (typically diagonal, positive definite) for velocity error (Kd).
                   Determines the responsiveness to velocity tracking errors.
        :param disturbance_bound: Assumed upper bound (d_B) for external disturbances and unmodeled dynamics.
                                  Used in the robust control term.
        :param use_robust_term: Boolean flag to enable or disable the robustifying term (e.g., sgn(eta) or tanh(eta)).
        :param min_params: Minimum allowable values for parameter estimates [m_min, I_min]. Used for projection.
        :param max_params: Maximum allowable values for parameter estimates [m_max, I_max]. Used for projection.
        """
        super().__init__()
        if dt <= 0:
            raise ValueError("Time step dt must be positive for AdaptiveDynamicController.")
        self.dt = dt
        self.kinematic_controller = kinematic_controller

        self.p_hat = np.array(initial_p_hat, dtype=float) # Parameter estimate vector [m_hat, I_hat]^T
        self.Gamma_p = np.array(gamma_p, dtype=float)     # Adaptation gain matrix Gamma_p
        self.Kd = np.array(kd, dtype=float)               # Velocity error feedback gain matrix Kd
        self.dB = disturbance_bound                       # Assumed disturbance bound d_B
        self.use_robust_term = use_robust_term

        self.min_params = np.array(min_params, dtype=float)
        self.max_params = np.array(max_params, dtype=float)
        if np.any(self.min_params <= 1e-3): # Parameters like mass/inertia should be strictly positive
            print("Warning: min_params in AdaptiveDynamicController contains values very close to or <= zero. "
                  "They should be small positive numbers.")
        if np.any(self.max_params <= self.min_params):
            raise ValueError("max_params must be strictly greater than min_params for all elements.")

        # Precompute B2_inv matrix: transforms generalized forces [F_v1_cmd, F_omega_cmd] 
        # into wheel torques [tau_right_cmd, tau_left_cmd].
        # The robot's dynamic equation is often M2 * acc_actual = generalized_forces_applied.
        # Generalized forces from wheel torques: [F_v1; F_omega] = B2_matrix_form @ [tau_r; tau_l]
        # where F_v1 = (1/r)*(tau_r + tau_l), F_omega = (W/2r)*(tau_r - tau_l).
        # We need to calculate [tau_r; tau_l] = B2_inv @ [F_v1_cmd; F_omega_cmd].
        r = wheel_radius
        W = wheel_width
        if W == 0: # Should have been caught by Simulation, but good to check.
            raise ValueError("Wheel width (W) cannot be zero in AdaptiveDynamicController.")
        
        # B2_matrix_form = (1/r) * np.array([[1, 1], [W/2, -W/2]])
        # self.B2_inv = np.linalg.inv(B2_matrix_form)
        # Direct calculation of B2_inv:
        # tau_r = (r/2)*F_v1_cmd + (r/W)*F_omega_cmd
        # tau_l = (r/2)*F_v1_cmd - (r/W)*F_omega_cmd
        self.B2_inv = np.array([
            [r/2.0, r/W],    # Row for tau_right
            [r/2.0, -r/W]    # Row for tau_left
        ])

        # Store previous desired kinematic velocities (v1d_kin, omegad_kin)
        # for numerical differentiation to get vd_dot.
        self.prev_v1d_kin_for_deriv = 0.0
        self.prev_omegad_kin_for_deriv = 0.0
        
        self.first_run = True # Flag to correctly initialize prev_..._for_deriv on the first call.

    def compute_control(self, robot_state, predefined_path):
        """
        Computes adaptive torque commands for the robot's wheels.

        :param robot_state: Current full state of the robot (x, y, theta, v1_actual, omega_actual).
        :param predefined_path: Target path (N, 2) for the kinematic controller.
        :return: Tuple (tau_left_cmd, tau_right_cmd, status_dict)
                 tau_left_cmd: Commanded torque for the left wheel.
                 tau_right_cmd: Commanded torque for the right wheel.
                 status_dict: Dictionary containing intermediate values for analysis and debugging,
                              e.g., {'eta', 'p_hat', 'kin_errors', 'v_d', 'vd_dot', 'tau_bar_cmd'}.
        """
        _, _, _, v1_actual, omega_actual = robot_state # Unpack current actual velocities
        v_actual = np.array([v1_actual, omega_actual]) # Actual velocity vector [v1, omega]

        # 1. Get desired kinematic velocities (v_d) from the kinematic controller (outer loop).
        v1d_kin, omegad_kin, kin_errors_tuple, finished_kin = \
            self.kinematic_controller.compute_desired_velocities(robot_state, predefined_path)
        v_d = np.array([v1d_kin, omegad_kin]) # Desired velocity vector [v1_d, omega_d]

        # Initialize derivative states on the first run or if kinematic controller resets/finishes.
        # This prevents a large spike in vd_dot if v_d suddenly goes to zero.
        if self.first_run or (v1d_kin == 0.0 and omegad_kin == 0.0 and 
                              (self.prev_v1d_kin_for_deriv != 0.0 or self.prev_omegad_kin_for_deriv != 0.0)):
            self.prev_v1d_kin_for_deriv = v1d_kin
            self.prev_omegad_kin_for_deriv = omegad_kin
            self.first_run = False

        # If kinematic controller signals path completion, command zero torques.
        if finished_kin:
             eta_final = v_actual - v_d # Velocity error using the final (likely zero) v_d
             status_final = {'eta': eta_final, 'p_hat': self.p_hat.copy(), 
                             'kin_errors': kin_errors_tuple, 'v_d': v_d.copy(),
                             'vd_dot': np.zeros_like(v_d), 'tau_bar_cmd': np.zeros_like(v_d)}
             # Ensure derivatives are zero for the next potential call (though unlikely if finished)
             self.prev_v1d_kin_for_deriv = 0.0
             self.prev_omegad_kin_for_deriv = 0.0
             return 0.0, 0.0, status_final

        # 2. Estimate derivative of desired velocities (vd_dot = [v1d_dot, omegad_dot]^T).
        # Using simple numerical differentiation (backward difference).
        # More sophisticated filtering or estimation could be used here for noisy v_d signals.
        v1d_dot_est = (v1d_kin - self.prev_v1d_kin_for_deriv) / self.dt
        omegad_dot_est = (omegad_kin - self.prev_omegad_kin_for_deriv) / self.dt
        
        # Update stored previous desired velocities for the next iteration's derivative calculation.
        self.prev_v1d_kin_for_deriv = v1d_kin
        self.prev_omegad_kin_for_deriv = omegad_kin
        
        vd_dot_est = np.array([v1d_dot_est, omegad_dot_est]) # Desired acceleration vector

        # 3. Calculate velocity tracking error (eta = v_actual - v_desired).
        eta = v_actual - v_d

        # 4. Calculate Regressor Matrix Yc.
        # The robot dynamics (simplified) are M2 * v_actual_dot = tau_bar_applied.
        # The feedforward part of the control is often Yc(vd_dot) * p_hat, aiming to replicate M2_hat * vd_dot.
        # If M2 = diag(m, I), then M2_hat * vd_dot = [m_hat*v1d_dot; I_hat*omegad_dot].
        # This can be written as Yc @ p_hat where p_hat = [m_hat, I_hat]^T and Yc = diag(vd_dot).
        Yc = np.diag(vd_dot_est)

        # 5. Update Parameter Estimates (p_hat = [m_hat, I_hat]^T) using the adaptation law.
        # The standard adaptation law is p_hat_dot = -Gamma_p * Yc^T * eta.
        p_hat_dot = -self.Gamma_p @ Yc.T @ eta

        # Integrate p_hat_dot using Euler method to get p_hat for the next step.
        self.p_hat += p_hat_dot * self.dt

        # Apply projection: ensure parameter estimates stay within predefined bounds.
        self.p_hat = np.maximum(self.p_hat, self.min_params)
        self.p_hat = np.minimum(self.p_hat, self.max_params)

        # 6. Calculate Robust Control Term (u_robust).
        # This term helps to compensate for uncertainties, disturbances, and modeling errors.
        # Typically u_robust = d_B * sgn(eta), but a smoothed version (tanh) is used to reduce chattering.
        if self.use_robust_term and self.dB > 0:
            epsilon_boundary_layer = 0.05 # Thickness of the boundary layer for tanh smoothing.
            tanh_argument = eta / epsilon_boundary_layer
            # Clip argument to prevent tanh from reaching exactly +/-1, which can be problematic.
            tanh_argument = np.clip(tanh_argument, -10, 10) # tanh(-10) is very close to -1.
            sgn_eta_smoothed = np.tanh(tanh_argument)
            u_robust = self.dB * sgn_eta_smoothed
        else:
            u_robust = np.zeros_like(eta)

        # 7. Calculate Commanded Generalized Forces (tau_bar_cmd).
        # The control law for generalized forces F_bar (or tau_bar) is:
        # tau_bar_cmd = Feedforward (model-based) - Feedback (error correction) - Robust Term
        # tau_bar_cmd = Yc @ p_hat - Kd @ eta - u_robust
        feedforward_term = Yc @ self.p_hat  # M_hat * vd_dot_est
        feedback_term = self.Kd @ eta       # Kd * (v_actual - v_d)
        
        tau_bar_cmd = feedforward_term - feedback_term - u_robust # Generalized forces [F_v1_cmd, F_omega_cmd]^T

        # Convert generalized forces command to actual wheel torques command [tau_right, tau_left]^T.
        wheel_torques_cmd = self.B2_inv @ tau_bar_cmd
        tau_right_cmd, tau_left_cmd = wheel_torques_cmd[0], wheel_torques_cmd[1]

        # Optional: Implement torque saturation if motors have physical limits.
        # max_abs_motor_torque = 10.0 # Example limit in Nm
        # tau_left_cmd = np.clip(tau_left_cmd, -max_abs_motor_torque, max_abs_motor_torque)
        # tau_right_cmd = np.clip(tau_right_cmd, -max_abs_motor_torque, max_abs_motor_torque)

        # Store intermediate values for analysis, debugging, or plotting.
        status_dict = {
            'eta': eta.copy(),                      # Velocity tracking error
            'p_hat': self.p_hat.copy(),             # Current parameter estimates
            'kin_errors': kin_errors_tuple,         # Errors from the kinematic controller
            'v_d': v_d.copy(),                      # Desired velocities from kinematic controller
            'vd_dot': vd_dot_est.copy(),            # Estimated desired accelerations
            'tau_bar_cmd': tau_bar_cmd.copy(),      # Commanded generalized forces
            'u_robust': u_robust.copy()             # Robust term applied
        }

        return tau_left_cmd, tau_right_cmd, status_dict
