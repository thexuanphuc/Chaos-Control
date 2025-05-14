# Controller.py
import numpy as np
import time

class Controller:
    """
    Base class for controllers.
    Provides a common interface for different control strategies.
    All controllers should inherit from this class and implement compute_control.
    """
    def __init__(self):
        """ Initializes the base controller. """
        self.target_path_idx = 0 # Common attribute for controllers that track paths

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
        to avoid division by zero when the orientation error is small.

        :param x: Input angle in radians.
        :return: sin(x)/x.
        """
        if np.abs(x) < 1e-6:  # Threshold for Taylor expansion
            return 1.0 - x**2 / 6.0 + x**4 / 120.0
        else:
            return np.sin(x) / x

    def _normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _get_path_information(self, robot_x, robot_y, robot_theta, predefined_path, current_target_idx, lookahead_time=0.2, dt_val=0.02):
        """
        Determines the reference point on the path and its derivatives.
        This version uses a moving horizon: it finds the closest point and then looks ahead.

        :param robot_x: Current x position of the robot.
        :param robot_y: Current y position of the robot.
        :param robot_theta: Current orientation of the robot.
        :param predefined_path: Array of (x,y) waypoints.
        :param current_target_idx: Current target index on the path (managed by the controller instance).
        :param lookahead_time: Time to look ahead for v_r, omega_r estimation.
        :param dt_val: Simulation time step, used for approximating derivatives.
        :return: Tuple containing (
            target_idx (updated),
            x_r, y_r, theta_r (reference pose),
            v_r, omega_r (reference velocities),
            v_r_dot, omega_r_dot (reference accelerations)
        )
        """
        num_path_points = predefined_path.shape[0]
        if num_path_points == 0:
            # No path, return current robot state as reference, no movement
            return current_target_idx, robot_x, robot_y, robot_theta, 0, 0, 0, 0
        if num_path_points == 1:
            # Path is a single point, target it
            pt = predefined_path[0]
            return 0, pt[0], pt[1], robot_theta, 0,0,0,0 # Use robot_theta if no path orientation


        robot_pos = np.array([robot_x, robot_y])
        distances = np.linalg.norm(predefined_path - robot_pos, axis=1)
        
        # Update target_idx logic:
        # Start searching for closest from a window around the current target_idx to avoid jumps
        # This prevents jumping to a far part of the path if the path self-intersects near the robot.
        search_window_size = 20 # Number of points to check around current_target_idx
        search_start_idx = max(0, current_target_idx - search_window_size // 2)
        search_end_idx = min(num_path_points, current_target_idx + search_window_size // 2 + 1)
        
        if search_start_idx >= search_end_idx: # Path is too short for window, search whole path
            search_start_idx = 0
            search_end_idx = num_path_points

        relevant_distances = distances[search_start_idx:search_end_idx]
        if len(relevant_distances) > 0:
            local_closest_idx = np.argmin(relevant_distances)
            updated_target_idx = search_start_idx + local_closest_idx # Index in the full path
        else: 
            updated_target_idx = np.argmin(distances) # Fallback to global closest


        # Simple progression: if robot is close to the updated_target_idx, try to advance it slightly.
        # This helps move along the path rather than getting stuck on one point.
        if distances[updated_target_idx] < 0.5 : # If close to the current "closest"
            if updated_target_idx < num_path_points - 1:
                # Heuristic: Advance if the next point is generally "further" or robot is very close
                vec_robot_to_current = predefined_path[updated_target_idx] - robot_pos
                vec_current_to_next = predefined_path[updated_target_idx + 1] - predefined_path[updated_target_idx]
                # If robot is "behind" current target (dot product > 0 roughly) or very close, and next point exists
                if np.dot(vec_robot_to_current, vec_current_to_next) > -0.1 or distances[updated_target_idx] < 0.1: # -0.1 to allow slight overshoot
                     updated_target_idx = min(updated_target_idx + 1, num_path_points - 1)


        final_target_idx = min(updated_target_idx, num_path_points - 1)

        x_r = predefined_path[final_target_idx, 0]
        y_r = predefined_path[final_target_idx, 1]

        # Estimate theta_r (orientation of the path segment at x_r, y_r)
        if final_target_idx < num_path_points - 1:
            dx_path = predefined_path[final_target_idx + 1, 0] - x_r
            dy_path = predefined_path[final_target_idx + 1, 1] - y_r
        elif final_target_idx > 0 : 
            dx_path = x_r - predefined_path[final_target_idx - 1, 0]
            dy_path = y_r - predefined_path[final_target_idx - 1, 1]
        else: 
            dx_path = 1.0 
            dy_path = 0.0
        theta_r = np.arctan2(dy_path, dx_path)

        # Estimate v_r, omega_r by looking ahead
        lookahead_points = max(1, int(lookahead_time / dt_val))
        future_idx1 = min(final_target_idx + lookahead_points, num_path_points - 1)
        
        v_r, omega_r, v_r_dot, omega_r_dot = 0.0, 0.0, 0.0, 0.0 # Initialize

        if future_idx1 > final_target_idx: 
            time_diff1 = (future_idx1 - final_target_idx) * dt_val
            
            x_curr_seg = predefined_path[final_target_idx, 0]
            y_curr_seg = predefined_path[final_target_idx, 1]
            x_fut1_seg = predefined_path[future_idx1, 0]
            y_fut1_seg = predefined_path[future_idx1, 1]

            dist_moved1 = np.sqrt((x_fut1_seg - x_curr_seg)**2 + (y_fut1_seg - y_curr_seg)**2)
            v_r = dist_moved1 / time_diff1 if time_diff1 > 1e-6 else 0.0
            
            theta_curr_path_seg = theta_r 
            if future_idx1 < num_path_points - 1:
                theta_fut1_path_seg = np.arctan2(predefined_path[future_idx1+1,1]-y_fut1_seg, predefined_path[future_idx1+1,0]-x_fut1_seg)
            elif future_idx1 > 0:
                theta_fut1_path_seg = np.arctan2(y_fut1_seg-predefined_path[future_idx1-1,1], x_fut1_seg-predefined_path[future_idx1-1,0])
            else:
                theta_fut1_path_seg = theta_curr_path_seg
            
            d_theta1 = self._normalize_angle(theta_fut1_path_seg - theta_curr_path_seg)
            omega_r = d_theta1 / time_diff1 if time_diff1 > 1e-6 else 0.0

            future_idx2 = min(future_idx1 + lookahead_points, num_path_points - 1)
            if future_idx2 > future_idx1:
                time_diff2 = (future_idx2 - future_idx1) * dt_val
                if time_diff2 > 1e-6:
                    x_fut2_seg = predefined_path[future_idx2, 0]
                    y_fut2_seg = predefined_path[future_idx2, 1]
                    
                    dist_moved2 = np.sqrt((x_fut2_seg - x_fut1_seg)**2 + (y_fut2_seg - y_fut1_seg)**2)
                    v_r_fut1 = dist_moved2 / time_diff2
                    v_r_dot = (v_r_fut1 - v_r) / time_diff1 # Approx over interval of v_r calculation

                    if future_idx2 < num_path_points - 1:
                        theta_fut2_path_seg = np.arctan2(predefined_path[future_idx2+1,1]-y_fut2_seg, predefined_path[future_idx2+1,0]-x_fut2_seg)
                    elif future_idx2 > 0:
                        theta_fut2_path_seg = np.arctan2(y_fut2_seg-predefined_path[future_idx2-1,1], x_fut2_seg-predefined_path[future_idx2-1,0])
                    else:
                        theta_fut2_path_seg = theta_fut1_path_seg

                    d_theta2 = self._normalize_angle(theta_fut2_path_seg - theta_fut1_path_seg)
                    omega_r_fut1 = d_theta2 / time_diff2
                    omega_r_dot = (omega_r_fut1 - omega_r) / time_diff1
        
        # Update the controller's internal target index for the next call
        # self.target_path_idx = final_target_idx # This should be done by the calling controller instance
        return final_target_idx, x_r, y_r, theta_r, v_r, omega_r, v_r_dot, omega_r_dot


class LyapunovKinematicController(Controller):
    """
    Lyapunov-based Kinematic Controller for a differential drive robot.
    Uses Kanayama-like error definitions.
    Outputs desired linear (v1) and angular (omega) velocities.
    """
    def __init__(self, k_x, k_y, k_theta, dt=0.02):
        super().__init__() # Initializes self.target_path_idx = 0
        self.k_x = k_x # Gain for longitudinal error e_x_local
        self.k_y = k_y # Gain for lateral error e_y_local (often scaled by v_r)
        self.k_theta = k_theta # Gain for orientation error e_theta_local
        self.dt = dt
        self.prev_v1_d = 0.0
        self.prev_omega_d = 0.0

    def compute_control(self, robot_state, predefined_path):
        x, y, theta, _, _ = robot_state # Actual v1, omega not used by pure kinematic controller

        # Get reference point on path and its properties
        updated_target_idx, x_r, y_r, theta_r, v_r, omega_r, _, _ = \
            self._get_path_information(x, y, theta, predefined_path, self.target_path_idx, dt_val=self.dt)
        self.target_path_idx = updated_target_idx # Update controller's internal state

        # Compute Kanayama-like errors (errors in robot's local frame)
        e_x_global = x_r - x
        e_y_global = y_r - y
        
        e_x_local = e_x_global * np.cos(theta) + e_y_global * np.sin(theta) # Error along robot's x-axis
        e_y_local = -e_x_global * np.sin(theta) + e_y_global * np.cos(theta) # Error along robot's y-axis
        e_theta_local = self._normalize_angle(theta_r - theta) # Orientation error

        # Kanayama control law
        v1_d = v_r * np.cos(e_theta_local) + self.k_x * e_x_local
        # A common form for omega_d:
        # omega_d = omega_r + self.k_y * v_r * e_y_local + self.k_theta * e_theta_local
        # Original Kanayama uses sinc(e_theta_local) for the e_y_local term:
        omega_d = omega_r + self.k_y * v_r * self._safe_sinc(e_theta_local) * e_y_local + self.k_theta * e_theta_local
        # Another variant for the last term:
        # omega_d = omega_r + self.k_y * v_r * e_y_local + self.k_theta * v_r * self._safe_sinc(e_theta_local) * e_theta_local

        # Estimate derivatives of v1_d, omega_d numerically for output
        v1_d_dot = (v1_d - self.prev_v1_d) / self.dt
        omega_d_dot = (omega_d - self.prev_omega_d) / self.dt

        self.prev_v1_d = v1_d
        self.prev_omega_d = omega_d

        status_dict = {
            'kin_errors': (e_x_local, e_y_local, e_theta_local),
            'v_d': np.array([v1_d, omega_d]),
            'vd_dot': np.array([v1_d_dot, omega_d_dot]), # Estimated derivatives
            'ref_pose': (x_r, y_r, theta_r),
            'ref_vel': (v_r, omega_r),
            'target_path_idx': self.target_path_idx
        }
        # Output is desired velocities and the status dictionary
        return np.array([v1_d, omega_d]), status_dict


class BacksteppingDynamicController(Controller):
    """
    Backstepping-based Dynamic Controller for a differential drive robot.
    Integrates kinematic control objectives with dynamic compensation using adaptive backstepping.
    Estimates robot's effective mass (m_eff) and inertia (I_eff).
    """
    def __init__(self, wheel_radius, wheel_width,
                 k_v, k_omega, k_delta,  # Kinematic gains for internal virtual controller
                 Kd, K_bs,                # Dynamic and Backstepping gain matrices (or lists for diagonal)
                 gamma_p, initial_p_hat,  # Adaptation gain matrix and initial parameter estimates [m_eff, I_eff]
                 dB=0.1, use_robust_term=True, # Robustness term parameters
                 min_params=None, max_params=None, # Bounds for parameter estimates
                 dt=0.02):
        super().__init__() # Initializes self.target_path_idx = 0
        self.r = wheel_radius
        self.W = wheel_width
        self.dt = dt

        # Kinematic control parameters (for generating v_d, omega_d internally)
        self.k_v = k_v         # Gain for e1 (longitudinal error) in v1_d
        self.k_omega = k_omega # Gain for e2 (lateral error, scaled by v_r) in omega_d
        self.k_delta = k_delta # Gain for e3 (orientation error, often with sinc) in omega_d
        
        # parameter for kinematics controller
        self.k_xnew = self.k_v        # -> assumed this one is kx
        self.k_ynew = self.k_omega        # -> assumed this one is ky
        self.k_thetanew = self.k_delta        # -> assumed this one is k_theta

        # Dynamic control parameters
        self.Kd = np.diag(Kd) if isinstance(Kd, (list, tuple)) else Kd # Dynamic feedback gain matrix for velocity error eta
        # self.K_bs = np.diag(K_bs) if isinstance(K_bs, (list, tuple)) else K_bs # Backstepping gain matrix for J_bs term
        # for backstepping of V2
        self.K_bs = np.array([[1, 0], [0, 1/self.k_ynew]])
        
        # Adaptive control parameters for p_hat = [m_eff, I_eff]^T
        self.p_hat = np.array(initial_p_hat, dtype=float).reshape(-1, 1) if initial_p_hat is not None else np.array([[10.0], [1.0]])
        if self.p_hat.shape != (2,1): raise ValueError("initial_p_hat must be for [m_eff, I_eff]")
        
        self.gamma_p = np.diag(gamma_p) if isinstance(gamma_p, (list, tuple)) else gamma_p # Adaptation gain matrix

        # Robust term parameters
        self.use_robust_term = use_robust_term
        self.dB = dB # Gain for the robust term (scales smoothed sgn(eta))

        # Parameter projection bounds
        default_min = np.array([[1e-3],[1e-4]]) # Small positive defaults
        default_max = np.array([[100.0],[20.0]])
        self.min_params = np.array(min_params, dtype=float).reshape(-1,1) if min_params is not None else default_min
        self.max_params = np.array(max_params, dtype=float).reshape(-1,1) if max_params is not None else default_max
        if self.min_params.shape != (2,1) or self.max_params.shape != (2,1): raise ValueError("min/max_params must be for [m_eff, I_eff]")


        # Transformation matrix B2_inv: maps generalized forces [Fx_body; Mz_body] to wheel torques [tau_R; tau_L]
        self.B2_inv = np.array([[self.r / 2.0, self.r / self.W],
                                [self.r / 2.0, -self.r / self.W]])
        
        # For numerical differentiation of desired kinematic velocities
        self.prev_v1_d_kin = 0.0
        self.prev_omega_d_kin = 0.0

    def _kinematic_control_law(self, robot_state_kin, predefined_path_kin):
        """
        Internal kinematic controller to generate virtual controls (v1_d, omega_d)
        and their time derivatives.
        Errors e1, e2, e3 are defined in the robot's local frame.
        e1: error along robot's x-axis (forward/longitudinal)
        e2: error along robot's y-axis (lateral)
        e3: orientation error
        """
        x, y, theta, _, _ = robot_state_kin

        updated_target_idx, x_r, y_r, theta_r, v_r, omega_r, v_r_dot, omega_r_dot = \
            self._get_path_information(x, y, theta, predefined_path_kin, self.target_path_idx, dt_val=self.dt)
        self.target_path_idx = updated_target_idx # Update controller's path tracking index

        # Kinematic errors in robot's local frame
        e1 = (x_r - x) * np.cos(theta) + (y_r - y) * np.sin(theta) 
        e2 = -(x_r - x) * np.sin(theta) + (y_r - y) * np.cos(theta) 
        e3 = self._normalize_angle(theta_r - theta)

        # Virtual control laws (desired velocities)
        v1_d = v_r * np.cos(e3) + self.k_v * e1
        # Common form for omega_d, using sinc for stability with e3 in Lyapunov function
        # it should be : omega_d = omega_r + k_theta * e3 + k_y * v_x * sinc(e_theta) * e_y
        omega_d = omega_r + self.k_thetanew * e3 + self.k_ynew * v_r * self._safe_sinc(e3) * e2
        # Alternative without sinc if Lyapunov function uses e.g. (1-cos(e3)):
        # omega_d = omega_r + self.k_omega * v_r * e2 * self._safe_sinc(e3) + self.k_delta * e3 

        # Numerical differentiation for derivatives of virtual controls
        v1_d_dot = (v1_d - self.prev_v1_d_kin) / self.dt
        omega_d_dot = (omega_d - self.prev_omega_d_kin) / self.dt

        self.prev_v1_d_kin = v1_d
        self.prev_omega_d_kin = omega_d

        kin_errors_tuple = (e1, e2, e3)
        desired_velocities_virtual = np.array([v1_d, omega_d]).reshape(2,1)
        desired_accelerations_virtual = np.array([v1_d_dot, omega_d_dot]).reshape(2,1)
        ref_pose_tuple = (x_r, y_r, theta_r)
        ref_vel_tuple = (v_r, omega_r) # Reference v_r, omega_r from path

        return kin_errors_tuple, desired_velocities_virtual, desired_accelerations_virtual, ref_pose_tuple, ref_vel_tuple

    def _compute_regressor_Yc(self, v1_actual, omega_actual, v1_d_dot_est, omega_d_dot_est):
        """
        Computes the regressor matrix Yc for the adaptive law, assuming p_hat = [m_eff, I_eff]^T.
        This corresponds to a simplified dynamic model M_bar * nu_dot = tau_bar, where M_bar = diag(m_eff, I_eff).
        If Coriolis or complex friction terms were included, Yc would be more complex.
        """
        # Yc should be 2x2 for p_hat = [m_eff, I_eff]^T
        Yc = np.array([[v1_d_dot_est, 0.0],
                       [0.0, omega_d_dot_est]])
        return Yc

    def compute_backstepping_term(self, kin_errors_tuple, desired_velocities_virtual):
        """
        Computes the backstepping term J_bs (or K_bs @ J_bs_derived).
        This term arises from the derivative of the kinematic Lyapunov function V_kin
        and links kinematic objectives to dynamic control.
        The exact form depends on the chosen V_kin and kinematic control laws.

        A common derivation for V_kin = 0.5*(e1^2 + e2^2 + e3^2/lambda_coeff) yields
        terms in V_kin_dot like -e1*eta_v1 and (-e3/lambda_coeff)*eta_omega.
        So, the derived J_bs_components would be [-e1, -e3/lambda_coeff]^T.
        The self.K_bs matrix then scales these components.
        """
        e1, e2, e3 = kin_errors_tuple
        # v1_d = desired_velocities_virtual[0,0] # May be needed for some J_bs forms

        # Derived components of J_bs based on typical Lyapunov analysis for (e1,e2,e3) tracking
        j_bs_v1_component = e1 
        j_bs_omega_component = e3 # This is a common simplification. A specific lambda_coeff might appear.
                                   # For example, if V_kin has e3^2 / k_some_gain, then this might be -e3 / k_some_gain.

        # Apply K_bs gains. K_bs is diagonal: [[k_bs_v1_link, 0], [0, k_bs_omega_link]]
        # The term to be subtracted in the dynamic law is K_bs @ [j_bs_v1_comp; j_bs_omega_comp]
        j_bs1_final = self.K_bs[0,0] * j_bs_v1_component
        j_bs2_final = self.K_bs[1,1] * j_bs_omega_component
        
        return np.array([j_bs1_final, j_bs2_final]).reshape(2,1)

    def compute_control(self, robot_state, predefined_path):
        x, y, theta, v1_actual, omega_actual = robot_state
        actual_velocities = np.array([v1_actual, omega_actual]).reshape(2,1)

        # 1. Kinematic Layer: Generate desired virtual velocities (v_d) and their derivatives (vd_dot_est)
        kin_errors_tuple, v_d, vd_dot_est, ref_pose, ref_vel = \
            self._kinematic_control_law(robot_state, predefined_path)

        # 2. Compute Velocity Error (eta)
        eta = actual_velocities - v_d # Velocity tracking error

        # 3. Compute Regressor Yc for adaptive law
        Yc = self._compute_regressor_Yc(v1_actual, omega_actual, vd_dot_est[0,0], vd_dot_est[1,0])
        
        # 4. Adaptation Law for p_hat = [m_eff, I_eff]^T
        # p_hat_dot = Gamma_p * Yc^T * eta
        p_hat_dot = self.gamma_p @ Yc.T @ eta
        self.p_hat += p_hat_dot * self.dt

        # Parameter Projection
        self.p_hat = np.clip(self.p_hat, self.min_params, self.max_params)

        # 5. Robust Term u_robust (optional, for unmodeled dynamics/disturbances)
        u_robust = np.zeros((2,1))
        if self.use_robust_term and self.dB > 0:
            epsilon_robust_smoothing = 0.01 # Small value for tanh smoothing layer
            tanh_argument = eta / epsilon_robust_smoothing 
            tanh_argument = np.clip(tanh_argument, -10, 10) # Avoid large values for tanh
            sgn_eta_smoothed = np.tanh(tanh_argument)
            u_robust = self.dB * sgn_eta_smoothed # self.dB is the gain for the robust term

        # 6. Compute the Backstepping Term (J_bs or K_bs @ J_bs_derived)
        backstepping_term_val = self.compute_backstepping_term(kin_errors_tuple, v_d)

        # 7. Calculate Commanded Generalized Forces (tau_bar_cmd = [Fx_bar_cmd, Mz_bar_cmd]^T)
        # Standard adaptive backstepping control law:
        # tau_bar_cmd = Yc @ p_hat - Kd @ eta - u_robust - backstepping_term_val
        # Yc @ p_hat: Adaptive feedforward (model compensation: M_hat*vd_dot + C_hat*v_d + F_hat)
        # -Kd @ eta: Feedback for velocity error stabilization
        # -u_robust: Robustness against uncertainties
        # -backstepping_term_val: Term from kinematic Lyapunov analysis to ensure overall stability
        feedforward_term = Yc @ self.p_hat
        feedback_term = self.Kd @ eta 

        tau_bar_cmd = feedforward_term - feedback_term - u_robust + backstepping_term_val

        # 8. Convert generalized forces command to actual wheel torques
        wheel_torques_cmd = self.B2_inv @ tau_bar_cmd
        tau_right_cmd, tau_left_cmd = wheel_torques_cmd[0,0], wheel_torques_cmd[1,0]

        status_dict = {
            'eta': eta.flatten().copy(),
            'p_hat': self.p_hat.flatten().copy(),
            'kin_errors': kin_errors_tuple, # (e1, e2, e3)
            'v_d': v_d.flatten().copy(), # Desired virtual velocities [v1_d, omega_d]
            'vd_dot': vd_dot_est.flatten().copy(), # Estimated derivatives
            'tau_bar_cmd': tau_bar_cmd.flatten().copy(),
            'u_robust': u_robust.flatten().copy(),
            'backstepping_term': backstepping_term_val.flatten().copy(),
            'ref_pose': ref_pose, # (x_r, y_r, theta_r)
            'ref_vel': ref_vel,   # (v_r, omega_r)
            'target_path_idx': self.target_path_idx
        }
        return np.array([tau_right_cmd, tau_left_cmd]), status_dict


class AdaptiveDynamicController(Controller):
    """
    Adaptive Dynamic Controller for tracking externally provided desired velocities.
    Assumes v_d, omega_d, and their derivatives are set via `set_desired_velocities`.
    Adapts for robot parameters m_eff and I_eff.
    """
    def __init__(self, wheel_radius, wheel_width,
                 Kd, gamma_p, initial_p_hat,              
                 dB=0.1, use_robust_term=True,
                 min_params=None, max_params=None,     
                 dt=0.02):
        super().__init__()
        self.r = wheel_radius
        self.W = wheel_width
        self.dt = dt

        self.Kd = np.diag(Kd) if isinstance(Kd, (list, tuple)) else Kd 
        self.p_hat = np.array(initial_p_hat, dtype=float).reshape(-1,1) if initial_p_hat is not None else np.array([[10.0], [1.0]])
        if self.p_hat.shape != (2,1): raise ValueError("initial_p_hat must be for [m_eff, I_eff]")

        self.gamma_p = np.diag(gamma_p) if isinstance(gamma_p, (list, tuple)) else gamma_p
        self.use_robust_term = use_robust_term
        self.dB = dB 

        default_min = np.array([[1e-3],[1e-4]])
        default_max = np.array([[100.0],[20.0]])
        self.min_params = np.array(min_params, dtype=float).reshape(-1,1) if min_params is not None else default_min
        self.max_params = np.array(max_params, dtype=float).reshape(-1,1) if max_params is not None else default_max
        if self.min_params.shape != (2,1) or self.max_params.shape != (2,1): raise ValueError("min/max_params must be for [m_eff, I_eff]")


        self.B2_inv = np.array([[self.r / 2.0, self.r / self.W],
                                [self.r / 2.0, -self.r / self.W]])
        
        # To store externally provided desired velocities and accelerations
        self.v_d_external = np.zeros((2,1))
        self.vd_dot_external = np.zeros((2,1))

    def set_desired_velocities(self, v1_d, omega_d, v1_d_dot, omega_d_dot):
        """
        Method to provide desired velocities and accelerations if this controller
        is used with an external kinematic planner.
        """
        self.v_d_external[0,0] = v1_d
        self.v_d_external[1,0] = omega_d
        self.vd_dot_external[0,0] = v1_d_dot
        self.vd_dot_external[1,0] = omega_d_dot

    def _compute_regressor_Yc(self, v1_d_dot_est, omega_d_dot_est):
        """ Computes Yc for p_hat = [m_eff, I_eff]^T. """
        Yc = np.array([[v1_d_dot_est, 0.0],
                       [0.0, omega_d_dot_est]])
        return Yc

    def compute_control(self, robot_state, predefined_path=None): # predefined_path is ignored
        _, _, _, v1_actual, omega_actual = robot_state # x,y,theta not directly used
        actual_velocities = np.array([v1_actual, omega_actual]).reshape(2,1)

        # Use externally set desired velocities and accelerations
        v_d = self.v_d_external
        vd_dot_est = self.vd_dot_external
        
        if np.all(np.abs(v_d) < 1e-9) and predefined_path is not None: 
            # This is a fallback if used without calling set_desired_velocities but path is given
            # It's not its primary mode of operation.
            print("AdaptiveDynamicController Warning: Desired velocities (v_d) are zero or not set. "
                  "This controller expects v_d via set_desired_velocities(). Outputting zero torques.")
        
        eta = actual_velocities - v_d # Velocity tracking error
        Yc = self._compute_regressor_Yc(vd_dot_est[0,0], vd_dot_est[1,0])
        
        p_hat_dot = self.gamma_p @ Yc.T @ eta
        self.p_hat += p_hat_dot * self.dt
        self.p_hat = np.clip(self.p_hat, self.min_params, self.max_params)

        u_robust = np.zeros((2,1))
        if self.use_robust_term and self.dB > 0:
            epsilon_robust_smoothing = 0.01
            tanh_argument = eta / epsilon_robust_smoothing
            tanh_argument = np.clip(tanh_argument, -10, 10)
            sgn_eta_smoothed = np.tanh(tanh_argument)
            u_robust = self.dB * sgn_eta_smoothed

        # Control law for dynamic tracking: tau_bar_cmd = Yc @ p_hat - Kd @ eta - u_robust
        feedforward_term = Yc @ self.p_hat
        feedback_term = self.Kd @ eta 
        tau_bar_cmd = feedforward_term - feedback_term - u_robust
        # No explicit backstepping_term_val here, as this controller assumes it's implicitly
        # handled by the source of v_d, omega_d (e.g., an outer backstepping loop).

        wheel_torques_cmd = self.B2_inv @ tau_bar_cmd
        tau_right_cmd, tau_left_cmd = wheel_torques_cmd[0,0], wheel_torques_cmd[1,0]

        status_dict = {
            'eta': eta.flatten().copy(),
            'p_hat': self.p_hat.flatten().copy(),
            'v_d': v_d.flatten().copy(),
            'vd_dot': vd_dot_est.flatten().copy(),
            'tau_bar_cmd': tau_bar_cmd.flatten().copy(),
            'u_robust': u_robust.flatten().copy(),
        }
        return np.array([tau_right_cmd, tau_left_cmd]), status_dict
