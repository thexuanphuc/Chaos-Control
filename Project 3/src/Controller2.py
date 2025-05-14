import numpy as np
import time

class Controller:
    def __init__(self):
        pass

    def compute_control(self, robot_state, predefined_path):
        raise NotImplementedError("The compute_control method must be implemented by subclasses.")

    def _safe_sinc(self, x):
        if np.abs(x) < 1e-6:
            return 1.0 - x**2 / 6.0
        else:
            return np.sin(x) / x

class LyapunovKinematicController(Controller):
    def __init__(self, k_forward=1.0, k_theta=2.0, k_lateral_gain_factor=1.0,
                 v_ref=1.0, omega_ref=0.0, omega_max=np.pi, lookahead_dist=0.5):
        super().__init__()
        self.kf = k_forward  # K_x
        self.ktt = k_theta   # K_theta
        self.k_lat_factor = k_lateral_gain_factor  # Contributes to K_y
        self.v_ref = v_ref   # v_r
        self.omega_ref = omega_ref  # omega_r
        self.omega_max = omega_max
        if lookahead_dist <= 0:
            print("Warning: Kinematic controller lookahead_dist should be positive. Setting to a small default (0.1).")
            self.lookahead_dist = 0.1
        else:
            self.lookahead_dist = lookahead_dist
        self.closest_index = 0
        self.prev_v1d = 0.0
        self.prev_omegad = 0.0
        self.finished_flag = False

    def find_target_point(self, robot_pos_tuple, path_coords):
        current_pos_np = np.array(robot_pos_tuple)
        distances_to_path = np.linalg.norm(path_coords - current_pos_np, axis=1)
        search_radius = 20
        start_idx = max(0, self.closest_index - search_radius)
        end_idx = min(len(path_coords), self.closest_index + search_radius + 1)
        path_segment_for_closest = path_coords[start_idx:end_idx]
        if path_segment_for_closest.shape[0] == 0:
            if len(path_coords) > 0:
                self.closest_index = np.argmin(distances_to_path)
            else:
                return None, None
        else:
            relative_closest_idx = np.argmin(np.linalg.norm(path_segment_for_closest - current_pos_np, axis=1))
            self.closest_index = start_idx + relative_closest_idx
        target_idx = self.closest_index
        found_lookahead = False
        for i in range(self.closest_index, len(path_coords)):
            dist_to_candidate_point = np.linalg.norm(path_coords[i] - current_pos_np)
            if dist_to_candidate_point >= self.lookahead_dist:
                target_idx = i
                found_lookahead = True
                break
        if not found_lookahead:
            target_idx = len(path_coords) - 1
        target_point_coords = path_coords[target_idx]
        dist_to_final_path_point = np.linalg.norm(path_coords[-1] - current_pos_np)
        finish_threshold = self.lookahead_dist * 0.3
        if target_idx == len(path_coords) - 1 and dist_to_final_path_point < finish_threshold:
            self.finished_flag = True
        if target_idx > 0:
            path_segment_vector = path_coords[target_idx] - path_coords[target_idx - 1]
        elif len(path_coords) > 1:
            path_segment_vector = path_coords[1] - path_coords[0]
        else:
            path_segment_vector = np.array([1.0, 0.0])
        path_segment_norm = np.linalg.norm(path_segment_vector)
        if path_segment_norm < 1e-6:
            if target_idx + 1 < len(path_coords):
                path_segment_vector = path_coords[target_idx+1] - path_coords[target_idx]
                path_segment_norm = np.linalg.norm(path_segment_vector)
            if path_segment_norm < 1e-6:
                path_segment_vector = np.array([1.0, 0.0])
        theta_path_tangent_at_target = np.arctan2(path_segment_vector[1], path_segment_vector[0])
        return target_point_coords, theta_path_tangent_at_target

    def compute_desired_velocities(self, robot_state, predefined_path):
        if self.finished_flag:
            return 0.0, 0.0, (0.0, 0.0, 0.0), True
        x_robot, y_robot, theta_robot, _, _ = robot_state
        current_robot_pos = (x_robot, y_robot)
        if predefined_path is None or len(predefined_path) < 2:
            print("Kinematic Controller: Path is invalid (None or < 2 points).")
            self.finished_flag = True
            return 0.0, 0.0, (0.0, 0.0, 0.0), True
        target_point, theta_path_tangent = self.find_target_point(current_robot_pos, predefined_path)
        if self.finished_flag:
            self.prev_v1d = 0.0
            self.prev_omegad = 0.0
            return 0.0, 0.0, (0.0, 0.0, 0.0), True
        if target_point is None:
            print("Kinematic Controller: Failed to find a valid target point.")
            self.finished_flag = True
            return 0.0, 0.0, (0.0, 0.0, 0.0), True
        x_target, y_target = target_point
        error_x_world = x_target - x_robot
        error_y_world = y_target - y_robot
        cos_theta_robot = np.cos(theta_robot)
        sin_theta_robot = np.sin(theta_robot)
        # Error in the robot's frame
        error_forward = error_x_world * cos_theta_robot + error_y_world * sin_theta_robot
        error_lateral = -error_x_world * sin_theta_robot + error_y_world * cos_theta_robot
        error_theta = theta_path_tangent - theta_robot
        error_theta = (error_theta + np.pi) % (2 * np.pi) - np.pi
        # Desired velocities
        v1_d = self.v_ref * np.cos(error_theta) + self.kf * error_forward
        effective_lateral_gain = self.ktt * self.k_lat_factor
        omega_d = self.omega_ref + self.ktt * error_theta + self.v_ref * error_lateral * effective_lateral_gain * self._safe_sinc(error_theta)
        omega_d = np.clip(omega_d, -self.omega_max, self.omega_max)
        self.prev_v1d = v1_d
        self.prev_omegad = omega_d
        current_errors = (error_forward, error_lateral, error_theta)
        return v1_d, omega_d, current_errors, self.finished_flag

class BacksteppingDynamicController(Controller):
    def __init__(self, dt, wheel_radius, wheel_width, gamma,
                 kinematic_controller: LyapunovKinematicController,
                 robot_mass=12.0, robot_inertia=0.8,
                 kd=np.diag([50.0, 25.0]), kz=np.diag([15.0, 15.0]),
                 disturbance_bound=360.0, epsilon=0.005,
                 torque_limit=500.0, ky=1.0, gamma_p=np.diag([0.1, 0.01]),
                 initial_p_hat=np.array([12.0, 0.8]),
                 min_p=np.array([0.1, 0.01]), max_p=np.array([50.0, 10.0])):
        super().__init__()
        if dt <= 0:
            raise ValueError("Time step dt must be positive.")
        if gamma <= 0:
            raise ValueError("Motor time constant gamma must be positive.")
        self.dt = dt
        self.gamma = gamma
        self.kinematic_controller = kinematic_controller
        self.m_true = robot_mass
        self.I_true = robot_inertia
        self.M2_true = np.diag([self.m_true, self.I_true])
        self.Kd = np.array(kd, dtype=float)
        self.Kz = np.array(kz, dtype=float)
        self.dB = disturbance_bound
        self.epsilon = epsilon
        self.torque_limit = torque_limit
        self.ky = ky
        self.r = wheel_radius
        self.W = wheel_width
        if self.W == 0:
            raise ValueError("Wheel width cannot be zero.")
        self.B2_inv = np.array([
            [self.r/2.0, self.r/self.W],
            [self.r/2.0, -self.r/self.W]
        ])
        self.prev_v1d = 0.0
        self.prev_omegad = 0.0
        self.prev_vd_dot = np.zeros(2)
        self.prev_tau_d = np.zeros(2)
        self.prev_tau_d_dot = np.zeros(2)
        self.tau = np.zeros(2)
        self.first_run = True
        self.alpha = 0.7
        self.p_hat = np.array(initial_p_hat, dtype=float).copy()
        self.Gamma_p = np.array(gamma_p, dtype=float)
        self.min_p = np.array(min_p, dtype=float)
        self.max_p = np.array(max_p, dtype=float)
        if np.any(self.min_p <= 0):
            raise ValueError("min_p must be positive.")
        if np.any(self.max_p <= self.min_p):
            raise ValueError("max_p must be greater than min_p.")

    def compute_control(self, robot_state, predefined_path):
        _, _, _, v1_actual, omega_actual = robot_state
        v_actual = np.array([v1_actual, omega_actual])
        v1d, omegad, kin_errors, finished = self.kinematic_controller.compute_desired_velocities(robot_state, predefined_path)
        v_d = np.array([v1d, omegad])
        if self.first_run:
            self.prev_v1d = v1d
            self.prev_omegad = omegad
            self.first_run = False
        if finished:
            self.prev_v1d = 0.0
            self.prev_omegad = 0.0
            self.prev_vd_dot = np.zeros(2)
            self.prev_tau_d = np.zeros(2)
            self.prev_tau_d_dot = np.zeros(2)
            self.tau = np.zeros(2)
            status = {
                'eta': v_actual - v_d,
                'z': np.zeros(2),
                'kin_errors': kin_errors,
                'v_d': v_d.copy(),
                'vd_dot': np.zeros(2),
                'tau_bar_d': np.zeros(2),
                'tau_d': np.zeros(2),
                'u': np.zeros(2),
                'tau': self.tau.copy(),
                'p_hat': self.p_hat.copy()
            }
            return 0.0, 0.0, status
        v1d_dot_raw = (v1d - self.prev_v1d) / self.dt
        omegad_dot_raw = (omegad - self.prev_omegad) / self.dt
        vd_dot_raw = np.array([v1d_dot_raw, omegad_dot_raw])
        vd_dot = self.alpha * self.prev_vd_dot + (1 - self.alpha) * vd_dot_raw
        # vd_dot = vd_dot_raw
        self.prev_v1d = v1d
        self.prev_omegad = omegad
        self.prev_vd_dot = vd_dot
        Yc = np.diag(vd_dot)
        eta = v_actual - v_d
        # Desired torque
        tau_bar_d = np.diag(self.p_hat) @ vd_dot - self.Kd @ eta - self.dB * np.tanh(eta / self.epsilon) + np.array([kin_errors[0], kin_errors[2] / self.ky])
        tau_d = self.B2_inv @ tau_bar_d
        tau_d = np.clip(tau_d, -self.torque_limit, self.torque_limit)
        tau_d_dot_raw = (tau_d - self.prev_tau_d) / self.dt
        tau_d_dot = self.alpha * self.prev_tau_d_dot + (1 - self.alpha) * tau_d_dot_raw
        # tau_d_dot = tau_d_dot_raw
        z = self.tau - tau_d
        self.p_hat = np.maximum(self.p_hat, self.min_p)
        self.p_hat = np.minimum(self.p_hat, self.max_p)
        p_hat_dot = -self.Gamma_p @ Yc.T @ eta
        self.p_hat += p_hat_dot * self.dt
        u = tau_d + self.gamma * (tau_d_dot - self.Kz @ z)
        u = np.clip(u, -self.torque_limit, self.torque_limit)
        self.tau += (u - self.tau) * self.dt / self.gamma
        status = {
            'eta': eta.copy(),
            'z': z.copy(),
            'kin_errors': kin_errors,
            'v_d': v_d.copy(),
            'vd_dot': vd_dot.copy(),
            'tau_bar_d': tau_bar_d.copy(),
            'tau_d': tau_d.copy(),
            'u': u.copy(),
            'tau': self.tau.copy(),
            'p_hat': self.p_hat.copy()
        }
        self.prev_tau_d = tau_d.copy()
        self.prev_tau_d_dot = tau_d_dot
        return u[1], u[0], status
    
class BacksteppingDynamicController2(Controller):
    def __init__(self, dt, wheel_radius, wheel_width, gamma,
                 kinematic_controller: LyapunovKinematicController,
                 robot_mass=12.0, robot_inertia=0.8,
                 kd=np.diag([50.0, 25.0]), kz=np.diag([15.0, 15.0]),
                 disturbance_bound=360.0, epsilon=0.005,
                 torque_limit=500.0, ky=1.0, gamma_p=np.diag([0.1, 0.01]),
                 initial_p_hat=np.array([12.0, 0.8]),
                 min_p=np.array([0.1, 0.01]), max_p=np.array([50.0, 10.0])):
        super().__init__()
        if dt <= 0:
            raise ValueError("Time step dt must be positive.")
        if gamma <= 0:
            raise ValueError("Motor time constant gamma must be positive.")
        self.dt = dt
        self.gamma = gamma
        self.kinematic_controller = kinematic_controller
        self.m_true = robot_mass
        self.I_true = robot_inertia
        self.M2_true = np.diag([self.m_true, self.I_true])
        self.Kd = np.array(kd, dtype=float)
        self.Kz = np.array(kz, dtype=float)
        self.dB = disturbance_bound
        self.epsilon = epsilon
        self.torque_limit = torque_limit
        self.ky = ky
        self.r = wheel_radius
        self.W = wheel_width
        if self.W == 0:
            raise ValueError("Wheel width cannot be zero.")
        self.B2_inv = np.array([
            [self.r/2.0, self.r/self.W],
            [self.r/2.0, -self.r/self.W]
        ])
        self.prev_v1d = 0.0
        self.prev_omegad = 0.0
        self.prev_vd_dot = np.zeros(2)
        self.prev_tau_d = np.zeros(2)
        self.prev_tau_d_dot = np.zeros(2)
        self.tau = np.zeros(2)
        self.first_run = True
        self.p_hat = np.array(initial_p_hat, dtype=float).copy()
        self.Gamma_p = np.array(gamma_p, dtype=float)
        self.min_p = np.array(min_p, dtype=float)
        self.max_p = np.array(max_p, dtype=float)
        if np.any(self.min_p <= 0):
            raise ValueError("min_p must be positive.")
        if np.any(self.max_p <= self.min_p):
            raise ValueError("max_p must be greater than min_p.")

    def compute_control(self, robot_state, predefined_path):
        _, _, _, v1_actual, omega_actual = robot_state
        v_actual = np.array([v1_actual, omega_actual])
        v1d, omegad, kin_errors, finished = self.kinematic_controller.compute_desired_velocities(robot_state, predefined_path)
        v_d = np.array([v1d, omegad])
        if self.first_run:
            self.prev_v1d = v1d
            self.prev_omegad = omegad
            self.first_run = False
        if finished:
            self.prev_v1d = 0.0
            self.prev_omegad = 0.0
            self.prev_vd_dot = np.zeros(2)
            self.prev_tau_d = np.zeros(2)
            self.prev_tau_d_dot = np.zeros(2)
            self.tau = np.zeros(2)
            status = {
                'eta': v_actual - v_d,
                'z': np.zeros(2),
                'kin_errors': kin_errors,
                'v_d': v_d.copy(),
                'vd_dot': np.zeros(2),
                'tau_bar_d': np.zeros(2),
                'tau_d': np.zeros(2),
                'u': np.zeros(2),
                'tau': self.tau.copy(),
                'p_hat': self.p_hat.copy()
            }
            return 0.0, 0.0, status
        v1d_dot_raw = (v1d - self.prev_v1d) / self.dt
        omegad_dot_raw = (omegad - self.prev_omegad) / self.dt
        vd_dot = np.array([v1d_dot_raw, omegad_dot_raw])
        self.prev_v1d = v1d
        self.prev_omegad = omegad
        self.prev_vd_dot = vd_dot
        Yc = np.diag(vd_dot)
        eta = v_actual - v_d
        tau_bar_d = np.diag(self.p_hat) @ vd_dot - self.Kd @ eta - self.dB * np.tanh(eta / self.epsilon) + np.array([kin_errors[0], kin_errors[2] / self.ky])
        tau_d = self.B2_inv @ tau_bar_d
        tau_d = np.clip(tau_d, -self.torque_limit, self.torque_limit)
        tau_d_dot = (tau_d - self.prev_tau_d) / self.dt
        z = self.tau - tau_d
        self.p_hat = np.maximum(self.p_hat, self.min_p)
        self.p_hat = np.minimum(self.p_hat, self.max_p)
        p_hat_dot = -self.Gamma_p @ Yc.T @ eta
        self.p_hat += p_hat_dot * self.dt
        u = tau_d + self.gamma * (tau_d_dot - self.Kz @ z)
        u = np.clip(u, -self.torque_limit, self.torque_limit)
        self.tau += (u - self.tau) * self.dt / self.gamma
        status = {
            'eta': eta.copy(),
            'z': z.copy(),
            'kin_errors': kin_errors,
            'v_d': v_d.copy(),
            'vd_dot': vd_dot.copy(),
            'tau_bar_d': tau_bar_d.copy(),
            'tau_d': tau_d.copy(),
            'u': u.copy(),
            'tau': self.tau.copy(),
            'p_hat': self.p_hat.copy()
        }
        self.prev_tau_d = tau_d.copy()
        self.prev_tau_d_dot = tau_d_dot
        return u[1], u[0], status