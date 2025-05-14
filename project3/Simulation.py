import numpy as np
import time

class Simulation:
    """
    Simulates a differential drive mobile robot with dynamic effects.
    The robot model includes mass, inertia, and can simulate disturbances.
    It updates the robot's state based on commanded wheel torques.
    """
    def __init__(self, dt, desired_path, wheel_radius, wheel_width,
                 m, I, dB=0.0,
                 initial_pose=np.array([0.0, 0.0, 0.0]),
                 initial_velocity=np.array([0.0, 0.0]), # initial [v1, omega]
                 kick_path_target_index=None,
                 kick_path_trigger_distance=0.5,
                 kick_duration=0.1,
                 kick_magnitude=np.array([0.0, 0.0])
                 ):
        """
        Initializes the simulation environment.

        :param dt: Time step duration (seconds).
        :param desired_path: A numpy array of shape (N, 2) representing the desired path (x, y coordinates).
        :param wheel_radius: Radius of the wheels (r) in meters.
        :param wheel_width: Distance between the wheels (W) in meters.
        :param m: Mass of the robot (kg).
        :param I: Moment of inertia of the robot around its center (kg*m^2).
        :param dB: Upper bound for continuous random disturbance torque magnitude (Nm).
                   If dB > 0, a random disturbance torque within [-dB, dB] is applied to each effective axis (v1, omega).
        :param initial_pose: Initial pose of the robot (x, y, theta) as a numpy array.
        :param initial_velocity: Initial linear (v1) and angular (omega) velocities as a numpy array.
        :param kick_path_target_index: Index of the point on desired_path to trigger a kick disturbance near.
                                       If None, trajectory-based kick is disabled.
        :param kick_path_trigger_distance: Distance (m) the robot must be within the target path point to trigger the kick.
        :param kick_duration: Duration (seconds) the kick disturbance is applied after being triggered.
        :param kick_magnitude: Numpy array [tau_d_v1, tau_d_omega] representing the effective kick torque components.
        """
        self.desired_path = np.array(desired_path)
        self.wheel_radius = wheel_radius
        self.wheel_width = wheel_width
        self.dt = dt

        # Dynamic Parameters
        if m <= 0 or I <= 0:
            raise ValueError("Mass (m) and Inertia (I) must be positive.")
        self.m = m  # mass
        self.I = I  # inertia
        self.M_matrix = np.diag([self.m, self.I]) # Mass-inertia matrix
        self.M_inv = np.linalg.inv(self.M_matrix) # Inverse of mass-inertia matrix

        if wheel_radius <= 0:
            raise ValueError("Wheel radius must be positive.")
        if wheel_width <= 0:
            raise ValueError("Wheel width must be positive.")

        # B2 matrix: relates wheel torques [tau_r; tau_l] to generalized forces [F_v1; N_omega]
        # F_v1 = (1/r) * (tau_r + tau_l)
        # N_omega = (W/(2r)) * (tau_r - tau_l)
        # So, [F_v1; N_omega] = B2_matrix_form @ [tau_r; tau_l]
        r = self.wheel_radius
        W = self.wheel_width
        self.B2_matrix_form = (1.0 / r) * np.array([
            [1.0, 1.0],      # Contribution to F_v1
            [W / 2.0, -W / 2.0] # Contribution to N_omega
        ])

        # State variables: pose (x, y, theta) and velocities (v1, omega)
        self.x, self.y, self.theta = initial_pose
        self.v1, self.omega = initial_velocity

        # History Storage for analysis and visualization
        self.actual_path_history = [list(initial_pose)] # Stores [x, y, theta] at each step
        self.time_stamps = [0.0]
        self.torques_commanded_history = [(0.0, 0.0)] # Stores (tau_l_cmd, tau_r_cmd)
        self.velocities_actual_history = [list(initial_velocity)] # Stores [v1, omega]
        self.accelerations_actual_history = [(0.0, 0.0)] # Stores [v1_dot, omega_dot]
        self.disturbances_applied_history = [(0.0, 0.0)] # Stores [tau_d_v1, tau_d_omega]

        # --- Kick Disturbance Parameters and State ---
        self.dB_continuous = dB # Continuous disturbance bound
        self.kick_path_target_index = kick_path_target_index
        self.kick_path_trigger_distance = kick_path_trigger_distance
        self.kick_path_trigger_distance_sq = kick_path_trigger_distance**2 # Precompute for efficiency
        self.kick_duration = kick_duration
        self.kick_magnitude = np.array(kick_magnitude)

        self.kick_target_point_coords = None
        if self.kick_path_target_index is not None:
            if not (0 <= self.kick_path_target_index < len(self.desired_path)):
                raise ValueError(f"kick_path_target_index ({self.kick_path_target_index}) is out of bounds "
                                 f"for desired_path length ({len(self.desired_path)}).")
            self.kick_target_point_coords = self.desired_path[self.kick_path_target_index]

        self.kick_triggered_flag = False # Becomes true once the kick condition is met
        self.kick_start_time_actual = -1.0 # Records the simulation time when the kick was triggered

    def _apply_disturbance(self):
        """
        Determines and returns the disturbance torque vector for the current step.
        This method handles both continuous random disturbances and the trajectory-based kick.

        :return: Numpy array [tau_d_v1, tau_d_omega] representing the effective disturbance torques.
        """
        current_time = self.time_stamps[-1]
        current_robot_pos = np.array([self.x, self.y])
        disturbance_v1_eff = 0.0
        disturbance_omega_eff = 0.0

        # 1. Check for Trajectory-Based Kick
        kick_is_active_this_step = False
        if self.kick_target_point_coords is not None: # If kick is configured
            if not self.kick_triggered_flag: # And not already triggered
                # Calculate distance to the target point on the path
                dist_sq_to_target = np.sum((current_robot_pos - self.kick_target_point_coords)**2)
                if dist_sq_to_target < self.kick_path_trigger_distance_sq:
                    print(f"--- Kick Triggered near Path Index {self.kick_path_target_index} at Time: {current_time:.2f}s ---")
                    self.kick_triggered_flag = True
                    self.kick_start_time_actual = current_time
            
            if self.kick_triggered_flag: # If kick has been triggered (either now or previously)
                # Check if current time is within the kick duration window
                if current_time < self.kick_start_time_actual + self.kick_duration:
                    kick_is_active_this_step = True

        # 2. Apply Disturbance
        if kick_is_active_this_step:
            # Apply the predefined kick magnitude
            disturbance_v1_eff, disturbance_omega_eff = self.kick_magnitude
            # print(f"Kick active at {current_time:.2f}s: Mag=({disturbance_v1_eff:.2f}, {disturbance_omega_eff:.2f})")
        elif self.dB_continuous > 0:
            # Apply continuous random disturbance if kick is not active
            disturbance_v1_eff = np.random.uniform(-self.dB_continuous, self.dB_continuous)
            disturbance_omega_eff = np.random.uniform(-self.dB_continuous, self.dB_continuous)
        
        return np.array([disturbance_v1_eff, disturbance_omega_eff])

    def execute_cmd(self, tau_left_cmd, tau_right_cmd):
        """
        Updates the robot's state for one time step based on commanded wheel torques.
        The dynamic model used is: M_matrix * [v1_dot; omega_dot] = B2_matrix_form * [tau_r; tau_l] - tau_disturbance_effective

        :param tau_left_cmd: Commanded torque for the left wheel (Nm).
        :param tau_right_cmd: Commanded torque for the right wheel (Nm).
        """
        # Store commanded torques for this step interval (k to k+1)
        self.torques_commanded_history.append((tau_left_cmd, tau_right_cmd))

        # Input torque vector [tau_right; tau_left]
        tau_wheels_cmd = np.array([tau_right_cmd, tau_left_cmd])

        # Get effective disturbance torques for this step
        tau_disturbance_effective = self._apply_disturbance()
        self.disturbances_applied_history.append(tuple(tau_disturbance_effective))

        # Calculate generalized forces from wheel torques
        generalized_forces_from_torques = self.B2_matrix_form @ tau_wheels_cmd
        
        # Calculate net generalized forces (including disturbances)
        net_generalized_forces = generalized_forces_from_torques - tau_disturbance_effective
        
        # Calculate accelerations using the dynamic model: v_dot = M_inv * F_net
        # v_dot = [v1_dot, omega_dot]^T
        v_dot_actual = self.M_inv @ net_generalized_forces
        v1_dot, omega_dot = v_dot_actual
        self.accelerations_actual_history.append(list(v_dot_actual))

        # Update velocities using Euler integration (velocities at time k+1)
        self.v1 += v1_dot * self.dt
        self.omega += omega_dot * self.dt
        self.velocities_actual_history.append([self.v1, self.omega])

        # Update pose (x, y, theta) using kinematic model with *updated* velocities
        # This uses v1(k+1) and omega(k+1) to update pose from q(k) to q(k+1)
        delta_x = self.v1 * np.cos(self.theta) * self.dt
        delta_y = self.v1 * np.sin(self.theta) * self.dt
        delta_theta = self.omega * self.dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to the range [-pi, pi]
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

        # Store updated pose (pose at time k+1)
        self.actual_path_history.append([self.x, self.y, self.theta])

        # Update simulation time (time at k+1)
        self.time_stamps.append(self.time_stamps[-1] + self.dt)

    def get_robot_state(self):
        """
        Returns the current full state of the robot.

        :return: Tuple (x, y, theta, v1, omega).
        """
        return self.x, self.y, self.theta, self.v1, self.omega

    def get_simulation_data(self):
        """
        Returns all collected simulation data in a structured dictionary.
        Ensures all history lists are converted to numpy arrays for consistent processing.

        :return: Dictionary containing time series data for various simulation variables.
        """
        # All history lists should have length (N_steps + 1) after N_steps calls to execute_cmd
        return {
            "time": np.array(self.time_stamps),
            "actual_path": np.array(self.actual_path_history), # Columns: x, y, theta
            "desired_path": self.desired_path, # Static
            "torques_cmd": np.array(self.torques_commanded_history), # Columns: tau_l, tau_r
            "robot_vels": np.array(self.velocities_actual_history), # Columns: v1, omega
            "robot_accels": np.array(self.accelerations_actual_history), # Columns: v1_dot, omega_dot
            "disturbances": np.array(self.disturbances_applied_history) # Columns: tau_d_v1, tau_d_omega
        }
