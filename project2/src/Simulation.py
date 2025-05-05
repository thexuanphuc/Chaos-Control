import numpy as np
import time

class Simulation:
    def __init__(self, dt, desired_path, wheel_radius, wheel_width,
                 m, I, dB=0.0, # Dynamic parameters: mass, inertia, disturbance bound
                 initial_pose=np.array([0.0, 0.0, 0.0]),
                 initial_velocity=np.array([0.0, 0.0]), # v1, omega
                 # --- New/Modified parameters for trajectory-based kick ---
                 kick_path_target_index=None, # Index on desired_path to trigger near (e.g., 50)
                 kick_path_trigger_distance=0.5, # How close robot must be to path point (m)
                 kick_duration=0.1,       # How long the kick force is applied *after triggering* (s)
                 kick_magnitude=np.array([0.0, 0.0]) # Effective torque [v1_disturbance, omega_disturbance]
                 ):
        """
        Initialize the simulation with dynamic model.
        :param dt: Time step duration.
        :param desired_path: A numpy array of shape (N, 2) representing the desired path.
        :param wheel_radius: Radius of the wheels (r).
        :param wheel_width: Distance between the wheels (W).
        :param m: Mass of the robot.
        :param I: Moment of inertia of the robot around center.
        :param dB: Upper bound for continuous disturbance torque magnitude (optional).
        :param initial_pose: Initial pose (x, y, theta).
        :param initial_velocity: Initial velocity (v1, omega).
        :param kick_path_target_index: Index of the point on desired_path to trigger the kick near. None to disable.
        :param kick_path_trigger_distance: Distance (m) the robot must be within the target path point to trigger.
        :param kick_duration: Duration (seconds) the kick disturbance is applied after triggering.
        :param kick_magnitude: Numpy array [tau_d_v1, tau_d_omega] representing the effective kick torque.
        """
        self.desired_path = desired_path
        self.wheel_radius = wheel_radius # r
        self.wheel_width = wheel_width   # W
        self.dt = dt

        # Dynamic Parameters
        self.m = m
        self.I = I
        self.dB = dB # Continuous disturbance bound
        if m <= 0 or I <= 0: raise ValueError("Mass and Inertia must be positive.")
        self.M_inv = np.linalg.inv(np.diag([m, I]))
        if wheel_radius <= 0: raise ValueError("Wheel radius must be positive.")
        self.B2 = (1.0 / self.wheel_radius) * np.array([
            [1.0, 1.0],
            [self.wheel_width / 2.0, -self.wheel_width / 2.0]
        ])

        # State variables
        self.x, self.y, self.theta = initial_pose
        self.v1, self.omega = initial_velocity

        # History Storage
        self.actual_path = np.array([[self.x, self.y, self.theta]])
        self.time_stamps = [0.0]
        self.torques_cmd = [(0.0, 0.0)]
        self.robot_velocities_actual = [initial_velocity.tolist()]
        self.robot_accelerations_actual = [(0.0, 0.0)]
        self.disturbances_actual = [(0.0, 0.0)]

        # --- Store kick parameters and state flags ---
        self.kick_path_target_index = kick_path_target_index
        self.kick_path_trigger_distance = kick_path_trigger_distance
        self.kick_path_trigger_distance_sq = kick_path_trigger_distance**2 # Precompute square
        self.kick_duration = kick_duration
        self.kick_magnitude = np.array(kick_magnitude)

        # Validate target index
        if self.kick_path_target_index is not None:
             if not (0 <= self.kick_path_target_index < len(self.desired_path)):
                  raise ValueError(f"kick_path_target_index ({self.kick_path_target_index}) is out of bounds for desired_path length ({len(self.desired_path)})")
             self.kick_target_point_coords = self.desired_path[self.kick_path_target_index] # Store target coords
        else:
             self.kick_target_point_coords = None


        self.kick_triggered_flag = False # Has the kick been triggered yet?
        self.kick_start_time_actual = -1.0 # Time when the kick was actually triggered


    def _apply_disturbance(self):
        """
        Generate disturbance torque vector.
        Applies a predefined kick for a fixed duration once the robot gets close
        to a specific point on the desired trajectory,
        otherwise applies continuous random disturbance based on self.dB.
        """
        current_time = self.time_stamps[-1]
        current_pos = np.array([self.x, self.y])

        # 1. Check if robot is close to the target path point (if kick is enabled and not already triggered)
        if self.kick_target_point_coords is not None and not self.kick_triggered_flag:
            dist_sq = np.sum((current_pos - self.kick_target_point_coords)**2)
            if dist_sq < self.kick_path_trigger_distance_sq:
                print(f"--- Kick Triggered near Path Index {self.kick_path_target_index} at Time: {current_time:.2f}s ---")
                self.kick_triggered_flag = True
                self.kick_start_time_actual = current_time # Record the time it was triggered

        # 2. Determine if the kick should be active *in this timestep*
        kick_active_now = False
        if self.kick_triggered_flag:
            # Check if current time is within the kick duration window after triggering
            if current_time < self.kick_start_time_actual + self.kick_duration:
                kick_active_now = True

        # 3. Apply the appropriate disturbance
        if kick_active_now:
            # Apply the predefined kick magnitude
            tau_d_effective = self.kick_magnitude
        else:
            # Apply the standard continuous random disturbance
            disturbance_v1 = np.random.uniform(-self.dB, self.dB) if self.dB > 0 else 0.0
            disturbance_omega = np.random.uniform(-self.dB, self.dB) if self.dB > 0 else 0.0
            tau_d_effective = np.array([disturbance_v1, disturbance_omega])

        return tau_d_effective


    def execute_cmd(self, tau_left, tau_right):
        """
        Update the robot's state based on the given wheel TORQUE commands.
        Simplified: M2*v_dot = B2*tau - tau_d_effective
        :param tau_left: Torque command for the left wheel (Nm).
        :param tau_right: Torque command for the right wheel (Nm).
        """
        # Store command *before* using it for the step k -> k+1 interval
        self.torques_cmd.append((tau_left, tau_right))

        # Input torque vector (tau_r, tau_l)
        tau_cmd = np.array([tau_right, tau_left])

        # Calculate actual disturbance for this step (applied during k -> k+1)
        # This now includes potential kick or standard disturbance
        tau_d_actual = self._apply_disturbance()
        self.disturbances_actual.append(tuple(tau_d_actual))

        # Calculate accelerations using the dynamic model
        v_dot = self.M_inv @ (self.B2 @ tau_cmd - tau_d_actual)
        v1_dot, omega_dot = v_dot
        self.robot_accelerations_actual.append([v1_dot, omega_dot])

        # Update velocities using Euler integration (state at k+1)
        self.v1 += v1_dot * self.dt
        self.omega += omega_dot * self.dt
        self.robot_velocities_actual.append([self.v1, self.omega])

        # Update the robot's position and orientation using kinematic model with *updated* velocities
        delta_x = self.v1 * np.cos(self.theta) * self.dt
        delta_y = self.v1 * np.sin(self.theta) * self.dt
        delta_theta = self.omega * self.dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

        # Store updated pose (state at k+1)
        current_pose = np.array([[self.x, self.y, self.theta]])
        self.actual_path = np.append(self.actual_path, current_pose, axis=0)

        # Update time (time at k+1)
        self.time_stamps.append(self.time_stamps[-1] + self.dt)


    def get_robot_state(self):
        """ Get the current full state of the robot. """
        return self.x, self.y, self.theta, self.v1, self.omega

    def get_simulation_data(self):
        """ Returns all collected data, ensuring consistent lengths. """
        # All lists now have length N_steps + 1
        return {
            "time": np.array(self.time_stamps),
            "actual_path": self.actual_path, # x, y, theta
            "desired_path": self.desired_path, # x, y
            "torques_cmd": np.array(self.torques_cmd), # tau_l, tau_r (includes initial placeholder)
            "robot_vels": np.array(self.robot_velocities_actual), # v1, omega
            "robot_accels": np.array(self.robot_accelerations_actual), # v1_dot, omega_dot
            "disturbances": np.array(self.disturbances_actual) # Applied disturbance torque
        }