import numpy as np
import time

class Simulation:
    def __init__(self, dt, desired_path, wheel_radius, wheel_width, m, I, dB,
                 initial_pose, initial_velocity, kick_path_target_index=None,
                 kick_path_trigger_distance=0.3, kick_duration=1, kick_magnitude=None):
        self.dt = dt
        self.wheel_radius = wheel_radius
        self.wheel_width = wheel_width
        self.m = m
        self.I = I
        self.M2 = np.diag([self.m, self.I])
        self.M_inv = np.linalg.inv(self.M2)
        self.B2 = np.array([[1/self.wheel_radius, 1/self.wheel_radius],
                           [self.wheel_width/(2*self.wheel_radius), -self.wheel_width/(2*self.wheel_radius)]])
        self.dB = dB
        self.time_stamps = [0.0]
        self.robot_state = np.concatenate([initial_pose, initial_velocity])
        self.x, self.y, self.theta = initial_pose
        self.v1, self.omega = initial_velocity
        self.actual_path = [[self.x, self.y, self.theta]]
        self.robot_vels = [[self.v1, self.omega]]
        self.torques_cmd = [[0.0, 0.0]]  # Placeholder for t=0: [tau_left, tau_right]
        self.disturbances = [[0.0, 0.0]]  # Placeholder for t=0: [d_v1, d_omega]
        self.desired_path = desired_path
        self.kick_target_index = kick_path_target_index
        self.kick_trigger_distance = kick_path_trigger_distance
        self.kick_duration = kick_duration
        self.kick_magnitude = np.array(kick_magnitude) if kick_magnitude is not None else np.zeros(2)
        self.kick_target_point_coords = desired_path[kick_path_target_index] if kick_path_target_index is not None else None
        self.kick_active = False
        self.kick_start_time = None
        # Motor dynamics parameters
        self.gamma = 0.05  # Matches controller's gamma (updated in main.py)
        self.tau = np.array([0.0, 0.0])  # [tau_right, tau_left]

    def execute_cmd(self, u_left, u_right):
        """
        Execute the control command u(t) = [u_right, u_left], integrate motor dynamics,
        and update the robot state.
        """
        u = np.array([u_right, u_left])
        # Integrate motor dynamics: dot(tau) = (1/gamma)(-tau + u)
        tau_dot = (1/self.gamma) * (-self.tau + u)
        self.tau += tau_dot * self.dt
        # Apply torque to robot dynamics
        tau_bar = self.B2 @ self.tau
        # Add disturbance
        disturbance = self._compute_disturbance()
        v_dot = self.M_inv @ (tau_bar - disturbance)
        # Update velocities
        self.v1 += v_dot[0] * self.dt
        self.omega += v_dot[1] * self.dt
        self.robot_state[3:] = np.array([self.v1, self.omega])
        # Update position and orientation
        x_dot = self.v1 * np.cos(self.theta)
        y_dot = self.v1 * np.sin(self.theta)
        theta_dot = self.omega
        self.x += x_dot * self.dt
        self.y += y_dot * self.dt
        self.theta += theta_dot * self.dt
        # Normalize theta to [-pi, pi]
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi
        self.robot_state[:3] = np.array([self.x, self.y, self.theta])
        # Update history
        self.time_stamps.append(self.time_stamps[-1] + self.dt)
        self.actual_path.append([self.x, self.y, self.theta])
        self.robot_vels.append([self.v1, self.omega])
        self.torques_cmd.append(self.tau[[1, 0]].copy())  # [tau_left, tau_right]
        self.disturbances.append(disturbance.copy())

    def _compute_disturbance(self):
        """Compute disturbance (continuous + kick)."""
        disturbance = np.zeros(2)
        if self.kick_target_index is not None:
            current_pos = self.robot_state[:2]
            kick_target_pos = self.desired_path[self.kick_target_index]
            distance_to_kick = np.linalg.norm(current_pos - kick_target_pos)
            if distance_to_kick < self.kick_trigger_distance and not self.kick_active:
                self.kick_active = True
                self.kick_start_time = self.time_stamps[-1]
                print(f"--- Kick Triggered near Path Index {self.kick_target_index} at Time: {self.time_stamps[-1]:.2f}s ---")
            if self.kick_active:
                time_since_kick = self.time_stamps[-1] - self.kick_start_time
                if time_since_kick < self.kick_duration:
                    disturbance += self.kick_magnitude
                else:
                    self.kick_active = False
        # Add continuous disturbance
        if self.dB > 0:
            disturbance += np.random.uniform(-self.dB, self.dB, size=2)
        return disturbance

    def get_robot_state(self):
        """Returns the current full state of the robot."""
        return self.robot_state.copy()

    def get_simulation_data(self):
        """Returns all collected simulation data in a structured dictionary."""
        return {
            'time': np.array(self.time_stamps),
            'actual_path': np.array(self.actual_path),  # Columns: x, y, theta
            'robot_vels': np.array(self.robot_vels),    # Columns: v1, omega
            'torques_cmd': np.array(self.torques_cmd),  # Columns: tau_left, tau_right
            'disturbances': np.array(self.disturbances) # Columns: d_v1, d_omega
        }