# Simulation.py
import numpy as np
import time

class Simulation:
    """
    Simulates a differential drive mobile robot with dynamic effects.
    The robot model includes mass, inertia, and can simulate disturbances
    (continuous random and discrete "kick").
    It updates the robot's state based on commanded wheel torques and
    can optionally add sensor noise to velocity measurements.
    """
    def __init__(self, dt, desired_path, wheel_radius, wheel_width,
                 m, I, 
                 dB_sim_continuous=0.0, # Continuous disturbance bound
                 initial_pose=np.array([0.0, 0.0, 0.0]), # x, y, theta
                 initial_velocity=np.array([0.0, 0.0]), # initial [v1, omega]
                 # Kick disturbance parameters
                 kick_active=False,
                 kick_start_time=10.0, 
                 kick_duration=0.5, 
                 kick_magnitude=np.array([0.0, 0.0]), 
                 # Sensor noise parameters (standard deviation)
                 velocity_noise_std=0.0, 
                 omega_noise_std=0.0
                 ):
        """
        Initializes the simulation environment.
        :param dt: Time step duration (seconds).
        :param desired_path: A numpy array of shape (N, 2) representing the target path (x, y coordinates).
        :param wheel_radius: Radius of the wheels (r) in meters.
        :param wheel_width: Distance between the wheels (W) in meters.
        :param m: True mass of the robot (kg).
        :param I: True moment of inertia of the robot around its center (kg*m^2).
        :param dB_sim_continuous: Continuous disturbance. Scalar for general magnitude, 
                                  or [max_Fx_dist, max_Mz_dist].
        :param initial_pose: Initial [x, y, theta] of the robot.
        :param initial_velocity: Initial [v1, omega] of the robot.
        :param kick_active: Boolean to activate kick disturbance.
        :param kick_start_time: Simulation time at which kick disturbance begins.
        :param kick_duration: Duration of the kick disturbance in seconds.
        :param kick_magnitude: Magnitude of kick as [kick_Fx_body, kick_Mz_body].
        :param velocity_noise_std: Standard deviation of Gaussian noise on v1 measurement.
        :param omega_noise_std: Standard deviation of Gaussian noise on omega measurement.
        """
        if dt <= 0:
            raise ValueError("Time step dt must be positive.")
        self.dt = dt
        self.desired_path = np.array(desired_path) if desired_path is not None and desired_path.size > 0 else np.array([[0.0,0.0]]) # Handle empty path
        self.r = wheel_radius
        self.W = wheel_width
        if self.r <= 0 or self.W <= 0:
            raise ValueError("Wheel radius and width must be positive.")

        self.m_true = m
        self.I_true = I
        if self.m_true <= 0 or self.I_true <= 0:
            raise ValueError("True mass and inertia must be positive.")

        self.dB_sim_continuous = dB_sim_continuous
        self.velocity_noise_std = velocity_noise_std
        self.omega_noise_std = omega_noise_std

        # Internal true state variables (not directly accessible by controller)
        self._x_true, self._y_true, self._theta_true = float(initial_pose[0]), float(initial_pose[1]), float(initial_pose[2])
        self._v1_true, self._omega_true = float(initial_velocity[0]), float(initial_velocity[1])

        # Measured state variables (output to controller, possibly with noise)
        self.x, self.y, self.theta = self._x_true, self._y_true, self._theta_true
        self.v1 = self._v1_true + np.random.normal(0, self.velocity_noise_std) if self.velocity_noise_std > 0 else self._v1_true
        self.omega = self._omega_true + np.random.normal(0, self.omega_noise_std) if self.omega_noise_std > 0 else self._omega_true
        
        # History storage
        self.current_time = 0.0
        self.time_stamps = [self.current_time]
        self.actual_path_history = [[self.x, self.y, self.theta]] # Measured pose history
        self.true_path_history = [[self._x_true, self._y_true, self._theta_true]] # True pose history
        
        self.torques_commanded_history = [] # Stores [tau_right_cmd, tau_left_cmd]
        self.velocities_actual_history = [[self.v1, self.omega]] # Measured velocity history
        self.true_velocities_history = [[self._v1_true, self._omega_true]] # True velocity history
        self.disturbances_applied_history = [] # Stores [d_Fx, d_Mz] for each step

        # Kick disturbance parameters
        self.kick_active_flag = kick_active
        self.kick_start_time_val = kick_start_time
        self.kick_end_time_val = kick_start_time + kick_duration
        self.kick_magnitude_val = np.array(kick_magnitude)
        self.is_kick_currently_active = False # Tracks if kick is ongoing in current step
        
        # Matrix B_matrix_dynamics: Relates wheel torques [tau_R, tau_L]^T to generalized forces [Fx_body, Mz_body]^T
        # Fx_body = (1/r) * (tau_R + tau_L)
        # Mz_body = (W/(2*r)) * (tau_R - tau_L)
        self.B_matrix_dynamics = np.array([
            [1.0/self.r, 1.0/self.r],
            [self.W/(2.0*self.r), -self.W/(2.0*self.r)]
        ])

    def _apply_disturbances(self):
        """
        Calculates and returns generalized disturbances (d_Fx, d_Mz) for the current time step.
        Includes continuous random disturbances and a discrete kick disturbance.
        """
        d_Fx_cont, d_Mz_cont = 0.0, 0.0
        if isinstance(self.dB_sim_continuous, (list, np.ndarray)) and len(self.dB_sim_continuous) == 2:
            d_Fx_cont = self.dB_sim_continuous[0] * (2 * np.random.rand() - 1)
            d_Mz_cont = self.dB_sim_continuous[1] * (2 * np.random.rand() - 1)
        elif isinstance(self.dB_sim_continuous, (int, float)) and self.dB_sim_continuous > 0:
            d_Fx_cont = self.dB_sim_continuous * (2 * np.random.rand() - 1)
            # Example: Make moment disturbance smaller relative to force disturbance if dB_sim is scalar
            d_Mz_cont = self.dB_sim_continuous * (2 * np.random.rand() - 1) * 0.3 

        d_Fx_kick, d_Mz_kick = 0.0, 0.0
        if self.kick_active_flag:
            if self.kick_start_time_val <= self.current_time < self.kick_end_time_val:
                if not self.is_kick_currently_active: # First step of the kick
                    print(f"SIM INFO: Kick disturbance STARTED at t={self.current_time:.2f}s (duration: {self.kick_end_time_val - self.kick_start_time_val:.2f}s).")
                self.is_kick_currently_active = True
                d_Fx_kick = self.kick_magnitude_val[0]
                d_Mz_kick = self.kick_magnitude_val[1]
            elif self.is_kick_currently_active and self.current_time >= self.kick_end_time_val:
                # Kick just ended in the previous step, or current time marks end
                print(f"SIM INFO: Kick disturbance ENDED at t={self.current_time:.2f}s (was active up to {self.kick_end_time_val:.2f}s).")
                self.is_kick_currently_active = False
        
        total_d_Fx = d_Fx_cont + d_Fx_kick
        total_d_Mz = d_Mz_cont + d_Mz_kick
        
        return total_d_Fx, total_d_Mz

    def execute_commanded_torques(self, tau_right_cmd, tau_left_cmd):
        """
        Updates the robot's true and measured states based on commanded wheel torques and simulation dynamics.
        """
        # Store commands at the beginning of the step they influence
        self.torques_commanded_history.append([tau_right_cmd, tau_left_cmd])

        # Calculate generalized forces from wheel torques
        wheel_torques_vec = np.array([[tau_right_cmd], [tau_left_cmd]])
        Fx_body_torques, Mz_body_torques = (self.B_matrix_dynamics @ wheel_torques_vec).flatten()

        # Get disturbances for the current step
        d_Fx_applied, d_Mz_applied = self._apply_disturbances()
        self.disturbances_applied_history.append([d_Fx_applied, d_Mz_applied])


        # Equations of motion using true mass and inertia
        v1_dot = (Fx_body_torques + d_Fx_applied) / self.m_true
        omega_dot = (Mz_body_torques + d_Mz_applied) / self.I_true

        # Integrate to update true velocities
        self._v1_true += v1_dot * self.dt
        self._omega_true += omega_dot * self.dt
        
        # Integrate to update true pose
        dx_true = self._v1_true * np.cos(self._theta_true) * self.dt
        dy_true = self._v1_true * np.sin(self._theta_true) * self.dt
        dtheta_true = self._omega_true * self.dt

        self._x_true += dx_true
        self._y_true += dy_true
        self._theta_true = (self._theta_true + dtheta_true + np.pi) % (2 * np.pi) - np.pi # Normalize
        
        # Update measured state (apply sensor noise if any)
        self.v1 = self._v1_true + np.random.normal(0, self.velocity_noise_std) if self.velocity_noise_std > 0 else self._v1_true
        self.omega = self._omega_true + np.random.normal(0, self.omega_noise_std) if self.omega_noise_std > 0 else self._omega_true
        
        # For simplicity, assume pose measurement is accurate (or add noise model here)
        self.x, self.y, self.theta = self._x_true, self._y_true, self._theta_true
        
        # Advance simulation time
        self.current_time += self.dt
        
        # Store history for the new state at self.current_time
        self.time_stamps.append(self.current_time)
        self.actual_path_history.append([self.x, self.y, self.theta])
        self.true_path_history.append([self._x_true, self._y_true, self._theta_true])
        self.velocities_actual_history.append([self.v1, self.omega])
        self.true_velocities_history.append([self._v1_true, self._omega_true])


    def get_current_state(self):
        """ Returns the current *measured* state of the robot for the controller. """
        return self.x, self.y, self.theta, self.v1, self.omega

    def get_results(self):
        """
        Returns all collected simulation data.
        Pads torque and disturbance histories to match length of state histories for plotting.
        """
        num_states = len(self.time_stamps) # Number of recorded states (includes initial state)
        num_commands = len(self.torques_commanded_history) # Number of commands issued

        torques_hist = np.array(self.torques_commanded_history)
        if num_commands < num_states -1 : # Should not happen if logic is correct
            print(f"Warning: Mismatch in torque history ({num_commands}) vs state history ({num_states-1} steps). Padding with zeros.")
            torques_hist = np.pad(torques_hist, ((0, num_states - 1 - num_commands), (0,0)), 'constant', constant_values=0)
        if num_commands > 0: # Pad last command to align with last state for plotting
             torques_hist = np.vstack([torques_hist, torques_hist[-1]])
        elif num_states > 0 : # No commands, but states exist (e.g. only initial state)
             torques_hist = np.zeros((num_states, 2))


        disturbances_hist = np.array(self.disturbances_applied_history)
        if len(disturbances_hist) < num_states -1:
            print(f"Warning: Mismatch in disturbance history ({len(disturbances_hist)}) vs state history ({num_states-1} steps). Padding with zeros.")
            disturbances_hist = np.pad(disturbances_hist, ((0, num_states - 1 - len(disturbances_hist)),(0,0)), 'constant', constant_values=0)
        if len(disturbances_hist) > 0:
            disturbances_hist = np.vstack([disturbances_hist, disturbances_hist[-1]])
        elif num_states > 0:
            disturbances_hist = np.zeros((num_states, 2))
            
        return {
            "dt": self.dt, 
            "time": np.array(self.time_stamps),
            "actual_path": np.array(self.actual_path_history), 
            "true_path": np.array(self.true_path_history), 
            "desired_path": self.desired_path,
            "torques_cmd": torques_hist,
            "robot_vels": np.array(self.velocities_actual_history), 
            "true_robot_vels": np.array(self.true_velocities_history), 
            "disturbances_applied": disturbances_hist
        }
