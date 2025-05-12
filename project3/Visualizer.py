import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np

class Visualizer:
    """
    Handles plotting and animation of the robot simulation results.
    Creates a multi-panel figure showing the robot's path, velocities,
    torques, errors, parameter estimates, and disturbances over time.
    """
    def __init__(self, desired_path):
        """
        Initializes the visualizer and sets up the plot structure.

        :param desired_path: A numpy array of shape (N, 2) representing the desired path.
        """
        self.desired_path = desired_path
        self.fig = plt.figure(figsize=(22, 14)) # Adjusted figure size for better layout

        # Define a GridSpec for more control over subplot sizes and spacing
        # 6 rows for time plots, 2 columns (path plot spans all rows in col 0)
        grid = plt.GridSpec(6, 2, height_ratios=[3, 1, 1, 1, 1, 1], hspace=0.9, wspace=0.25)

        # --- Column 0: Path Plot (spans all rows) ---
        self.path_ax = self.fig.add_subplot(grid[:, 0])
        self.path_ax.set_aspect('equal', adjustable='box')
        self.path_ax.set_title("Robot Path Tracking (Adaptive Dynamic Control)")
        self.path_ax.set_xlabel("X-axis (m)")
        self.path_ax.set_ylabel("Y-axis (m)")
        self.path_ax.plot(desired_path[:, 0], desired_path[:, 1], 'r--', label='Desired Path', linewidth=1.5, alpha=0.7)
        self.actual_path_line, = self.path_ax.plot([], [], 'g-', label='Actual Path', linewidth=2.0)
        self.robot_marker, = self.path_ax.plot([], [], 'bo', markersize=10, label="Robot Position", markeredgecolor='k')
        # Robot orientation arrow
        self.robot_orientation_arrow = patches.Arrow(0, 0, 0, 0, width=0.2, color='blue', zorder=5, alpha=0.8)
        self.path_ax.add_patch(self.robot_orientation_arrow)
        # Robot velocity vector arrow (optional, can be noisy for animation)
        self.robot_velocity_arrow = patches.Arrow(0, 0, 0, 0, width=0.1, color='cyan', alpha=0.6, zorder=4)
        self.path_ax.add_patch(self.robot_velocity_arrow)
        self.path_ax.legend(loc='upper right')
        self.path_ax.grid(True, linestyle=':', alpha=0.7)

        # --- Column 1: Time Plots ---
        # Row 0: Robot Velocities (Actual vs Desired)
        self.velocity_ax = self.fig.add_subplot(grid[0, 1])
        self.velocity_ax.set_title("Robot Velocities")
        self.velocity_ax.set_ylabel("Velocity")
        self.v1_actual_line, = self.velocity_ax.plot([], [], 'r-', label="$v_1$ Actual (m/s)")
        self.omega_actual_line, = self.velocity_ax.plot([], [], 'm-', label="$\\omega$ Actual (rad/s)")
        self.v1_desired_line, = self.velocity_ax.plot([], [], 'r:', label="$v_{1d}$ Desired", linewidth=1.5)
        self.omega_desired_line, = self.velocity_ax.plot([], [], 'm:', label="$\\omega_d$ Desired", linewidth=1.5)
        self.velocity_ax.legend(fontsize='small', loc='best')
        self.velocity_ax.grid(True, linestyle=':', alpha=0.7)
        self.velocity_ax.tick_params(axis='x', labelbottom=False) # Hide x-axis labels for stacked plots

        # Row 1: Control Torques
        self.torque_ax = self.fig.add_subplot(grid[1, 1], sharex=self.velocity_ax) # Share x-axis
        self.torque_ax.set_title("Control Torques (Commanded)")
        self.torque_ax.set_ylabel("Torque (Nm)")
        self.tau_left_line, = self.torque_ax.plot([], [], 'b-', label="$\\tau_L$ Left")
        self.tau_right_line, = self.torque_ax.plot([], [], 'g-', label="$\\tau_R$ Right")
        self.torque_ax.legend(fontsize='small', loc='best')
        self.torque_ax.grid(True, linestyle=':', alpha=0.7)
        self.torque_ax.tick_params(axis='x', labelbottom=False)

        # Row 2: Kinematic Errors (from Kinematic Controller)
        self.kin_error_ax = self.fig.add_subplot(grid[2, 1], sharex=self.velocity_ax)
        self.kin_error_ax.set_title("Kinematic Errors (Path Following Layer)")
        self.kin_error_ax.set_ylabel("Error Value")
        self.error_fwd_line, = self.kin_error_ax.plot([], [], 'c-', label="Forward Err (m)")
        self.error_lat_line, = self.kin_error_ax.plot([], [], 'y-', label="Lateral Err (m)")
        self.error_theta_line, = self.kin_error_ax.plot([], [], 'k-', label="$\\theta$ Err (rad)")
        self.kin_error_ax.legend(fontsize='small', loc='best')
        self.kin_error_ax.grid(True, linestyle=':', alpha=0.7)
        self.kin_error_ax.tick_params(axis='x', labelbottom=False)

        # Row 3: Velocity Errors (eta = v_actual - v_desired)
        self.vel_error_ax = self.fig.add_subplot(grid[3, 1], sharex=self.velocity_ax)
        self.vel_error_ax.set_title("Velocity Errors ($\\eta = v - v_d$)")
        self.vel_error_ax.set_ylabel("Velocity Error")
        self.eta1_line, = self.vel_error_ax.plot([], [], 'r-', label="$\\eta_1 = v_1 - v_{1d}$")
        self.eta2_line, = self.vel_error_ax.plot([], [], 'm-', label="$\\eta_2 = \\omega - \\omega_d$")
        self.vel_error_ax.legend(fontsize='small', loc='best')
        self.vel_error_ax.grid(True, linestyle=':', alpha=0.7)
        self.vel_error_ax.tick_params(axis='x', labelbottom=False)

        # Row 4: Parameter Estimates (p_hat)
        self.param_ax = self.fig.add_subplot(grid[4, 1], sharex=self.velocity_ax)
        self.param_ax.set_title("Parameter Estimates ($\\hat{p}$)")
        self.param_ax.set_ylabel("Estimated Value")
        self.m_hat_line, = self.param_ax.plot([], [], 'b-', label="$\\hat{m}$ (mass est.)")
        self.I_hat_line, = self.param_ax.plot([], [], 'g-', label="$\\hat{I}$ (inertia est.)")
        self.m_true_line, = self.param_ax.plot([], [], 'b--', label="$m$ True", alpha=0.6)
        self.I_true_line, = self.param_ax.plot([], [], 'g--', label="$I$ True", alpha=0.6)
        self.param_ax.legend(fontsize='small', loc='best')
        self.param_ax.grid(True, linestyle=':', alpha=0.7)
        self.param_ax.tick_params(axis='x', labelbottom=False)

        # Row 5: Applied Disturbances
        self.dist_ax = self.fig.add_subplot(grid[5, 1], sharex=self.velocity_ax)
        self.dist_ax.set_title("Applied Disturbances (Effective Torque on Dynamics)")
        self.dist_ax.set_xlabel("Time (s)") # X-label only on the bottom-most plot
        self.dist_ax.set_ylabel("Disturbance")
        self.dist1_line, = self.dist_ax.plot([], [], 'r-', label="$\\tau_{d,v1}$ (on $v_1$ axis)")
        self.dist2_line, = self.dist_ax.plot([], [], 'm-', label="$\\tau_{d,\\omega}$ (on $\\omega$ axis)")
        self.dist_ax.legend(fontsize='small', loc='best')
        self.dist_ax.grid(True, linestyle=':', alpha=0.7)

        plt.tight_layout(rect=[0, 0, 1, 0.98]) # Adjust layout to prevent title overlap
        
        # Animation related attributes
        self.sim_data = None
        self.controller_status_list = None
        self.anim_step = 1 # Number of simulation steps per animation frame
        self.true_m = None # True mass for plotting
        self.true_I = None # True inertia for plotting
        self.artists_to_animate = [] # List of artists to be updated in animation

    def _init_animation(self):
        """ Initializes elements for animation drawing. Called once at the start of animation. """
        # Combine all line artists that need to be reset
        self.artists_to_animate = [
            self.actual_path_line, self.robot_marker,
            self.v1_actual_line, self.omega_actual_line, self.v1_desired_line, self.omega_desired_line,
            self.tau_left_line, self.tau_right_line,
            self.error_fwd_line, self.error_lat_line, self.error_theta_line,
            self.eta1_line, self.eta2_line,
            self.m_hat_line, self.I_hat_line,
            self.m_true_line, self.I_true_line, # True parameter lines also need to be set
            self.dist1_line, self.dist2_line
        ]
        for artist in self.artists_to_animate:
            if isinstance(artist, plt.Line2D):
                 artist.set_data([], [])
        
        # Reset arrows (remove and re-add to handle blitting correctly if it were used)
        if self.robot_orientation_arrow.axes: self.robot_orientation_arrow.remove()
        if self.robot_velocity_arrow.axes: self.robot_velocity_arrow.remove()

        self.robot_orientation_arrow = patches.Arrow(0, 0, 0.01, 0, width=0.2, color='blue', zorder=5, alpha=0.8) # Small initial arrow
        self.robot_velocity_arrow = patches.Arrow(0, 0, 0, 0, width=0.1, color='cyan', alpha=0.6, zorder=4)
        self.path_ax.add_patch(self.robot_orientation_arrow)
        self.path_ax.add_patch(self.robot_velocity_arrow)
        self.artists_to_animate.extend([self.robot_orientation_arrow, self.robot_velocity_arrow]) # Add arrows to artist list

        # Set dynamic plot limits based on the full dataset
        max_time = self.sim_data["time"][-1] if len(self.sim_data["time"]) > 0 else 1.0

        # Extract data for calculating y-limits
        num_controller_pts = len(self.controller_status_list)
        vels_actual = self.sim_data["robot_vels"]
        vels_desired = np.array([s['v_d'] for s in self.controller_status_list]) if num_controller_pts > 0 else np.zeros((0,2))
        
        # Ensure torques and disturbances match controller status length for plotting
        torques = self.sim_data["torques_cmd"][1:num_controller_pts+1]
        disturbances = self.sim_data["disturbances"][1:num_controller_pts+1]
        
        kin_errors = np.array([s['kin_errors'] for s in self.controller_status_list]) if num_controller_pts > 0 else np.zeros((0,3))
        vel_errors_eta = np.array([s['eta'] for s in self.controller_status_list]) if num_controller_pts > 0 else np.zeros((0,2))
        params_hat = np.array([s['p_hat'] for s in self.controller_status_list]) if num_controller_pts > 0 else np.zeros((0,2))

        # Helper function to get robust y-limits
        def get_ylims(data_list, pad_factor=0.1, zero_center=False):
            min_val, max_val = np.inf, -np.inf
            has_data = False
            for data_arr in data_list:
                if data_arr is not None and data_arr.size > 0:
                    finite_data = data_arr[np.isfinite(data_arr)]
                    if finite_data.size > 0:
                        has_data = True
                        min_val = min(min_val, np.min(finite_data))
                        max_val = max(max_val, np.max(finite_data))
            if not has_data: return (-0.1, 0.1)
            if np.isclose(min_val, max_val): min_val -= 0.1; max_val += 0.1 # Handle flat lines
            
            padding = abs(max_val - min_val) * pad_factor
            if not np.isfinite(padding) or padding == 0: padding = abs(max_val)*pad_factor if np.isfinite(max_val) and max_val !=0 else 0.1

            lim_min, lim_max = min_val - padding, max_val + padding
            if zero_center:
                abs_max_val = max(abs(lim_min), abs(lim_max))
                if not np.isfinite(abs_max_val): abs_max_val = 1.0
                lim_min, lim_max = -abs_max_val, abs_max_val
            if lim_max <= lim_min: lim_max = lim_min + 0.2 # Ensure positive range
            return lim_min, lim_max

        # Set y-limits for each time plot
        self.velocity_ax.set_ylim(get_ylims([vels_actual, vels_desired], zero_center=True))
        self.torque_ax.set_ylim(get_ylims([torques], zero_center=True))
        self.kin_error_ax.set_ylim(get_ylims([kin_errors], zero_center=True))
        self.vel_error_ax.set_ylim(get_ylims([vel_errors_eta], zero_center=True))
        
        param_plot_data = [params_hat]
        if self.true_m is not None: param_plot_data.append(np.array([[self.true_m, self.true_I]])) # Add true values for limit calc
        param_lims = get_ylims(param_plot_data, pad_factor=0.2)
        if param_lims[0] > -0.1 and param_lims[0] < 0.1 : param_lims = (min(param_lims[0], -0.1), param_lims[1]) # Ensure origin is visible if params are positive
        self.param_ax.set_ylim(param_lims)

        self.dist_ax.set_ylim(get_ylims([disturbances], zero_center=True))

        # Set x-limits for all time plots
        for ax in [self.velocity_ax, self.torque_ax, self.kin_error_ax, self.vel_error_ax, self.param_ax, self.dist_ax]:
            ax.set_xlim(0, max_time)

        # Adjust path plot limits dynamically based on actual and desired paths
        all_x_coords = np.concatenate((self.desired_path[:, 0], self.sim_data["actual_path"][:, 0]))
        all_y_coords = np.concatenate((self.desired_path[:, 1], self.sim_data["actual_path"][:, 1]))
        self.path_ax.set_xlim(get_ylims([all_x_coords]))
        self.path_ax.set_ylim(get_ylims([all_y_coords]))

        # Set true parameter lines if available (these are static once set)
        if self.true_m is not None:
            self.m_true_line.set_data([0, max_time], [self.true_m, self.true_m])
        if self.true_I is not None:
            self.I_true_line.set_data([0, max_time], [self.true_I, self.true_I])
        
        self.path_ax.set_title(f"Robot Path Tracking (T=0.00s)") # Initial title
        return self.artists_to_animate

    def _animate(self, frame_idx):
        """ Animation update function. Called for each frame. """
        # Calculate simulation index based on animation frame and step
        sim_idx = min(frame_idx * self.anim_step, len(self.sim_data["time"]) - 1)
        # Controller data often corresponds to the command *for* the interval ending at sim_idx+1,
        # or status *at* sim_idx. Let's use status at sim_idx, and command for interval sim_idx to sim_idx+1.
        # So, controller_status_list[sim_idx] is status at time[sim_idx].
        # torques_cmd[sim_idx+1] is command applied during time[sim_idx] to time[sim_idx+1].
        controller_data_idx = sim_idx # Index for status lists (p_hat, eta, kin_errors, v_d)
        
        # Data slices for plotting up to current simulation time
        time_slice = self.sim_data["time"][:sim_idx+1]
        actual_path_slice = self.sim_data["actual_path"][:sim_idx+1]
        vels_actual_slice = self.sim_data["robot_vels"][:sim_idx+1]
        
        # For torques and disturbances, we plot up to the command/disturbance that *caused* the state at sim_idx+1
        # So, if sim_idx is current state time, torques_cmd[sim_idx+1] is relevant.
        # Time axis for these should be time_slice[1:] if data is N long for N+1 time points.
        # Or, if data is N+1 long (like disturbances_applied), use time_slice.
        
        # Ensure controller_status_list is long enough
        if controller_data_idx < len(self.controller_status_list):
            current_status = self.controller_status_list[controller_data_idx]
            
            # Slices for controller-derived data (up to controller_data_idx)
            time_controller_slice = self.sim_data["time"][:controller_data_idx+1] # Time for p_hat, eta etc.

            vels_desired_plot_data = np.array([s['v_d'] for s in self.controller_status_list[:controller_data_idx+1]])
            kin_errors_plot_data = np.array([s['kin_errors'] for s in self.controller_status_list[:controller_data_idx+1]])
            vel_errors_eta_plot_data = np.array([s['eta'] for s in self.controller_status_list[:controller_data_idx+1]])
            params_hat_plot_data = np.array([s['p_hat'] for s in self.controller_status_list[:controller_data_idx+1]])

            # Torques and disturbances are commanded/applied for the *next* interval
            # So torques_cmd[k] is applied from t[k-1] to t[k].
            # For plotting at time t[sim_idx], we need torques up to command torques_cmd[sim_idx]
            # The time axis for these is time_slice for torques_cmd[0...sim_idx]
            # However, torques_cmd[0] is often a placeholder.
            # Let's plot torques_cmd[1...sim_idx+1] against time[1...sim_idx+1]
            time_for_cmds_dist = self.sim_data["time"][1:sim_idx+2] # Correct time axis for N commands over N intervals
            torques_plot_data = self.sim_data["torques_cmd"][1:sim_idx+2]
            disturbances_plot_data = self.sim_data["disturbances"][1:sim_idx+2]


            # Update Path Plot
            current_pose = actual_path_slice[-1]
            x, y, theta = current_pose
            self.actual_path_line.set_data(actual_path_slice[:, 0], actual_path_slice[:, 1])
            self.robot_marker.set_data([x], [y])

            # Update Robot Orientation Arrow
            arrow_length = max(0.2, np.mean(np.abs(self.path_ax.get_xlim())) * 0.03) # Dynamic arrow size
            if self.robot_orientation_arrow.axes: self.robot_orientation_arrow.remove()
            self.robot_orientation_arrow = patches.Arrow(x, y, arrow_length * np.cos(theta), arrow_length * np.sin(theta),
                                                     width=arrow_length*0.5, color='blue', zorder=5, alpha=0.8)
            self.path_ax.add_patch(self.robot_orientation_arrow)

            # Update Robot Velocity Arrow (optional)
            v1_current = vels_actual_slice[-1,0]
            vel_arrow_scale = arrow_length * 0.8 # Scale velocity arrow
            if self.robot_velocity_arrow.axes: self.robot_velocity_arrow.remove()
            self.robot_velocity_arrow = patches.Arrow(x, y, v1_current * vel_arrow_scale * np.cos(theta), v1_current * vel_arrow_scale * np.sin(theta),
                                                    width=arrow_length*0.25, color='cyan', alpha=0.6, zorder=4)
            self.path_ax.add_patch(self.robot_velocity_arrow)
            
            # Update Time Plots
            self.v1_actual_line.set_data(time_slice, vels_actual_slice[:, 0])
            self.omega_actual_line.set_data(time_slice, vels_actual_slice[:, 1])
            
            if vels_desired_plot_data.shape[0] == time_controller_slice.shape[0]:
                self.v1_desired_line.set_data(time_controller_slice, vels_desired_plot_data[:, 0])
                self.omega_desired_line.set_data(time_controller_slice, vels_desired_plot_data[:, 1])

            if torques_plot_data.shape[0] == time_for_cmds_dist.shape[0]:
                self.tau_left_line.set_data(time_for_cmds_dist, torques_plot_data[:, 0])
                self.tau_right_line.set_data(time_for_cmds_dist, torques_plot_data[:, 1])

            if kin_errors_plot_data.shape[0] == time_controller_slice.shape[0]:
                self.error_fwd_line.set_data(time_controller_slice, kin_errors_plot_data[:, 0])
                self.error_lat_line.set_data(time_controller_slice, kin_errors_plot_data[:, 1])
                self.error_theta_line.set_data(time_controller_slice, kin_errors_plot_data[:, 2])

            if vel_errors_eta_plot_data.shape[0] == time_controller_slice.shape[0]:
                self.eta1_line.set_data(time_controller_slice, vel_errors_eta_plot_data[:, 0])
                self.eta2_line.set_data(time_controller_slice, vel_errors_eta_plot_data[:, 1])

            if params_hat_plot_data.shape[0] == time_controller_slice.shape[0]:
                self.m_hat_line.set_data(time_controller_slice, params_hat_plot_data[:, 0])
                self.I_hat_line.set_data(time_controller_slice, params_hat_plot_data[:, 1])
            
            if disturbances_plot_data.shape[0] == time_for_cmds_dist.shape[0]:
                self.dist1_line.set_data(time_for_cmds_dist, disturbances_plot_data[:, 0])
                self.dist2_line.set_data(time_for_cmds_dist, disturbances_plot_data[:, 1])

        self.path_ax.set_title(f"Robot Path Tracking (T={time_slice[-1]:.2f}s)")
        
        # Update the list of artists to be returned for blitting (though blit=False is used)
        self.artists_to_animate = [
            self.actual_path_line, self.robot_marker, self.robot_orientation_arrow, self.robot_velocity_arrow,
            self.v1_actual_line, self.omega_actual_line, self.v1_desired_line, self.omega_desired_line,
            self.tau_left_line, self.tau_right_line,
            self.error_fwd_line, self.error_lat_line, self.error_theta_line,
            self.eta1_line, self.eta2_line,
            self.m_hat_line, self.I_hat_line,
            self.dist1_line, self.dist2_line
        ]
        # True param lines are static so don't need to be in artists_to_animate after _init_animation
        return self.artists_to_animate

    def create_animation(self, simulation_data, controller_status_list, interval=50, step=10, true_m=None, true_I=None):
        """
        Creates the animation object.

        :param simulation_data: Dictionary of simulation history from Simulation.get_simulation_data().
        :param controller_status_list: List of status dictionaries from the controller.
        :param interval: Delay between frames in milliseconds.
        :param step: Number of simulation steps per animation frame.
        :param true_m: True mass of the robot (for plotting reference).
        :param true_I: True inertia of the robot (for plotting reference).
        :return: matplotlib.animation.FuncAnimation object.
        """
        self.sim_data = simulation_data
        # Ensure controller_status_list length matches number of actual control steps taken
        num_sim_intervals = len(simulation_data['time']) -1 # Number of steps/intervals is one less than time points
        self.controller_status_list = controller_status_list[:num_sim_intervals]
        
        if len(self.controller_status_list) != num_sim_intervals :
             print(f"Warning: Length mismatch in create_animation. Status list {len(self.controller_status_list)}, Sim intervals {num_sim_intervals}")
             # This can happen if simulation stops early. Trim controller_status_list to match.
             self.controller_status_list = controller_status_list[:num_sim_intervals]


        self.anim_step = step
        self.true_m = true_m
        self.true_I = true_I

        num_total_sim_points = len(self.sim_data["time"])
        # Calculate number of frames needed to show all simulation points
        num_animation_frames = (num_total_sim_points -1) // self.anim_step + 1

        # Create animation. blit=False is generally safer when dealing with complex artists like Patches (Arrows).
        ani = animation.FuncAnimation(self.fig, self._animate, frames=num_animation_frames,
                                      init_func=self._init_animation, blit=False,
                                      interval=interval, repeat=False)
        return ani

    def plot_final_results(self, simulation_data, controller_status_list, path_type_name="", true_m=None, true_I=None):
        """
        Generates and shows static plots of key simulation variables vs. time.
        This is useful for a final overview after the simulation or animation.

        :param simulation_data: Dictionary of simulation history.
        :param controller_status_list: List of status dictionaries from the controller.
        :param path_type_name: Name of the path type for the plot title.
        :param true_m: True mass of the robot.
        :param true_I: True inertia of the robot.
        """
        # Create a new figure for static plots to avoid conflicts with animation figure
        fig_static, axs_static = plt.subplots(5, 1, figsize=(12, 18), sharex=True)
        fig_static.suptitle(f"Adaptive Control Simulation Results - {path_type_name} Path", fontsize=16)

        time_data = simulation_data["time"]
        num_control_outputs = len(controller_status_list)

        if num_control_outputs == 0:
             print("No controller status data to plot for final results.")
             return

        # Time axis for controller data (status at t_k)
        time_controller_axis = time_data[:num_control_outputs]
        
        # Time axis for commands/disturbances (applied over interval t_k to t_{k+1}, plotted at t_{k+1})
        time_command_axis = time_data[1:num_control_outputs+1]


        # Extract controller data safely, matching the length of time_controller_axis
        kin_errors = np.array([s['kin_errors'] for s in controller_status_list])
        vel_errors_eta = np.array([s['eta'] for s in controller_status_list])
        params_hat = np.array([s['p_hat'] for s in controller_status_list])

        # Torques and disturbances (match length of time_command_axis)
        torques_cmd = simulation_data["torques_cmd"][1:num_control_outputs+1]
        disturbances_applied = simulation_data["disturbances"][1:num_control_outputs+1]
        
        # Plot Kinematic Errors
        axs_static[0].plot(time_controller_axis, kin_errors[:, 0], 'c-', label="Forward Err (m)")
        axs_static[0].plot(time_controller_axis, kin_errors[:, 1], 'y-', label="Lateral Err (m)")
        axs_static[0].plot(time_controller_axis, kin_errors[:, 2], 'k-', label="$\\theta$ Err (rad)")
        axs_static[0].set_title("Kinematic Errors vs. Time")
        axs_static[0].set_ylabel("Error Value")
        axs_static[0].legend(fontsize='small')
        axs_static[0].grid(True, linestyle=':', alpha=0.7)

        # Plot Velocity Errors (eta)
        axs_static[1].plot(time_controller_axis, vel_errors_eta[:, 0], 'r-', label="$\\eta_1 = v_1 - v_{1d}$")
        axs_static[1].plot(time_controller_axis, vel_errors_eta[:, 1], 'm-', label="$\\eta_2 = \\omega - \\omega_d$")
        axs_static[1].set_title("Velocity Errors ($\\eta$) vs. Time")
        axs_static[1].set_ylabel("Velocity Error")
        axs_static[1].legend(fontsize='small')
        axs_static[1].grid(True, linestyle=':', alpha=0.7)

        # Plot Parameter Estimates
        axs_static[2].plot(time_controller_axis, params_hat[:, 0], 'b-', label="$\\hat{m}$ (mass est.)")
        axs_static[2].plot(time_controller_axis, params_hat[:, 1], 'g-', label="$\\hat{I}$ (inertia est.)")
        if true_m is not None: axs_static[2].axhline(true_m, color='b', linestyle='--', alpha=0.6, label="$m$ True")
        if true_I is not None: axs_static[2].axhline(true_I, color='g', linestyle='--', alpha=0.6, label="$I$ True")
        axs_static[2].set_title("Parameter Estimates vs. Time")
        axs_static[2].set_ylabel("Estimated Value")
        axs_static[2].legend(fontsize='small')
        axs_static[2].grid(True, linestyle=':', alpha=0.7)

        # Plot Torques
        if torques_cmd.shape[0] == time_command_axis.shape[0] and torques_cmd.size > 0 :
             axs_static[3].plot(time_command_axis, torques_cmd[:, 0], 'b-', label="$\\tau_L$ Left Cmd")
             axs_static[3].plot(time_command_axis, torques_cmd[:, 1], 'g-', label="$\\tau_R$ Right Cmd")
        axs_static[3].set_title("Commanded Control Torques vs. Time")
        axs_static[3].set_ylabel("Torque (Nm)")
        axs_static[3].legend(fontsize='small')
        axs_static[3].grid(True, linestyle=':', alpha=0.7)

        # Plot Applied Disturbances
        if disturbances_applied.shape[0] == time_command_axis.shape[0] and disturbances_applied.size > 0:
             axs_static[4].plot(time_command_axis, disturbances_applied[:, 0], 'r-', label="$\\tau_{d,v1}$ (on $v_1$ axis)")
             axs_static[4].plot(time_command_axis, disturbances_applied[:, 1], 'm-', label="$\\tau_{d,\\omega}$ (on $\\omega$ axis)")
        axs_static[4].set_title("Applied Disturbances vs. Time")
        axs_static[4].set_xlabel("Time (s)")
        axs_static[4].set_ylabel("Disturbance Torque")
        axs_static[4].legend(fontsize='small')
        axs_static[4].grid(True, linestyle=':', alpha=0.7)

        fig_static.tight_layout(rect=[0, 0.03, 1, 0.96]) # Adjust layout
        # plt.show() # Optionally show this plot if not run in a script that exits
