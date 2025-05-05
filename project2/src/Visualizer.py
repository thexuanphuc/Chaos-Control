import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np

class Visualizer:
    def __init__(self, desired_path):
        """
        Initialize the visualizer. Sets up the plot structure with improved layout and labels.
        """
        self.desired_path = desired_path
        # Use constrained_layout for better automatic spacing
        self.fig = plt.figure(figsize=(18, 10), constrained_layout=True)

        # Create a grid for subplots: 6 rows, 2 columns
        # Column 0 spans all rows for the main path plot
        # Column 1 has 6 smaller plots for time-series data
        gs = self.fig.add_gridspec(6, 2, width_ratios=[3, 2]) # Adjust width ratio

        # --- Column 0: Path Plot ---
        self.path_ax = self.fig.add_subplot(gs[:, 0])
        self.path_ax.set_aspect('equal', adjustable='box')
        self.path_ax.set_title("Robot Path Tracking")
        self.path_ax.set_xlabel("X Position (m)")
        self.path_ax.set_ylabel("Y Position (m)")
        self.path_ax.plot(desired_path[:, 0], desired_path[:, 1], 'r--', label='Desired Path', linewidth=1.5, alpha=0.7)
        self.actual_path_line, = self.path_ax.plot([], [], 'g-', label='Actual Path', linewidth=1.5)
        # Use a more distinct marker for the robot
        self.robot_marker, = self.path_ax.plot([], [], marker='^', color='blue', markersize=9, label="Robot Position & Orientation")
        # Note: Robot orientation is shown by the marker '^' and the blue arrow
        self.robot_arrow = patches.Arrow(0, 0, 0, 0, width=0.2, color='blue', zorder=5, alpha=0.8) # Orientation
        self.path_ax.add_patch(self.robot_arrow)
        self.velocity_arrow = patches.Arrow(0, 0, 0, 0, width=0.1, color='orange', alpha=0.7, zorder=4, label="Velocity Vector (Scaled)") # Velocity
        self.path_ax.add_patch(self.velocity_arrow)
        # Place legend outside the main plot area if possible, or use 'best'
        self.path_ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1), borderaxespad=0.)
        self.path_ax.grid(True, linestyle=':', alpha=0.6)

        # --- Column 1: Time Plots ---
        # Shared X-axis for all time plots
        time_axes = []

        # 1. Robot Velocities (Actual vs Desired)
        self.velocity_ax = self.fig.add_subplot(gs[0, 1])
        time_axes.append(self.velocity_ax)
        self.velocity_ax.set_title("Velocities")
        self.velocity_ax.set_ylabel("Value")
        self.v1_actual_line, = self.velocity_ax.plot([], [], 'r-', label="$v_1$ Actual (m/s)")
        self.omega_actual_line, = self.velocity_ax.plot([], [], 'm-', label="$\\omega$ Actual (rad/s)")
        self.v1_desired_line, = self.velocity_ax.plot([], [], 'r:', label="$v_{1d}$ Desired (m/s)")
        self.omega_desired_line, = self.velocity_ax.plot([], [], 'm:', label="$\\omega_d$ Desired (rad/s)")
        self.velocity_ax.legend(fontsize='small', ncol=2) # Use 2 columns for legend
        self.velocity_ax.grid(True, linestyle=':', alpha=0.6)
        self.velocity_ax.tick_params(axis='x', labelbottom=False)

        # 2. Control Torques
        self.torque_ax = self.fig.add_subplot(gs[1, 1], sharex=self.velocity_ax)
        time_axes.append(self.torque_ax)
        self.torque_ax.set_title("Control Torques")
        self.torque_ax.set_ylabel("Torque (Nm)")
        self.tau_left_line, = self.torque_ax.plot([], [], 'b-', label="$\\tau_L$ Left Cmd")
        self.tau_right_line, = self.torque_ax.plot([], [], 'g-', label="$\\tau_R$ Right Cmd")
        self.torque_ax.legend(fontsize='small')
        self.torque_ax.grid(True, linestyle=':', alpha=0.6)
        self.torque_ax.tick_params(axis='x', labelbottom=False)

        # 3. Kinematic Errors (Path Following)
        self.kin_error_ax = self.fig.add_subplot(gs[2, 1], sharex=self.velocity_ax)
        time_axes.append(self.kin_error_ax)
        self.kin_error_ax.set_title("Kinematic Errors (Path Following)")
        self.kin_error_ax.set_ylabel("Error Value")
        self.error_fwd_line, = self.kin_error_ax.plot([], [], 'c-', label="Forward Error (m)")
        self.error_lat_line, = self.kin_error_ax.plot([], [], 'y-', label="Lateral Error (m)")
        self.error_theta_line, = self.kin_error_ax.plot([], [], 'k-', label="Angle Error $\\Delta\\theta$ (rad)")
        self.kin_error_ax.legend(fontsize='small')
        self.kin_error_ax.grid(True, linestyle=':', alpha=0.6)
        self.kin_error_ax.tick_params(axis='x', labelbottom=False)

        # 4. Velocity Errors (Tracking)
        self.vel_error_ax = self.fig.add_subplot(gs[3, 1], sharex=self.velocity_ax)
        time_axes.append(self.vel_error_ax)
        self.vel_error_ax.set_title("Velocity Tracking Errors ($\\eta = v - v_d$)")
        self.vel_error_ax.set_ylabel("Velocity Error")
        self.eta1_line, = self.vel_error_ax.plot([], [], 'r-', label="$\\eta_1 = v_1 - v_{1d}$ (m/s)")
        self.eta2_line, = self.vel_error_ax.plot([], [], 'm-', label="$\\eta_2 = \\omega - \\omega_d$ (rad/s)")
        self.vel_error_ax.legend(fontsize='small')
        self.vel_error_ax.grid(True, linestyle=':', alpha=0.6)
        self.vel_error_ax.tick_params(axis='x', labelbottom=False)

        # 5. Parameter Estimates
        self.param_ax = self.fig.add_subplot(gs[4, 1], sharex=self.velocity_ax)
        time_axes.append(self.param_ax)
        self.param_ax.set_title("Parameter Estimates ($\\hat{p}$)")
        self.param_ax.set_ylabel("Estimated Value")
        self.m_hat_line, = self.param_ax.plot([], [], 'b-', label="$\\hat{m}$ Estimate (kg)")
        self.I_hat_line, = self.param_ax.plot([], [], 'g-', label="$\\hat{I}$ Estimate (kg m$^2$)")
        self.m_true_line, = self.param_ax.plot([], [], 'b--', label="$m$ True Value", alpha=0.6)
        self.I_true_line, = self.param_ax.plot([], [], 'g--', label="$I$ True Value", alpha=0.6)
        self.param_ax.legend(fontsize='small', ncol=2) # 2 columns
        self.param_ax.grid(True, linestyle=':', alpha=0.6)
        self.param_ax.tick_params(axis='x', labelbottom=False)

        # 6. Disturbances
        self.dist_ax = self.fig.add_subplot(gs[5, 1], sharex=self.velocity_ax)
        time_axes.append(self.dist_ax)
        self.dist_ax.set_title("Applied Disturbances (Effective Torque)")
        self.dist_ax.set_xlabel("Time (s)") # Only the last plot needs the x-label
        self.dist_ax.set_ylabel("Disturbance")
        self.dist1_line, = self.dist_ax.plot([], [], 'r-', label="$\\tau_{d,v1}$ (Linear Eq.)")
        self.dist2_line, = self.dist_ax.plot([], [], 'm-', label="$\\tau_{d,\\omega}$ (Angular Eq.)")
        self.dist_ax.legend(fontsize='small')
        self.dist_ax.grid(True, linestyle=':', alpha=0.6)

        # Add a main title to the figure
        self.fig.suptitle("Adaptive Dynamic Control Simulation Analysis", fontsize=16, weight='bold')

        # Animation related attributes
        self.sim_data = None
        self.controller_status_list = None
        self.anim_step = 1
        self.true_m = None
        self.true_I = None
        self.artists = [] # Store artists for blitting

    def _get_lims(self, data, pad_factor=0.1, zero_center=False):
        """ Helper function to calculate plot limits safely. """
        if not isinstance(data, np.ndarray): data = np.array(data)
        if data.size == 0: return (-0.1, 0.1)
        finite_data = data[np.isfinite(data)]
        if finite_data.size == 0: return (-0.1, 0.1)
        min_val, max_val = np.min(finite_data), np.max(finite_data)
        if np.isclose(min_val, max_val): min_val -= 0.1; max_val += 0.1
        data_range = max_val - min_val
        if not np.isfinite(data_range): data_range = abs(max_val)*0.2 if np.isfinite(max_val) else 1.0
        pad = data_range * pad_factor
        lim_min, lim_max = min_val - pad, max_val + pad
        if zero_center:
            abs_max = max(abs(lim_min), abs(lim_max))
            if not np.isfinite(abs_max): abs_max = 1.0
            lim_min, lim_max = -abs_max, abs_max
        # Ensure a minimal range
        if np.isclose(lim_max, lim_min): lim_max = lim_min + 0.1
        elif lim_max < lim_min: lim_max = lim_min + 0.1 # Handle potential inversion
        return lim_min, lim_max

    def _init_animation(self):
        """ Initialize elements for animation drawing """
        # Combine all line artists for easier reset
        line_artists = [
            self.actual_path_line, self.robot_marker,
            self.v1_actual_line, self.omega_actual_line, self.v1_desired_line, self.omega_desired_line,
            self.tau_left_line, self.tau_right_line,
            self.error_fwd_line, self.error_lat_line, self.error_theta_line,
            self.eta1_line, self.eta2_line,
            self.m_hat_line, self.I_hat_line,
            self.m_true_line, self.I_true_line,
            self.dist1_line, self.dist2_line
        ]
        for line in line_artists:
            line.set_data([], [])

        # Reset arrows by removing and re-adding (simpler for non-blit animation)
        if hasattr(self.robot_arrow, 'axes') and self.robot_arrow.axes: self.robot_arrow.remove()
        if hasattr(self.velocity_arrow, 'axes') and self.velocity_arrow.axes: self.velocity_arrow.remove()

        # Recreate arrows at origin for init state
        self.robot_arrow = patches.Arrow(0, 0, 0.01, 0, width=0.2, color='blue', zorder=5, alpha=0.8)
        self.velocity_arrow = patches.Arrow(0, 0, 0, 0, width=0.1, color='orange', alpha=0.7, zorder=4)
        self.path_ax.add_patch(self.robot_arrow)
        self.path_ax.add_patch(self.velocity_arrow)

        # Store all artists that need updating (lines + patches)
        self.artists = line_artists + [self.robot_arrow, self.velocity_arrow]

        # --- Set dynamic plot limits based on full data ---
        max_time = self.sim_data["time"][-1] if len(self.sim_data["time"]) > 0 else 1.0

        # Extract data slices, handling potential early termination
        num_steps_completed = len(self.controller_status_list)
        num_time_points = len(self.sim_data['time'])
        valid_controller_indices = slice(0, num_steps_completed)
        valid_sim_indices = slice(0, num_time_points)
        valid_command_indices = slice(1, num_steps_completed + 1) # For torques/disturbances

        vels_actual = self.sim_data["robot_vels"][valid_sim_indices]
        vels_desired = np.array([s['v_d'] for s in self.controller_status_list]) if self.controller_status_list else np.zeros((0,2))
        all_vels_for_lim = np.vstack((vels_actual, vels_desired)) if vels_desired.size > 0 else vels_actual

        torques = self.sim_data["torques_cmd"][valid_command_indices]
        kin_errors = np.array([s['kin_errors'] for s in self.controller_status_list]) if self.controller_status_list else np.zeros((0,3))
        vel_errors = np.array([s['eta'] for s in self.controller_status_list]) if self.controller_status_list else np.zeros((0,2))
        params_hat = np.array([s['p_hat'] for s in self.controller_status_list]) if self.controller_status_list else np.zeros((0,2))
        disturbances = self.sim_data["disturbances"][valid_command_indices]

        # Calculate limits using the helper function
        vel_lim = self._get_lims(all_vels_for_lim, zero_center=False) # Velocities can be non-zero centered
        torque_lim = self._get_lims(torques, zero_center=True)
        kin_err_lim = self._get_lims(kin_errors, zero_center=True)
        vel_err_lim = self._get_lims(vel_errors, zero_center=True)

        param_data_for_lims = params_hat
        true_params_exist = self.true_m is not None and self.true_I is not None
        if true_params_exist and params_hat.size > 0:
             # Include true values in limit calculation if they exist
             true_vals = np.array([[self.true_m, self.true_I]])
             param_data_for_lims = np.vstack((params_hat, true_vals))
        param_lim = self._get_lims(param_data_for_lims, pad_factor=0.2)
        # Ensure lower bound of params is at least slightly below zero if estimates are positive
        if param_lim[0] > -0.1: param_lim = (-0.1, param_lim[1])

        dist_lim = self._get_lims(disturbances, zero_center=True)

        # Apply limits to time-series plots
        axes = [self.velocity_ax, self.torque_ax, self.kin_error_ax, self.vel_error_ax, self.param_ax, self.dist_ax]
        lims = [vel_lim, torque_lim, kin_err_lim, vel_err_lim, param_lim, dist_lim]
        for ax, lim in zip(axes, lims):
             ax.set_xlim(0, max_time)
             ax.set_ylim(lim)

        # Adjust path plot limits dynamically based on all path data
        path_data = self.sim_data["actual_path"][valid_sim_indices]
        all_x = np.concatenate((self.desired_path[:, 0], path_data[:, 0])) if path_data.size > 0 else self.desired_path[:, 0]
        all_y = np.concatenate((self.desired_path[:, 1], path_data[:, 1])) if path_data.size > 0 else self.desired_path[:, 1]
        x_lim = self._get_lims(all_x)
        y_lim = self._get_lims(all_y)
        self.path_ax.set_xlim(x_lim)
        self.path_ax.set_ylim(y_lim)
        # Re-apply aspect ratio after setting limits
        self.path_ax.set_aspect('equal', adjustable='box')


        # Set true parameter horizontal lines if available
        if true_params_exist:
            self.m_true_line.set_data([0, max_time], [self.true_m, self.true_m])
            self.I_true_line.set_data([0, max_time], [self.true_I, self.true_I])
        else: # Clear them if true values not provided
             self.m_true_line.set_data([],[])
             self.I_true_line.set_data([],[])


        # Initial title update
        self.path_ax.set_title(f"Robot Path Tracking (T=0.00s)")

        return self.artists


    def _animate(self, i):
        """ Animation update function for FuncAnimation """
        # Calculate indices carefully to avoid off-by-one errors
        # sim_index corresponds to the state at time t_k (length N+1)
        sim_index = min(i * self.anim_step, len(self.sim_data["time"]) - 1)
        # controller_index corresponds to data calculated *during* step k-1 (length N)
        controller_index = min(sim_index, len(self.controller_status_list) - 1) # Max index is N-1

        # Time data up to current simulation index
        time_data = self.sim_data["time"][:sim_index+1] # Slice includes sim_index

        # State data up to current simulation index
        actual_path_data = self.sim_data["actual_path"][:sim_index+1]
        vels_actual_data = self.sim_data["robot_vels"][:sim_index+1]

        # Controller/Command data corresponds to steps leading up to current state
        # We need data points from step 0 up to step controller_index (total controller_index + 1 points)
        # Time axis for these plots should match the number of data points
        time_controller = self.sim_data["time"][1:controller_index+2] # Time t_1 to t_{controller_index+1}

        # Slice controller and command data up to controller_index
        if controller_index >= 0:
            torques_cmd_data = self.sim_data["torques_cmd"][1:controller_index+2] # Commands applied during steps 0..controller_index
            dist_data_slice = self.sim_data["disturbances"][1:controller_index+2] # Disturbances during steps 0..controller_index

            vels_desired_data = np.array([s['v_d'] for s in self.controller_status_list[:controller_index+1]])
            kin_errors_data = np.array([s['kin_errors'] for s in self.controller_status_list[:controller_index+1]])
            vel_errors_data = np.array([s['eta'] for s in self.controller_status_list[:controller_index+1]])
            params_hat_data = np.array([s['p_hat'] for s in self.controller_status_list[:controller_index+1]])
        else: # Handle first frame where controller_index might be < 0
             time_controller = np.array([])
             torques_cmd_data=np.empty((0,2)); dist_data_slice=np.empty((0,2))
             vels_desired_data=np.empty((0,2)); kin_errors_data=np.empty((0,3))
             vel_errors_data=np.empty((0,2)); params_hat_data=np.empty((0,2))

        # Get current pose and velocity for markers/arrows (at sim_index)
        current_pose = self.sim_data["actual_path"][sim_index]
        current_vel = self.sim_data["robot_vels"][sim_index]
        x, y, theta = current_pose
        v1, _ = current_vel # Only need v1 for velocity arrow magnitude/direction

        # --- Update Path Plot elements ---
        self.actual_path_line.set_data(actual_path_data[:, 0], actual_path_data[:, 1])
        self.robot_marker.set_data([x], [y])

        # Update Arrows - Remove old, create new (simpler than updating properties for non-blit)
        if hasattr(self.robot_arrow, 'axes') and self.robot_arrow.axes: self.robot_arrow.remove()
        # Make arrow size relative to plot scale for better consistency
        x_range = np.diff(self.path_ax.get_xlim())[0]
        y_range = np.diff(self.path_ax.get_ylim())[0]
        arrow_base_size = min(x_range, y_range) * 0.03 # Base size relative to plot view
        arrow_length = max(0.1, arrow_base_size) # Ensure minimum length

        self.robot_arrow = patches.Arrow(x, y, arrow_length * np.cos(theta), arrow_length * np.sin(theta),
                                         width=arrow_length * 0.4, color='blue', zorder=5, alpha=0.8)
        self.path_ax.add_patch(self.robot_arrow)

        if hasattr(self.velocity_arrow, 'axes') and self.velocity_arrow.axes: self.velocity_arrow.remove()
        # Scale velocity arrow by v1 magnitude, relative to orientation arrow
        vel_arrow_scale = arrow_length * 1.5
        # Ensure arrow direction matches v1 (forward/backward)
        vel_arrow_dx = np.sign(v1) * vel_arrow_scale * abs(v1) * np.cos(theta)
        vel_arrow_dy = np.sign(v1) * vel_arrow_scale * abs(v1) * np.sin(theta)
        # Prevent huge arrows if velocity explodes
        max_vel_arrow_len = arrow_base_size * 5
        current_vel_arrow_len = np.sqrt(vel_arrow_dx**2 + vel_arrow_dy**2)
        if current_vel_arrow_len > max_vel_arrow_len:
            scale_factor = max_vel_arrow_len / current_vel_arrow_len
            vel_arrow_dx *= scale_factor
            vel_arrow_dy *= scale_factor

        self.velocity_arrow = patches.Arrow(x, y, vel_arrow_dx, vel_arrow_dy,
                                            width=arrow_length * 0.2, color='orange', alpha=0.7, zorder=4)
        self.path_ax.add_patch(self.velocity_arrow)


        # --- Update Time Plots safely checking array lengths ---
        self.v1_actual_line.set_data(time_data, vels_actual_data[:, 0])
        self.omega_actual_line.set_data(time_data, vels_actual_data[:, 1])

        # Check if time_controller axis has data points before plotting controller-related data
        if time_controller.size > 0:
            # Check length consistency for each plot
            if vels_desired_data.shape[0] == time_controller.size:
                self.v1_desired_line.set_data(time_controller, vels_desired_data[:, 0])
                self.omega_desired_line.set_data(time_controller, vels_desired_data[:, 1])
            else: self.v1_desired_line.set_data([],[]); self.omega_desired_line.set_data([],[])

            if torques_cmd_data.shape[0] == time_controller.size:
                 self.tau_left_line.set_data(time_controller, torques_cmd_data[:, 0])
                 self.tau_right_line.set_data(time_controller, torques_cmd_data[:, 1])
            else: self.tau_left_line.set_data([],[]); self.tau_right_line.set_data([],[])

            if kin_errors_data.shape[0] == time_controller.size:
                self.error_fwd_line.set_data(time_controller, kin_errors_data[:, 0])
                self.error_lat_line.set_data(time_controller, kin_errors_data[:, 1])
                self.error_theta_line.set_data(time_controller, kin_errors_data[:, 2])
            else: self.error_fwd_line.set_data([],[]); self.error_lat_line.set_data([],[]); self.error_theta_line.set_data([],[])

            if vel_errors_data.shape[0] == time_controller.size:
                self.eta1_line.set_data(time_controller, vel_errors_data[:, 0])
                self.eta2_line.set_data(time_controller, vel_errors_data[:, 1])
            else: self.eta1_line.set_data([],[]); self.eta2_line.set_data([],[])

            if params_hat_data.shape[0] == time_controller.size:
                self.m_hat_line.set_data(time_controller, params_hat_data[:, 0])
                self.I_hat_line.set_data(time_controller, params_hat_data[:, 1])
            else: self.m_hat_line.set_data([],[]); self.I_hat_line.set_data([],[])

            if dist_data_slice.shape[0] == time_controller.size:
                 self.dist1_line.set_data(time_controller, dist_data_slice[:, 0])
                 self.dist2_line.set_data(time_controller, dist_data_slice[:, 1])
            else: self.dist1_line.set_data([],[]); self.dist2_line.set_data([],[])

        else: # If time_controller is empty (first frame), clear all controller-related plots
            self.v1_desired_line.set_data([],[]); self.omega_desired_line.set_data([],[])
            self.tau_left_line.set_data([],[]); self.tau_right_line.set_data([],[])
            self.error_fwd_line.set_data([],[]); self.error_lat_line.set_data([],[]); self.error_theta_line.set_data([],[])
            self.eta1_line.set_data([],[]); self.eta2_line.set_data([],[])
            self.m_hat_line.set_data([],[]); self.I_hat_line.set_data([],[])
            self.dist1_line.set_data([],[]); self.dist2_line.set_data([],[])


        # Update path plot title with current time
        self.path_ax.set_title(f"Robot Path Tracking (T={time_data[-1]:.2f}s)")

        # Update artist list for blitting (though blit=False is used)
        self.artists = [
            self.actual_path_line, self.robot_marker,
            self.v1_actual_line, self.omega_actual_line, self.v1_desired_line, self.omega_desired_line,
            self.tau_left_line, self.tau_right_line,
            self.error_fwd_line, self.error_lat_line, self.error_theta_line,
            self.eta1_line, self.eta2_line,
            self.m_hat_line, self.I_hat_line,
            self.m_true_line, self.I_true_line,
            self.dist1_line, self.dist2_line,
            self.robot_arrow, self.velocity_arrow # Include patches
        ]
        return self.artists


    def create_animation(self, simulation_data, controller_status_list, interval=50, step=10, true_m=None, true_I=None):
        """ Creates the animation object using the simulation data. """
        self.sim_data = simulation_data
        # Ensure controller_status_list matches number of control steps taken
        num_control_steps = len(simulation_data['time']) - 1 # N steps produce N+1 states
        self.controller_status_list = controller_status_list[:num_control_steps]
        if len(self.controller_status_list) != num_control_steps:
             print(f"Warning: Length mismatch in create_animation. Status list {len(self.controller_status_list)}, Control steps {num_control_steps}")
             # Pad or truncate controller status if needed? For now, rely on slicing in _animate.

        self.anim_step = step
        self.true_m = true_m
        self.true_I = true_I

        num_sim_points = len(self.sim_data["time"])
        # Calculate frames needed to show *all* simulation points
        num_frames = (num_sim_points + self.anim_step - 1) // self.anim_step

        # Use blit=False because we are modifying patch artists (Arrows), which is complex with blitting.
        ani = animation.FuncAnimation(self.fig, self._animate, frames=num_frames,
                                      init_func=self._init_animation, blit=False,
                                      interval=interval, repeat=False)
        return ani


    def plot_final_results(self, simulation_data, controller_status_list, path_type_name="", true_m=None, true_I=None):
        """ Generates static plots of errors, params, etc. vs. time after the simulation. """
        # Create a new figure specifically for the final results
        fig_res, axs = plt.subplots(5, 1, figsize=(10, 15), sharex=True) # 5 rows, 1 column
        fig_res.suptitle(f"Adaptive Control Simulation Results - {path_type_name} Path", fontsize=16, weight='bold')

        time_data = simulation_data["time"]
        num_control_steps = len(controller_status_list)
        if num_control_steps == 0:
             print("No controller status data to plot for final results.")
             plt.close(fig_res) # Close the empty figure
             return

        # Time axis corresponding to controller data points
        time_controller = time_data[1:num_control_steps+1] # Time t_1 to t_N
        if len(time_controller) != num_control_steps:
             print(f"Warning: Final plot time axis length ({len(time_controller)}) != controller steps ({num_control_steps}). Adjusting.")
             # Adjust data length to match the shorter time axis or controller steps
             min_len = min(len(time_controller), num_control_steps)
             time_controller = time_controller[:min_len]
             controller_status_list = controller_status_list[:min_len]
             num_control_steps = min_len # Update the number of steps to use

        # Extract controller data safely using the adjusted length
        kin_errors = np.array([s['kin_errors'] for s in controller_status_list])
        vel_errors = np.array([s['eta'] for s in controller_status_list])
        params_hat = np.array([s['p_hat'] for s in controller_status_list])

        # Extract command/disturbance data matching the number of control steps
        torques = simulation_data["torques_cmd"][1:num_control_steps+1]
        disturbances = simulation_data["disturbances"][1:num_control_steps+1]

        # --- Plotting with improved labels and legends ---

        # Plot Kinematic Errors
        axs[0].plot(time_controller, kin_errors[:, 0], 'c-', label="Forward Error (m)")
        axs[0].plot(time_controller, kin_errors[:, 1], 'y-', label="Lateral Error (m)")
        axs[0].plot(time_controller, kin_errors[:, 2], 'k-', label="Angle Error $\\Delta\\theta$ (rad)")
        axs[0].set_title("Kinematic Errors (Path Following)")
        axs[0].set_ylabel("Error Value")
        axs[0].legend(fontsize='small', loc='best')
        axs[0].grid(True, linestyle=':', alpha=0.6)

        # Plot Velocity Errors (eta)
        axs[1].plot(time_controller, vel_errors[:, 0], 'r-', label="$\\eta_1 = v_1 - v_{1d}$ (m/s)")
        axs[1].plot(time_controller, vel_errors[:, 1], 'm-', label="$\\eta_2 = \\omega - \\omega_d$ (rad/s)")
        axs[1].set_title("Velocity Tracking Errors ($\\eta$)")
        axs[1].set_ylabel("Velocity Error")
        axs[1].legend(fontsize='small', loc='best')
        axs[1].grid(True, linestyle=':', alpha=0.6)

        # Plot Parameter Estimates
        axs[2].plot(time_controller, params_hat[:, 0], 'b-', label="$\\hat{m}$ Estimate (kg)")
        axs[2].plot(time_controller, params_hat[:, 1], 'g-', label="$\\hat{I}$ Estimate (kg m$^2$)")
        if true_m is not None: axs[2].axhline(true_m, color='b', linestyle='--', alpha=0.7, label=f"$m$ True ({true_m:.2f} kg)")
        if true_I is not None: axs[2].axhline(true_I, color='g', linestyle='--', alpha=0.7, label=f"$I$ True ({true_I:.2f} kg m$^2$)")
        axs[2].set_title("Parameter Estimates")
        axs[2].set_ylabel("Estimated Value")
        axs[2].legend(fontsize='small', loc='best')
        axs[2].grid(True, linestyle=':', alpha=0.6)
        # Optional: Add bounds if they were used
        # min_m, min_I = adaptive_controller.min_params # Need access to controller instance or pass bounds
        # max_m, max_I = adaptive_controller.max_params
        # axs[2].axhline(min_m, color='grey', linestyle=':', alpha=0.5, label="Min Bound")
        # axs[2].axhline(max_m, color='grey', linestyle=':', alpha=0.5, label="Max Bound")


        # Plot Torques
        if torques.shape[0] == num_control_steps:
             axs[3].plot(time_controller, torques[:, 0], 'b-', label="$\\tau_L$ Left Cmd (Nm)")
             axs[3].plot(time_controller, torques[:, 1], 'g-', label="$\\tau_R$ Right Cmd (Nm)")
        axs[3].set_title("Control Torques")
        axs[3].set_ylabel("Torque (Nm)")
        axs[3].legend(fontsize='small', loc='best')
        axs[3].grid(True, linestyle=':', alpha=0.6)

        # Plot Disturbances
        if disturbances.shape[0] == num_control_steps:
             axs[4].plot(time_controller, disturbances[:, 0], 'r-', label="$\\tau_{d,v1}$ (Linear Eq.)")
             axs[4].plot(time_controller, disturbances[:, 1], 'm-', label="$\\tau_{d,\\omega}$ (Angular Eq.)")
        axs[4].set_title("Applied Disturbances (Effective Torque)")
        axs[4].set_xlabel("Time (s)") # X-label only on the bottom plot
        axs[4].set_ylabel("Disturbance")
        axs[4].legend(fontsize='small', loc='best')
        axs[4].grid(True, linestyle=':', alpha=0.6)

        # Improve overall layout
        fig_res.tight_layout(rect=[0, 0.03, 1, 0.95]) # Adjust rect to prevent title overlap


