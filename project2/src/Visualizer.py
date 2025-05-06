import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np

class Visualizer:
    def __init__(self, desired_path):
        """
        Initialize the visualizer. Sets up the plot structure with improved layout and labels.
        The layout will be:
        - Column 0: Main path plot (spans all 3 rows)
        - Column 1: 3 time-series plots (Velocities, Torques, Kinematic Errors)
        - Column 2: 3 time-series plots (Velocity Errors, Parameter Estimates, Disturbances)
        """
        self.desired_path = desired_path
        # Use constrained_layout for better automatic spacing
        # Adjusted figsize for a wider layout to accommodate three columns
        self.fig = plt.figure(figsize=(22, 10), constrained_layout=True)

        # Create a grid for subplots: 3 rows, 3 columns
        # Column 0 spans all rows for the main path plot
        # Columns 1 and 2 each have 3 smaller plots for time-series data
        gs = self.fig.add_gridspec(3, 3, width_ratios=[3, 2, 2]) # Main plot wider

        # --- Column 0: Path Plot ---
        self.path_ax = self.fig.add_subplot(gs[:, 0])
        self.path_ax.set_aspect('equal', adjustable='box')
        self.path_ax.set_title("Robot Path Tracking")
        self.path_ax.set_xlabel("X Position (m)")
        self.path_ax.set_ylabel("Y Position (m)")
        self.path_ax.plot(desired_path[:, 0], desired_path[:, 1], 'r--', label='Desired Path', linewidth=1.5, alpha=0.7)
        self.actual_path_line, = self.path_ax.plot([], [], 'g-', label='Actual Path', linewidth=1.5)
        self.robot_marker, = self.path_ax.plot([], [], marker='^', color='blue', markersize=9, label="Robot Position & Orientation")
        self.robot_arrow = patches.Arrow(0, 0, 0, 0, width=0.2, color='blue', zorder=5, alpha=0.8) # Orientation
        self.path_ax.add_patch(self.robot_arrow)
        self.velocity_arrow = patches.Arrow(0, 0, 0, 0, width=0.1, color='orange', alpha=0.7, zorder=4, label="Velocity Vector (Scaled)") # Velocity
        self.path_ax.add_patch(self.velocity_arrow)
        # Adjusted legend placement slightly due to new column layout
        self.path_ax.legend(loc='upper right', bbox_to_anchor=(0.98, 0.98), borderaxespad=0.)
        self.path_ax.grid(True, linestyle=':', alpha=0.6)

        # --- Time Plots ---
        # All time plots will share the x-axis of the first time plot (velocity_ax)

        # --- Column 1: Time Plots (First Set) ---
        # 1. Robot Velocities (Actual vs Desired)
        self.velocity_ax = self.fig.add_subplot(gs[0, 1])
        self.velocity_ax.set_title("Velocities")
        self.velocity_ax.set_ylabel("Value")
        self.v1_actual_line, = self.velocity_ax.plot([], [], 'r-', label="$v_1$ Actual (m/s)")
        self.omega_actual_line, = self.velocity_ax.plot([], [], 'm-', label="$\\omega$ Actual (rad/s)")
        self.v1_desired_line, = self.velocity_ax.plot([], [], 'r:', label="$v_{1d}$ Desired (m/s)")
        self.omega_desired_line, = self.velocity_ax.plot([], [], 'm:', label="$\\omega_d$ Desired (rad/s)")
        self.velocity_ax.legend(fontsize='small', ncol=2)
        self.velocity_ax.grid(True, linestyle=':', alpha=0.6)
        self.velocity_ax.tick_params(axis='x', labelbottom=False) # No x-label for upper plots

        # 2. Control Torques
        self.torque_ax = self.fig.add_subplot(gs[1, 1], sharex=self.velocity_ax)
        self.torque_ax.set_title("Control Torques")
        self.torque_ax.set_ylabel("Torque (Nm)")
        self.tau_left_line, = self.torque_ax.plot([], [], 'b-', label="$\\tau_L$ Left Cmd")
        self.tau_right_line, = self.torque_ax.plot([], [], 'g-', label="$\\tau_R$ Right Cmd")
        self.torque_ax.legend(fontsize='small')
        self.torque_ax.grid(True, linestyle=':', alpha=0.6)
        self.torque_ax.tick_params(axis='x', labelbottom=False) # No x-label for middle plots

        # 3. Kinematic Errors (Path Following)
        self.kin_error_ax = self.fig.add_subplot(gs[2, 1], sharex=self.velocity_ax)
        self.kin_error_ax.set_title("Kinematic Errors (Path Following)")
        self.kin_error_ax.set_ylabel("Error Value")
        self.error_fwd_line, = self.kin_error_ax.plot([], [], 'c-', label="Forward Error (m)")
        self.error_lat_line, = self.kin_error_ax.plot([], [], 'y-', label="Lateral Error (m)")
        self.error_theta_line, = self.kin_error_ax.plot([], [], 'k-', label="Angle Error $\\Delta\\theta$ (rad)")
        self.kin_error_ax.legend(fontsize='small')
        self.kin_error_ax.grid(True, linestyle=':', alpha=0.6)
        self.kin_error_ax.set_xlabel("Time (s)") # X-label for bottom plot in this column

        # --- Column 2: Time Plots (Second Set) ---
        # 4. Velocity Errors (Tracking)
        self.vel_error_ax = self.fig.add_subplot(gs[0, 2], sharex=self.velocity_ax)
        self.vel_error_ax.set_title("Velocity Tracking Errors ($\\eta = v - v_d$)")
        self.vel_error_ax.set_ylabel("Velocity Error")
        self.eta1_line, = self.vel_error_ax.plot([], [], 'r-', label="$\\eta_1 = v_1 - v_{1d}$ (m/s)")
        self.eta2_line, = self.vel_error_ax.plot([], [], 'm-', label="$\\eta_2 = \\omega - \\omega_d$ (rad/s)")
        self.vel_error_ax.legend(fontsize='small')
        self.vel_error_ax.grid(True, linestyle=':', alpha=0.6)
        self.vel_error_ax.tick_params(axis='x', labelbottom=False) # No x-label for upper plots

        # 5. Parameter Estimates
        self.param_ax = self.fig.add_subplot(gs[1, 2], sharex=self.velocity_ax)
        self.param_ax.set_title("Parameter Estimates ($\\hat{p}$)")
        self.param_ax.set_ylabel("Estimated Value")
        self.m_hat_line, = self.param_ax.plot([], [], 'b-', label="$\\hat{m}$ Estimate (kg)")
        self.I_hat_line, = self.param_ax.plot([], [], 'g-', label="$\\hat{I}$ Estimate (kg m$^2$)")
        self.m_true_line, = self.param_ax.plot([], [], 'b--', label="$m$ True Value", alpha=0.6)
        self.I_true_line, = self.param_ax.plot([], [], 'g--', label="$I$ True Value", alpha=0.6)
        self.param_ax.legend(fontsize='small', ncol=2)
        self.param_ax.grid(True, linestyle=':', alpha=0.6)
        self.param_ax.tick_params(axis='x', labelbottom=False) # No x-label for middle plots

        # 6. Disturbances
        self.dist_ax = self.fig.add_subplot(gs[2, 2], sharex=self.velocity_ax)
        self.dist_ax.set_title("Applied Disturbances (Effective Torque)")
        self.dist_ax.set_xlabel("Time (s)") # X-label for bottom plot in this column
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
        if not np.isfinite(data_range) or data_range == 0: # check for zero range
             data_range = abs(max_val)*0.2 if np.isfinite(max_val) and max_val != 0 else 1.0
             if data_range == 0: data_range = 0.2 # Default if max_val is 0
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

        if hasattr(self.robot_arrow, 'axes') and self.robot_arrow.axes: self.robot_arrow.remove()
        if hasattr(self.velocity_arrow, 'axes') and self.velocity_arrow.axes: self.velocity_arrow.remove()

        self.robot_arrow = patches.Arrow(0, 0, 0.01, 0, width=0.2, color='blue', zorder=5, alpha=0.8)
        self.velocity_arrow = patches.Arrow(0, 0, 0, 0, width=0.1, color='orange', alpha=0.7, zorder=4)
        self.path_ax.add_patch(self.robot_arrow)
        self.path_ax.add_patch(self.velocity_arrow)

        self.artists = line_artists + [self.robot_arrow, self.velocity_arrow]

        max_time = self.sim_data["time"][-1] if len(self.sim_data["time"]) > 0 else 1.0
        num_steps_completed = len(self.controller_status_list)
        num_time_points = len(self.sim_data['time'])
        # valid_controller_indices = slice(0, num_steps_completed) # Used implicitly by slicing controller_status_list
        valid_sim_indices = slice(0, num_time_points)
        valid_command_indices = slice(1, num_steps_completed + 1) # For torques/disturbances

        vels_actual = self.sim_data["robot_vels"][valid_sim_indices]
        vels_desired = np.array([s['v_d'] for s in self.controller_status_list]) if self.controller_status_list else np.zeros((0,2))
        all_vels_for_lim = np.vstack((vels_actual, vels_desired)) if vels_desired.size > 0 and vels_actual.size > 0 else (vels_actual if vels_actual.size > 0 else vels_desired)


        torques = self.sim_data["torques_cmd"][valid_command_indices] if num_steps_completed > 0 else np.zeros((0,2))
        kin_errors = np.array([s['kin_errors'] for s in self.controller_status_list]) if self.controller_status_list else np.zeros((0,3))
        vel_errors = np.array([s['eta'] for s in self.controller_status_list]) if self.controller_status_list else np.zeros((0,2))
        params_hat = np.array([s['p_hat'] for s in self.controller_status_list]) if self.controller_status_list else np.zeros((0,2))
        disturbances = self.sim_data["disturbances"][valid_command_indices] if num_steps_completed > 0 else np.zeros((0,2))


        vel_lim = self._get_lims(all_vels_for_lim, zero_center=False)
        torque_lim = self._get_lims(torques, zero_center=True)
        kin_err_lim = self._get_lims(kin_errors, zero_center=True)
        vel_err_lim = self._get_lims(vel_errors, zero_center=True)

        param_data_for_lims = params_hat
        true_params_exist = self.true_m is not None and self.true_I is not None
        if true_params_exist and params_hat.size > 0:
            true_vals = np.array([[self.true_m, self.true_I]])
            param_data_for_lims = np.vstack((params_hat, true_vals))
        elif true_params_exist: # params_hat is empty but true values exist
            param_data_for_lims = np.array([[self.true_m, self.true_I]])

        param_lim = self._get_lims(param_data_for_lims, pad_factor=0.2)
        if param_lim[0] > -0.1 and np.any(param_data_for_lims >= 0): param_lim = (-0.1, param_lim[1])


        dist_lim = self._get_lims(disturbances, zero_center=True)

        # Order matches the creation order in __init__ for column 1 then column 2
        time_series_axes = [
            self.velocity_ax, self.torque_ax, self.kin_error_ax, # Column 1
            self.vel_error_ax, self.param_ax, self.dist_ax      # Column 2
        ]
        time_series_lims = [vel_lim, torque_lim, kin_err_lim, vel_err_lim, param_lim, dist_lim]

        for ax, lim in zip(time_series_axes, time_series_lims):
            ax.set_xlim(0, max_time)
            ax.set_ylim(lim)

        path_data = self.sim_data["actual_path"][valid_sim_indices]
        all_x = np.concatenate((self.desired_path[:, 0], path_data[:, 0])) if path_data.size > 0 else self.desired_path[:, 0]
        all_y = np.concatenate((self.desired_path[:, 1], path_data[:, 1])) if path_data.size > 0 else self.desired_path[:, 1]
        x_lim = self._get_lims(all_x)
        y_lim = self._get_lims(all_y)
        self.path_ax.set_xlim(x_lim)
        self.path_ax.set_ylim(y_lim)
        self.path_ax.set_aspect('equal', adjustable='box')

        if true_params_exist:
            self.m_true_line.set_data([0, max_time], [self.true_m, self.true_m])
            self.I_true_line.set_data([0, max_time], [self.true_I, self.true_I])
        else:
            self.m_true_line.set_data([],[])
            self.I_true_line.set_data([],[])

        self.path_ax.set_title(f"Robot Path Tracking (T=0.00s)")
        return self.artists

    def _animate(self, i):
        sim_index = min(i * self.anim_step, len(self.sim_data["time"]) - 1)
        controller_index = min(sim_index, len(self.controller_status_list) - 1)

        time_data = self.sim_data["time"][:sim_index+1]
        actual_path_data = self.sim_data["actual_path"][:sim_index+1]
        vels_actual_data = self.sim_data["robot_vels"][:sim_index+1]

        time_controller = np.array([])
        if controller_index >= 0:
            # Time axis for controller data should correspond to when commands/estimates were made
            # If controller_status_list has N items, they correspond to t_1, ..., t_N
            # sim_data['time'] is t_0, t_1, ..., t_N
            # So we take time[1] up to time[controller_index + 1]
            time_controller = self.sim_data["time"][1:controller_index+2]


        if controller_index >= 0 and time_controller.size > 0:
            # Ensure controller_status_list has enough data for the current controller_index
            current_status_list = self.controller_status_list[:controller_index+1]

            torques_cmd_data = self.sim_data["torques_cmd"][1:controller_index+2]
            dist_data_slice = self.sim_data["disturbances"][1:controller_index+2]

            vels_desired_data = np.array([s['v_d'] for s in current_status_list])
            kin_errors_data = np.array([s['kin_errors'] for s in current_status_list])
            vel_errors_data = np.array([s['eta'] for s in current_status_list])
            params_hat_data = np.array([s['p_hat'] for s in current_status_list])
        else:
            time_controller = np.array([]) # Ensure it's empty if no controller data
            torques_cmd_data=np.empty((0,2)); dist_data_slice=np.empty((0,2))
            vels_desired_data=np.empty((0,2)); kin_errors_data=np.empty((0,3))
            vel_errors_data=np.empty((0,2)); params_hat_data=np.empty((0,2))


        current_pose = self.sim_data["actual_path"][sim_index]
        current_vel = self.sim_data["robot_vels"][sim_index]
        x, y, theta = current_pose
        v1, _ = current_vel

        self.actual_path_line.set_data(actual_path_data[:, 0], actual_path_data[:, 1])
        self.robot_marker.set_data([x], [y])

        if hasattr(self.robot_arrow, 'axes') and self.robot_arrow.axes: self.robot_arrow.remove()
        x_range = np.diff(self.path_ax.get_xlim())[0]
        y_range = np.diff(self.path_ax.get_ylim())[0]
        # Handle case where x_range or y_range might be non-positive or zero
        arrow_base_size = 0.1
        if x_range > 0 and y_range > 0 :
             arrow_base_size = min(x_range, y_range) * 0.03
        arrow_length = max(0.01, arrow_base_size) # Ensure minimum length for visibility

        self.robot_arrow = patches.Arrow(x, y, arrow_length * np.cos(theta), arrow_length * np.sin(theta),
                                         width=arrow_length * 0.4, color='blue', zorder=5, alpha=0.8)
        self.path_ax.add_patch(self.robot_arrow)

        if hasattr(self.velocity_arrow, 'axes') and self.velocity_arrow.axes: self.velocity_arrow.remove()
        vel_arrow_scale = arrow_length * 1.5
        vel_arrow_dx = np.sign(v1) * vel_arrow_scale * abs(v1) * np.cos(theta)
        vel_arrow_dy = np.sign(v1) * vel_arrow_scale * abs(v1) * np.sin(theta)
        max_vel_arrow_len = arrow_base_size * 5
        current_vel_arrow_len = np.sqrt(vel_arrow_dx**2 + vel_arrow_dy**2)
        if current_vel_arrow_len > max_vel_arrow_len and max_vel_arrow_len > 0: # Check max_vel_arrow_len >0
            scale_factor = max_vel_arrow_len / current_vel_arrow_len
            vel_arrow_dx *= scale_factor
            vel_arrow_dy *= scale_factor

        self.velocity_arrow = patches.Arrow(x, y, vel_arrow_dx, vel_arrow_dy,
                                            width=arrow_length * 0.2, color='orange', alpha=0.7, zorder=4)
        self.path_ax.add_patch(self.velocity_arrow)

        self.v1_actual_line.set_data(time_data, vels_actual_data[:, 0])
        self.omega_actual_line.set_data(time_data, vels_actual_data[:, 1])

        if time_controller.size > 0:
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
        else: # Clear all controller-related plots if no time_controller data
            self.v1_desired_line.set_data([],[]); self.omega_desired_line.set_data([],[])
            self.tau_left_line.set_data([],[]); self.tau_right_line.set_data([],[])
            self.error_fwd_line.set_data([],[]); self.error_lat_line.set_data([],[]); self.error_theta_line.set_data([],[])
            self.eta1_line.set_data([],[]); self.eta2_line.set_data([],[])
            self.m_hat_line.set_data([],[]); self.I_hat_line.set_data([],[])
            self.dist1_line.set_data([],[]); self.dist2_line.set_data([],[])

        self.path_ax.set_title(f"Robot Path Tracking (T={time_data[-1]:.2f}s)")
        self.artists = [
            self.actual_path_line, self.robot_marker,
            self.v1_actual_line, self.omega_actual_line, self.v1_desired_line, self.omega_desired_line,
            self.tau_left_line, self.tau_right_line,
            self.error_fwd_line, self.error_lat_line, self.error_theta_line,
            self.eta1_line, self.eta2_line,
            self.m_hat_line, self.I_hat_line,
            self.m_true_line, self.I_true_line,
            self.dist1_line, self.dist2_line,
            self.robot_arrow, self.velocity_arrow
        ]
        return self.artists

    def create_animation(self, simulation_data, controller_status_list, interval=50, step=10, true_m=None, true_I=None):
        self.sim_data = simulation_data
        num_control_steps = len(simulation_data['time']) -1 if len(simulation_data['time']) > 0 else 0
        
        # Ensure controller_status_list is correctly sized. It should have one entry per control step.
        # If simulation ran for N steps (N+1 time points), controller_status_list should have N entries.
        if len(controller_status_list) > num_control_steps:
            print(f"Warning: controller_status_list (len {len(controller_status_list)}) is longer than num_control_steps ({num_control_steps}). Truncating.")
            self.controller_status_list = controller_status_list[:num_control_steps]
        elif len(controller_status_list) < num_control_steps:
             print(f"Warning: controller_status_list (len {len(controller_status_list)}) is shorter than num_control_steps ({num_control_steps}). Animation might miss some controller data.")
             self.controller_status_list = controller_status_list
        else:
            self.controller_status_list = controller_status_list


        self.anim_step = step
        self.true_m = true_m
        self.true_I = true_I

        num_sim_points = len(self.sim_data["time"])
        num_frames = (num_sim_points + self.anim_step - 1) // self.anim_step if self.anim_step > 0 else num_sim_points

        ani = animation.FuncAnimation(self.fig, self._animate, frames=num_frames,
                                      init_func=self._init_animation, blit=False,
                                      interval=interval, repeat=False)
        return ani

    def plot_final_results(self, simulation_data, controller_status_list, path_type_name="", true_m=None, true_I=None):
        """ Generates static plots of errors, params, etc. vs. time after the simulation.
            Uses a 3 rows, 2 columns layout for the time-series data.
        """
        # Create a new figure for final results, with a 3x2 layout for the plots
        # This makes it 5 plots in total, the last subplot in the grid can be turned off.
        fig_res, axs = plt.subplots(3, 2, figsize=(15, 12), sharex=True)
        fig_res.suptitle(f"Adaptive Control Simulation Results - {path_type_name} Path", fontsize=16, weight='bold')

        time_data = simulation_data["time"]
        
        num_sim_data_points = len(time_data)
        if num_sim_data_points == 0:
            print("No simulation data to plot for final results.")
            plt.close(fig_res)
            return

        # Number of control steps is N if there are N+1 time points (t_0 to t_N)
        num_control_steps_from_time = num_sim_data_points - 1
        
        # Validate controller_status_list length
        num_controller_entries = len(controller_status_list)

        if num_controller_entries == 0:
            print("No controller status data to plot for final results.")
            # We might still be able to plot things like torques if they exist for the steps
            # For now, let's assume if controller_status is empty, critical data is missing.
            # plt.close(fig_res) # Keep open if other data might be plottable
            # return

        # Determine the actual number of control steps to plot based on available data
        # It should ideally be num_control_steps_from_time
        # And data arrays (torques, disturbances, controller_status outputs) should match this.
        
        # This is the number of intervals, corresponding to commands and controller states
        # for steps 0 to N-1 (resulting in states 1 to N)
        plot_n_steps = num_control_steps_from_time 
        
        if num_controller_entries < plot_n_steps:
            print(f"Warning: Final plot - controller_status_list has {num_controller_entries} entries, expected {plot_n_steps}. Truncating plot range.")
            plot_n_steps = num_controller_entries
        
        # Time axis for controller data, torques, disturbances (t_1 to t_N)
        # This corresponds to the end time of each control interval
        time_controller = time_data[1 : plot_n_steps + 1]

        if len(time_controller) != plot_n_steps:
             print(f"Warning: Final plot time_controller axis length ({len(time_controller)}) != plot_n_steps ({plot_n_steps}). This indicates an issue.")
             # Adjust plot_n_steps to the shortest available data length if this happens
             plot_n_steps = min(len(time_controller), plot_n_steps)
             time_controller = time_controller[:plot_n_steps]


        # Safely slice all data arrays up to plot_n_steps
        kin_errors = np.array([s['kin_errors'] for s in controller_status_list[:plot_n_steps]]) if plot_n_steps > 0 else np.empty((0,3))
        vel_errors = np.array([s['eta'] for s in controller_status_list[:plot_n_steps]]) if plot_n_steps > 0 else np.empty((0,2))
        params_hat = np.array([s['p_hat'] for s in controller_status_list[:plot_n_steps]]) if plot_n_steps > 0 else np.empty((0,2))
        
        # Torques and disturbances are indexed from 1 up to plot_n_steps+1 in sim_data
        # This corresponds to commands for step 0, 1, ..., plot_n_steps-1
        torques = simulation_data["torques_cmd"][1:plot_n_steps+1] if plot_n_steps > 0 else np.empty((0,2))
        disturbances = simulation_data["disturbances"][1:plot_n_steps+1] if plot_n_steps > 0 else np.empty((0,2))

        # Flatten axs for easier iteration if needed, or use direct indexing
        # axs_flat = axs.flatten()

        # Plot Kinematic Errors
        if kin_errors.shape[0] == plot_n_steps and plot_n_steps > 0:
            axs[0, 0].plot(time_controller, kin_errors[:, 0], 'c-', label="Forward Error (m)")
            axs[0, 0].plot(time_controller, kin_errors[:, 1], 'y-', label="Lateral Error (m)")
            axs[0, 0].plot(time_controller, kin_errors[:, 2], 'k-', label="Angle Error $\\Delta\\theta$ (rad)")
        axs[0, 0].set_title("Kinematic Errors")
        axs[0, 0].set_ylabel("Error Value")
        axs[0, 0].legend(fontsize='small', loc='best')
        axs[0, 0].grid(True, linestyle=':', alpha=0.6)

        # Plot Velocity Errors (eta)
        if vel_errors.shape[0] == plot_n_steps and plot_n_steps > 0:
            axs[0, 1].plot(time_controller, vel_errors[:, 0], 'r-', label="$\\eta_1 = v_1 - v_{1d}$ (m/s)")
            axs[0, 1].plot(time_controller, vel_errors[:, 1], 'm-', label="$\\eta_2 = \\omega - \\omega_d$ (rad/s)")
        axs[0, 1].set_title("Velocity Tracking Errors ($\\eta$)")
        axs[0, 1].set_ylabel("Velocity Error")
        axs[0, 1].legend(fontsize='small', loc='best')
        axs[0, 1].grid(True, linestyle=':', alpha=0.6)

        # Plot Parameter Estimates
        if params_hat.shape[0] == plot_n_steps and plot_n_steps > 0:
            axs[1, 0].plot(time_controller, params_hat[:, 0], 'b-', label="$\\hat{m}$ Estimate (kg)")
            axs[1, 0].plot(time_controller, params_hat[:, 1], 'g-', label="$\\hat{I}$ Estimate (kg m$^2$)")
        if true_m is not None: axs[1, 0].axhline(true_m, color='b', linestyle='--', alpha=0.7, label=f"$m$ True ({true_m:.2f} kg)")
        if true_I is not None: axs[1, 0].axhline(true_I, color='g', linestyle='--', alpha=0.7, label=f"$I$ True ({true_I:.2f} kg m$^2$)")
        axs[1, 0].set_title("Parameter Estimates")
        axs[1, 0].set_ylabel("Estimated Value")
        axs[1, 0].legend(fontsize='small', loc='best')
        axs[1, 0].grid(True, linestyle=':', alpha=0.6)

        # Plot Torques
        if torques.shape[0] == plot_n_steps and plot_n_steps > 0:
            axs[1, 1].plot(time_controller, torques[:, 0], 'b-', label="$\\tau_L$ Left Cmd (Nm)")
            axs[1, 1].plot(time_controller, torques[:, 1], 'g-', label="$\\tau_R$ Right Cmd (Nm)")
        axs[1, 1].set_title("Control Torques")
        axs[1, 1].set_ylabel("Torque (Nm)")
        axs[1, 1].legend(fontsize='small', loc='best')
        axs[1, 1].grid(True, linestyle=':', alpha=0.6)

        # Plot Disturbances
        if disturbances.shape[0] == plot_n_steps and plot_n_steps > 0:
            axs[2, 0].plot(time_controller, disturbances[:, 0], 'r-', label="$\\tau_{d,v1}$ (Linear Eq.)")
            axs[2, 0].plot(time_controller, disturbances[:, 1], 'm-', label="$\\tau_{d,\\omega}$ (Angular Eq.)")
        axs[2, 0].set_title("Applied Disturbances")
        axs[2, 0].set_xlabel("Time (s)")
        axs[2, 0].set_ylabel("Disturbance")
        axs[2, 0].legend(fontsize='small', loc='best')
        axs[2, 0].grid(True, linestyle=':', alpha=0.6)

        # Turn off the unused subplot in the 3x2 grid
        axs[2, 1].axis('off')

        # Improve overall layout
        fig_res.tight_layout(rect=[0, 0.03, 1, 0.95]) # Adjust rect to prevent title overlap
        # plt.show() # Optional: show immediately, or let user handle it.

# Example Usage (assuming you have simulation_data and controller_status_list)
if __name__ == '__main__':
    # --- Create Dummy Data for Demonstration ---
    num_points = 200
    dt = 0.1
    time_vector = np.arange(0, num_points * dt, dt)

    # Desired path (e.g., a circle or a line)
    # desired_path_data = np.array([[t, np.sin(t*0.5)] for t in np.linspace(0, 10, 100)])
    radius = 5
    center_x, center_y = 0, 5
    theta_path = np.linspace(0, 2 * np.pi, 150)
    desired_path_data = np.array([
        center_x + radius * np.cos(theta_path),
        center_y + radius * np.sin(theta_path)
    ]).T


    # Initialize simulation_data dictionary
    simulation_data = {
        "time": time_vector,
        "actual_path": np.zeros((num_points, 3)), # x, y, theta
        "robot_vels": np.zeros((num_points, 2)),  # v1, omega
        "torques_cmd": np.zeros((num_points, 2)), # tau_L, tau_R (Note: often N entries for N+1 states)
        "disturbances": np.zeros((num_points, 2)) # tau_d1, tau_d2 (Note: often N entries for N+1 states)
    }
    # Populate with some dummy data
    for i in range(num_points):
        simulation_data["actual_path"][i, :] = [desired_path_data[i % len(desired_path_data),0] * (0.5 + 0.5*i/num_points) + np.random.randn()*0.1,
                                                desired_path_data[i % len(desired_path_data),1] * (0.5 + 0.5*i/num_points) + np.random.randn()*0.1,
                                                np.arctan2(np.cos(time_vector[i]*0.2), -np.sin(time_vector[i]*0.2)) + np.random.randn()*0.05]
        simulation_data["robot_vels"][i, :] = [1.0 + 0.1 * np.sin(time_vector[i]), 0.2 * np.cos(time_vector[i]*0.5)]
        if i > 0: # Torques and disturbances are commands for the *interval*
            simulation_data["torques_cmd"][i, :] = [5 + np.sin(time_vector[i-1]), 5 - np.sin(time_vector[i-1])]
            simulation_data["disturbances"][i, :] = [0.1 * np.sin(time_vector[i-1]*2), -0.1 * np.cos(time_vector[i-1]*2)]

    # Controller status list (N entries if N+1 time points/states)
    # Each entry is a dict, corresponding to data calculated at each control step
    num_control_steps = num_points -1
    controller_status_list = []
    if num_control_steps > 0:
        for k in range(num_control_steps): # k from 0 to N-2
            status = {
                'v_d': np.array([1.05, 0.18 * np.cos(time_vector[k]*0.5)]), # Desired velocities at step k
                'kin_errors': np.array([0.1*np.exp(-k*dt*0.1), -0.05*np.exp(-k*dt*0.1), 0.02*np.exp(-k*dt*0.1)]), # Kinematic errors after step k
                'eta': np.array([0.05*np.sin(time_vector[k]), 0.02*np.cos(time_vector[k])]), # Velocity errors at step k
                'p_hat': np.array([10 - 2*np.exp(-k*dt*0.2), 0.5 - 0.1*np.exp(-k*dt*0.2)]) # Parameter estimates at step k
            }
            controller_status_list.append(status)

    true_m_val = 8.0
    true_I_val = 0.4

    # --- Test Animation ---
    visualizer = Visualizer(desired_path=desired_path_data)
    anim = visualizer.create_animation(simulation_data, controller_status_list,
                                       interval=30, step=2, # interval in ms, step = how many sim points per anim frame
                                       true_m=true_m_val, true_I=true_I_val)
    # To save the animation:
    # print("Saving animation... (this may take a while)")
    # anim.save('simulation_animation.mp4', writer='ffmpeg', fps=30, dpi=150)
    # print("Animation saved as simulation_animation.mp4")
    plt.show() # Show animation plot

    # --- Test Final Results Plot ---
    # Need a separate call as the figure for animation is typically closed or reused
    # Re-create a Visualizer instance or call plot_final_results statically if it were designed that way.
    # For this class structure, we can call it on the same instance, it will create a new figure.
    visualizer.plot_final_results(simulation_data, controller_status_list,
                                  path_type_name="Circular",
                                  true_m=true_m_val, true_I=true_I_val)
    plt.show() # Show final results plot