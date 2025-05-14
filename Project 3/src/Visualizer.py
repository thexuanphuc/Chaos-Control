import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np

class Visualizer:
    """
    Handles plotting and animation of the robot simulation results.
    Creates a multi-panel figure showing the robot's path, velocities,
    torques, errors, parameter estimates or torque errors, and disturbances over time.
    """
    def __init__(self, desired_path):
        """
        Initializes the visualizer and sets up the plot structure.

        :param desired_path: A numpy array of shape (N, 2) representing the desired path.
        """
        self.desired_path = desired_path
        self.fig = plt.figure(figsize=(22, 14))

        # Define a GridSpec for subplot layout
        grid = plt.GridSpec(6, 2, height_ratios=[3, 1, 1, 1, 1, 1], hspace=0.9, wspace=0.25)

        # --- Column 0: Path Plot ---
        self.path_ax = self.fig.add_subplot(grid[:, 0])
        self.path_ax.set_aspect('equal', adjustable='box')
        self.path_ax.set_title("Robot Path Tracking")
        self.path_ax.set_xlabel("X-axis (m)")
        self.path_ax.set_ylabel("Y-axis (m)")
        self.path_ax.plot(desired_path[:, 0], desired_path[:, 1], 'r--', label='Desired Path', linewidth=1.5, alpha=0.7)
        self.actual_path_line, = self.path_ax.plot([], [], 'g-', label='Actual Path', linewidth=2.0)
        self.robot_marker, = self.path_ax.plot([], [], 'bo', markersize=10, label="Robot Position", markeredgecolor='k')
        self.robot_orientation_arrow = patches.Arrow(0, 0, 0, 0, width=0.2, color='blue', zorder=5, alpha=0.8)
        self.path_ax.add_patch(self.robot_orientation_arrow)
        self.robot_velocity_arrow = patches.Arrow(0, 0, 0, 0, width=0.1, color='cyan', alpha=0.6, zorder=4)
        self.path_ax.add_patch(self.robot_velocity_arrow)
        self.path_ax.legend(loc='upper right')
        self.path_ax.grid(True, linestyle=':', alpha=0.7)

        # --- Column 1: Time Plots ---
        # Row 0: Robot Velocities
        self.velocity_ax = self.fig.add_subplot(grid[0, 1])
        self.velocity_ax.set_title("Robot Velocities")
        self.velocity_ax.set_ylabel("Velocity")
        self.v1_actual_line, = self.velocity_ax.plot([], [], 'r-', label="$v_1$ Actual (m/s)")
        self.omega_actual_line, = self.velocity_ax.plot([], [], 'm-', label="$\\omega$ Actual (rad/s)")
        self.v1_desired_line, = self.velocity_ax.plot([], [], 'r:', label="$v_{1d}$ Desired", linewidth=1.5)
        self.omega_desired_line, = self.velocity_ax.plot([], [], 'm:', label="$\\omega_d$ Desired", linewidth=1.5)
        self.velocity_ax.legend(fontsize='small', loc='best')
        self.velocity_ax.grid(True, linestyle=':', alpha=0.7)
        self.velocity_ax.tick_params(axis='x', labelbottom=False)

        # Row 1: Control Torques
        self.torque_ax = self.fig.add_subplot(grid[1, 1], sharex=self.velocity_ax)
        self.torque_ax.set_title("Control Torques (Commanded)")
        self.torque_ax.set_ylabel("Torque (Nm)")
        self.tau_left_line, = self.torque_ax.plot([], [], 'b-', label="$\\tau_L$ Left")
        self.tau_right_line, = self.torque_ax.plot([], [], 'g-', label="$\\tau_R$ Right")
        self.torque_ax.legend(fontsize='small', loc='best')
        self.torque_ax.grid(True, linestyle=':', alpha=0.7)
        self.torque_ax.tick_params(axis='x', labelbottom=False)

        # Row 2: Kinematic Errors
        self.kin_error_ax = self.fig.add_subplot(grid[2, 1], sharex=self.velocity_ax)
        self.kin_error_ax.set_title("Kinematic Errors (Path Following Layer)")
        self.kin_error_ax.set_ylabel("Error Value")
        self.error_fwd_line, = self.kin_error_ax.plot([], [], 'c-', label="Forward Err (m)")
        self.error_lat_line, = self.kin_error_ax.plot([], [], 'y-', label="Lateral Err (m)")
        self.error_theta_line, = self.kin_error_ax.plot([], [], 'k-', label="$\\theta$ Err (rad)")
        self.kin_error_ax.legend(fontsize='small', loc='best')
        self.kin_error_ax.grid(True, linestyle=':', alpha=0.7)
        self.kin_error_ax.tick_params(axis='x', labelbottom=False)

        # Row 3: Velocity Errors
        self.vel_error_ax = self.fig.add_subplot(grid[3, 1], sharex=self.velocity_ax)
        self.vel_error_ax.set_title("Velocity Errors ($\\eta = v - v_d$)")
        self.vel_error_ax.set_ylabel("Velocity Error")
        self.eta1_line, = self.vel_error_ax.plot([], [], 'r-', label="$\\eta_1 = v_1 - v_{1d}$")
        self.eta2_line, = self.vel_error_ax.plot([], [], 'm-', label="$\\eta_2 = \\omega - \\omega_d$")
        self.vel_error_ax.legend(fontsize='small', loc='best')
        self.vel_error_ax.grid(True, linestyle=':', alpha=0.7)
        self.vel_error_ax.tick_params(axis='x', labelbottom=False)

        # Row 4: Parameter Estimates or Torque Errors
        self.param_ax = self.fig.add_subplot(grid[4, 1], sharex=self.velocity_ax)
        self.param_ax.set_title("Parameter Estimates ($\\hat{p}$) or Torque Errors ($z$)")
        self.param_ax.set_ylabel("Value")
        self.line1, = self.param_ax.plot([], [], 'b-', label="$\\hat{m}$ or $z_1$ (left)")
        self.line2, = self.param_ax.plot([], [], 'g-', label="$\\hat{I}$ or $z_2$ (right)")
        self.m_true_line, = self.param_ax.plot([], [], 'b--', label="$m$ True", alpha=0.6)
        self.I_true_line, = self.param_ax.plot([], [], 'g--', label="$I$ True", alpha=0.6)
        self.param_ax.legend(fontsize='small', loc='best')
        self.param_ax.grid(True, linestyle=':', alpha=0.7)
        self.param_ax.tick_params(axis='x', labelbottom=False)

        # Row 5: Applied Disturbances
        self.dist_ax = self.fig.add_subplot(grid[5, 1], sharex=self.velocity_ax)
        self.dist_ax.set_title("Applied Disturbances")
        self.dist_ax.set_xlabel("Time (s)")
        self.dist_ax.set_ylabel("Disturbance")
        self.dist1_line, = self.dist_ax.plot([], [], 'r-', label="$\\tau_{d,v1}$ (on $v_1$ axis)")
        self.dist2_line, = self.dist_ax.plot([], [], 'm-', label="$\\tau_{d,\\omega}$ (on $\\omega$ axis)")
        self.dist_ax.legend(fontsize='small', loc='best')
        self.dist_ax.grid(True, linestyle=':', alpha=0.7)

        # Adjust layout manually to avoid tight_layout issues
        self.fig.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05, hspace=0.9, wspace=0.25)

        # Animation attributes
        self.sim_data = None
        self.controller_status_list = None
        self.anim_step = 1
        self.true_m = None
        self.true_I = None
        self.artists_to_animate = []
        self.use_torque_errors = False  # Flag to indicate if plotting z instead of p_hat

    def _init_animation(self):
        """Initializes elements for animation drawing."""
        # Determine if we're plotting p_hat or z
        self.use_torque_errors = 'z' in self.controller_status_list[0] if self.controller_status_list else False
        if self.use_torque_errors:
            self.param_ax.set_title("Torque Errors ($z$)")
            self.line1.set_label("$z_1$ (left torque err, Nm)")
            self.line2.set_label("$z_2$ (right torque err, Nm)")
            self.param_ax.legend(fontsize='small', loc='best')
        else:
            self.param_ax.set_title("Parameter Estimates ($\\hat{p}$)")
            self.line1.set_label("$\\hat{m}$ (mass est.)")
            self.line2.set_label("$\\hat{I}$ (inertia est.)")
            self.param_ax.legend(fontsize='small', loc='best')

        self.artists_to_animate = [
            self.actual_path_line, self.robot_marker,
            self.v1_actual_line, self.omega_actual_line, self.v1_desired_line, self.omega_desired_line,
            self.tau_left_line, self.tau_right_line,
            self.error_fwd_line, self.error_lat_line, self.error_theta_line,
            self.eta1_line, self.eta2_line,
            self.line1, self.line2,
            self.m_true_line, self.I_true_line,
            self.dist1_line, self.dist2_line
        ]
        for artist in self.artists_to_animate:
            if isinstance(artist, plt.Line2D):
                artist.set_data([], [])

        # Reset arrows
        if self.robot_orientation_arrow.axes:
            self.robot_orientation_arrow.remove()
        if self.robot_velocity_arrow.axes:
            self.robot_velocity_arrow.remove()
        self.robot_orientation_arrow = patches.Arrow(0, 0, 0.01, 0, width=0.2, color='blue', zorder=5, alpha=0.8)
        self.robot_velocity_arrow = patches.Arrow(0, 0, 0, 0, width=0.1, color='cyan', alpha=0.6, zorder=4)
        self.path_ax.add_patch(self.robot_orientation_arrow)
        self.path_ax.add_patch(self.robot_velocity_arrow)
        self.artists_to_animate.extend([self.robot_orientation_arrow, self.robot_velocity_arrow])

        # Set dynamic plot limits
        max_time = self.sim_data["time"][-1] if len(self.sim_data["time"]) > 0 else 1.0
        num_controller_pts = len(self.controller_status_list)
        vels_actual = self.sim_data["robot_vels"]
        vels_desired = np.array([s['v_d'] for s in self.controller_status_list]) if num_controller_pts > 0 else np.zeros((0,2))
        torques = self.sim_data["torques_cmd"][1:num_controller_pts+1]
        disturbances = self.sim_data["disturbances"][1:num_controller_pts+1]
        kin_errors = np.array([s['kin_errors'] for s in self.controller_status_list]) if num_controller_pts > 0 else np.zeros((0,3))
        vel_errors_eta = np.array([s['eta'] for s in self.controller_status_list]) if num_controller_pts > 0 else np.zeros((0,2))
        if self.use_torque_errors:
            data_to_plot = np.array([s['z'] for s in self.controller_status_list]) if num_controller_pts > 0 else np.zeros((0,2))
        else:
            data_to_plot = np.array([s['p_hat'] for s in self.controller_status_list]) if num_controller_pts > 0 else np.zeros((0,2))

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
            if not has_data:
                return (-0.1, 0.1)
            if np.isclose(min_val, max_val):
                min_val -= 0.1
                max_val += 0.1
            padding = abs(max_val - min_val) * pad_factor
            if not np.isfinite(padding) or padding == 0:
                padding = abs(max_val)*pad_factor if np.isfinite(max_val) and max_val !=0 else 0.1
            lim_min, lim_max = min_val - padding, max_val + padding
            if zero_center:
                abs_max_val = max(abs(lim_min), abs(lim_max))
                if not np.isfinite(abs_max_val):
                    abs_max_val = 1.0
                lim_min, lim_max = -abs_max_val, abs_max_val
            if lim_max <= lim_min:
                lim_max = lim_min + 0.2
            return lim_min, lim_max

        self.velocity_ax.set_ylim(get_ylims([vels_actual, vels_desired], zero_center=True))
        self.torque_ax.set_ylim(get_ylims([torques], zero_center=True))
        self.kin_error_ax.set_ylim(get_ylims([kin_errors], zero_center=True))
        self.vel_error_ax.set_ylim(get_ylims([vel_errors_eta], zero_center=True))
        param_plot_data = [data_to_plot]
        if not self.use_torque_errors and self.true_m is not None:
            param_plot_data.append(np.array([[self.true_m, self.true_I]]))
        param_lims = get_ylims(param_plot_data, pad_factor=0.2)
        if not self.use_torque_errors and param_lims[0] > -0.1 and param_lims[0] < 0.1:
            param_lims = (min(param_lims[0], -0.1), param_lims[1])
        self.param_ax.set_ylim(param_lims)
        self.dist_ax.set_ylim(get_ylims([disturbances], zero_center=True))

        for ax in [self.velocity_ax, self.torque_ax, self.kin_error_ax, self.vel_error_ax, self.param_ax, self.dist_ax]:
            ax.set_xlim(0, max_time)

        all_x_coords = np.concatenate((self.desired_path[:, 0], self.sim_data["actual_path"][:, 0]))
        all_y_coords = np.concatenate((self.desired_path[:, 1], self.sim_data["actual_path"][:, 1]))
        self.path_ax.set_xlim(get_ylims([all_x_coords]))
        self.path_ax.set_ylim(get_ylims([all_y_coords]))

        if not self.use_torque_errors:
            if self.true_m is not None:
                self.m_true_line.set_data([0, max_time], [self.true_m, self.true_m])
            if self.true_I is not None:
                self.I_true_line.set_data([0, max_time], [self.true_I, self.true_I])

        self.path_ax.set_title(f"Robot Path Tracking (T=0.00s)")
        return self.artists_to_animate

    def _animate(self, frame_idx):
        """Animation update function."""
        sim_idx = min(frame_idx * self.anim_step, len(self.sim_data["time"]) - 1)
        controller_data_idx = sim_idx
        time_slice = self.sim_data["time"][:sim_idx+1]
        actual_path_slice = self.sim_data["actual_path"][:sim_idx+1]
        vels_actual_slice = self.sim_data["robot_vels"][:sim_idx+1]

        if controller_data_idx < len(self.controller_status_list):
            current_status = self.controller_status_list[controller_data_idx]
            time_controller_slice = self.sim_data["time"][:controller_data_idx+1]
            vels_desired_plot_data = np.array([s['v_d'] for s in self.controller_status_list[:controller_data_idx+1]])
            kin_errors_plot_data = np.array([s['kin_errors'] for s in self.controller_status_list[:controller_data_idx+1]])
            vel_errors_eta_plot_data = np.array([s['eta'] for s in self.controller_status_list[:controller_data_idx+1]])
            data_to_plot = np.array([s['z' if self.use_torque_errors else 'p_hat'] for s in self.controller_status_list[:controller_data_idx+1]])
            time_for_cmds_dist = self.sim_data["time"][1:sim_idx+2]
            torques_plot_data = self.sim_data["torques_cmd"][1:sim_idx+2]
            disturbances_plot_data = self.sim_data["disturbances"][1:sim_idx+2]

            current_pose = actual_path_slice[-1]
            x, y, theta = current_pose
            self.actual_path_line.set_data(actual_path_slice[:, 0], actual_path_slice[:, 1])
            self.robot_marker.set_data([x], [y])
            arrow_length = max(0.2, np.mean(np.abs(self.path_ax.get_xlim())) * 0.03)
            if self.robot_orientation_arrow.axes:
                self.robot_orientation_arrow.remove()
            self.robot_orientation_arrow = patches.Arrow(x, y, arrow_length * np.cos(theta), arrow_length * np.sin(theta),
                                                        width=arrow_length*0.5, color='blue', zorder=5, alpha=0.8)
            self.path_ax.add_patch(self.robot_orientation_arrow)
            v1_current = vels_actual_slice[-1,0]
            vel_arrow_scale = arrow_length * 0.8
            if self.robot_velocity_arrow.axes:
                self.robot_velocity_arrow.remove()
            self.robot_velocity_arrow = patches.Arrow(x, y, v1_current * vel_arrow_scale * np.cos(theta), v1_current * vel_arrow_scale * np.sin(theta),
                                                    width=arrow_length*0.25, color='cyan', alpha=0.6, zorder=4)
            self.path_ax.add_patch(self.robot_velocity_arrow)

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
            if data_to_plot.shape[0] == time_controller_slice.shape[0]:
                self.line1.set_data(time_controller_slice, data_to_plot[:, 0])
                self.line2.set_data(time_controller_slice, data_to_plot[:, 1])
            if disturbances_plot_data.shape[0] == time_for_cmds_dist.shape[0]:
                self.dist1_line.set_data(time_for_cmds_dist, disturbances_plot_data[:, 0])
                self.dist2_line.set_data(time_for_cmds_dist, disturbances_plot_data[:, 1])

        self.path_ax.set_title(f"Robot Path Tracking (T={time_slice[-1]:.2f}s)")
        self.artists_to_animate = [
            self.actual_path_line, self.robot_marker, self.robot_orientation_arrow, self.robot_velocity_arrow,
            self.v1_actual_line, self.omega_actual_line, self.v1_desired_line, self.omega_desired_line,
            self.tau_left_line, self.tau_right_line,
            self.error_fwd_line, self.error_lat_line, self.error_theta_line,
            self.eta1_line, self.eta2_line,
            self.line1, self.line2,
            self.dist1_line, self.dist2_line
        ]
        return self.artists_to_animate

    def create_animation(self, simulation_data, controller_status_list, interval=50, step=10, true_m=None, true_I=None):
        """
        Creates the animation object.
        """
        self.sim_data = simulation_data
        num_sim_intervals = len(simulation_data['time']) - 1
        self.controller_status_list = controller_status_list[:num_sim_intervals]
        if len(self.controller_status_list) != num_sim_intervals:
            print(f"Warning: Length mismatch in create_animation. Status list {len(self.controller_status_list)}, Sim intervals {num_sim_intervals}")
        self.anim_step = step
        self.true_m = true_m
        self.true_I = true_I
        num_total_sim_points = len(self.sim_data["time"])
        num_animation_frames = (num_total_sim_points - 1) // self.anim_step + 1
        ani = animation.FuncAnimation(self.fig, self._animate, frames=num_animation_frames,
                                      init_func=self._init_animation, blit=False,
                                      interval=interval, repeat=False)
        return ani

    def plot_final_results(self, simulation_data, controller_status_list, path_type_name="", true_m=None, true_I=None):
        """
        Generates static plots of key simulation variables vs. time.
        """
        fig_static, axs_static = plt.subplots(5, 1, figsize=(12, 18), sharex=True)
        fig_static.suptitle(f"Control Simulation Results - {path_type_name} Path", fontsize=16)
        time_data = simulation_data["time"]
        num_control_outputs = len(controller_status_list)
        if num_control_outputs == 0:
            print("No controller status data to plot for final results.")
            return
        time_controller_axis = time_data[:num_control_outputs]
        time_command_axis = time_data[1:num_control_outputs+1]
        kin_errors = np.array([s['kin_errors'] for s in controller_status_list])
        vel_errors_eta = np.array([s['eta'] for s in controller_status_list])
        use_torque_errors = 'z' in controller_status_list[0] if controller_status_list else False
        if use_torque_errors:
            data_to_plot = np.array([s['z'] for s in controller_status_list])
            title = "Torque Errors ($z$) vs. Time"
            ylabel = "Torque Error (Nm)"
            labels = ["$z_1$ (left torque err)", "$z_2$ (right torque err)"]
        else:
            data_to_plot = np.array([s['p_hat'] for s in controller_status_list])
            title = "Parameter Estimates vs. Time"
            ylabel = "Estimated Value"
            labels = ["$\\hat{m}$ (mass est.)", "$\\hat{I}$ (inertia est.)"]
        torques_cmd = simulation_data["torques_cmd"][1:num_control_outputs+1]
        disturbances_applied = simulation_data["disturbances"][1:num_control_outputs+1]
        axs_static[0].plot(time_controller_axis, kin_errors[:, 0], 'c-', label="Forward Err (m)")
        axs_static[0].plot(time_controller_axis, kin_errors[:, 1], 'y-', label="Lateral Err (m)")
        axs_static[0].plot(time_controller_axis, kin_errors[:, 2], 'k-', label="$\\theta$ Err (rad)")
        axs_static[0].set_title("Kinematic Errors vs. Time")
        axs_static[0].set_ylabel("Error Value")
        axs_static[0].legend(fontsize='small')
        axs_static[0].grid(True, linestyle=':', alpha=0.7)
        axs_static[1].plot(time_controller_axis, vel_errors_eta[:, 0], 'r-', label="$\\eta_1 = v_1 - v_{1d}$")
        axs_static[1].plot(time_controller_axis, vel_errors_eta[:, 1], 'm-', label="$\\eta_2 = \\omega - \\omega_d$")
        axs_static[1].set_title("Velocity Errors ($\\eta$) vs. Time")
        axs_static[1].set_ylabel("Velocity Error")
        axs_static[1].legend(fontsize='small')
        axs_static[1].grid(True, linestyle=':', alpha=0.7)
        axs_static[2].plot(time_controller_axis, data_to_plot[:, 0], 'b-', label=labels[0])
        axs_static[2].plot(time_controller_axis, data_to_plot[:, 1], 'g-', label=labels[1])
        if not use_torque_errors:
            if true_m is not None:
                axs_static[2].axhline(true_m, color='b', linestyle='--', alpha=0.6, label="$m$ True")
            if true_I is not None:
                axs_static[2].axhline(true_I, color='g', linestyle='--', alpha=0.6, label="$I$ True")
        axs_static[2].set_title(title)
        axs_static[2].set_ylabel(ylabel)
        axs_static[2].legend(fontsize='small')
        axs_static[2].grid(True, linestyle=':', alpha=0.7)
        if torques_cmd.shape[0] == time_command_axis.shape[0] and torques_cmd.size > 0:
            axs_static[3].plot(time_command_axis, torques_cmd[:, 0], 'b-', label="$\\tau_L$ Left Cmd")
            axs_static[3].plot(time_command_axis, torques_cmd[:, 1], 'g-', label="$\\tau_R$ Right Cmd")
        axs_static[3].set_title("Commanded Control Torques vs. Time")
        axs_static[3].set_ylabel("Torque (Nm)")
        axs_static[3].legend(fontsize='small')
        axs_static[3].grid(True, linestyle=':', alpha=0.7)
        if disturbances_applied.shape[0] == time_command_axis.shape[0] and disturbances_applied.size > 0:
            axs_static[4].plot(time_command_axis, disturbances_applied[:, 0], 'r-', label="$\\tau_{d,v1}$ (on $v_1$ axis)")
            axs_static[4].plot(time_command_axis, disturbances_applied[:, 1], 'm-', label="$\\tau_{d,\\omega}$ (on $\\omega$ axis)")
        axs_static[4].set_title("Applied Disturbances vs. Time")
        axs_static[4].set_xlabel("Time (s)")
        axs_static[4].set_ylabel("Disturbance Torque")
        axs_static[4].legend(fontsize='small')
        axs_static[4].grid(True, linestyle=':', alpha=0.7)
        fig_static.subplots_adjust(left=0.1, right=0.9, top=0.95, bottom=0.05, hspace=0.4)