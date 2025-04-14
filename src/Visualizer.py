import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np

class Visualizer:
    def __init__(self, desired_path):
        """
        Initialize the visualizer. Sets up the plot structure.
        :param desired_path: A numpy array of shape (N, 2) representing the desired path.
        """
        self.desired_path = desired_path
        self.fig = plt.figure(figsize=(18, 10)) # Increased figure size

        # Create a grid for subplots
        grid = plt.GridSpec(4, 3, hspace=0.7, wspace=0.3) # Adjusted spacing

        # Large plot for robot path (takes 4 rows, 2 columns)
        self.path_ax = self.fig.add_subplot(grid[:, :2])
        self.path_ax.set_aspect('equal', adjustable='box')
        self.path_ax.set_title("Robot Path Tracking")
        self.path_ax.set_xlabel("X-axis (m)")
        self.path_ax.set_ylabel("Y-axis (m)")
        self.path_ax.plot(desired_path[:, 0], desired_path[:, 1], 'r--', label='Desired Path', linewidth=1.5)
        # Initialize animated elements placeholders
        self.actual_path_line, = self.path_ax.plot([], [], 'g-', label='Actual Path', linewidth=1.5)
        self.robot_marker, = self.path_ax.plot([], [], 'bo', markersize=8, label="Robot Position")
        self.robot_arrow = patches.Arrow(0, 0, 0, 0, width=0.2, color='blue') # Placeholder arrow
        self.path_ax.add_patch(self.robot_arrow)
        self.path_ax.legend()
        self.path_ax.grid(True)

        # Small plot for wheel angular velocities (Top right)
        self.wheel_ax = self.fig.add_subplot(grid[0, 2])
        self.wheel_ax.set_title("Wheel Angular Velocities (Cmd)")
        self.wheel_ax.set_xlabel("Time (s)")
        self.wheel_ax.set_ylabel("Ang. Vel (rad/s)")
        self.left_wheel_line, = self.wheel_ax.plot([], [], 'b-', label="Left Wheel")
        self.right_wheel_line, = self.wheel_ax.plot([], [], 'g-', label="Right Wheel")
        self.wheel_ax.legend()
        self.wheel_ax.grid(True)

        # Small plot for forward velocity and yaw angular velocity (Middle right)
        self.velocity_ax = self.fig.add_subplot(grid[1, 2])
        self.velocity_ax.set_title("Robot Velocities (Actual)")
        self.velocity_ax.set_xlabel("Time (s)")
        self.velocity_ax.set_ylabel("Velocity")
        self.forward_velocity_line, = self.velocity_ax.plot([], [], 'r-', label="Forward (m/s)")
        self.yaw_velocity_line, = self.velocity_ax.plot([], [], 'm-', label="Yaw (rad/s)")
        self.velocity_ax.legend()
        self.velocity_ax.grid(True)

        # Small plot for Errors (Bottom right)
        self.error_ax = self.fig.add_subplot(grid[2, 2])
        self.error_ax.set_title("Tracking Errors")
        self.error_ax.set_xlabel("Time (s)")
        self.error_ax.set_ylabel("Error Value")
        self.error_fwd_line, = self.error_ax.plot([], [], 'c-', label="Forward Error (m)")
        self.error_lat_line, = self.error_ax.plot([], [], 'y-', label="Lateral Error (m)")
        self.error_theta_line, = self.error_ax.plot([], [], 'k-', label="Theta Error (rad)")
        self.error_ax.legend()
        self.error_ax.grid(True)

        # Small plot for Lyapunov Energy (Bottom right)
        self.energy_ax = self.fig.add_subplot(grid[3, 2])
        self.energy_ax.set_title("Lyapunov Energy (V)")
        self.energy_ax.set_xlabel("Time (s)")
        self.energy_ax.set_ylabel("Energy")
        self.energy_line, = self.energy_ax.plot([], [], 'orange', label="Energy V")
        self.energy_ax.legend()
        self.energy_ax.grid(True)

        # Animation related attributes
        self.sim_data = None
        self.errors_list = None
        self.energy_list = None
        self.anim_step = 1

    def _init_animation(self):
        """ Initialize elements for animation drawing """
        # Reset lines
        self.actual_path_line.set_data([], [])
        self.robot_marker.set_data([], [])
        self.left_wheel_line.set_data([], [])
        self.right_wheel_line.set_data([], [])
        self.forward_velocity_line.set_data([], [])
        self.yaw_velocity_line.set_data([], [])
        self.error_fwd_line.set_data([], [])
        self.error_lat_line.set_data([], [])
        self.error_theta_line.set_data([], [])
        self.energy_line.set_data([], [])

        # Reset arrow (remove and add new one, easier than updating position/angle)
        self.robot_arrow.remove()
        self.robot_arrow = patches.Arrow(0, 0, 0, 0, width=0.2, color='blue')
        self.path_ax.add_patch(self.robot_arrow)

        # Set dynamic plot limits based on data
        max_time = self.sim_data["time"][-1]
        max_wheel_vel = np.max(np.abs(self.sim_data["wheel_cmds"])) * 1.1
        max_robot_fwd_vel = np.max(np.abs(self.sim_data["robot_vels"][:, 0])) * 1.1
        max_robot_yaw_vel = np.max(np.abs(self.sim_data["robot_vels"][:, 1])) * 1.1
        max_vel_axis = max(max_robot_fwd_vel, max_robot_yaw_vel)
        max_err = np.max(np.abs(self.errors_list)) * 1.1 if self.errors_list else 0.1
        max_energy = np.max(np.abs(self.energy_list)) * 1.1 if self.energy_list else 0.1

        self.wheel_ax.set_xlim(0, max_time)
        self.wheel_ax.set_ylim(-max_wheel_vel, max_wheel_vel)
        self.velocity_ax.set_xlim(0, max_time)
        self.velocity_ax.set_ylim(-max_vel_axis, max_vel_axis)
        self.error_ax.set_xlim(0, max_time)
        self.error_ax.set_ylim(-max_err, max_err)
        self.energy_ax.set_xlim(0, max_time)
        self.energy_ax.set_ylim(0, max_energy) # Energy usually non-negative

        # Adjust path plot limits dynamically
        all_x = np.concatenate((self.desired_path[:, 0], self.sim_data["actual_path"][:, 0]))
        all_y = np.concatenate((self.desired_path[:, 1], self.sim_data["actual_path"][:, 1]))
        x_min, x_max = np.min(all_x), np.max(all_x)
        y_min, y_max = np.min(all_y), np.max(all_y)
        x_range = x_max - x_min
        y_range = y_max - y_min
        padding = max(x_range, y_range) * 0.1 # Add 10% padding
        self.path_ax.set_xlim(x_min - padding, x_max + padding)
        self.path_ax.set_ylim(y_min - padding, y_max + padding)


        return (self.actual_path_line, self.robot_marker, self.robot_arrow,
                self.left_wheel_line, self.right_wheel_line,
                self.forward_velocity_line, self.yaw_velocity_line,
                self.error_fwd_line, self.error_lat_line, self.error_theta_line,
                self.energy_line)

    def _animate(self, i):
        """ Animation update function for FuncAnimation """
        sim_index = i * self.anim_step
        if sim_index >= len(self.sim_data["time"]):
             sim_index = len(self.sim_data["time"]) - 1 # Ensure index is valid

        # Get data up to current simulation index
        time_data = self.sim_data["time"][:sim_index+1]
        actual_path_data = self.sim_data["actual_path"][:sim_index+1]
        current_pose = self.sim_data["actual_path"][sim_index]
        wheel_cmds_data = self.sim_data["wheel_cmds"][:sim_index+1]
        robot_vels_data = self.sim_data["robot_vels"][:sim_index+1]
        # Error and energy lists have one less element than time/path if collected inside loop starting step 1
        error_idx_end = min(sim_index, len(self.errors_list))
        energy_idx_end = min(sim_index, len(self.energy_list))
        errors_data = np.array(self.errors_list[:error_idx_end]) if self.errors_list else np.empty((0,3))
        energy_data = np.array(self.energy_list[:energy_idx_end]) if self.energy_list else np.empty(0)
        time_for_err_energy = self.sim_data["time"][1:max(error_idx_end, energy_idx_end)+1] # Time corresponding to errors/energy

        # Update Path Plot
        self.actual_path_line.set_data(actual_path_data[:, 0], actual_path_data[:, 1])
        self.robot_marker.set_data(current_pose[0], current_pose[1])
        # Update Arrow
        arrow_length = 0.5 # Visual length of arrow
        theta = current_pose[2]
        self.robot_arrow.remove() # Remove old arrow
        self.robot_arrow = patches.Arrow(current_pose[0], current_pose[1],
                                         arrow_length * np.cos(theta), arrow_length * np.sin(theta),
                                         width=0.2, color='blue')
        self.path_ax.add_patch(self.robot_arrow) # Add updated arrow

        # Update Wheel Velocity Plot
        self.left_wheel_line.set_data(time_data, wheel_cmds_data[:, 0])
        self.right_wheel_line.set_data(time_data, wheel_cmds_data[:, 1])

        # Update Robot Velocity Plot
        self.forward_velocity_line.set_data(time_data, robot_vels_data[:, 0])
        self.yaw_velocity_line.set_data(time_data, robot_vels_data[:, 1])

        # Update Error Plot
        if errors_data.shape[0] > 0:
            self.error_fwd_line.set_data(time_for_err_energy, errors_data[:, 0])
            self.error_lat_line.set_data(time_for_err_energy, errors_data[:, 1])
            self.error_theta_line.set_data(time_for_err_energy, errors_data[:, 2])

        # Update Energy Plot
        if energy_data.shape[0] > 0:
            self.energy_line.set_data(time_for_err_energy, energy_data)


        return (self.actual_path_line, self.robot_marker, self.robot_arrow,
                self.left_wheel_line, self.right_wheel_line,
                self.forward_velocity_line, self.yaw_velocity_line,
                self.error_fwd_line, self.error_lat_line, self.error_theta_line,
                self.energy_line)

    def create_animation(self, simulation_data, errors_list, energy_list, interval=50, step=10):
        """
        Creates the animation object.
        :param simulation_data: Dictionary from simulation.get_simulation_data().
        :param errors_list: List of error tuples (err_fwd, err_lat, err_theta).
        :param energy_list: List of energy values V.
        :param interval: Delay between frames in milliseconds.
        :param step: Number of simulation steps per animation frame.
        :return: matplotlib.animation.FuncAnimation object.
        """
        self.sim_data = simulation_data
        self.errors_list = errors_list
        self.energy_list = energy_list
        self.anim_step = step

        num_sim_steps = len(self.sim_data["time"])
        num_frames = num_sim_steps // step + (1 if num_sim_steps % step > 0 else 0)

        ani = animation.FuncAnimation(self.fig, self._animate, frames=num_frames,
                                      init_func=self._init_animation, blit=True, interval=interval, repeat=False)
        return ani

    def plot_final_results(self, simulation_data, errors_list, energy_list, path_type_name=""):
        """
        Generates static plots of errors and energy vs. time after the simulation.
        """
        # Create a new figure for final results
        fig_res, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        fig_res.suptitle(f"Simulation Results - {path_type_name} Path", fontsize=16)

        time_data = simulation_data["time"]
        # Error and energy lists have one less element if collected inside loop
        error_idx_end = len(errors_list)
        energy_idx_end = len(energy_list)
        errors_np = np.array(errors_list) if errors_list else np.empty((0,3))
        energy_np = np.array(energy_list) if energy_list else np.empty(0)
        time_for_err_energy = time_data[1:max(error_idx_end, energy_idx_end)+1]

        # Plot Errors
        if errors_np.shape[0] > 0:
            axs[0].plot(time_for_err_energy, errors_np[:, 0], 'c-', label="Forward Error (m)")
            axs[0].plot(time_for_err_energy, errors_np[:, 1], 'y-', label="Lateral Error (m)")
            axs[0].plot(time_for_err_energy, errors_np[:, 2], 'k-', label="Theta Error (rad)")
        axs[0].set_title("Tracking Errors vs. Time")
        axs[0].set_ylabel("Error")
        axs[0].legend()
        axs[0].grid(True)

        # Plot Energy
        if energy_np.shape[0] > 0:
            axs[1].plot(time_for_err_energy, energy_np, 'orange', label="Energy V")
        axs[1].set_title("Lyapunov Energy (V) vs. Time")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Energy")
        axs[1].legend()
        axs[1].grid(True)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Adjust layout to prevent title overlap