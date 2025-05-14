# Visualizer.py
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np

class Visualizer:
    """
    Handles plotting and animation of the robot simulation results.
    Creates a multi-panel figure showing the robot's path, velocities,
    torques, errors, parameter estimates, and disturbances over time.
    Includes a separate comparison plot for velocities, u_robust, eta, p_hat, tau, Lyapunov functions, and control.
    """
    def __init__(self, desired_path):
        """
        Initializes the visualizer.
        :param desired_path: A numpy array of shape (N, 2) representing the desired path.
        """
        self.desired_path = desired_path if desired_path is not None and desired_path.ndim == 2 and desired_path.shape[0] > 0 else np.array([[0.0,0.0]])
        
        # Attributes for animation - initialized when create_animation is called
        self.fig_anim = None
        self.anim_axes = {} # Store axes by key: 'path', 'vel', 'err', 'dyn'
        self.anim_lines = {} # Store line2D objects for animation
        self.anim_patches = {} # Store patches like arrows

        self.sim_data_anim = None
        self.controller_status_list_anim = None
        self.anim_step_size = 1
        self.true_m_anim, self.true_I_anim = None, None
        self.robot_radius_anim = 0.15
        self.dt_anim = 0.02 

    def _compute_lyapunov_values(self, controller_status_list, true_m=None, true_I=None):
        """
        Computes Lyapunov function values for kinematic and dynamic layers.
        :param controller_status_list: List of controller status dictionaries.
        :param true_m: True mass for parameter error computation.
        :param true_I: True inertia for parameter error computation.
        :return: Tuple of (V_kin, V_dyn) arrays, where V_kin is kinematic Lyapunov function
                 and V_dyn is dynamic Lyapunov function.
        """
        V_kin = np.full(len(controller_status_list), np.nan)
        V_dyn = np.full(len(controller_status_list), np.nan)

        for i, status in enumerate(controller_status_list):
            if not isinstance(status, dict):
                continue
            # Kinematic Lyapunov: V_kin = 0.5 * (e1^2 + e2^2 + e3^2)
            kin_errors = status.get('kin_errors', [np.nan]*3)
            if len(kin_errors) >= 3 and np.all(np.isfinite(kin_errors)):
                e1, e2, e3 = kin_errors
                V_kin[i] = 0.5 * (e1**2 + e2**2 + e3**2)

            # Dynamic Lyapunov: V_dyn ≈ 0.5 * (eta1^2 + eta2^2)
            eta = status.get('eta', [np.nan]*2)
            if len(eta) >= 2 and np.all(np.isfinite(eta)):
                V_dyn[i] = 0.5 * (eta[0]**2 + eta[1]**2)

        return V_kin, V_dyn

    def _plot_comparison(self, sim_data, controller_status_list, true_m=None, true_I=None):
        """
        Creates a separate figure comparing velocities, u_robust, eta, p_hat, tau, Lyapunov functions, and control.
        :param sim_data: Simulation data dictionary.
        :param controller_status_list: List of controller status dictionaries.
        :param true_m: True mass for plotting.
        :param true_I: True inertia for plotting.
        """
        fig, axs = plt.subplots(4, 2, figsize=(16, 18)) # Add this line to define fig and axs

        time_axis = sim_data["time"]
        num_time_points = len(time_axis)
        
        # Process controller status list
        processed_status_list = []
        if controller_status_list:
            for i in range(num_time_points):
                if i < len(controller_status_list) and isinstance(controller_status_list[i], dict):
                    processed_status_list.append(controller_status_list[i])
                else:
                    last_valid = next((s for s in reversed(controller_status_list[:i]) if isinstance(s, dict)), {})
                    processed_status_list.append(last_valid)
        else:
            processed_status_list = [{}] * num_time_points

        # Extract data with defaults
        def get_status_array(key, num_elements, default_val=np.nan):
            data_list = [s.get(key, np.full(num_elements, default_val)) for s in processed_status_list]
            arr = np.array(data_list)
            return arr if arr.ndim == 2 and arr.shape[1] == num_elements else np.full((len(processed_status_list), num_elements), default_val)

        actual_vels = sim_data["robot_vels"]
        desired_vels = get_status_array('v_d', 2)
        u_robust = get_status_array('u_robust', 2)
        eta = get_status_array('eta', 2)
        p_hats = get_status_array('p_hat', 2)
        tau_cmd = sim_data["torques_cmd"]
        tau_bar_cmd = get_status_array('tau_bar_cmd', 2)
        V_kin, V_dyn = self._compute_lyapunov_values(processed_status_list, true_m, true_I)

        # Create figure
        fig.suptitle("Comparison of Key Variables", fontsize=16, fontweight='bold')

        # Velocities
        axs[0, 0].plot(time_axis, actual_vels[:, 0], 'b-', label="$v_1$ Act")
        axs[0, 0].plot(time_axis, desired_vels[:, 0], 'b:', label="$v_1$ Des")
        axs[0, 0].plot(time_axis, actual_vels[:, 1], 'g-', label="$\omega$ Act")
        axs[0, 0].plot(time_axis, desired_vels[:, 1], 'g:', label="$\omega$ Des")
        axs[0, 0].set_title("Velocities")
        axs[0, 0].set_ylabel("Velocity (m/s, rad/s)")
        axs[0, 0].legend(fontsize='x-small')
        axs[0, 0].grid(True, ls=':')

        # u_robust
        axs[0, 1].plot(time_axis, u_robust[:, 0], 'brown', label="Robust $F_x$")
        axs[0, 1].plot(time_axis, u_robust[:, 1], 'olive', label="Robust $M_z$")
        axs[0, 1].set_title("Robust Terms")
        axs[0, 1].set_ylabel("Force/Torque")
        axs[0, 1].legend(fontsize='x-small')
        axs[0, 1].grid(True, ls=':')

        # eta
        axs[1, 0].plot(time_axis, eta[:, 0], 'r-', label="$\eta_1 (v_1 err)$")
        axs[1, 0].plot(time_axis, eta[:, 1], 'purple', label="$\eta_2 (\omega err)$")
        axs[1, 0].set_title("Velocity Errors ($\eta$)")
        axs[1, 0].set_ylabel("Error")
        axs[1, 0].legend(fontsize='x-small')
        axs[1, 0].grid(True, ls=':')

        # p_hat
        axs[1, 1].plot(time_axis, p_hats[:, 0], 'k-', label="$\hat{m}$")
        if true_m is not None:
            axs[1, 1].axhline(true_m, color='k', ls=':', label="$m_{true}$")
        axs[1, 1].plot(time_axis, p_hats[:, 1], 'dimgray', label="$\hat{I}$")
        if true_I is not None:
            axs[1, 1].axhline(true_I, color='dimgray', ls=':', label="$I_{true}$")
        axs[1, 1].set_title("Parameter Estimates")
        axs[1, 1].set_ylabel("Params (kg, kg·m²)")
        axs[1, 1].legend(fontsize='x-small')
        axs[1, 1].grid(True, ls=':')

        # tau
        axs[2, 0].plot(time_axis, tau_cmd[:, 0], 'c-', label="$\\tau_R$")
        axs[2, 0].plot(time_axis, tau_cmd[:, 1], 'm-', label="$\\tau_L$")
        axs[2, 0].set_title("Wheel Torques")
        axs[2, 0].set_ylabel("Torque (Nm)")
        axs[2, 0].legend(fontsize='x-small')
        axs[2, 0].grid(True, ls=':')

        # tau_bar_cmd
        axs[2, 1].plot(time_axis, tau_bar_cmd[:, 0], 'teal', label="$F_x$ Cmd")
        axs[2, 1].plot(time_axis, tau_bar_cmd[:, 1], 'orange', label="$M_z$ Cmd")
        axs[2, 1].set_title("Generalized Forces")
        axs[2, 1].set_ylabel("Force/Torque")
        axs[2, 1].legend(fontsize='x-small')
        axs[2, 1].grid(True, ls=':')

        # Lyapunov Functions
        axs[3, 0].plot(time_axis, V_kin, 'blue', label="$V_{kin}$")
        axs[3, 0].plot(time_axis, V_dyn, 'red', label="$V_{dyn}$")
        axs[3, 0].set_title("Lyapunov Functions")
        axs[3, 0].set_ylabel("Value")
        axs[3, 0].set_xlabel("Time (s)")
        axs[3, 0].legend(fontsize='x-small')
        axs[3, 0].grid(True, ls=':')

        # Empty subplot (or additional plot if needed)
        axs[3, 1].axis('off')

        for ax_r in axs[:-1, :]:
            for ax_c in ax_r:
                plt.setp(ax_c.get_xticklabels(), visible=False)
        
        fig.tight_layout(rect=[0, 0, 1, 0.96])

    def _setup_animation_plots(self):
        """ Sets up the subplots for the animation figure. """
        if self.fig_anim is not None: plt.close(self.fig_anim) # Close previous figure if any

        self.fig_anim = plt.figure(figsize=(16, 9)) # Common 16:9 aspect ratio
        # GridSpec: Path plot on the left (larger), time series plots on the right
        gs = self.fig_anim.add_gridspec(3, 2, width_ratios=[2,1.5], height_ratios=[1,1,1], hspace=0.4, wspace=0.3)

        self.anim_axes['path'] = self.fig_anim.add_subplot(gs[:, 0]) # Path plot spans all rows, first column
        self.anim_axes['vel'] = self.fig_anim.add_subplot(gs[0, 1])  # Velocities
        self.anim_axes['err'] = self.fig_anim.add_subplot(gs[1, 1], sharex=self.anim_axes['vel']) # Combined errors
        self.anim_axes['dyn'] = self.fig_anim.add_subplot(gs[2, 1], sharex=self.anim_axes['vel']) # Torques, Params, Dyn Terms

        # --- Path Plot Setup ---
        ax = self.anim_axes['path']
        ax.set_title("Robot Path Tracking")
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        if self.desired_path.size > 0:
            ax.plot(self.desired_path[:, 0], self.desired_path[:, 1], 'r--', label="Desired Path", lw=1.5, alpha=0.6)
        self.anim_lines['actual_path'], = ax.plot([], [], 'b-', label="Actual Path (Measured)", lw=2)
        self.anim_lines['true_path'], = ax.plot([], [], 'k:', label="True Path", lw=1, alpha=0.5)
        self.anim_lines['robot_marker'], = ax.plot([], [], 'o', color='deepskyblue', ms=7, mec='black', mew=0.5, label="Robot (Measured)")
        self.anim_patches['orientation_arrow'] = patches.Arrow(0,0,0,0, width=0.1, color='deepskyblue', alpha=0.7, zorder=10)
        ax.add_patch(self.anim_patches['orientation_arrow'])
        ax.legend(loc='best', fontsize='x-small')
        ax.axis('equal')
        ax.grid(True, linestyle=':', alpha=0.5)

        # --- Time Plots Setup ---
        time_plot_configs = {
            'vel': {"title": "Velocities", "ylabel": "Velocity"},
            'err': {"title": "Errors (Kinematic & Velocity)", "ylabel": "Error Value"},
            'dyn': {"title": "Dynamics & Control Inputs", "ylabel": "Value"}
        }
        for key, config in time_plot_configs.items():
            ax = self.anim_axes[key]
            ax.set_title(config["title"], fontsize=10)
            ax.set_ylabel(config["ylabel"], fontsize=9)
            ax.grid(True, linestyle=':', alpha=0.5)
            if key != 'dyn': plt.setp(ax.get_xticklabels(), visible=False)
        self.anim_axes['dyn'].set_xlabel("Time (s)", fontsize=9)

        # Initialize lines for animation plots
        ax = self.anim_axes['vel']
        self.anim_lines['v1_act'], = ax.plot([],[], 'b-', label="$v_1$ Act", lw=1.5)
        self.anim_lines['om_act'], = ax.plot([],[], 'g-', label="$\omega$ Act", lw=1.5)
        self.anim_lines['v1_des'], = ax.plot([],[], 'b:', label="$v_1$ Des", lw=1)
        self.anim_lines['om_des'], = ax.plot([],[], 'g:', label="$\omega$ Des", lw=1)
        ax.legend(fontsize='xx-small', loc='best')

        ax = self.anim_axes['err']
        self.anim_lines['e1'], = ax.plot([],[], color='orangered', label="$e_1$")
        self.anim_lines['e2'], = ax.plot([],[], color='darkviolet', label="$e_2$")
        self.anim_lines['e3'], = ax.plot([],[], color='teal', label="$e_3$")
        self.anim_lines['eta1'], = ax.plot([],[], color='orangered', ls=':', label="$\eta_1(v_1)$")
        self.anim_lines['eta2'], = ax.plot([],[], color='darkviolet', ls=':', label="$\eta_2(\omega)$")
        ax.legend(fontsize='xx-small', loc='best')
        
        ax = self.anim_axes['dyn'] # Primary y-axis for torques
        self.anim_lines['tau_r'], = ax.plot([], [], 'c-', label="$\\tau_R$")
        self.anim_lines['tau_l'], = ax.plot([], [], 'm-', label="$\\tau_L$")
        
        # Secondary y-axis for parameters on the 'dyn' plot
        ax_param_twin = ax.twinx()
        self.anim_axes['dyn_param_twin'] = ax_param_twin # Store for later use
        self.anim_lines['m_hat'], = ax_param_twin.plot([], [], 'k-', label="$\hat{m}$", lw=1.5, alpha=0.8)
        self.anim_lines['I_hat'], = ax_param_twin.plot([], [], 'dimgray', ls='-', label="$\hat{I}$", lw=1.5, alpha=0.8)
        self.anim_lines['m_true'], = ax_param_twin.plot([], [], 'k--', label="$m_{true}$", lw=1, alpha=0.6)
        self.anim_lines['I_true'], = ax_param_twin.plot([], [], 'dimgray', ls='--', label="$I_{true}$", lw=1, alpha=0.6)
        ax_param_twin.set_ylabel("Params", fontsize=9, color='dimgray')
        ax_param_twin.tick_params(axis='y', labelcolor='dimgray')
        
        # Consolidate legends for dyn plot
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax_param_twin.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2, fontsize='xx-small', loc='upper left')

        self.fig_anim.tight_layout(rect=[0, 0, 1, 0.96]) # Adjust layout for suptitle
        
        # Collect all line artists that need set_data called
        all_line_artists = [self.anim_lines[key] for key in self.anim_lines]
        return all_line_artists + [self.anim_patches['orientation_arrow']] # Arrow needs to be handled separately

    def _init_anim_data_ranges(self):
        """ Sets the y-axis limits for time plots based on the full dataset. """
        if not self.sim_data_anim or not self.controller_status_list_anim: return

        times = self.sim_data_anim["time"]
        max_time = times[-1] if len(times) > 0 else 1.0

        def get_min_max(data_arrays, margin_factor=0.1):
            min_val, max_val = np.inf, -np.inf
            valid_data_exists = False
            for arr_or_val in data_arrays:
                if arr_or_val is None: continue
                arr = np.asarray(arr_or_val) # Handle single values like true_m
                if arr.size > 0:
                    finite_arr = arr[np.isfinite(arr)]
                    if finite_arr.size > 0:
                        valid_data_exists = True
                        min_val = min(min_val, np.min(finite_arr))
                        max_val = max(max_val, np.max(finite_arr))
            if not valid_data_exists: return -0.1, 0.1
            if np.isclose(min_val, max_val):
                padding_abs = 0.1 * abs(min_val) if abs(min_val) > 1e-2 else 0.1
                min_val -= padding_abs
                max_val += padding_abs
            padding = (max_val - min_val) * margin_factor
            return min_val - padding, max_val + padding
        
        # Velocities
        actual_v = self.sim_data_anim["robot_vels"]
        des_v_list = [s.get('v_d', np.full(2, np.nan)) for s in self.controller_status_list_anim if isinstance(s, dict)]
        des_v = np.array(des_v_list) if des_v_list else np.array([[]])
        self.anim_axes['vel'].set_ylim(get_min_max([actual_v, des_v if des_v.ndim == 2 else None]))

        # Errors
        kin_e_list = [s.get('kin_errors', np.full(3, np.nan)) for s in self.controller_status_list_anim if isinstance(s, dict)]
        kin_e = np.array(kin_e_list) if kin_e_list else np.array([[]])
        vel_e_list = [s.get('eta', np.full(2, np.nan)) for s in self.controller_status_list_anim if isinstance(s, dict)]
        vel_e = np.array(vel_e_list) if vel_e_list else np.array([[]])
        self.anim_axes['err'].set_ylim(get_min_max([kin_e if kin_e.ndim == 2 else None, vel_e if vel_e.ndim == 2 else None]))
        
        # Dynamics (Torques on primary, Params on secondary)
        torques = self.sim_data_anim["torques_cmd"]
        self.anim_axes['dyn'].set_ylim(get_min_max([torques]))
        
        phat_list = [s.get('p_hat', np.full(2,np.nan)) for s in self.controller_status_list_anim if isinstance(s,dict)]
        phats = np.array(phat_list) if phat_list else np.array([[]])
        param_data_for_lim = [phats if phats.ndim == 2 else None]
        if self.true_m_anim is not None: param_data_for_lim.append(self.true_m_anim)
        if self.true_I_anim is not None: param_data_for_lim.append(self.true_I_anim)
        self.anim_axes['dyn_param_twin'].set_ylim(get_min_max(param_data_for_lim))

        for key in ['vel', 'err', 'dyn']: self.anim_axes[key].set_xlim(0, max_time)
        
        # Path plot limits
        all_x = self.desired_path[:,0].copy()
        all_y = self.desired_path[:,1].copy()
        if "actual_path" in self.sim_data_anim and self.sim_data_anim["actual_path"].size > 0:
            all_x = np.concatenate((all_x, self.sim_data_anim["actual_path"][:,0]))
            all_y = np.concatenate((all_y, self.sim_data_anim["actual_path"][:,1]))
        if all_x.size > 0:
            self.anim_axes['path'].set_xlim(np.nanmin(all_x) - 0.5, np.nanmax(all_x) + 0.5) # Increased padding
            self.anim_axes['path'].set_ylim(np.nanmin(all_y) - 0.5, np.nanmax(all_y) + 0.5)

        # Set data for true parameter lines (static throughout animation)
        if self.true_m_anim is not None: self.anim_lines['m_true'].set_data([0, max_time], [self.true_m_anim, self.true_m_anim])
        if self.true_I_anim is not None: self.anim_lines['I_true'].set_data([0, max_time], [self.true_I_anim, self.true_I_anim])

    def _init_anim_func(self):
        """Initializes the animation plot elements by clearing their data."""
        for key, line in self.anim_lines.items():
            line.set_data([], [])
        # Arrow patch is handled by removing and re-adding in _animate_frame
        self._init_anim_data_ranges() # Set fixed y-limits after setup
        
        # Return all line artists and the arrow patch
        return [self.anim_lines[key] for key in self.anim_lines] + [self.anim_patches['orientation_arrow']]

    def _animate_frame(self, i):
        """Updates the plot elements for frame i of the animation."""
        idx = min(i * self.anim_step_size, len(self.sim_data_anim["time"]) - 1)
        
        time_s = self.sim_data_anim["time"][:idx+1]
        actual_p_s = self.sim_data_anim["actual_path"][:idx+1, :]
        true_p_s = self.sim_data_anim.get("true_path", actual_p_s)[:idx+1, :]
        actual_v_s = self.sim_data_anim["robot_vels"][:idx+1, :]
        torques_s = self.sim_data_anim["torques_cmd"][:idx+1, :]
        
        status_s_list = self.controller_status_list_anim[:idx+1] # List of dicts

        # Path plot
        self.anim_lines['actual_path'].set_data(actual_p_s[:, 0], actual_p_s[:, 1])
        self.anim_lines['true_path'].set_data(true_p_s[:, 0], true_p_s[:, 1])
        curr_x, curr_y, curr_th = actual_p_s[-1, 0], actual_p_s[-1, 1], actual_p_s[-1, 2]
        self.anim_lines['robot_marker'].set_data([curr_x], [curr_y])
        
        # Update arrow
        arrow_patch = self.anim_patches['orientation_arrow']
        if arrow_patch.axes: arrow_patch.remove() # Remove old arrow
        arrow_len = self.robot_radius_anim * 1.2
        self.anim_patches['orientation_arrow'] = patches.Arrow(curr_x, curr_y,
                                                              arrow_len * np.cos(curr_th), arrow_len * np.sin(curr_th),
                                                              width=self.robot_radius_anim*0.4, color='deepskyblue', alpha=0.7, zorder=10)
        self.anim_axes['path'].add_patch(self.anim_patches['orientation_arrow'])
        
        # Time plots
        self.anim_lines['v1_act'].set_data(time_s, actual_v_s[:, 0])
        self.anim_lines['om_act'].set_data(time_s, actual_v_s[:, 1])
        
        # Safely extract controller status data
        if status_s_list and isinstance(status_s_list[0], dict):
            v_d_h = np.array([s.get('v_d', [np.nan, np.nan]) for s in status_s_list])
            if v_d_h.ndim == 2 and v_d_h.shape[0] == len(time_s):
                 self.anim_lines['v1_des'].set_data(time_s, v_d_h[:, 0])
                 self.anim_lines['om_des'].set_data(time_s, v_d_h[:, 1])

            kin_e_h = np.array([s.get('kin_errors', [np.nan]*3) for s in status_s_list])
            if kin_e_h.ndim == 2 and kin_e_h.shape[0] == len(time_s):
                self.anim_lines['e1'].set_data(time_s, kin_e_h[:,0])
                self.anim_lines['e2'].set_data(time_s, kin_e_h[:,1])
                self.anim_lines['e3'].set_data(time_s, kin_e_h[:,2])

            eta_h = np.array([s.get('eta', [np.nan]*2) for s in status_s_list])
            if eta_h.ndim == 2 and eta_h.shape[0] == len(time_s):
                self.anim_lines['eta1'].set_data(time_s, eta_h[:,0])
                self.anim_lines['eta2'].set_data(time_s, eta_h[:,1])
        
            phat_h = np.array([s.get('p_hat', [np.nan]*2) for s in status_s_list])
            if phat_h.ndim == 2 and phat_h.shape[0] == len(time_s):
                self.anim_lines['m_hat'].set_data(time_s, phat_h[:,0])
                self.anim_lines['I_hat'].set_data(time_s, phat_h[:,1])
        
        self.anim_lines['tau_r'].set_data(time_s, torques_s[:, 0])
        self.anim_lines['tau_l'].set_data(time_s, torques_s[:, 1])
        
        self.anim_axes['path'].set_title(f"Robot Path Tracking (T={time_s[-1]:.2f}s)")
        
        # Return all line artists and the arrow patch
        return [self.anim_lines[key] for key in self.anim_lines] + [self.anim_patches['orientation_arrow']]

    def create_animation(self, simulation_data, controller_status_list, 
                         robot_radius=0.15, steps_per_frame=10, save_path=None, interval=50,
                         true_m=None, true_I=None):
        """ Creates and optionally saves/shows an animation of the simulation. """
        self.sim_data_anim = simulation_data
        self.dt_anim = simulation_data.get("dt", 0.02)

        num_time_points = len(simulation_data["time"])
        processed_status_list = []
        if controller_status_list: # Check if it's not None and not accordion
            for i in range(num_time_points):
                if i < len(controller_status_list) and isinstance(controller_status_list[i], dict):
                    processed_status_list.append(controller_status_list[i])
                else: 
                    # Pad with last valid status if list is short, or empty dict if no valid status
                    last_valid_status = next((s for s in reversed(controller_status_list[:i]) if isinstance(s, dict)), {})
                    processed_status_list.append(last_valid_status)
        else: 
            processed_status_list = [{}] * num_time_points # List of empty dicts
        self.controller_status_list_anim = processed_status_list

        self.anim_step_size = max(1, steps_per_frame)
        self.true_m_anim, self.true_I_anim = true_m, true_I
        self.robot_radius_anim = robot_radius

        # Setup figure, axes, and initial plot elements
        initial_artists = self._setup_animation_plots() 
        
        num_sim_points = len(self.sim_data_anim["time"])
        num_frames = max(1, (num_sim_points -1) // self.anim_step_size + 1)

        ani = animation.FuncAnimation(self.fig_anim, self._animate_frame, frames=num_frames,
                                      init_func=self._init_anim_func, blit=False, 
                                      interval=interval, repeat=False)
        
        if save_path:
            try:
                print(f"Saving animation to {save_path}...")
                target_fps = max(1, int(1000.0 / interval))
                ani.save(save_path, writer='pillow', fps=target_fps, dpi=100, # Lower DPI for faster save
                         progress_callback=lambda i, n: print(f'  Saving frame {i+1}/{n}') if (i+1)%max(1,(n//10))==0 or i+1==n else None)
                print("Animation saved successfully.")
            except Exception as e:
                print(f"Error saving animation: {e}. Pillow installed? ('pip install pillow').")
        # else: # No save_path, so show it (if in interactive environment)
            # plt.show() # This might be blocking or handled by the environment
        return ani 

    def plot_final_results(self, sim_data, controller_status_list, title_suffix="",
                           true_m=None, true_I=None, robot_radius=0.15):
        """ Generates and shows static plots of key simulation variables. """
        time_axis = sim_data["time"]
        actual_path = sim_data["actual_path"] 
        true_path = sim_data.get("true_path", actual_path) 
        actual_vels = sim_data["robot_vels"] 
        torques_cmd = sim_data["torques_cmd"]
        disturbances = sim_data.get("disturbances_applied", np.zeros((len(time_axis),2)))

        num_time_points = len(time_axis)
        processed_status_list = []
        if controller_status_list:
            for i in range(num_time_points):
                if i < len(controller_status_list) and isinstance(controller_status_list[i], dict):
                     processed_status_list.append(controller_status_list[i])
                else: 
                    last_valid = next((s for s in reversed(controller_status_list[:i]) if isinstance(s, dict)), {})
                    processed_status_list.append(last_valid)
        else:
            processed_status_list = [{}] * num_time_points
            
        # Safely extract data, providing defaults for missing keys
        def get_status_array(key, num_elements, default_val=np.nan):
            data_list = [s.get(key, np.full(num_elements, default_val)) for s in processed_status_list]
            arr = np.array(data_list)
            return arr if arr.ndim == 2 and arr.shape[1] == num_elements else np.full((len(processed_status_list), num_elements), default_val)

        desired_vels = get_status_array('v_d', 2)
        kin_errors = get_status_array('kin_errors', 3)
        vel_errors_eta = get_status_array('eta', 2)
        p_hats = get_status_array('p_hat', 2) # Assuming p_hat is [m,I]
        robust_terms = get_status_array('u_robust', 2)
        bs_terms = get_status_array('backstepping_term', 2)

        # Original Plot
        fig, axs = plt.subplots(3, 2, figsize=(16, 14)) 
        fig.suptitle(f"Simulation Results: {title_suffix}", fontsize=16, fontweight='bold')

        # Path Plot
        axs[0,0].plot(self.desired_path[:, 0], self.desired_path[:, 1], 'r--', lw=1.5, label="Desired", alpha=0.7)
        axs[0,0].plot(actual_path[:, 0], actual_path[:, 1], 'b-', lw=1.5, label="Actual (Meas.)")
        if np.any(true_path != actual_path):
            axs[0,0].plot(true_path[:, 0], true_path[:, 1], 'k:', lw=1, label="True Path", alpha=0.6)
        axs[0,0].plot(self.desired_path[0, 0], self.desired_path[0, 1], 'go', ms=6, label="Start_D")
        axs[0,0].plot(self.desired_path[-1, 0], self.desired_path[-1, 1], 'mo', ms=6, label="End_D")
        axs[0,0].set_title("Path Tracking"); axs[0,0].set_xlabel("X (m)"); axs[0,0].set_ylabel("Y (m)")
        axs[0,0].legend(fontsize='x-small'); axs[0,0].axis('equal'); axs[0,0].grid(True, ls=':')

        # Velocities
        axs[0,1].plot(time_axis, actual_vels[:, 0], 'b-', label="$v_1$ Act")
        axs[0,1].plot(time_axis, desired_vels[:, 0], 'b:', label="$v_1$ Des")
        ax_om_static = axs[0,1].twinx()
        ax_om_static.plot(time_axis, actual_vels[:, 1], 'g-', label="$\omega$ Act")
        ax_om_static.plot(time_axis, desired_vels[:, 1], 'g:', label="$\omega$ Des")
        axs[0,1].set_title("Velocities"); axs[0,1].set_ylabel("$v_1$ (m/s)", color='b')
        axs[0,1].tick_params(axis='y', labelcolor='b')
        ax_om_static.set_ylabel("$\omega$ (rad/s)", color='g'); ax_om_static.tick_params(axis='y', labelcolor='g')
        axs[0,1].grid(True, ls=':'); 
        lines, labels = axs[0,1].get_legend_handles_labels(); lines2, labels2 = ax_om_static.get_legend_handles_labels()
        axs[0,1].legend(lines + lines2, labels + labels2, loc='best', fontsize='x-small')

        # Kinematic Errors
        axs[1,0].plot(time_axis, kin_errors[:,0], label="$e_1$ (Lon)")
        axs[1,0].plot(time_axis, kin_errors[:,1], label="$e_2$ (Lat)")
        axs[1,0].plot(time_axis, kin_errors[:,2], label="$e_3$ (Ori)")
        axs[1,0].set_title("Kinematic Errors"); axs[1,0].set_ylabel("Error"); axs[1,0].legend(fontsize='x-small'); axs[1,0].grid(True, ls=':')

        # Velocity Errors (eta)
        axs[1,1].plot(time_axis, vel_errors_eta[:,0], 'r-', label="$\eta_1 (v_1 err)$")
        axs[1,1].plot(time_axis, vel_errors_eta[:,1], 'purple', ls='-', label="$\eta_2 (\omega err)$")
        axs[1,1].set_title("Velocity Errors ($\eta$)"); axs[1,1].set_ylabel("Error"); axs[1,1].legend(fontsize='x-small'); axs[1,1].grid(True, ls=':')

        # Torques & Parameters
        axs[2,0].plot(time_axis, torques_cmd[:, 0], 'c-', label="$\\tau_R$ Cmd")
        axs[2,0].plot(time_axis, torques_cmd[:, 1], 'm-', label="$\\tau_L$ Cmd")
        axs[2,0].set_title("Torques & Params"); axs[2,0].set_ylabel("Torque (Nm)", color='c')
        axs[2,0].tick_params(axis='y', labelcolor='c'); axs[2,0].legend(loc='upper left', fontsize='x-small'); axs[2,0].grid(True, ls=':')
        axs[2,0].set_xlabel("Time (s)")
        ax_p_static = axs[2,0].twinx()
        ax_p_static.plot(time_axis, p_hats[:,0], 'k-', label="$\hat{m}$")
        if true_m is not None: ax_p_static.axhline(true_m, color='k', ls=':', lw=1, label="$m_{true}$")
        if p_hats.shape[1] > 1:
            ax_p_static.plot(time_axis, p_hats[:,1], 'dimgray', ls='-', label="$\hat{I}$")
            if true_I is not None: ax_p_static.axhline(true_I, color='dimgray', ls=':', lw=1, label="$I_{true}$")
        ax_p_static.set_ylabel("Params", color='k'); ax_p_static.tick_params(axis='y', labelcolor='k')
        
        # Dynamic Terms / Disturbances
        if np.any(np.isfinite(robust_terms)): axs[2,1].plot(time_axis, robust_terms[:,0], label="Robust $F_x$", alpha=0.7)
        if robust_terms.shape[1]>1 and np.any(np.isfinite(robust_terms[:,1])): axs[2,1].plot(time_axis, robust_terms[:,1], label="Robust $M_z$", alpha=0.7)
        if np.any(np.isfinite(bs_terms)): axs[2,1].plot(time_axis, bs_terms[:,0], ls=':', label="BS $F_x$", alpha=0.7)
        if bs_terms.shape[1]>1 and np.any(np.isfinite(bs_terms[:,1])): axs[2,1].plot(time_axis, bs_terms[:,1], ls=':', label="BS $M_z$", alpha=0.7)
        if np.any(disturbances != 0):
            axs[2,1].plot(time_axis, disturbances[:,0], 'brown', ls='--', label="$d_{Fx}$", alpha=0.5)
            if disturbances.shape[1]>1: axs[2,1].plot(time_axis, disturbances[:,1], 'olive', ls='--', label="$d_{Mz}$", alpha=0.5)
        axs[2,1].set_title("Dyn. Terms & Disturbances"); axs[2,1].set_ylabel("Value")
        axs[2,1].legend(fontsize='x-small', loc='best'); axs[2,1].grid(True, ls=':'); axs[2,1].set_xlabel("Time (s)")

        for ax_r in axs[:-1,:]: # Hide x-labels for non-bottom plots
            for ax_c in ax_r: plt.setp(ax_c.get_xticklabels(), visible=False)
        plt.setp(axs[0,1].get_xticklabels(), visible=False) # Velocity plot x-label also
        
        fig.tight_layout(rect=[0, 0, 1, 0.96])

        # Create Comparison Plot
        self._plot_comparison(sim_data, controller_status_list, true_m, true_I)
