import numpy as np
import matplotlib.pyplot as plt
import time
from Simulation import Simulation
from Controller import LyapunovKinematicController, AdaptiveDynamicController
from Visualizer import Visualizer

# Dictionary mapping path type index to a descriptive name for filenames/titles
PATH_TYPES = {
    1: "Circle", 2: "Ellipse", 3: "Spiral", 4: "Line", 5: "Lemniscate",
    6: "SineWave", 7: "Heart", 8: "SquareWave", 9: "Parabola", 10: "ComplexPath"
}

def generate_path(path_type_index=2, num_points=150):
    """
    Generates various types of predefined paths.

    :param path_type_index: Integer index corresponding to a path type in PATH_TYPES.
    :param num_points: Number of points to generate for the path.
    :return: A numpy array of shape (num_points, 2) representing the path (x, y coordinates).
    :raises ValueError: If an invalid path_type_index is provided.
    """
    path_type_str = str(path_type_index) # Convert to string for if/elif matching

    if path_type_str == '1': # Circle
        radius = 5.0
        t = np.linspace(0, 2 * np.pi, num_points)
        x = radius * np.cos(t)
        y = radius * np.sin(t)
    elif path_type_str == '2': # Ellipse
        rx, ry = 6.0, 3.5
        t = np.linspace(0, 2 * np.pi, num_points)
        x = rx * np.cos(t)
        y = ry * np.sin(t)
    elif path_type_str == '3': # Spiral
        a = 0.4 # Controls tightness
        b = 0.2 # Controls growth rate
        t = np.linspace(0, 8 * np.pi, num_points) # More turns for a visible spiral
        x = (a + b * t) * np.cos(t)
        y = (a + b * t) * np.sin(t)
    elif path_type_str == '4': # Straight Line
        x_start, y_start = 0, 1
        x_end, y_end = 10, 6
        x = np.linspace(x_start, x_end, num_points)
        y = np.linspace(y_start, y_end, num_points)
    elif path_type_str == '5': # Lemniscate of Gerono (Figure-eight)
        scale = 5.0
        t = np.linspace(0, 2 * np.pi, num_points)
        x = scale * np.cos(t)
        y = scale * np.sin(t) * np.cos(t) # Corrected formula for Lemniscate of Gerono
    elif path_type_str == '6': # Sine Wave
        amplitude = 2.0
        frequency = 1.0 # Cycles per 2*pi range
        length = 3 * (2 * np.pi / frequency) # Three full cycles
        x = np.linspace(0, length, num_points)
        y = amplitude * np.sin(frequency * x)
    elif path_type_str == '7': # Heart Shape (Cardioid-like)
        scale = 2.0 # Adjusted scale
        t = np.linspace(0, 2 * np.pi, num_points)
        x = scale * (16 * np.sin(t)**3) / 10 # Normalize and scale
        y = scale * (13 * np.cos(t) - 5 * np.cos(2*t) - 2 * np.cos(3*t) - np.cos(4*t)) / 10
    elif path_type_str == '8': # Smoothed Square Wave
        cycles = 3
        amplitude = 2.0
        period_points = num_points // cycles
        segment_points = period_points // 4
        x_coords, y_coords = [], []
        current_x = 0
        for _ in range(cycles):
            x_coords.extend(np.linspace(current_x, current_x + segment_points * 0.1, segment_points)) # Horizontal
            y_coords.extend(np.full(segment_points, 0))
            current_x += segment_points * 0.1
            x_coords.extend(np.full(segment_points, current_x)) # Vertical up
            y_coords.extend(np.linspace(0, amplitude, segment_points))
            x_coords.extend(np.linspace(current_x, current_x + segment_points * 0.1, segment_points)) # Horizontal
            y_coords.extend(np.full(segment_points, amplitude))
            current_x += segment_points * 0.1
            x_coords.extend(np.full(segment_points, current_x)) # Vertical down
            y_coords.extend(np.linspace(amplitude, 0, segment_points))
        # Ensure num_points by trimming or padding (simple trim here)
        x = np.array(x_coords[:num_points])
        y = np.array(y_coords[:num_points])

    elif path_type_str == '9': # Parabola
        a = 0.2 # Controls width of parabola
        x_range = 10.0
        x = np.linspace(-x_range/2, x_range/2, num_points)
        y = a * x**2
    elif path_type_str == '10': # Complex Path (Lissajous-like)
        a, b = 5, 3  # Amplitudes
        delta = np.pi / 2
        freq_x, freq_y = 1.5, 2.5 # Frequencies
        t = np.linspace(0, 4 * np.pi, num_points)
        x = a * np.sin(freq_x * t + delta)
        y = b * np.sin(freq_y * t)
    else:
        valid_keys = ", ".join(map(str, PATH_TYPES.keys()))
        raise ValueError(f"Invalid path_type_index: {path_type_index}. Choose from {valid_keys}")

    return np.column_stack((x, y))


def main():
    """
    Main function to set up and run the robot simulation.
    """
    # --- Simulation Configuration ---
    selected_path_type_idx = 4 # Choose path type (e.g., 7 for Heart)
    path_name_str = PATH_TYPES.get(selected_path_type_idx, "CustomPath")
    print(f"--- Starting Simulation for: {path_name_str} (Type {selected_path_type_idx}) ---")

    desired_path_coords = generate_path(selected_path_type_idx, num_points=200)
    if desired_path_coords.shape[0] < 2:
         print("Error: Generated path has less than 2 points. Exiting.")
         return

    # --- Initial Robot State ---
    # Start slightly offset from the beginning of the path with a small orientation error
    path_start_dx = desired_path_coords[1, 0] - desired_path_coords[0, 0]
    path_start_dy = desired_path_coords[1, 1] - desired_path_coords[0, 1]
    initial_robot_theta = np.arctan2(path_start_dy, path_start_dx)
    
    offset_distance = 0.5 # Perpendicular distance offset from the path start
    initial_robot_pose = np.array([
        desired_path_coords[0, 0] - offset_distance * np.sin(initial_robot_theta), # Offset x
        desired_path_coords[0, 1] + offset_distance * np.cos(initial_robot_theta), # Offset y
        initial_robot_theta + np.deg2rad(15) # Initial orientation with a 15-degree error
    ])
    initial_robot_velocity = np.array([0.0, 0.0]) # Start from rest [v1, omega]

    # --- Simulation Parameters ---
    simulation_dt = 0.02       # Simulation time step (seconds)
    robot_wheel_radius = 0.05  # Wheel radius (m)
    robot_wheel_width = 0.3    # Distance between wheels (m)
    max_simulation_steps = 3500 # Max steps (adjust based on path length and speed)

    # --- Robot Dynamic Parameters (Actual Values for Simulation) ---
    actual_robot_mass = 12.0      # kg (m)
    actual_robot_inertia = 0.8    # kg*m^2 (I)
    
    # --- Disturbance Configuration ---
    # Continuous random disturbance (effective torque on v1 and omega axes)
    continuous_disturbance_bound = 1 # Max magnitude (Nm) for each component. Set to 0 for no continuous disturbance.
    
    # Trajectory-based kick disturbance
    kick_target_idx_on_path = len(desired_path_coords) // 2 # E.g., halfway point
    kick_trigger_dist_from_target = 0.3 # Trigger when robot is within this distance (m)
    kick_active_duration = 1        # Apply kick for this duration (s) after triggering
    # Kick magnitude: [effective_torque_on_v1_axis, effective_torque_on_omega_axis] (Nm)
    kick_disturbance_magnitude = np.array([0.0, 220.0]) # Example: pushes backward and induces CW rotation
    # To disable kick, set kick_target_idx_on_path = None or kick_disturbance_magnitude = np.array([0.0, 0.0])

    # --- Kinematic Controller Parameters (Outer Loop) ---
    kin_k_forward = 1.5         # Gain for forward error correction
    kin_k_theta = 3.5           # Gain for orientation error correction (sensitive)
    kin_k_lateral_factor = 0.8  # Scales lateral error correction
    kin_v_ref = 2            # Target speed for kinematic controller (m/s)
    kin_omega_max = np.pi * 30 # Max desired angular velocity (rad/s)
    kin_lookahead = 0.35        # Lookahead distance for path following (m)

    # --- Adaptive Dynamic Controller Parameters (Inner Loop) ---
    # Initial estimates for [m, I] - Can be different from actual values
    initial_param_estimates = np.array([actual_robot_mass * 0.7, actual_robot_inertia * 1.3])
    # Adaptation gains for p_hat = [m_hat, I_hat] (diagonal matrix Gamma_p)
    # Smaller gains lead to slower but potentially more stable adaptation.
    adaptation_gains_gamma_p = np.diag([0.02, 0.002]) # Adjusted for potentially more stability
    
    # Velocity error feedback gains Kd = diag(kd1, kd2)
    # These gains determine how aggressively the controller corrects velocity tracking errors.
    # Proportional to initial parameter estimates can be a starting point.
    kd1_factor_vs_mass_est = 6.0
    kd2_factor_vs_inertia_est = 9.0
    feedback_gains_kd = np.diag([
        initial_param_estimates[0] * kd1_factor_vs_mass_est,
        initial_param_estimates[1] * kd2_factor_vs_inertia_est
    ])
    
    # Assumed disturbance bound for the controller's robust term.
    # Should ideally be slightly larger than the expected max actual disturbance (continuous + kick if applicable).
    controller_assumed_disturbance_bound = max(continuous_disturbance_bound, np.max(np.abs(kick_disturbance_magnitude))) * 1.2
    use_robust_control_term = True

    # Parameter estimate bounds (to prevent divergence of estimates)
    min_param_bounds = np.array([1.0, 0.1])     # Min [m, I]
    max_param_bounds = np.array([50.0, 20.0])   # Max [m, I]

    # --- Initialization of Simulation, Controllers, and Visualizer ---
    print("Initializing simulation environment...")
    simulation_instance = Simulation(
        dt=simulation_dt, desired_path=desired_path_coords,
        wheel_radius=robot_wheel_radius, wheel_width=robot_wheel_width,
        m=actual_robot_mass, I=actual_robot_inertia,
        dB=continuous_disturbance_bound,
        initial_pose=initial_robot_pose, initial_velocity=initial_robot_velocity,
        kick_path_target_index=kick_target_idx_on_path if np.any(kick_disturbance_magnitude) else None,
        kick_path_trigger_distance=kick_trigger_dist_from_target,
        kick_duration=kick_active_duration,
        kick_magnitude=kick_disturbance_magnitude
    )

    kinematic_ctrl = LyapunovKinematicController(
        k_forward=kin_k_forward, k_theta=kin_k_theta,
        k_lateral_gain_factor=kin_k_lateral_factor,
        v_ref=kin_v_ref, omega_max=kin_omega_max, lookahead_dist=kin_lookahead
    )

    adaptive_dynamic_ctrl = AdaptiveDynamicController(
        dt=simulation_dt, wheel_radius=robot_wheel_radius, wheel_width=robot_wheel_width,
        kinematic_controller=kinematic_ctrl,
        initial_p_hat=initial_param_estimates,
        gamma_p=adaptation_gains_gamma_p,
        kd=feedback_gains_kd,
        disturbance_bound=controller_assumed_disturbance_bound,
        use_robust_term=use_robust_control_term,
        min_params=min_param_bounds,
        max_params=max_param_bounds
    )

    # --- Data Storage for Analysis ---
    controller_status_history = [] # Stores dicts: {'eta', 'p_hat', 'kin_errors', 'v_d', ...}

    # --- Simulation Loop ---
    print("Starting simulation loop...")
    sim_start_time = time.time()
    kinematic_controller_finished = False

    for step_num in range(max_simulation_steps):
        current_robot_state = simulation_instance.get_robot_state()

        # Compute control torques only if kinematic controller hasn't signaled finish
        if not kinematic_controller_finished:
            try:
                tau_left_cmd, tau_right_cmd, status_dict = adaptive_dynamic_ctrl.compute_control(
                    current_robot_state, desired_path_coords
                )
                controller_status_history.append(status_dict)

                # Check if the kinematic controller (via adaptive controller) signaled finish
                if adaptive_dynamic_ctrl.kinematic_controller.finished_flag:
                    print(f"Kinematic controller signaled finish at step {step_num}.")
                    kinematic_controller_finished = True
                    # Command zero torque after finishing to allow robot to stop
                    tau_left_cmd, tau_right_cmd = 0.0, 0.0
            except Exception as e:
                print(f"Error in controller at step {step_num}: {e}")
                print(f"Current robot state: {current_robot_state}")
                tau_left_cmd, tau_right_cmd = 0.0, 0.0 # Apply zero torque on error
                # break # Optionally stop simulation on controller error
        else:
            # If finished, keep applying zero torque
            tau_left_cmd, tau_right_cmd = 0.0, 0.0
            # Append a dummy status if needed for consistent list lengths, or handle in plotting
            if controller_status_history: # Append based on last valid status
                 dummy_status = controller_status_history[-1].copy()
                 dummy_status['eta'] = np.zeros_like(dummy_status['eta']) # Zero errors
                 dummy_status['v_d'] = np.zeros_like(dummy_status['v_d'])   # Zero desired vels
                 controller_status_history.append(dummy_status)


        # Execute commanded torques in the simulation
        try:
            simulation_instance.execute_cmd(tau_left_cmd, tau_right_cmd)
        except Exception as e:
            print(f"Error during simulation step {step_num}: {e}")
            print(f"Commanded torques: Left={tau_left_cmd:.2f}, Right={tau_right_cmd:.2f}")
            break # Stop simulation if physics fail

        # Optional: Print progress periodically
        if (step_num + 1) % 250 == 0 and controller_status_history:
            last_status = controller_status_history[-1]
            print(f"Step: {step_num+1}/{max_simulation_steps}, Sim Time: {simulation_instance.time_stamps[-1]:.2f}s, "
                  f"Pos:({current_robot_state[0]:.2f},{current_robot_state[1]:.2f}), "
                  f"Vels:({current_robot_state[3]:.2f},{current_robot_state[4]:.2f}), "
                  f"p_hat:[m={last_status['p_hat'][0]:.2f}, I={last_status['p_hat'][1]:.2f}]")

        # Break loop if kinematic controller finished and robot has had a few steps to settle
        if kinematic_controller_finished and step_num > kinematic_ctrl.closest_index + 10:
            print(f"Simulation loop ending at step {step_num} as controller finished and robot settled.")
            break
    else: # Executed if the loop completes without an explicit break
        print(f"Simulation completed after {max_simulation_steps} steps.")
    
    sim_end_time = time.time()
    print(f"Total simulation wall-clock time: {sim_end_time - sim_start_time:.2f} seconds.")

    # --- Post-Simulation Processing and Visualization ---
    simulation_results_data = simulation_instance.get_simulation_data()
    
    # Ensure controller_status_history has a length corresponding to the number of control actions taken
    # Number of control actions = number of simulation intervals = len(time_stamps) - 1
    num_control_actions = len(simulation_results_data['time']) - 1
    controller_status_history = controller_status_history[:num_control_actions]


    print("Initializing visualization...")
    visualizer_instance = Visualizer(desired_path_coords)

    # Add a marker for the kick trigger point on the path plot for reference
    if simulation_instance.kick_target_point_coords is not None:
        kick_tp_coords = simulation_instance.kick_target_point_coords
        visualizer_instance.path_ax.plot(kick_tp_coords[0], kick_tp_coords[1], 'X', color='magenta',
                                         markersize=12, label='Kick Trigger Target Pt', alpha=0.8)
        visualizer_instance.path_ax.legend(loc='best') # Refresh legend

    # Animation settings
    animation_frame_interval_ms = 40  # Milliseconds between animation frames
    simulation_steps_per_frame = 4    # Number of simulation steps per animation frame

    print("Creating animation...")
    animation_object = visualizer_instance.create_animation(
        simulation_results_data, controller_status_history,
        interval=animation_frame_interval_ms, step=simulation_steps_per_frame,
        true_m=actual_robot_mass, true_I=actual_robot_inertia
    )

    # # Save animation (optional)
    # animation_filename = f'robot_sim_{path_name_str.replace(" ", "_")}_anim.gif'
    # print(f"Attempting to save animation to {animation_filename}...")
    # try:
    #     # Calculate FPS for saving: time_per_frame = steps_per_frame * dt
    #     time_per_anim_frame = simulation_steps_per_frame * simulation_dt
    #     animation_fps = max(1, int(1.0 / time_per_anim_frame)) if time_per_anim_frame > 0 else 15
    #     print(f"Calculated FPS for saving GIF: {animation_fps}")
        
    #     animation_object.save(animation_filename, writer='pillow', fps=animation_fps,
    #                           progress_callback=lambda i, n: print(f'Saving frame {i+1}/{n}') if (i+1)%20==0 or i+1==n else None)
    #     print(f"Animation saved successfully to {animation_filename}.")
    # except Exception as e:
    #     print(f"Error saving animation: {e}")
    #     print("Ensure 'pillow' is installed ('pip install pillow'). For MP4, 'ffmpeg' might be needed.")

    # Plot final static results
    print("Plotting final static results...")
    visualizer_instance.plot_final_results(
        simulation_results_data, controller_status_history, path_name_str,
        true_m=actual_robot_mass, true_I=actual_robot_inertia
    )

    # Show plots (behavior depends on Matplotlib backend and environment)
    try:
        plt.show()
        print("Plots displayed. Close plot windows to exit.")
    except Exception as e:
        print(f"Note: Could not display plots interactively ({e}). Check saved GIF/static plots.")

if __name__ == "__main__":
    main()
