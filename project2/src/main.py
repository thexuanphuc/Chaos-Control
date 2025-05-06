import numpy as np
import matplotlib.pyplot as plt
import time
from Simulation import Simulation
# Import the new controller
from Controller import LyapunovKinematicController, AdaptiveDynamicController, NonAdaptiveDynamicController
from Visualizer import Visualizer

# (Keep your PATH_TYPES dictionary and generate_path function as they are)
PATH_TYPES = {
    1: "Circle", 2: "Ellipse", 3: "Spiral", 4: "Line", 5: "Lemniscate",
    6: "SineWave", 7: "Heart", 8: "SquareWave", 9: "Parabola", 10: "Complex"
}

def generate_path(path_type_index=2):
    num_points = 150 # Number of points in the path
    path_type = str(path_type_index)

    if path_type == '1': # Circle
        radius = 5
        t = np.linspace(0, 2 * np.pi, num_points)
        x = radius * np.cos(t)
        y = radius * np.sin(t)
    elif path_type == '2': # Ellipse
        rx, ry = 6, 4
        t = np.linspace(0, 2 * np.pi, num_points)
        x = rx * np.cos(t)
        y = ry * np.sin(t)
    elif path_type == '3': # Spiral
        a = 0.5 # Controls tightness
        t = np.linspace(0, 6 * np.pi, num_points) # More turns
        x = a * t * np.cos(t)
        y = a * t * np.sin(t)
    elif path_type == '4': # Line
        x = np.linspace(0, 10, num_points)
        y = 0.5 * x + 1 # Example: y = 0.5x + 1
    elif path_type == '5': # Lemniscate (Figure-eight)
        scale = 5
        t = np.linspace(0, 2 * np.pi, num_points)
        x = scale * np.cos(t) / (1 + np.sin(t)**2)
        y = scale * np.sin(t) * np.cos(t) / (1 + np.sin(t)**2)
    elif path_type == '6': # Sine wave
        amp = 2
        freq = 1.5
        x = np.linspace(0, 4 * np.pi / freq, num_points)
        y = amp * np.sin(freq * x)
    elif path_type == '7': # Heart shape
        t = np.linspace(0, 2 * np.pi, num_points)
        x = 16 * np.sin(t)**3
        y = 13 * np.cos(t) - 5 * np.cos(2*t) - 2 * np.cos(3*t) - np.cos(4*t)
        x = x/3 # Scale down
        y = y/3 # Scale down
    elif path_type == '8': # Square wave-like path (smoothed slightly)
        x_seg = np.linspace(0, 10, 6) # Define corners
        y_seg = [0, 2, 2, -2, -2, 0]
        x, y = [], []
        points_per_seg = num_points // (len(x_seg)-1) if len(x_seg) > 1 else num_points
        for i in range(len(x_seg)-1):
             x_pts = np.linspace(x_seg[i], x_seg[i+1], points_per_seg)
             y_pts = np.linspace(y_seg[i], y_seg[i+1], points_per_seg)
             x.extend(x_pts[:-1])
             y.extend(y_pts[:-1])
        x.append(x_seg[-1])
        y.append(y_seg[-1])
        x = np.array(x)
        y = np.array(y)

    elif path_type == '9': # Parabola
        x = np.linspace(-5, 5, num_points)
        y = 0.3*x**2
    elif path_type == '10': # Complex
        t = np.linspace(0, 5 * np.pi, num_points)
        x = (t/3) * np.cos(t) + 3 * np.sin(3 * t)
        y = (t/3) * np.sin(t) + 2 * np.cos(t/2)
    else:
        raise ValueError(f"Invalid path type index: {path_type_index}. Choose from {list(PATH_TYPES.keys())}")

    return np.column_stack((x, y))


def main():
    # --- CHOOSE CONTROLLER TYPE ---
    # Set to 'adaptive' or 'non_adaptive'
    CONTROLLER_TYPE = 'adaptive' # TRY 'adaptive' AND 'non_adaptive' HERE

    # --- Simulation Parameters ---
    selected_path_type = 3
    path_name = PATH_TYPES.get(selected_path_type, "Custom")
    print(f"Generating path: {path_name} (Type {selected_path_type})")
    print(f"Using Controller: {CONTROLLER_TYPE.upper()}")

    desired_path = generate_path(selected_path_type)
    if desired_path.shape[0] < 2:
         print("Error: Generated path has less than 2 points.")
         return

    start_dx = desired_path[1, 0] - desired_path[0, 0]
    start_dy = desired_path[1, 1] - desired_path[0, 1]
    initial_theta = np.arctan2(start_dy, start_dx)
    initial_offset = 0.3
    initial_pose = np.array([
        desired_path[0, 0] - initial_offset * np.sin(initial_theta),
        desired_path[0, 1] + initial_offset * np.cos(initial_theta),
        initial_theta + 0.2
    ])
    initial_velocity = np.array([0.0, 0.0])

    dt = 0.02
    wheel_radius = 0.15 #0.05
    wheel_width = 0.75/2 #0.3
    max_steps = 3000

    robot_mass = 20
    robot_inertia = 5
    # --- Make sure disturbance is significant ---
    disturbance_level =0 # Max continuous random disturbance torque (Nm) - Keep low if focusing on kick
    
    # --- Kinematic Controller Parameters ---
    kin_k_forward = 1.8
    kin_k_theta = 4.5
    kin_k_lateral_factor = 1.0
    kin_v_ref = 1
    kin_omega_max = np.pi * 3
    kin_lookahead = 0.4

    # --- Shared Dynamic Controller Parameters ---
    # These initial/assumed estimates will be used by NonAdaptiveDynamicController
    # And as initial estimates for AdaptiveDynamicController
    # Consider making the assumed parameters for NonAdaptive controller deliberately "wrong"
    # if the true parameters are robot_mass and robot_inertia, to show parameter mismatch issues.
    assumed_mass = robot_mass * 0.7  # Example: 70% of true mass
    assumed_inertia = robot_inertia * 1.5 # Example: 150% of true inertia
    
    p_estimates_for_controller = np.array([assumed_mass, assumed_inertia])

    kd1_factor = 5.0 
    kd2_factor = 8.0 
    # Kd should ideally be tuned for stability for *both* controllers,
    # or use values that are known to be reasonably stable for the assumed parameters.
    kd_gains = np.diag([p_estimates_for_controller[0] * kd1_factor, p_estimates_for_controller[1] * kd2_factor])


    # --- Adaptive Controller Specific Parameters ---
    gamma_p_adaptive = np.diag([0.3, 0.05]) 
    controller_dB_adaptive = 0.9 # Disturbance bound for adaptive controller's robust term
    use_robust_adaptive = True 
    min_params_adaptive=np.array([1.0, 0.1])
    max_params_adaptive=np.array([50.0, 10.0])

    # --- Define Kick Parameters (Ensure it's significant) ---
    kick_target_idx = len(desired_path) // 2
    kick_target_idx = max(0, min(kick_target_idx, len(desired_path) - 1))
    kick_trigger_dist = 0.5
    kick_duration_time = 1# Make it last a bit longer to see effect
    # This kick should be strong enough to destabilize the non-adaptive controller
    kick_mag = np.array([-100.0, 0]) # Forward and CW rotational kick
    # kick_mag = np.array([0.0, 0.0]) # Uncomment to disable kick for a baseline run

    print(f"Initializing Simulation with Trajectory Kick: Target Index={kick_target_idx}, Trigger Dist={kick_trigger_dist}m, Duration={kick_duration_time}s, Mag={kick_mag}")
    simulation = Simulation(
        dt, desired_path, wheel_radius, wheel_width,
        m=robot_mass, I=robot_inertia,
        dB=disturbance_level,
        initial_pose=initial_pose, initial_velocity=initial_velocity,
        kick_path_target_index=kick_target_idx if np.any(kick_mag) else None,
        kick_path_trigger_distance=kick_trigger_dist,
        kick_duration=kick_duration_time,
        kick_magnitude=kick_mag
    )

    kinematic_controller = LyapunovKinematicController(
        k_forward=kin_k_forward, k_theta=kin_k_theta,
        k_lateral_gain_factor=kin_k_lateral_factor,
        v_ref=kin_v_ref, omega_max=kin_omega_max, lookahead_dist=kin_lookahead
    )

    # --- Instantiate the chosen controller ---
    active_controller = None
    if CONTROLLER_TYPE == 'adaptive':
        active_controller = AdaptiveDynamicController(
            dt=dt, wheel_radius=wheel_radius, wheel_width=wheel_width,
            kinematic_controller=kinematic_controller,
            initial_p_hat=p_estimates_for_controller, # Start with the same estimates
            gamma_p=gamma_p_adaptive,
            kd=kd_gains, # Use the common Kd
            disturbance_bound=controller_dB_adaptive,
            use_robust_term=use_robust_adaptive,
            min_params=min_params_adaptive,
            max_params=max_params_adaptive
        )
        print("Using AdaptiveDynamicController.")
    elif CONTROLLER_TYPE == 'non_adaptive':
        active_controller = NonAdaptiveDynamicController(
            dt=dt, wheel_radius=wheel_radius, wheel_width=wheel_width,
            kinematic_controller=kinematic_controller,
            assumed_p=p_estimates_for_controller, # Uses the fixed assumed parameters
            kd=kd_gains # Use the common Kd
        )
        print("Using NonAdaptiveDynamicController.")
    else:
        raise ValueError(f"Unknown CONTROLLER_TYPE: {CONTROLLER_TYPE}")

    controller_status_list = []
    print("Starting simulation...")
    start_time_sim = time.time()
    controller_finished = False

    for step in range(max_steps):
        current_state = simulation.get_robot_state()
        tau_left_cmd, tau_right_cmd = 0.0, 0.0 # Initialize

        if not controller_finished:
            try:
                # Use the 'active_controller' instance
                tau_left_cmd, tau_right_cmd, status = active_controller.compute_control(current_state, desired_path)
                controller_status_list.append(status)

                if active_controller.kinematic_controller.finished_flag:
                    print(f"Controller signaled finish at step {step}.")
                    controller_finished = True
                    tau_left_cmd, tau_right_cmd = 0.0, 0.0
            except Exception as e:
                print(f"Error in controller at step {step}: {e}")
                tau_left_cmd, tau_right_cmd = 0.0, 0.0
        else:
            tau_left_cmd, tau_right_cmd = 0.0, 0.0
        
        try:
            simulation.execute_cmd(tau_left_cmd, tau_right_cmd)
        except Exception as e:
            print(f"Error in simulation step {step}: {e}")
            break

        if (step + 1) % 200 == 0 and controller_status_list:
            last_status = controller_status_list[-1]
            # The 'p_hat' in status will be adaptive estimates for adaptive, assumed for non-adaptive
            p_display = last_status['p_hat']
            print(f"Step: {step+1}/{max_steps}, Time: {simulation.time_stamps[-1]:.2f}s, "
                  f"Pos:({current_state[0]:.2f},{current_state[1]:.2f}), "
                  f"Vels:({current_state[3]:.2f},{current_state[4]:.2f}), "
                  f"Params Est/Assumed:[{p_display[0]:.2f}, {p_display[1]:.2f}]")

        if controller_finished and step > active_controller.kinematic_controller.closest_index + 5:
            print(f"Simulation loop ending early at step {step} due to controller finish.")
            break
    else:
        print(f"Simulation finished after {max_steps} steps.")
    
    end_time_sim = time.time()
    print(f"Simulation duration: {end_time_sim - start_time_sim:.2f} seconds")

    simulation_data = simulation.get_simulation_data()
    num_sim_data_points = len(simulation_data['time'])
    num_controller_points = len(controller_status_list)
    if num_controller_points > num_sim_data_points -1 :
         controller_status_list = controller_status_list[:num_sim_data_points-1]

    print("Creating visualization and animation...")
    visualizer = Visualizer(desired_path)

    if simulation.kick_target_point_coords is not None:
        target_coords = simulation.kick_target_point_coords
        visualizer.path_ax.plot(target_coords[0], target_coords[1], 'rx', markersize=10, label='Kick Trigger Point')
        visualizer.path_ax.legend(loc='best')

    anim_interval = 50
    anim_step = 3
    
    # In the visualizer, p_hat plot will show adaptive estimates or assumed fixed values
    # depending on the controller used.
    ani = visualizer.create_animation(simulation_data, controller_status_list,
                                      interval=anim_interval, step=anim_step,
                                      true_m=robot_mass, true_I=robot_inertia)

    # # # # --- Adjust GIF filename based on controller type ---
    # gif_filename_base = f'robot_animation_{path_name.replace(" ", "")}_{CONTROLLER_TYPE}'
    # # Add _kick if kick_mag is not zero
    # if np.any(kick_mag):
    #     gif_filename = f'{gif_filename_base}_kick.gif'
    # else:
    #     gif_filename = f'{gif_filename_base}_nokick.gif'

    # print(f"Saving animation to {gif_filename}...")
    # try:
    #     time_per_frame = anim_step * dt
    #     fps = max(1, int(1.0 / time_per_frame)) if time_per_frame > 0 else 10
    #     ani.save(gif_filename, writer='pillow', fps=fps, progress_callback=lambda i, n: print(f'Saving frame {i+1}/{n}') if (i+1)%50==0 or i+1==n else None)
    #     print("Animation saved successfully.")
    # except Exception as e:
    #     print(f"Error saving animation: {e}")

    print("Plotting final results...")
    # --- Adjust plot title for final results ---
    final_plot_title_suffix = f"{path_name} Path ({CONTROLLER_TYPE.upper()})"
    if np.any(kick_mag):
        final_plot_title_suffix += " with Disturbance Kick"
    
    visualizer.plot_final_results(simulation_data, controller_status_list, 
                                 final_plot_title_suffix, # Pass the modified title
                                 true_m=robot_mass, true_I=robot_inertia)

    try:
        plt.show()
    except Exception as e:
        print(f"Note: Could not display plots interactively ({e}). Check saved GIF/static plots.")

if __name__ == "__main__":
    main()