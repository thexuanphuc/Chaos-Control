import numpy as np
import matplotlib.pyplot as plt
import time
from Simulation import Simulation
from Controller import LyapunovKinematicController, BacksteppingDynamicController, AdaptiveDynamicController
from Visualizer import Visualizer

# Dictionary mapping path type index to a descriptive name for filenames/titles
PATH_TYPES = {
    1: "Circle", 2: "Ellipse", 3: "Spiral", 4: "Line", 5: "Lemniscate",
    6: "SineWave", 7: "Heart", 8: "SquareWave", 9: "Parabola", 10: "ComplexPath"
}

def generate_path(path_type_index=2, num_points=500, scale=5.0, origin=(0,0)):
    """
    Generates various types of predefined paths.

    :param path_type_index: Integer index corresponding to a path type in PATH_TYPES.
    :param num_points: Number of points to generate for the path.
    :param scale: General scaling factor for the path size.
    :param origin: Tuple (x0, y0) for the starting point/center of the path.
    :return: A numpy array of shape (num_points, 2) representing the path (x, y coordinates).
    """
    path_type_str = str(path_type_index)
    t_end = 2 * np.pi
    
    if path_type_str == '4':
        t_end = 1.0
    elif path_type_str == '8':
        t_end = 4.0
    
    t = np.linspace(0, t_end, num_points)
    if path_type_str == '9':
         t = np.linspace(-2.5, 2.5, num_points)

    x0, y0 = origin

    if path_type_str == '1':
        x = x0 + scale * np.cos(t)
        y = y0 + scale * np.sin(t)
    elif path_type_str == '2':
        x = x0 + scale * np.cos(t)
        y = y0 + scale * 0.6 * np.sin(t)
    elif path_type_str == '3':
        radius = scale * (t / (t_end if t_end > 1e-6 else 1.0))
        angle = t * 3.5
        x = x0 + radius * np.cos(angle)
        y = y0 + radius * np.sin(angle)
    elif path_type_str == '4':
        x = x0 + scale * t * np.cos(np.pi/6)
        y = y0 + scale * t * np.sin(np.pi/6)
    elif path_type_str == '5':
        a_lemniscate = scale
        x = x0 + a_lemniscate * np.cos(t) / (1 + np.sin(t)**2 + 1e-9)
        y = y0 + a_lemniscate * np.sin(t) * np.cos(t) / (1 + np.sin(t)**2 + 1e-9)
    elif path_type_str == '6':
        x_wave = scale * t / (np.pi if np.pi > 1e-6 else 1.0)
        y = y0 + scale * 0.3 * np.sin(x_wave * 2)
        x = x0 + x_wave
    elif path_type_str == '7':
        a_heart = scale / 16.0
        x_h = a_heart * (16 * np.sin(t)**3)
        y_h = a_heart * (13 * np.cos(t) - 5 * np.cos(2*t) - 2 * np.cos(3*t) - np.cos(4*t))
        x = x0 + x_h
        y = y0 + y_h + a_heart * 12
    elif path_type_str == '8':
        segment_len_param = t_end / 4.0
        x = np.zeros_like(t)
        y = np.zeros_like(t)
        for i_pt, ti_pt in enumerate(t):
            if segment_len_param > 1e-9:
                if ti_pt <= segment_len_param:
                    current_segment_progress = ti_pt / segment_len_param
                    x[i_pt] = x0 + scale * current_segment_progress
                    y[i_pt] = y0
                elif ti_pt <= 2 * segment_len_param:
                    current_segment_progress = (ti_pt - segment_len_param) / segment_len_param
                    x[i_pt] = x0 + scale
                    y[i_pt] = y0 + scale * current_segment_progress
                elif ti_pt <= 3 * segment_len_param:
                    current_segment_progress = (ti_pt - 2*segment_len_param)/segment_len_param
                    x[i_pt] = x0 + scale * (1 - current_segment_progress)
                    y[i_pt] = y0 + scale
                else:
                    current_segment_progress = (ti_pt - 3*segment_len_param)/segment_len_param
                    x[i_pt] = x0
                    y[i_pt] = y0 + scale * (1 - current_segment_progress)
            else:
                x[i_pt] = x0
                y[i_pt] = y0
    elif path_type_str == '9':
        x_par = scale * t / 2.5
        y = y0 + scale * 0.08 * x_par**2
        x = x0 + x_par
    elif path_type_str == '10':
        x = x0 + scale * np.sin(1.5 * t + np.pi/3)
        y = y0 + scale * 0.8 * np.sin(t + np.pi/6)
    else:
        raise ValueError(f"Invalid path_type_index: {path_type_index}. Choose from {list(PATH_TYPES.keys())}.")

    return np.column_stack((x, y))

def run_simulation_for_path(path_idx, sim_duration=50.0, dt=0.02, controller_name="BacksteppingDynamic"):
    """
    Run a single simulation for a given path index.
    """
    # --- Path Generation ---
    path_scale = 3.5
    path_origin = (0.0, 0.0)
    if path_idx == 1:
        path_origin = (path_scale, 0.0)
    
    predefined_path = generate_path(
        path_idx, 
        num_points=int(sim_duration / (dt*1.0)),
        scale=path_scale, 
        origin=path_origin
    )

    # --- Initial Robot State ---
    initial_robot_pose = np.array([predefined_path[0,0] - 0.5, 
                                   predefined_path[0,1] - 0.5, 
                                   np.pi/3])
    if predefined_path.shape[0] > 1:
        dx_init = predefined_path[1,0] - predefined_path[0,0]
        dy_init = predefined_path[1,1] - predefined_path[0,1]
        if abs(dx_init) > 1e-6 or abs(dy_init) > 1e-6:
            initial_robot_pose[2] = np.arctan2(dy_init, dx_init)

    # --- Robot Physical Parameters ---
    wheel_r = 0.05
    wheel_W = 0.28
    actual_robot_mass = 10.0
    actual_robot_inertia = 1.0

    # --- Disturbance Configuration ---
    sim_continuous_disturbance_bound = np.array([0.1, 0.05])
    use_kick_disturbance = True
    kick_start_time_sim = sim_duration / 3.0
    kick_duration_sim = 0.6
    kick_magnitude_sim = np.array([-5.0, 60.0])

    # --- Controller Initialization ---
    controller = None
    kin_for_adaptive = None

    if controller_name == "LyapunovKinematic":
        k_x_lyap, k_y_lyap, k_theta_lyap = 2.8, 18.0, 2.5
        controller = LyapunovKinematicController(k_x_lyap, k_y_lyap, k_theta_lyap, dt=dt)
        print(f"Using Lyapunov Kinematic Controller for {PATH_TYPES[path_idx]}.")
    elif controller_name == "BacksteppingDynamic":
        k_v_bs, k_omega_bs, k_delta_bs = 2.5, 2.0, 1.5
        Kd_bs = [20.0, 12.0]
        K_bs_gains = [1.2, 0.8]
        gamma_p_bs = [0.01, 0.001]
        initial_p_hat_bs = np.array([actual_robot_mass * 0.5, actual_robot_inertia * 0.4])
        min_est_params = np.array([1.0, 0.1])
        max_est_params = np.array([30.0, 5.0])
        use_robust_bs, dB_robust_bs = True, 0.25

        controller = BacksteppingDynamicController(
            wheel_radius=wheel_r, wheel_width=wheel_W,
            k_v=k_v_bs, k_omega=k_omega_bs, k_delta=k_delta_bs,
            Kd=Kd_bs, K_bs=K_bs_gains,
            gamma_p=gamma_p_bs, initial_p_hat=initial_p_hat_bs,
            dB=dB_robust_bs, use_robust_term=use_robust_bs,
            min_params=min_est_params, max_params=max_est_params,
            dt=dt
        )
        print(f"Using Backstepping Dynamic Controller for {PATH_TYPES[path_idx]}.")
    elif controller_name == "AdaptiveDynamic":
        Kd_ad, gamma_p_ad = [25.0, 15.0], [0.018, 0.0018]
        initial_p_hat_ad = np.array([actual_robot_mass * 0.6, actual_robot_inertia * 0.5])
        min_est_params_ad, max_est_params_ad = np.array([1.0, 0.1]), np.array([40.0, 8.0])
        use_robust_ad, dB_robust_ad = True, 0.3

        controller = AdaptiveDynamicController(
            wheel_radius=wheel_r, wheel_width=wheel_W,
            Kd=Kd_ad, gamma_p=gamma_p_ad, initial_p_hat=initial_p_hat_ad,
            dB=dB_robust_ad, use_robust_term=use_robust_ad,
            min_params=min_est_params_ad, max_params=max_est_params_ad,
            dt=dt
        )
        kin_for_adaptive = LyapunovKinematicController(k_x=2.5, k_y=15.0, k_theta=2.0, dt=dt)
        print(f"Using Adaptive Dynamic Controller for {PATH_TYPES[path_idx]}.")
    else:
        raise ValueError(f"Unknown controller name: {controller_name}")

    # --- Simulation Setup ---
    simulation = Simulation(
        dt=dt, desired_path=predefined_path,
        wheel_radius=wheel_r, wheel_width=wheel_W,
        m=actual_robot_mass, I=actual_robot_inertia,
        dB_sim_continuous=sim_continuous_disturbance_bound,
        initial_pose=initial_robot_pose,
        kick_active=use_kick_disturbance,
        kick_start_time=kick_start_time_sim,
        kick_duration=kick_duration_sim,
        kick_magnitude=kick_magnitude_sim,
        velocity_noise_std=0.005,
        omega_noise_std=0.005
    )

    # --- Run Simulation ---
    num_steps = int(sim_duration / dt)
    controller_status_history = []

    print(f"\nStarting simulation: {controller_name} on {PATH_TYPES[path_idx]}")
    print(f"Duration: {sim_duration}s, Steps: {num_steps}, Path Points: {len(predefined_path)}")
    sim_start_time = time.time()

    for step in range(num_steps):
        current_robot_state = simulation.get_current_state()
        torques_cmd_tuple = np.array([0.0, 0.0])
        status_info = {}

        try:
            if controller_name == "LyapunovKinematic":
                (v_cmd_kin, _), status_info = controller.compute_control(current_robot_state, predefined_path)
                if step == 0:
                    print("INFO: LyapunovKinematic selected for dynamic sim; applying zero torques.")
            elif controller_name == "AdaptiveDynamic":
                (v1_d_val, omega_d_val), kin_status = kin_for_adaptive.compute_control(current_robot_state, predefined_path)
                v1_d_dot_val, omega_d_dot_val = kin_status['vd_dot']
                controller.set_desired_velocities(v1_d_val, omega_d_val, v1_d_dot_val, omega_d_dot_val)
                torques_cmd_tuple, status_info = controller.compute_control(current_robot_state, predefined_path)
            else:
                torques_cmd_tuple, status_info = controller.compute_control(current_robot_state, predefined_path)
        except Exception as e:
            print(f"\nERROR at step {step + 1} in controller.compute_control() for {PATH_TYPES[path_idx]}: {e}")
            import traceback; print(traceback.format_exc())
            return False

        controller_status_history.append(status_info)
        simulation.execute_commanded_torques(torques_cmd_tuple[0], torques_cmd_tuple[1])

        if (step + 1) % (max(1, num_steps // 10)) == 0:
            print(f"Progress: {((step + 1) / num_steps) * 100:.0f}% (Sim Time: {simulation.current_time:.2f}s)")

    sim_end_time = time.time()
    print(f"\nSimulation finished for {PATH_TYPES[path_idx]}. Real time elapsed: {sim_end_time - sim_start_time:.2f}s.")

    # --- Results and Visualization ---
    simulation_results_data = simulation.get_results()
    path_name_str = PATH_TYPES[path_idx]

    visualizer_instance = Visualizer(desired_path=predefined_path)
    
    animate_results = True
    if num_steps > 40000:
        animate_results = False

    if animate_results:
        print(f"Creating animation for {path_name_str}...")
        animation_filename = f"robot_animation_{path_name_str.replace(' ','_')}_{controller_name}.gif"
        try:
            target_anim_fps = 25
            anim_interval = int(1000 / target_anim_fps)
            anim_steps_per_frame = max(1, int((1.0/target_anim_fps) / dt))

            visualizer_instance.create_animation(
                simulation_results_data, controller_status_history,
                robot_radius=wheel_W/1.5, steps_per_frame=anim_steps_per_frame,
                save_path=animation_filename,
                interval=anim_interval,
                true_m=actual_robot_mass, true_I=actual_robot_inertia
            )
            print(f"Animation saved to {animation_filename}")
        except Exception as e:
            print(f"Could not create animation for {path_name_str}: {e}")
    
    print(f"\nPlotting final static results for {path_name_str}...")
    try:
        visualizer_instance.plot_final_results(
            simulation_results_data, controller_status_history,
            title_suffix=f"{path_name_str} ({controller_name})",
            true_m=actual_robot_mass, true_I=actual_robot_inertia,
            robot_radius=wheel_W/1.5
        )
        plt.savefig(f"plot_{path_name_str.replace(' ','_')}_{controller_name}.png")
        plt.close()
    except Exception as e:
        print(f"Could not plot results for {path_name_str}: {e}")

    return True

def main():
    sim_duration = 50.0
    dt = 0.02
    controller_name = "BacksteppingDynamic"

    for path_idx in range(1, 10):
        print(f"\n=== Starting Simulation for Path {path_idx}: {PATH_TYPES[path_idx]} ===")
        try:
            success = run_simulation_for_path(path_idx, sim_duration, dt, controller_name)
            if not success:
                print(f"Simulation for {PATH_TYPES[path_idx]} failed, continuing to next path.")
        except Exception as e:
            print(f"\nError running simulation for {PATH_TYPES[path_idx]}: {e}")
            import traceback
            print(traceback.format_exc())
        print(f"=== Completed Simulation for Path {path_idx}: {PATH_TYPES[path_idx]} ===\n")

    print("\n--- All simulations finished ---")

if __name__ == "__main__":
    try:
        main()
    except FileNotFoundError as e:
        print(f"\nExecution Error: {e}. Ensure Simulation.py, Controller.py, Visualizer.py are accessible.")
    except ImportError as e:
        print(f"\nImport Error: {e}. Check required libraries (numpy, matplotlib).")
    except Exception as e:
        import traceback
        print(f"\nAn unexpected error occurred in main execution:")
        print(traceback.format_exc())