import numpy as np
import matplotlib.pyplot as plt
import time
import os
from matplotlib.animation import PillowWriter
from Simulation import Simulation
from Controller2 import LyapunovKinematicController, BacksteppingDynamicController, BacksteppingDynamicController2
from Visualizer import Visualizer

PATH_TYPES = {
    1: "Circle", 2: "Ellipse", 3: "Spiral", 4: "Line", 5: "Lemniscate",
    6: "SineWave", 7: "Heart", 8: "SquareWave", 9: "Parabola", 10: "ComplexPath"
}

def generate_path(path_type_index=2, num_points=150):
    path_type_str = str(path_type_index)
    if path_type_str == '1':
        radius = 5.0
        t = np.linspace(0, 2 * np.pi, num_points)
        x = radius * np.cos(t)
        y = radius * np.sin(t)
    elif path_type_str == '2':
        rx, ry = 6.0, 3.5
        t = np.linspace(0, 2 * np.pi, num_points)
        x = rx * np.cos(t)
        y = ry * np.sin(t)
    elif path_type_str == '3':
        a = 0.4
        b = 0.2
        t = np.linspace(0, 8 * np.pi, num_points)
        x = (a + b * t) * np.cos(t)
        y = (a + b * t) * np.sin(t)
    elif path_type_str == '4':
        x_start, y_start = 0, 1
        x_end, y_end = 10, 6
        x = np.linspace(x_start, x_end, num_points)
        y = np.linspace(y_start, y_end, num_points)
    elif path_type_str == '5':
        scale = 5.0
        t = np.linspace(0, 2 * np.pi, num_points)
        x = scale * np.cos(t)
        y = scale * np.sin(t) * np.cos(t)
    elif path_type_str == '6':
        amplitude = 2.0
        frequency = 1.0
        length = 3 * (2 * np.pi / frequency)
        x = np.linspace(0, length, num_points)
        y = amplitude * np.sin(frequency * x)
    elif path_type_str == '7':
        scale = 2.0
        t = np.linspace(0, 2 * np.pi, num_points)
        x = scale * (16 * np.sin(t)**3) / 10
        y = scale * (13 * np.cos(t) - 5 * np.cos(2*t) - 2 * np.cos(3*t) - np.cos(4*t)) / 10
    elif path_type_str == '8':
        cycles = 3
        amplitude = 2.0
        period_points = num_points // cycles
        segment_points = period_points // 4
        x_coords, y_coords = [], []
        current_x = 0
        for _ in range(cycles):
            x_coords.extend(np.linspace(current_x, current_x + segment_points * 0.1, segment_points))
            y_coords.extend(np.full(segment_points, 0))
            current_x += segment_points * 0.1
            x_coords.extend(np.full(segment_points, current_x))
            y_coords.extend(np.linspace(0, amplitude, segment_points))
            x_coords.extend(np.linspace(current_x, current_x + segment_points * 0.1, segment_points))
            y_coords.extend(np.full(segment_points, amplitude))
            current_x += segment_points * 0.1
            x_coords.extend(np.full(segment_points, current_x))
            y_coords.extend(np.linspace(amplitude, 0, segment_points))
        x = np.array(x_coords[:num_points])
        y = np.array(y_coords[:num_points])
    elif path_type_str == '9':
        a = 0.2
        x_range = 10.0
        x = np.linspace(-x_range/2, x_range/2, num_points)
        y = a * x**2
    elif path_type_str == '10':
        a, b = 5, 3
        delta = np.pi / 2
        freq_x, freq_y = 1.5, 2.5
        t = np.linspace(0, 4 * np.pi, num_points)
        x = a * np.sin(freq_x * t + delta)
        y = b * np.sin(freq_y * t)
    else:
        valid_keys = ", ".join(map(str, PATH_TYPES.keys()))
        raise ValueError(f"Invalid path_type_index: {path_type_index}. Choose from {valid_keys}")
    return np.column_stack((x, y))

def main():
    selected_path_type_idx = 7
    path_name_str = PATH_TYPES.get(selected_path_type_idx, "CustomPath")
    print(f"--- Starting Simulation for: {path_name_str} (Type {selected_path_type_idx}) ---")
    desired_path_coords = generate_path(selected_path_type_idx, num_points=200)
    if desired_path_coords.shape[0] < 2:
        print("Error: Generated path has less than 2 points. Exiting.")
        return
    path_start_dx = desired_path_coords[1, 0] - desired_path_coords[0, 0]
    path_start_dy = desired_path_coords[1, 1] - desired_path_coords[0, 1]
    initial_robot_theta = np.arctan2(path_start_dy, path_start_dx)
    offset_distance = 5
    initial_robot_pose = np.array([
        desired_path_coords[0, 0] - offset_distance * np.sin(initial_robot_theta),
        desired_path_coords[0, 1] + offset_distance * np.cos(initial_robot_theta),
        initial_robot_theta + np.deg2rad(15)
    ])
    initial_robot_velocity = np.array([0.0, 0.0])
    simulation_dt = 0.02
    robot_wheel_radius = 0.05
    robot_wheel_width = 0.3
    max_simulation_steps = 5000  # Increased to allow more recovery time
    actual_robot_mass = 12.0
    actual_robot_inertia = 0.8
    continuous_disturbance_bound = 10
    kick_target_idx_on_path = len(desired_path_coords) // 2
    kick_trigger_dist_from_target = 0.3
    kick_active_duration = 1
    kick_disturbance_magnitude = np.array([0.0, 220.0])
    kin_k_forward = 1.5
    kin_k_theta = 3.5
    kin_k_lateral_factor = 0.8
    kin_v_ref = 2
    kin_omega_max = np.pi * 30
    kin_lookahead = 0.35
    gamma = 0.05  # Increased for faster torque response
    kd = np.diag([50.0, 25.0])
    kz = np.diag([15.0, 15.0])
    controller_assumed_disturbance_bound = max(continuous_disturbance_bound, np.max(np.abs(kick_disturbance_magnitude))) * 1.2
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
        k_forward=kin_k_forward, k_theta=kin_k_theta, k_lateral_gain_factor=kin_k_lateral_factor,
        v_ref=kin_v_ref, omega_max=kin_omega_max, lookahead_dist=kin_lookahead
    )
    backstepping_dynamic_ctrl = BacksteppingDynamicController(
        dt=simulation_dt, wheel_radius=robot_wheel_radius, wheel_width=robot_wheel_width,
        gamma=gamma, kinematic_controller=kinematic_ctrl,
        robot_mass=actual_robot_mass, robot_inertia=actual_robot_inertia,
        kd=kd, kz=kz, disturbance_bound=controller_assumed_disturbance_bound,
        epsilon=0.005, torque_limit=1000.0
    )
    controller_status_history = []
    print("Starting simulation loop...")
    sim_start_time = time.time()
    kinematic_controller_finished = False
    for step_num in range(max_simulation_steps):
        current_robot_state = simulation_instance.get_robot_state()
        if not kinematic_controller_finished:
            try:
                u_left, u_right, status_dict = backstepping_dynamic_ctrl.compute_control(
                    current_robot_state, desired_path_coords
                )
                controller_status_history.append(status_dict)
                if backstepping_dynamic_ctrl.kinematic_controller.finished_flag:
                    print(f"Kinematic controller signaled finish at step {step_num}.")
                    kinematic_controller_finished = True
                    u_left, u_right = 0.0, 0.0
            except Exception as e:
                print(f"Error in controller at step {step_num}: {e}")
                print(f"Current robot state: {current_robot_state}")
                u_left, u_right = 0.0, 0.0
        else:
            u_left, u_right = 0.0, 0.0
            if controller_status_history:
                dummy_status = controller_status_history[-1].copy()
                dummy_status['eta'] = np.zeros_like(dummy_status['eta'])
                dummy_status['z'] = np.zeros_like(dummy_status['z'])
                dummy_status['v_d'] = np.zeros_like(dummy_status['v_d'])
                controller_status_history.append(dummy_status)
        try:
            simulation_instance.execute_cmd(u_left, u_right)
        except Exception as e:
            print(f"Error during simulation step {step_num}: {e}")
            print(f"Commanded inputs: Left={u_left:.2f}, Right={u_right:.2f}")
            break
        if (step_num + 1) % 250 == 0 and controller_status_history:
            last_status = controller_status_history[-1]
            eta_norm = np.linalg.norm(last_status['eta'])
            z_norm = np.linalg.norm(last_status['z'])
            print(f"Step: {step_num+1}/{max_simulation_steps}, Sim Time: {simulation_instance.time_stamps[-1]:.2f}s, "
                  f"Pos:({current_robot_state[0]:.2f},{current_robot_state[1]:.2f}), "
                  f"Vels:({current_robot_state[3]:.2f},{current_robot_state[4]:.2f}), "
                  f"Vel Error Norm: {eta_norm:.4f}, Torque Error Norm: {z_norm:.4f}")
        if kinematic_controller_finished and step_num > kinematic_ctrl.closest_index + 50:  # Increased threshold
            print(f"Simulation loop ending at step {step_num} as controller finished and robot settled.")
            break
    else:
        print(f"Simulation completed after {max_simulation_steps} steps.")
    sim_end_time = time.time()
    print(f"Total simulation wall-clock time: {sim_end_time - sim_start_time:.2f} seconds.")
    simulation_results_data = simulation_instance.get_simulation_data()
    num_control_actions = len(simulation_results_data['time']) - 1
    controller_status_history = controller_status_history[:num_control_actions]
    print("Initializing visualization...")
    visualizer_instance = Visualizer(desired_path_coords)
    if simulation_instance.kick_target_point_coords is not None:
        kick_tp_coords = simulation_instance.kick_target_point_coords
        visualizer_instance.path_ax.plot(kick_tp_coords[0], kick_tp_coords[1], 'X', color='magenta',
                                        markersize=12, label='Kick Trigger Target Pt', alpha=0.8)
        visualizer_instance.path_ax.legend(loc='best')
    animation_frame_interval_ms = 40
    simulation_steps_per_frame = 4
    print("Creating animation...")
    anim = visualizer_instance.create_animation(
        simulation_results_data, controller_status_history,
        interval=animation_frame_interval_ms, step=simulation_steps_per_frame,
        true_m=actual_robot_mass, true_I=actual_robot_inertia
    )
    # Save animation as GIF in media folder
    gif_filename = f"{path_name_str}_path.gif"
    gif_filepath = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "media", gif_filename))
    try:
        anim.save(gif_filepath, writer=PillowWriter(fps=1000/animation_frame_interval_ms))
        print(f"Animation saved to {gif_filepath}")
    except Exception as e:
        print(f"Failed to save animation GIF: {e}")
    print("Plotting final static results...")
    visualizer_instance.plot_final_results(
        simulation_results_data, controller_status_history, path_name_str,
        true_m=actual_robot_mass, true_I=actual_robot_inertia
    )
    try:
        plt.show()
        print("Plots displayed. Close plot windows to exit.")
    except Exception as e:
        print(f"Note: Could not display plots interactively ({e}). Check saved GIF/static plots.")

if __name__ == "__main__":
    main()