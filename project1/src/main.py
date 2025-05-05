import numpy as np
import matplotlib.pyplot as plt
from Simulation import Simulation
from Controller import LyapunovEnergyBasedController
from Visualizer import Visualizer # Import the Visualizer

# Dictionary mapping type index to name for filenames/titles
PATH_TYPES = {
    1: "Circle", 2: "Ellipse", 3: "Spiral", 4: "Line", 5: "Lemniscate",
    6: "SineWave", 7: "Heart", 8: "SquareWave", 9: "Parabola", 10: "Complex"
}

def generate_path(path_type_index=2):
    """
    Generate a predefined path for the robot to follow.
    :param path_type_index: Integer index corresponding to PATH_TYPES keys.
    :return: A numpy array of shape (N, 2) representing the desired path.
    """
    num_points = 150 # Number of points in the path
    path_type = str(path_type_index) # Keep original logic using string comparison for now

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
        a = 1 # Controls tightness
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
        amp = 3
        freq = 1
        x = np.linspace(0, 4 * np.pi / freq, num_points)
        y = amp * np.sin(freq * x)
    elif path_type == '7': # Heart shape
        t = np.linspace(0, 2 * np.pi, num_points)
        x = 16 * np.sin(t)**3
        y = 13 * np.cos(t) - 5 * np.cos(2*t) - 2 * np.cos(3*t) - np.cos(4*t)
        x = x/2 # Scale down
        y = y/2 # Scale down
    elif path_type == '8': # Square wave-like path (smoothed slightly)
        x = np.linspace(0, 10, num_points)
        y = 2 * np.sign(np.sin(np.pi * x)) # Amplitude 2
        # Add some intermediate points for smoother corners in simulation
        x_detailed, y_detailed = [], []
        for i in range(len(x)-1):
            x_detailed.extend([x[i], x[i+1], x[i+1]])
            y_detailed.extend([y[i], y[i], y[i+1]])
        x = np.array(x_detailed)
        y = np.array(y_detailed)

    elif path_type == '9': # Parabola
        x = np.linspace(-5, 5, num_points)
        y = 0.5*x**2
    elif path_type == '10': # Super complicated path (combination)
        t = np.linspace(0, 4 * np.pi, num_points)
        x = t * np.cos(t) + 3 * np.sin(2 * t)
        y = t * np.sin(t) + 3 * np.cos(t) # Changed 2nd term
    else:
        raise ValueError(f"Invalid path type index: {path_type_index}. Choose from {list(PATH_TYPES.keys())}")

    return np.column_stack((x, y))

def main():
    # --- Simulation Parameters ---
    selected_path_type = 10 # Choose path type index (e.g., 2 for Ellipse)
    path_name = PATH_TYPES.get(selected_path_type, "Custom")
    print(f"Generating path: {path_name} (Type {selected_path_type})")

    desired_path = generate_path(selected_path_type)

    # Initial robot state (slightly offset from path start, with initial orientation)
    start_dx = desired_path[1, 0] - desired_path[0, 0]
    start_dy = desired_path[1, 1] - desired_path[0, 1]
    initial_theta = np.arctan2(start_dy, start_dx)
    initial_offset = 0.2 # Distance offset from start
    initial_pose = np.array([
        desired_path[0, 0] - initial_offset * np.sin(initial_theta),
        desired_path[0, 1] + initial_offset * np.cos(initial_theta),
        initial_theta + 0.1 # Small angle offset
    ])

    dt = 0.02           # Simulation time step (s) - Increased slightly
    wheel_radius = 0.05 # m
    wheel_width = 0.3   # m
    max_steps = 2000    # Maximum simulation steps

    # --- Controller Parameters ---
    k_forward = 1.5                # Gain for forward error
    k_theta = 4.0                  # Gain for orientation error
    k_lateral_gain_factor = 0.8    # Factor for lateral gain term (adjust as needed)
    v_ref = 0.8                    # Reference forward speed (m/s)
    omega_max = np.pi * 1.5        # Max robot angular velocity (rad/s)

    # --- Initialization ---
    simulation = Simulation(dt, desired_path, wheel_radius, wheel_width, initial_pose=initial_pose)
    controller = LyapunovEnergyBasedController(dt, k_forward=k_forward, k_theta=k_theta,
                                              k_lateral_gain_factor=k_lateral_gain_factor,
                                              predefined_path=desired_path, wheel_width=wheel_width,
                                              wheel_radius=wheel_radius, v_ref=v_ref, omega_max=omega_max)

    # --- Data Storage ---
    errors_list = []
    energy_list = []

    # --- Simulation Loop ---
    print("Starting simulation...")
    for step in range(max_steps):
        current_state = simulation.get_robot_state()

        # Compute control commands
        omega_left_wheel, omega_right_wheel, errors, V = controller.compute_control(current_state, desired_path)

        # Store errors and energy
        errors_list.append(errors)
        energy_list.append(V)

        # Execute command in simulation
        simulation.execute_cmd(omega_left_wheel, omega_right_wheel)

        # Check for completion condition (e.g., controller signals stop)
        if omega_left_wheel == 0 and omega_right_wheel == 0 and step > 10:
             # Add check for distance to target as well if needed
             current_pos = np.array(current_state[:2])
             dist_to_end = np.linalg.norm(current_pos - desired_path[-1])
             if dist_to_end < 0.15: # If controller stopped and close to end
                 print(f"Simulation ended early at step {step} due to reaching target.")
                 break
    else: # Runs if loop completes without break
        print(f"Simulation finished after {max_steps} steps.")


    # --- Post-Simulation Processing ---
    simulation_data = simulation.get_simulation_data()

    # --- Visualization and Animation ---
    print("Creating visualization and animation...")
    visualizer = Visualizer(desired_path)

    # Animation settings
    anim_interval = 60  # milliseconds between frames
    anim_step = 5       # Number of simulation steps per animation frame (adjust for speed)
    fps = 1000 / (anim_interval * anim_step / dt) # Calculate approximate FPS for saving

    # Create animation
    ani = visualizer.create_animation(simulation_data, errors_list, energy_list,
                                      interval=anim_interval, step=anim_step)

    # Save animation
    gif_filename = f'robot_animation_{path_name.replace(" ", "")}.gif'
    print(f"Saving animation to {gif_filename}...")
    try:
        # --- CORRECTED FPS Calculation ---
        time_per_frame = anim_step * dt
        if time_per_frame > 0:
            fps = 1.0 / time_per_frame
        else:
            print("Warning: Calculated time per frame is zero. Using default FPS=10.")
            fps = 10 # Default FPS if dt or anim_step is zero

        print(f"Calculated FPS for saving: {fps:.2f}")
        ani.save(gif_filename, writer='pillow', fps=int(fps)) # Use calculated FPS (converted to int)
        # --- End CORRECTED Part ---
        print("Animation saved successfully.")
    except Exception as e:
        print(f"Error saving animation: {e}")
        print("Ensure you have 'pillow' installed (`pip install pillow`)") # Pillow is needed, but wasn't the cause here.

    # Plot final static results (errors, energy)
    print("Plotting final results...")
    visualizer.plot_final_results(simulation_data, errors_list, energy_list, path_name)

    # Show plots (animation figure and results figure)
    plt.show()


if __name__ == "__main__":
    main()