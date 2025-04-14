import numpy as np
from Simulation import Simulation
from Controller import LyaponovEnergyBasedController


def generate_path(path_type="1"):
    """
    Generate a predefined path for the robot to follow.
    :return: A numpy array of shape (N, 2) representing the desired path.
    """

    if path_type == 1:
        # Circle
        t = np.linspace(0, 2 * np.pi, 500)
        x = 5 * np.cos(t)
        y = 5 * np.sin(t)
    elif path_type == 2:
        # Ellipse
        t = np.linspace(0, 2 * np.pi, 500)
        x = 6 * np.cos(t)
        y = 4 * np.sin(t)
    elif path_type == 3:
        # Spiral
        t = np.linspace(0, 4 * np.pi, 500)
        x = t * np.cos(t)
        y = t * np.sin(t)
    elif path_type == 4:
        # Line
        x = np.linspace(0, 10, 500)
        y = 2 * x
    elif path_type == 5:
        # Lemniscate (Figure-eight)
        t = np.linspace(0, 5 * np.pi, 500)
        x = 5 * np.sin(t)
        y = 5 * np.sin(t) * np.cos(t)
    elif path_type == 6:
        # Sine wave
        x = np.linspace(0, 10, 500)
        y = 5 * np.sin(x)
    elif path_type == 7:
        # Heart shape
        t = np.linspace(0, 2 * np.pi, 500)
        x = 16 * np.sin(t)**3
        y = 13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)
    elif path_type == 8:
        # Square wave-like path
        x = np.linspace(0, 10, 500)
        y = np.sign(np.sin(2 * np.pi * x / 2))
    elif path_type == 9:
        # Parabola
        x = np.linspace(-5, 5, 500)
        y = x**2
    else:
        raise ValueError("Invalid path type selected!")

    return np.column_stack((x, y))

def main():
    # Generate the desired path
    desired_path = generate_path(7)

    # Initial robot state (x, y, theta)
    robot_state = desired_path[0] + np.array([0.1, 0.1])
    robot_state = np.append(robot_state, 0.0)  # Add theta

    # Simulation parameters
    dt = 0.01  # Time stepq
    wheel_radius = 0.1
    wheel_width = 0.5
    k_forward = 3.0
    k_theta = 1.0
    max_steps = 1000
    v_max = 5.0
    omega_max = 6.5
    # Initialize the simulation
    simulation = Simulation(dt, desired_path, wheel_radius, wheel_width, intial_pose=robot_state)

    # Initialize the controller
    controller = LyaponovEnergyBasedController(k_forward=k_forward, k_theta=k_theta, predefind_path=desired_path, wheel_width=wheel_width, wheel_radius=wheel_radius, v_max=v_max, omega_max=omega_max)

    # Run the simulation
    for step in range(max_steps):

        current_state = simulation.get_robot_state()

        # Compute control commands
        omega_left_wheel, omega_right_wheel = controller.compute_control(current_state, desired_path)

        # Update the robot state
        robot_state = simulation.execute_cmd(omega_left_wheel, omega_right_wheel)

        # Plot the robot
        simulation.plot_robot()
        simulation.show()


if __name__ == "__main__":
    main()