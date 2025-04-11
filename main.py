import numpy as np
from Simulation import Simulation
from Controller import LyaponovEnergyBasedController


def generate_path():
    """
    Generate a predefined path for the robot to follow.
    :return: A numpy array of shape (N, 2) representing the desired path.
    """
    t = np.linspace(0, 1.5 * np.pi, 200)
    x = 5 * np.cos(t)
    y = 4 * np.sin(t)
    return np.column_stack((x, y))

def main():
    # Generate the desired path
    desired_path = generate_path()

    # Initial robot state (x, y, theta)
    robot_state = desired_path[0] + np.array([0.1, 0.1])
    robot_state = np.append(robot_state, 0.0)  # Add theta

    # Simulation parameters
    dt = 0.01  # Time step
    wheel_radius = 0.1
    wheel_width = 0.5
    k_forward = 1.0
    k_theta = 0.1
    max_steps = 1000
    v_max = 0.1
    omega_max = 0.1
    # Initialize the simulation
    simulation = Simulation(desired_path, wheel_radius, wheel_width, intial_pose=robot_state)

    # Initialize the controller
    controller = LyaponovEnergyBasedController(k_forward=k_forward, k_theta=k_theta, predefind_path=desired_path, wheel_width=wheel_width, wheel_radius=wheel_radius, v_max=v_max, omega_max=omega_max)

    # Run the simulation
    for step in range(max_steps):

        current_state = simulation.get_robot_state()

        # Compute control commands
        left_wheel_velocity, right_wheel_velocity = controller.compute_control(current_state, desired_path)

        # Update the robot state
        robot_state = simulation.execute_cmd(left_wheel_velocity, right_wheel_velocity)

        # Plot the robot
        simulation.plot_robot()
        simulation.show()


if __name__ == "__main__":
    main()