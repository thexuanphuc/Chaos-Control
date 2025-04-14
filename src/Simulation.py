import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

class Simulation:
    def __init__(self, dt, desired_path, wheel_radius, wheel_width, intial_pose = np.array([0.0, 0.0, 0.0])):
        """
        Initialize the map with a desired path.
        :param desired_path: A numpy array of shape (N, 2) representing the desired path.
        """

        self.desired_path = desired_path
        self.wheel_radius = wheel_radius
        self.wheel_width = wheel_width
        self.dt = dt
        self.x, self.y, self.theta = intial_pose
        self.actual_path = np.array([[self.x, self.y]])  # Initialize actual path with the first position

        # Create a figure and axis for the plot
        self.fig, self.ax = plt.subplots()
        self.ax.plot(self.desired_path[:, 0], self.desired_path[:, 1], 'r--', label="Desired Path")
        self.ax.plot(self.actual_path[:, 0], self.actual_path[:, 1], 'g-', label="Actual Path")
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.legend()
        self.ax.grid()
        plt.ion()

    def plot_robot(self):
        """
        Plot the robot's position and orientation on the map.
        """
        self.actual_path = np.append(self.actual_path, [[self.x, self.y]], axis=0)  # Append the current position to the actual path
        # Clear the plot
        self.ax.clear()
        # Plot the desired path
        self.ax.plot(self.desired_path[:, 0], self.desired_path[:, 1], 'r--', label="Desired Path")
        self.ax.plot(self.actual_path[:, 0], self.actual_path[:, 1], 'g-', label="Actual Path")

        # Plot the robot's position
        self.ax.plot(self.x, self.y, 'bo', label="Robot Position")

        # Draw the robot's orientation
        arrow_length = 0.5
        self.ax.arrow(self.x, self.y, arrow_length * np.cos(self.theta), arrow_length * np.sin(self.theta),
                      head_width=0.1, head_length=0.2, fc='blue', ec='blue', label="Orientation")

        plt.pause(0.0000000001)

    def execute_cmd(self, left_wheel_velocity, right_wheel_velocity):
        """
        Update the robot's state based on the given commands.
        :param robot_state: A tuple (x, y, theta) representing the robot's position and orientation.
        :param forward_velocity: Linear velocity of the robot.
        :param angular_velocity_z: Angular velocity of the robot around the z-axis.
        :param dt: Time step for the update.
        :param wheel_distance: Distance between the two wheels.
        :return: Updated robot state (x, y, theta).
        """
        # Calculate the robot's linear and angular velocities
        forward_velocity = (left_wheel_velocity + right_wheel_velocity) * self.wheel_width / 2
        angular_velocity_z = (right_wheel_velocity - left_wheel_velocity) * self.wheel_width/ self.wheel_width

        # Update the robot's position and orientation
        self.x += forward_velocity * np.cos(self.theta) * self.dt
        self.y += forward_velocity * np.sin(self.theta) * self.dt
        self.theta += angular_velocity_z * self.dt

        # Normalize theta to keep it within [-pi, pi]
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

        print(  f"Robot state updated: x={self.x}, y={self.y}, theta={self.theta}")

    def get_robot_state(self):
        """
        Get the current state of the robot.
        :return: A tuple (x, y, theta) representing the robot's position and orientation.
        """
        return self.x, self.y, self.theta

    def show(self):
        """
        Display the map.
        """
        plt.show()
