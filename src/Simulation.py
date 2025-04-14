import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time
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
        self.visualizer = Visualizer(self.desired_path)
        
        self.initial_time = time.time_ns()

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
        forward_velocity = (left_wheel_velocity + right_wheel_velocity) * self.wheel_radius / 2
        angular_velocity_z = (right_wheel_velocity - left_wheel_velocity) * self.wheel_radius/ self.wheel_width
        # Update the robot's position and orientation
        self.x += forward_velocity * np.cos(self.theta) * self.dt
        self.y += forward_velocity * np.sin(self.theta) * self.dt
        self.theta += angular_velocity_z * self.dt

        # Normalize theta to keep it within [-pi, pi]
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

        self.actual_path = np.append(self.actual_path, [[self.x, self.y]], axis=0)  # Append the current position to the actual path
        self.visualizer.update(self.actual_path,time.time_ns() - self.initial_time,
                                left_wheel_velocity, right_wheel_velocity, 
                                forward_velocity, angular_velocity_z)

        print(  f"Robot state updated: x={self.x}, y={self.y}, theta={self.theta}")

    def get_robot_state(self):
        """
        Get the current state of the robot.
        :return: A tuple (x, y, theta) representing the robot's position and orientation.
        """
        return self.x, self.y, self.theta
        

class Visualizer:
    def __init__(self, desired_path):
        """
        Initialize the visualizer with the desired and actual paths.
        :param desired_path: A numpy array of shape (N, 2) representing the desired path.
        """
        self.desired_path = desired_path
        self.time_data = []
        self.fig = plt.figure(figsize=(16, 9))
        # Create a grid for subplots
        grid = plt.GridSpec(4, 3, hspace=0.7, wspace=0.2)

        # Large plot for robot path
        self.path_ax = self.fig.add_subplot(grid[:, :2])
        self.path_ax.set_aspect('equal')
        self.path_ax.set_title("Robot Path")
        self.path_ax.set_xlabel("X-axis")
        self.path_ax.set_ylabel("Y-axis")
        self.path_ax.plot(desired_path[:, 0], desired_path[:, 1], 'r-', label='Desired Path')
        self.path_ax.legend()

        # Small plot for wheel angular velocities
        self.wheel_ax = self.fig.add_subplot(grid[:2, 2])
        self.wheel_ax.set_title("Wheel Angular Velocities")
        self.wheel_ax.set_xlabel("Time (s)")
        self.wheel_ax.set_ylabel("Angular Velocity (rad/s)")
        self.wheel_ax.grid()
        self.left_wheel_data = []
        self.right_wheel_data = []

        # Small plot for forward velocity and yaw angular velocity
        self.velocity_ax = self.fig.add_subplot(grid[2:, 2])
        self.velocity_ax.set_title("Forward and Yaw Velocities")
        self.velocity_ax.set_xlabel("Time (s)")
        self.velocity_ax.set_ylabel("Velocity")
        self.velocity_ax.grid()
        self.forward_velocity_data = []
        self.yaw_velocity_data = []

    def plot_robot(self, actual_path):
        # Update robot path
        self.path_ax.clear()
        self.path_ax.plot(self.desired_path[:, 0], self.desired_path[:, 1], 'r--', label="Desired Path")
        self.path_ax.plot(actual_path[:, 0], actual_path[:, 1], 'g-', label="Actual Path")
        self.path_ax.plot(actual_path[-1, 0], actual_path[-1, 1], 'bo', label="Robot Position")
        arrow_length = 0.5
        theta = np.arctan2(actual_path[-1, 1] - actual_path[-2, 1], actual_path[-1, 0] - actual_path[-2, 0]) if len(actual_path) > 1 else 0
        self.path_ax.arrow(actual_path[-1, 0], actual_path[-1, 1], arrow_length * np.cos(theta), arrow_length * np.sin(theta),
                           head_width=0.1, head_length=0.2, fc='blue', ec='blue')
        self.path_ax.set_aspect('equal')
        self.path_ax.set_title("Robot Path")
        self.path_ax.set_xlabel("X-axis (m)")
        self.path_ax.set_ylabel("Y-axis (m)")
        self.path_ax.legend()

    def plot_wheel_velocities(self, time, left_wheel_velocity, right_wheel_velocity):
        # Update wheel angular velocities
        self.time_data.append(time / 1e9)  # Convert time from nanoseconds to seconds
        self.left_wheel_data.append(left_wheel_velocity)
        self.right_wheel_data.append(right_wheel_velocity)
        self.wheel_ax.clear()
        self.wheel_ax.plot(self.time_data, self.left_wheel_data, 'b-', label="Left Wheel")
        self.wheel_ax.plot(self.time_data, self.right_wheel_data, 'g-', label="Right Wheel")
        self.wheel_ax.set_title("Wheel Angular Velocities")
        self.wheel_ax.set_xlabel("Time (s)")
        self.wheel_ax.set_ylabel("Angular Velocity (rad/s)")
        self.wheel_ax.legend()
        self.wheel_ax.grid()

    def plot_robot_velocities(self, time, forward_velocity, yaw_velocity):
        # Update forward and yaw velocities
        self.forward_velocity_data.append(forward_velocity)
        self.yaw_velocity_data.append(yaw_velocity)
        self.velocity_ax.clear()
        self.velocity_ax.plot(self.time_data, self.forward_velocity_data, 'r-', label="Forward Velocity")
        self.velocity_ax.plot(self.time_data, self.yaw_velocity_data, 'm-', label="Yaw Velocity")
        self.velocity_ax.set_title("Forward Velocity (m/s) and Angular Yaw Velocities (rad/s)")
        self.velocity_ax.set_xlabel("Time (s)")
        self.velocity_ax.set_ylabel("Velocity")
        self.velocity_ax.legend()
        self.velocity_ax.grid()

    def update(self, actual_path, time, left_wheel_velocity, right_wheel_velocity, forward_velocity, yaw_velocity):
        self.plot_robot(actual_path)
        self.plot_wheel_velocities(time, left_wheel_velocity, right_wheel_velocity)
        self.plot_robot_velocities(time, forward_velocity, yaw_velocity)
        plt.pause(0.0000001)