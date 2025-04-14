import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time
# Keep Visualizer class here or move to its own file if preferred
from Visualizer import Visualizer # Assuming Visualizer is in Visualizer.py

class Simulation:
    def __init__(self, dt, desired_path, wheel_radius, wheel_width, initial_pose = np.array([0.0, 0.0, 0.0])):
        """
        Initialize the map with a desired path.
        :param dt: Time step duration.
        :param desired_path: A numpy array of shape (N, 2) representing the desired path.
        :param wheel_radius: Radius of the wheels.
        :param wheel_width: Distance between the wheels.
        :param initial_pose: Initial pose (x, y, theta) of the robot.
        """
        self.desired_path = desired_path
        self.wheel_radius = wheel_radius
        self.wheel_width = wheel_width
        self.dt = dt
        self.x, self.y, self.theta = initial_pose
        self.actual_path = np.array([[self.x, self.y, self.theta]])  # Store pose (x, y, theta)
        # Removed Visualizer initialization from here, handle it in main

        # Data storage for analysis and animation
        self.time_stamps = [0.0]
        self.wheel_velocities_cmd = [(0.0, 0.0)] # (left_cmd, right_cmd)
        self.robot_velocities_actual = [(0.0, 0.0)] # (forward_vel, angular_vel)

    def execute_cmd(self, left_wheel_velocity, right_wheel_velocity):
        """
        Update the robot's state based on the given wheel angular velocity commands.
        :param left_wheel_velocity: Angular velocity command for the left wheel (rad/s).
        :param right_wheel_velocity: Angular velocity command for the right wheel (rad/s).
        """
        # Calculate the robot's linear and angular velocities based on commands
        forward_velocity = (left_wheel_velocity + right_wheel_velocity) * self.wheel_radius / 2
        angular_velocity_z = (right_wheel_velocity - left_wheel_velocity) * self.wheel_radius / self.wheel_width

        # Update the robot's position and orientation using kinematic model
        delta_x = forward_velocity * np.cos(self.theta) * self.dt
        delta_y = forward_velocity * np.sin(self.theta) * self.dt
        delta_theta = angular_velocity_z * self.dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to keep it within [-pi, pi]
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

        # Store current state and commands
        current_pose = np.array([[self.x, self.y, self.theta]])
        self.actual_path = np.append(self.actual_path, current_pose, axis=0)
        self.time_stamps.append(self.time_stamps[-1] + self.dt)
        self.wheel_velocities_cmd.append((left_wheel_velocity, right_wheel_velocity))
        self.robot_velocities_actual.append((forward_velocity, angular_velocity_z))

        # print(f"Step: {len(self.time_stamps)-1}, Time: {self.time_stamps[-1]:.2f}, X: {self.x:.2f}, Y: {self.y:.2f}, Theta: {self.theta:.2f}")

    def get_robot_state(self):
        """
        Get the current state of the robot.
        :return: A tuple (x, y, theta) representing the robot's position and orientation.
        """
        return self.x, self.y, self.theta

    def get_simulation_data(self):
        """ Returns all collected data """
        return {
            "time": np.array(self.time_stamps),
            "actual_path": self.actual_path,
            "desired_path": self.desired_path,
            "wheel_cmds": np.array(self.wheel_velocities_cmd),
            "robot_vels": np.array(self.robot_velocities_actual)
        }