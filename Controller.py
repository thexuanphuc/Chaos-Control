import numpy as np

class Controller:
    def __init__(self):
        """
        Base controller class. This will be extended by specific controllers.
        """
        pass

    def compute_control(self, robot_state, desired_state):
        """
        Compute control commands for the robot.
        :param robot_state: Current state of the robot (x, y, theta).
        :param desired_state: Desired state of the robot (x, y).
        :return: Tuple (forward_velocity, angular_velocity).
        """
        raise NotImplementedError("This method should be implemented by subclasses.")


class LyaponovEnergyBasedController(Controller):
    def __init__(self, k_forward=1.0, k_theta=2.0, predefind_path=None, wheel_width=0.5, wheel_radius = 0.1, v_max=1.0, omega_max=1.0):
        """
        Differential drive controller for a two-wheeled robot.
        :param kp_linear: Proportional gain for linear velocity.
        :param kp_angular: Proportional gain for angular velocity.
        """
        super().__init__()
        self.kf = k_forward
        self.ktt = k_theta
        self.predefind_path = predefind_path
        # self.v_forward_reference = None
        # self.omega_reference = None
        self.wheel_width = wheel_width 
        self.wheel_radius = wheel_radius
        self.v_max = v_max
        self.omega_max = omega_max
        #  TODO turn parameters
        self.v_f = v_max
        self.omega_f = 0.0


    def compute_control(self, robot_state, predefind_path: np.ndarray):
        """
        Compute forward and angular velocities to move the robot towards the desired state.
        :param robot_state: Current state of the robot (x, y, theta).
        :param desired_state: Desired state of the robot (x, y).
        :return: Tuple (forward_velocity, angular_velocity).
        """
        x, y, theta = robot_state
        # Find the closest point on the predefined path
        distances = np.linalg.norm(predefind_path - np.array([x, y]), axis=1)
        closest_index = np.argmin(distances) + 1
        if closest_index == len(predefind_path):
            return 0, 0
        else:
            next_point = predefind_path[closest_index + 1]

        theta_d = np.arctan2(next_point[1] - y, next_point[0] - x)
        x_d, y_d = next_point

        print("the desired point is ", x_d, y_d, "the desired angle is ", theta_d)
        print("the current point is ", x, y, "the current angle is ", theta)
        # Compute the error in position
        error_x = x_d - x
        error_y = y_d - y
        error_theta = theta_d - theta

        # Normalize the angle error to [-pi, pi]
        error_theta = (error_theta + np.pi) % (2 * np.pi) - np.pi

        # convert the error to the robot's frame
        error_forward = error_x * np.cos(theta) + error_y * np.sin(theta)
        error_lateral = -error_x * np.sin(theta) + error_y * np.cos(theta)
        error_theta = error_theta

        # Compute the control commands
        #  TODO : use the reference velocity, not just constant values
        # forward_velocity = self.v_forward_reference[closest_index] * np.cos(error_theta) + self.kf * error_forward
        # angular_velocity = self.omega_reference[closest_index] + self.ktt * error_theta + self.v_forward_reference[closest_index] * error_lateral * np.sin(error_theta) / error_theta
        forward_velocity = self.v_f * np.cos(error_theta) + self.kf * error_forward
        angular_velocity = self.omega_f + self.ktt * error_theta + self.v_f * error_lateral * np.sin(error_theta) / error_theta
        
        # Limit the velocities
        # forward_velocity = np.clip(forward_velocity, -self.v_max, self.v_max)
        # angular_velocity = np.clip(angular_velocity, -self.omega_max, self.omega_max)

        # calculate the wheels velocities
        v_left = forward_velocity - (self.wheel_width / 2) * angular_velocity
        v_right = forward_velocity + (self.wheel_width / 2) * angular_velocity

        # calculate the wheel angular velocities
        v_left = v_left / self.wheel_radius
        v_right = v_right / self.wheel_radius

        print(f" the wheel velocities is v_left: {v_left}, v_right: {v_right}")
        return v_left, v_right
