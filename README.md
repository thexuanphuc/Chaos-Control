# Chaos-Control
Mini_Project 01 for Advanced Control Course

# Lyapunov-Based Controller for Nonholonomic Mobile Robot

This repository implements a Lyapunov Energy-Based Controller for trajectory tracking of a nonholonomic mobile robot (e.g., a 3-wheeled or unicycle-type robot).

## Table of Contents
- [Lyapunov-Based Controller for Nonholonomic Mobile Robot](#lyapunov-based-controller-for-nonholonomic-mobile-robot)
- [Robot Kinematic Model](#robot-kinematic-model)
- [Error Definition](#error-definition)
- [Error Dynamics](#error-dynamics)
- [Lyapunov-Based Control Law](#lyapunov-based-control-law)
- [Closed-Loop Error Dynamics](#closed-loop-error-dynamics)
- [Features](#features)
- [Usage](#usage)
- [Dependencies](#dependencies)
- [License](#license)

## Robot Kinematic Model

The kinematic equations of the robot are given by:

```
x_dot = v * cos(theta)
y_dot = v * sin(theta)
theta_dot = omega
```

Where:  
- `v` is the linear velocity  
- `omega` is the angular velocity  
- `(x, y, theta)` represent the robot's position and orientation  

## Error Definition

The tracking error is defined in the robot's body frame as:

```
e_x = cos(theta) * (x_r - x) + sin(theta) * (y_r - y)
e_y = -sin(theta) * (x_r - x) + cos(theta) * (y_r - y)
e_theta = theta_r - theta
```

Where `(x_r, y_r, theta_r)` is the desired/reference trajectory.

## Error Dynamics

The error dynamics of the system are given by:

```
e_x_dot = omega * e_y - v + v_r * cos(e_theta)
e_y_dot = -omega * e_x + v_r * sin(e_theta)
e_theta_dot = omega_r - omega
```

## Lyapunov-Based Control Law

The control inputs (`v` and `omega`) are designed using Lyapunov stability theory:

```
v = v_r * cos(e_theta) + K_x * e_x
omega = omega_r + K_theta * e_theta + v_r * K_y * e_y
```

Where:  
- `K_x > 0`, `K_theta > 0`, `K_y > 0` are positive controller gains.

## Closed-Loop Error Dynamics

Substituting the control inputs gives the closed-loop error dynamics:

```
e_x_dot = omega_r * e_y + K_theta * e_theta * e_y + v_r * K_y * e_y^2 - K_x * e_x
e_y_dot = -omega_r * e_x - K_theta * e_theta * e_x - v_r * K_y * e_x * e_y + v_r * sin(e_theta)
e_theta_dot = -K_theta * e_theta - v_r * K_y * e_y
```

## Features

- Lyapunov-based globally asymptotically stable control law.
- Trajectory tracking for nonholonomic robots.
- Applicable to circular, spiral, or custom reference trajectories.
- Animation and visualization using Matplotlib.

## Usage

1. Clone the repository:

```bash
git clone https://github.com/your-username/your-repo-name.git
cd your-repo-name
```

2. Modify the desired trajectory in `trajectory_generator.py`.

3. Run the simulation:

```bash
python main.py
```

## Dependencies

- Python 3.x
- Numpy
- Matplotlib

Install required packages using:

```bash
pip install -r requirements.txt
```

## License

This project is open-source and available under the MIT License.

