# Chaos-Control
Mini_Project 01 for Advanced Control Course

# Lyapunov-Based Controller for Nonholonomic Mobile Robot

This repository implements a Lyapunov Energy-Based Controller for trajectory tracking of a nonholonomic mobile robot (e.g., a 3-wheeled or unicycle-type robot).

## Table of Contents
- [Lyapunov-Based Controller for Nonholonomic Mobile Robot](#lyapunov-based-controller-for-nonholonomic-mobile-robot)
- [Robot Kinematic Model](#robot-kinematic-model)
- [Error Definition](#error-definition)
- [Error Dynamics](#error-dynamics)
- [Candidate Lyapunov Function](#candidate-lyapunov-function)
- [Lyapunov-Based Control Law](#lyapunov-based-control-law)
- [Closed-Loop Error Dynamics](#closed-loop-error-dynamics)
- [Usage](#usage)
- [License](#license)

## Robot Kinematic Model

The kinematic equations of the robot are given by:

$$
\dot{x} = v \cos \theta
$$
$$
\dot{y} = v \sin \theta
$$
$$
\dot{\theta} = \omega
$$

Where:
- `v` is the linear velocity
- `omega` is the angular velocity
- `(x, y, theta)` represent the robot's position and orientation

## Error Definition

The tracking error is defined in the robot's body frame as:

$$
e_x = \cos \theta (x_r - x) + \sin \theta (y_r - y)
$$
$$
e_y = -\sin \theta (x_r - x) + \cos \theta (y_r - y)
$$
$$
e_\theta = \theta_r - \theta
$$

Where `(x_r, y_r, theta_r)` is the desired/reference trajectory.

## Error Dynamics

The error dynamics of the system are given by:


$$ 
\dot{e_x} = \omega e_y - v + v_r \cos(e_\theta)
$$

$$
\dot{e_y} = -\omega e_x + v_r \sin (e_\theta)
$$

$$
\dot{e_\theta} = \omega_r - \omega
$$

## Candidate Lyapunov Function

Candidate Lyapunov function:

$$
L = \frac{1}{2} e_x^2 + \frac{1}{2} e_y^2 + \frac{1}{2} e_\theta^2
$$

Derivative of Lyapunov function:

$$
\dot{L} = e_x \dot{e_x} + e_y \dot{e_y} + e_\theta \dot{e_\theta}
$$

Substitute error dynamics:

$$
\dot{L} = -e_x v + e_x v_r \cos e_\theta + e_y v_r \sin e_\theta + \omega_r e_\theta - \omega e_\theta
$$

## Lyapunov-Based Control Law

The control actions (`v` and `omega`) are designed using Lyapunov stability theory such that \( \dot{L} \leq 0 \):

$$
v = v_r \cos e_\theta + K_x e_x
$$

$$
\omega = \omega_r + K_\theta e_\theta + v_r K_y e_y
$$

Where:
- `K_x > 0`, `K_theta > 0`, `K_y > 0` are positive controller gains.
- These control actions make sure that $\dot{L} < 0$

## Closed-Loop Error Dynamics

Substituting control inputs gives the closed-loop error dynamics:

$$
\dot{e_x} = \omega_r e_y + K_\theta e_\theta e_y + v_r K_y e_y^2 - K_x e_x
$$

$$
\dot{e_y} = -\omega_r e_x - K_\theta e_\theta e_x - v_r K_y e_x e_y + v_r \sin e_\theta
$$

$$
\dot{e_\theta} = -K_\theta e_\theta - v_r K_y e_y
$$

## Usage

1. Clone the repository:

```bash
git clone https://github.com/thexuanphuc/Chaos-Control
```

2. Modify the desired trajectory in `trajectory_generator.py`.

3. Run the simulation:

```bash
python main.py
```

## License

This project is open-source and available under the MIT License.
