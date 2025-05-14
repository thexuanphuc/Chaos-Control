# Backstepping-Based Adaptive Control for Robust Path Following in Nonholonomic Mobile Robots

<div align="center">
    <img src="media/robot_backstepping_animation_Heart.gif" alt="Heart Trajectory" width="320">
    <img src="media/robot_backstepping_animation_Circle.gif" alt="Circle Trajectory" width="320">
    <img src="media/robot_backstepping_animation_Complex.gif" alt="Complex Trajectory" width="320">
</div>

<div align="center">
    <img src="media/robot_backstepping_animation_Lemniscate.gif" alt="Lemniscate Trajectory" width="320">
    <img src="media/robot_backstepping_animation_SineWave.gif" alt="Sine Wave Trajectory" width="320">
    <img src="media/robot_backstepping_animation_Line.gif" alt="Line Trajectory" width="320">
</div>

## Table of Contents

- [Introduction](#introduction)
- [Problem Statement](#problem-statement)
- [Kinematic Control Layer](#kinematic-control-layer)
  - [Reference Vehicle Dynamics](#reference-vehicle-dynamics)
  - [Error Definition in Local Coordinates](#error-definition-in-local-coordinates)
- [Backstepping Adaptive Control Strategy](#backstepping-adaptive-control-strategy)
  - [Candidate Lyapunov Function](#candidate-lyapunov-function)
  - [Backstepping Control Law](#backstepping-control-law)
  - [Adaptive Parameter Update Law](#adaptive-parameter-update-law)
  - [Robustness Term](#robustness-term)
- [Dynamic Model of the Robot](#dynamic-model-of-the-robot)
- [Regressor Matrix and Parameter Estimation](#regressor-matrix-and-parameter-estimation)
- [Implementation Details](#implementation-details)
  - [Path Generation](#path-generation)
  - [Simulation and Controller Parameters](#simulation-and-controller-parameters)
- [Usage](#usage)
- [Output](#output)
- [License](#license)

## Introduction

This repository presents a backstepping-based adaptive control framework for trajectory tracking of a nonholonomic differential-drive mobile robot. The two-layered control design combines:

1. A **Lyapunov-guided kinematic controller** to compute reference linear and angular velocities.
2. A **backstepping adaptive dynamic controller** that estimates unknown parameters (mass and inertia), compensates for bounded disturbances, and ensures stability via Lyapunov arguments.

The codebase includes modules for simulation, control, animation, and static plotting, enabling end-to-end evaluation of performance under various path geometries and disturbance scenarios.

## Problem Statement

A differential-drive robot must follow a desired planar trajectory in the presence of:

- Unknown mass ($m$) and moment of inertia ($I$).
- External disturbances (continuous torque noise and trajectory-based "kick").

Classic kinematic controllers ignore second-order dynamics and cannot guarantee performance under disturbances. Adaptive dynamic control with backstepping addresses parameter uncertainty and disturbance rejection, while retaining kinematic guidance.

## Kinematic Control Layer

The unicycle kinematic model:

$$
\dot{x} = v \cos \theta,
\quad
\dot{y} = v \sin \theta,
\quad
\dot{\theta} = \omega.
$$

### Reference Vehicle Dynamics

The controller treats a virtual reference vehicle with velocities $(v_r, \omega_r)$ along the desired path.

### Error Definition in Local Coordinates

Define error in robot frame:

$$
\begin{aligned}
 e_x &= \cos\theta (x_d - x) + \sin\theta (y_d - y), \\
 e_y &= -\sin\theta (x_d - x) + \cos\theta (y_d - y), \\
 e_\theta &= \theta_d - \theta,
\end{aligned}
$$

where $(x_d, y_d, \theta_d)$ is the lookahead point on the desired path, and $e_\theta$ is normalized to $[-\pi,\pi]$.

## Backstepping Adaptive Control Strategy

### Candidate Lyapunov Function

A suitable Lyapunov function for backstepping design:

$$
V_1 = \frac12 (e_x^2 + e_y^2) + \frac12 e_\theta^2.
$$

For the dynamic layer, velocity tracking error $\eta = [v - v_d, \omega - \omega_d]^T$ uses:

$$
V_2 = \frac12 \eta^T M_2 \eta + \frac12 \tilde{p}^T \Gamma_p^{-1} \tilde{p},
$$

with $M_2 = \mathrm{diag}(m, I)$ and parameter estimation error $\tilde{p} = \hat{p} - p$.

### Backstepping Control Law

Define desired velocities $(v_d, \omega_d)$ via the kinematic controller. The backstepping dynamic control law:

$$
\tau_{bar} = Y_c \hat{p} - K_d\,\eta - u_{b},
$$
$$
[\tau_R, \tau_L]^T = B_2^{-1} \tau_{bar},
$$

where $Y_c$ is the regressor matrix, $K_d$ is a positive-definite feedback gain, and $u_{b}$ is a backstepping term derived from $V_1$ to drive kinematic errors.  

### Adaptive Parameter Update Law

Parameter estimates $\hat{p} = [\hat{m}, \hat{I}]^T$ are updated:

$$
\dot{\hat{p}} = -\Gamma_p Y_c^T \eta,
$$

with adaptation gain $\Gamma_p = \mathrm{diag}(\gamma_m, \gamma_I)$. Projections enforce $\hat{p}_{\min} \le \hat{p} \le \hat{p}_{\max}$.

### Robustness Term

To reject bounded disturbances $d_B$, an optional robust term:

$$
u = d_B \tanh\bigl(\eta/\epsilon\bigr),$$

added to the control law to maintain Lyapunov decrease in presence of disturbances.

## Dynamic Model of the Robot

Second-order model:

$$
M_2(p) \dot{v} = B_2 \tau_{cmd} - \tau_d,
$$

with $v = [v, \omega]^T$, $M_2 = \mathrm{diag}(m, I)$, and $B_2$ mapping wheel torques to body forces/torques. Pose integrates via Euler:

$$
x_{k+1} = x_k + v_k \cos\theta_k \Delta t,
\quad
y_{k+1} = y_k + v_k \sin\theta_k \Delta t,
\quad
\theta_{k+1} = \theta_k + \omega_k \Delta t.
$$

Disturbances include continuous random dither and optional trajectory-based "kick".

## Regressor Matrix and Parameter Estimation

The regressor for dynamic control:

$$Y_c = \begin{bmatrix} \dot{v}_d & 0 \\ 0 & \dot{\omega}_d \end{bmatrix},$$

where derivatives of desired velocities are approximated by finite differences.

## Implementation Details

### Path Generation (`generate_path`)
- Support various geometries: Circle, Ellipse, Spiral, Line, Lemniscate, Sine Wave, Heart, Square Wave, Parabola, Complex.
- Returns NumPy array of $(x, y)$ points.

### Simulation and Controller Parameters

| Variable          | Description                                       | Example Value   |
|-------------------|---------------------------------------------------|-----------------|
| dt                | Simulation time step                              | 0.02 s          |
| max_steps         | Max number of simulation iterations               | 3000            |
| wheel_radius      | Wheel radius                                      | 0.005 m         |
| wheel_width       | Distance between wheels                           | 0.03 m          |
| robot_mass        | True robot mass                                   | 1.5 kg          |
| robot_inertia     | True moment of inertia                            | 1.2 kg·m²       |
| kin_k_forward     | Kinematic forward gain                            | 1.8             |
| kin_k_theta       | Kinematic orientation gain                        | 4.5             |
| kin_v_ref         | Reference linear velocity                         | 0.8 m/s         |
| kin_lookahead     | Lookahead distance                                | 0.4 m           |
| gamma_p           | Adaptation gains (mass, inertia)                  | [0.1, 0.1]      |
| Kd_factors        | Dynamic feedback gains factors                    | [5.0, 8.0]      |
| disturbance_level | Max continuous disturbance bound                  | 0.0 Nm          |
| use_robust        | Enable robust term                                | true            |

## Usage

1. Clone the repository and navigate to Project 3:
   ```bash
   cd "Project 3"
   ```
2. (Optional) Create a virtual environment and activate:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
4. Run the backstepping simulation:
   ```bash
   python3 src/mainbackstepping.py
   ```
5. View animation and final plots; GIFs are saved under `media/` if enabled.

## Output

- **Animated Trajectories**: GIFs under `media/robot_backstepping_animation_*.gif`.
- **Static Plots**: Multi-panel figures showing kinematic errors, velocity errors, parameter estimates, torques, and disturbances.
- **Console Logs**: Simulation progress, timing, and final summary.

## License

This project is licensed under the MIT License. See [LICENSE](../LICENSE) for details.

