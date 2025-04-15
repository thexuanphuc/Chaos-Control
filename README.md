# Lyapunov-Based Controller for Nonholonomic Mobile Robot

This project implements a Lyapunov-based controller for trajectory tracking of a nonholonomic mobile robot. The simulation runs using Python scripts and displays results in a live Matplotlib window.

<div align="center">
    <img src="media/robot_animation_Heart.gif" alt="Heart Trajectory" width="400">
    <img src="media/robot_animation_Circle.gif" alt="Circle Trajectory" width="400">
    <img src="media/robot_animation_Complex.gif" alt="Complex Trajectory" width="400">
</div>

<div align="center">
    <img src="media/robot_animation_Lemniscate.gif" alt="Lemniscate Trajectory" width="400">
    <img src="media/robot_animation_SineWave.gif" alt="SineWave Trajectory" width="400">
    <img src="media/robot_animation_Line.gif" alt="Line Trajectory" width="400">
</div>

## Table of Contents
- [Dependencies](#dependencies)
- [Installation](#installation)
- [File Structure](#file-structure)
- [Robot Kinematic Model](#robot-kinematic-model)
- [Path Following Strategy](#path-following-strategy)
- [Error Definition](#error-definition)
- [Control Strategy](#control-strategy)
  - [Candidate Lyapunov Function](#candidate-lyapunov-function)
  - [Control Law](#control-law)
- [Implementation Details](#implementation-details)
  - [Path Generation](#path-generation)
  - [Controller Parameters](#controller-parameters)
- [Usage](#usage)
- [Output](#output)

## Dependencies

- Python 3.x
- NumPy
- Matplotlib

## Installation

1.  Clone the repository:
    ```bash
    git clone https://github.com/thexuanphuc/Chaos-Control
    cd Chaos-Control
    ```

2.  Install the required Python libraries:
    ```bash
    pip install numpy matplotlib
    ```

## File Structure

The core logic is organized into the following files within the `src` directory:

- **main.py**: The main script to run the simulation. It contains parameter settings, path generation routines, the simulation loop, and visualization setup.
- **Simulation.py**: Defines the `Simulation` class, which handles robot state updates based on wheel commands and stores simulation history.
- **Controller.py**: Defines the base `Controller` class and the `LyapunovEnergyBasedController` that implements the control logic.
- **Visualizer.py**: Defines the `Visualizer` class, which creates and updates the live Matplotlib plots during the simulation.

## Robot Kinematic Model

The robot is modeled as a unicycle-type vehicle. Its motion is described by the following equations:
  
$$
\begin{aligned}
\dot{x} &= v \cos \theta, \\
\dot{y} &= v \sin \theta, \\
\dot{\theta} &= \omega,
\end{aligned}
$$

where  
- $(x, y)$ is the robot's position in the world frame,  
- $\theta$ is the robot's orientation (angle with the world X-axis),  
- $v$ is the forward linear velocity, and  
- $\omega$ is the angular velocity.  

The `Simulation` class uses these equations to convert wheel velocities (commands from the controller) into chassis motion and update the robot state.

## Path Following Strategy

The controller follows a predefined geometric path represented as a sequence of points. Its steps are:

1. **Closest Point:** Identify the point on the desired path closest to the robot's current position.
2. **Lookahead Point:** Select a target point $(x_d, y_d)$ on the path slightly ahead of the closest point.
3. **Reference Orientation:** Compute the desired orientation $\theta_d$ from the path segment following the target point.
4. **Reference Velocities:**  
   - Use a constant reference forward speed $v_{ref}$ (set in `main.py`).  
   - Estimate a reference angular velocity $\omega_{ref}$ based on the curvature of the path near the target.

## Error Definition

The tracking error is defined relative to the **lookahead point** $(x_d, y_d)$ and the **reference orientation** $\theta_d$. Expressed in the robot's body frame, the errors are:

$$
\begin{aligned}
e_x &= \cos \theta\, (x_d - x) + \sin \theta\, (y_d - y) \quad &\text{(Forward error)}, \\
e_y &= -\sin \theta\, (x_d - x) + \cos \theta\, (y_d - y) \quad &\text{(Lateral error)}, \\
e_\theta &= \theta_d - \theta \quad &\text{(Orientation error)}.
\end{aligned}
$$

The orientation error $e_\theta$ is normalized to the range $[-\pi, \pi]$. These errors correspond to `error_forward`, `error_lateral`, and `error_theta` used in `Controller.py`.

## Control Strategy

The controller is designed to drive the tracking errors to zero using Lyapunov stability principles.

### Candidate Lyapunov Function

A common candidate Lyapunov function is defined as:

$$
V := \frac{1}{2} \left(e_x^2 + e_y^2 + e_\theta^2\right).
$$

The goal is to design control inputs $v$ and $\omega$ such that $\frac{dV}{dt} \leq 0$.

*(Note: The code calculates `V = 0.5 * (e_x**2 + e_y**2 + e_theta**2)`.)*

### Control Law

The `LyapunovEnergyBasedController` computes the desired chassis velocities as follows:

$$
\begin{aligned}
v &= v_{ref} \cos(e_\theta) + K_x\, e_x, \\
\omega &= \omega_{ref} + K_\theta\, e_\theta + K_y\, v_{ref}\, \mathrm{sinc}(e_\theta)\, e_y,
\end{aligned}
$$

where  
- $v_{ref}$ is the reference forward speed,  
- $\omega_{ref}$ is the reference angular velocity derived from path curvature,  
- $K_x > 0$, $K_\theta > 0$, and $K_y > 0$ are positive controller gains, and  
- $\mathrm{sinc}(e_\theta) = \dfrac{\sin(e_\theta)}{e_\theta}$ with $\mathrm{sinc}(0) = 1$.  

These target chassis velocities are then converted to left and right wheel angular velocity commands based on the robot's physical parameters.

## Implementation Details

### Path Generation

The function `generate_path` in `main.py` generates various geometric paths (Circle, Ellipse, Spiral, Line, etc.). You select the desired path type via the `selected_path_type` variable.

### Controller Parameters

The main controller parameters (set near the top of `main.py`) include:

- `k_forward`: Corresponds to $K_x$.
- `k_theta`: Corresponds to $K_\theta$.
- `k_lateral_gain_factor`: Determines $K_y$ via $K_y = K_\theta \times \texttt{k\_lateral\_gain\_factor}$.
- `v_ref`: Corresponds to $v_{ref}$.
- `omega_max`: Maximum limit applied to the computed chassis angular velocity $\omega$.
- `wheel_radius`, `wheel_width`: Physical parameters used for simulation and control calculations.
- `dt`: Simulation time step.

## Usage

1. Ensure that the required dependencies (`numpy`, `matplotlib`) are installed.
2. Clone the repository and navigate to the project directory.
3. Modify simulation parameters, path type (using `selected_path_type`), and controller gains within `main.py` as desired.
4. Run the simulation from the terminal (ensure the working directory contains the `src` folder):
    ```bash
    python src/main.py
    ```
    You can also navigate into the `src` folder and run:
    ```bash
    python main.py
    ```

## Output

Running `main.py` will:

1. Open a Matplotlib window showing the live simulation.
2. Display the desired path (e.g., dashed red line) and the actual robot path (e.g., solid green line) with the current position and orientation indicated.
3. Plot the wheel velocity commands, forward and angular velocities over time.
4. Plot the tracking errors ($e_x$, $e_y$, $e_\theta$) and the Lyapunov energy function $V$ over time.
5. Print simulation status messages to the console.
6. Terminate when the simulation completes (based on `max_steps`, reaching the target, or an error). Close the Matplotlib window to exit the program.