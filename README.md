# Lyapunov-Based Controller for Nonholonomic Mobile Robot

This project implements a Lyapunov-based controller for trajectory tracking of a nonholonomic mobile robot. The simulation runs using Python scripts and displays results in a live Matplotlib window.

<div align="center">
    <img src="media/robot_animation_Heart.gif" alt="Heart Trajectory" width="350">
    <img src="media/robot_animation_Circle.gif" alt="Circle Trajectory" width="350">
    <img src="media/robot_animation_Complex.gif" alt="Complex Trajectory" width="350">
</div>

<div align="center">
    <img src="media/robot_animation_Lemniscate.gif" alt="Lemniscate Trajectory" width="350">
    <img src="media/robot_animation_SineWave.gif" alt="SineWave Trajectory" width="350">
    <img src="media/robot_animation_Line.gif" alt="Line Trajectory" width="350">
</div>


## Table of Contents
- [Dependencies](#dependencies)
- [Installation](#installation)
- [File Structure](#file-structure)
- [Robot Kinematic Model](#robot-kinematic-model)
- [Error Definition](#error-definition)
- [Control Strategy](#control-strategy)
  - [Candidate Lyapunov Function](#candidate-lyapunov-function)
  - [Control Law](#control-law)
- [Path Following Strategy](#path-following-strategy)
- [Implementation Details](#implementation-details)
  - [Path Generation](#path-generation)
  - [Controller Parameters](#controller-parameters)
- [Usage](#usage)
- [Output](#output)

## Dependencies

* Python 3.x
* NumPy
* Matplotlib

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

* `main.py`: The main script to run the simulation. Contains parameter settings, path generation call, simulation loop, and visualization setup.
* `Simulation.py`: Defines the `Simulation` class, handling robot state updates based on wheel commands and storing history.
* `Controller.py`: Defines the base `Controller` class and the `LyapunovEnergyBasedController` implementing the control logic.
* `Visualizer.py`: Defines the `Visualizer` class, responsible for creating and updating the live Matplotlib plots during the simulation.

## Robot Kinematic Model

The kinematic model of the unicycle-type robot is given by:

$$
\begin{aligned}
\dot{x} &= v \cos \theta \\
\dot{y} &= v \sin \theta \\
\dot{\theta} &= \omega
\end{aligned}
$$

Where:
* `(x, y)` is the robot's position in the world frame.
* `theta` is the robot's orientation (angle with the world X-axis).
* `v` is the forward linear velocity.
* `omega` is the angular velocity.

The `Simulation` class uses this model implicitly when converting wheel velocities (commands from the controller) into chassis motion (`v`, `omega`) and updating the state (`x`, `y`, `theta`).

### Reference Vehicle Dynamics

The tracking control problem involves following a reference vehicle with dynamics:

$$
\dot{x}_r = v_r \cos \theta_r
$$

$$
\dot{y}_r = v_r \sin \theta_r
$$

$$
\dot{\theta}_r = \omega_r
$$

where $v_r(t)$ and $\omega_r(t)$ are the velocity references.
The tracking error is defined relative to the **lookahead point** (`x_d`, `y_d`) and the **reference orientation** (`theta_d`). The errors are expressed in the robot's body frame:

### Error Definition in Local Coordinates


$$
\begin{aligned}
e_x &= \cos \theta (x_d - x) + \sin \theta (y_d - y) \quad &\text{(Forward error)} \\
e_y &= -\sin \theta (x_d - x) + \cos \theta (y_d - y) \quad &\text{(Lateral error)} \\
e_\theta &= \theta_d - \theta \quad &\text{(Orientation error)}
\end{aligned}
$$

Where `(x, y, theta)` is the robot's current state. $e_\theta$ is normalized to $[-\pi, \pi]$. These correspond to `error_forward`, `error_lateral`, and `error_theta` calculated in `Controller.py`.

### Error Dynamics in Local Coordinates

The error dynamics between the reference vehicle and the follower robot in local coordinates are given by:

$$
\dot{e_x} = \omega e_y - v + v_r(t) \cos(e_\theta)
$$

$$
\dot{e_y} = -\omega e_x + v_r(t) \sin(e_\theta)
$$

$$
\dot{e}_\theta = \omega_r(t) - \omega
$$

## Control Strategy

The controller aims to drive the tracking errors towards zero using a control law derived from Lyapunov stability principles.

### Candidate Lyapunov Function

A common candidate Lyapunov function for this system is:

$$ V := \frac{1}{2} (e_x^2 + e_y^2 +  \frac{1}{K_y}e_\theta^2) $$

$$
\dot{V} := e_x \dot{e_x} + e_y \dot{e_y} + \frac{1}{K_y}e_\theta \dot{e_\theta}
$$

Substitute error dynamics:

$$
\dot{V} := -e_x v + e_x v_r \cos e_\theta + e_y v_r \sin e_\theta + \frac{1}{K_y}(\omega_r e_\theta - \omega e_\theta)
$$


The goal is to design control inputs `v` and `omega` such that `dV/dt <= 0`.

*(Note: The code calculates `V = 0.5 * (e_x**2 + e_y**2 + e_theta**2)`).*

### Control Law

The `LyapunovEnergyBasedController` implements the following control law to calculate the desired chassis velocities (`v`, `omega`):

$$
v = v_r \cos e_\theta + K_x e_x 
$$

$$
\omega = \omega_r + K_\theta e_\theta + v_r  e_y K_y \frac{sin{e_\theta}}{e_\theta}
$$

Where:
- K_x > 0, K_theta > 0, K_y > 0 are positive controller gains.
- Based on these control actions $\dot{L} < 0$


$$ \dot{V} := -K_x e_x^2 - K_\theta e_\theta^2 $$

The controller then converts these target chassis velocities (`v`, `omega`) into left and right wheel angular velocity commands (`omega_left_cmd`, `omega_right_cmd`) based on the robot's wheel radius and width, which are sent to the `Simulation`.

## Path Following Strategy

This controller follows a predefined geometric path represented as a sequence of points. The strategy involves:

1.  **Finding the Closest Point:** Identifying the point on the desired path closest to the robot's current position.
2.  **Lookahead Point:** Selecting a target point (`x_d`, `y_d`) on the path slightly ahead of the closest point.
3.  **Reference Orientation (`theta_d`):** Determining the desired orientation by calculating the angle of the path segment *following* the target point.
4.  **Reference Velocities:**
    * A constant reference forward speed `v_ref` is used (parameter `v_ref` in `main.py`).
    * A reference angular velocity `omega_ref` is estimated based on the curvature of the path near the target point.



## Implementation Details

### Path Generation

The `generate_path` function in `main.py` can create various geometric paths (Circle, Ellipse, Spiral, Line, etc.). You select the desired path type using the `selected_path_type` variable in `main.py`.

### Controller Parameters

The key controller parameters are set near the top of the `main()` function in `main.py`:

* `k_forward`: Corresponds to $K_x$.
* `k_theta`: Corresponds to $K_\theta$.
* `k_lateral_gain_factor`: Used to determine $K_y$ ($K_y = K_{\theta} \times k_{lateral-gain-factor}$).
* `v_ref`: Corresponds to $v_{ref}$.
* `omega_max`: Maximum limit applied to the calculated chassis angular velocity $\omega$.
* `wheel_radius`, `wheel_width`: Robot physical parameters used for simulation and control calculations.
* `dt`: Simulation time step.

## Usage

1.  Ensure you have installed the dependencies (`numpy`, `matplotlib`).
2.  Clone the repository and navigate into the project directory.
3.  Modify simulation parameters, path type (`selected_path_type`), and controller gains directly within the `main.py` file (inside the `src` directory).
4.  Run the simulation from the terminal (ensure your terminal's working directory is the one *containing* the `src` folder):
    ```bash
    python src/main.py
    ```
    (Or navigate *into* the `src` directory and run `python main.py`).

## Output

Running `main.py` will:

1.  Open a Matplotlib window.
2.  Display the simulation live, showing:
    * The desired path (dashed red line).
    * The robot's actual path (solid green line).
    * The robot's current position and orientation (blue circle and arrow).
    * Plots of wheel velocity commands over time.
    * Plots of the robot's actual forward and angular velocities over time.
    * Plots of the tracking errors ($e_x$, $e_y$, $e_\theta$) over time.
    * A plot of the calculated Lyapunov energy function $V$ over time.
3.  Print simulation status messages to the console.
4.  The simulation runs until `max_steps` is reached, the target is achieved, or an error occurs.
5.  **Close the Matplotlib plot window to terminate the program** after the simulation finishes.
