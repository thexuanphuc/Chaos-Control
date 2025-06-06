# Mass and Inertia Adaptive Control for Robust Path Following in Nonholonomic Mobile Robots

<div align="center">
    <img src="media/robot_adaptive_animation_Heart.gif" alt="Heart Trajectory" width="350">
    <img src="media/robot_adaptive_animation_Circle.gif" alt="Circle Trajectory" width="350">
    <img src="media/robot_adaptive_animation_Complex.gif" alt="Complex Trajectory" width="350">
</div>

<div align="center">
    <img src="media/robot_animation_Lemniscate_disturbed.gif" alt="Lemniscate Trajectory" width="350">
    <img src="media/robot_adaptive_animation_SineWave.gif" alt="SineWave Trajectory" width="350">
    <img src="media/robot_adaptive_animation_Line.gif" alt="Line Trajectory" width="350">
</div>

## Table of Contents

- [Introduction](#introduction)
- [Problem Statement](#problem-statement)
- [Kinematic Control Layer](#kinematic-control-layer)
  - [Reference Vehicle Dynamics](#reference-vehicle-dynamics)
  - [Error Definition in Local Coordinates](#error-definition-in-local-coordinates)
  - [Error Dynamics in Local Coordinates](#error-dynamics-in-local-coordinates)
- [Control Strategy](#control-strategy)
  - [Candidate Lyapunov Function](#candidate-lyapunov-function)
  - [Control Law](#control-law)
- [Dynamic Model of the Robot](#dynamic-model-of-the-robot)
- [Regressor Matrix and Parameter Estimation](#regressor-matrix-and-parameter-estimation)
- [Lyapunov-Based Adaptive Control Design](#lyapunov-based-adaptive-control-design)
  - [Tracking Error and Lyapunov Candidate](#tracking-error-and-lyapunov-candidate)
  - [Time Derivative of the Lyapunov Function](#time-derivative-of-the-lyapunov-function)
    - [First Term](#first-term-fracddt-left-frac12-etat-m_2-eta-right)
    - [Second Term](#second-term-fracddt-left-frac12-delta-pt-gamma_p-1-delta-p-right)
  - [Control Law Design](#control-law-design)
  - [Adaptive Law Design](#adaptive-law-design)
  - [Robust Term Design](#robust-term-design)
- [Final Control and Adaptive Laws](#final-control-and-adaptive-laws)
- [Implementation Details](#implementation-details)
  - [Path Generation](#path-generation)
  - [Controller Parameters](#simulation-and-controller-parameters)
- [Usage](#usage)
- [Output](#output)

## Introduction

This repository presents a robust adaptive control strategy for path following in a 3-wheeled nonholonomic mobile robot under unknown mass, inertia, and wind disturbances. The controller combines a kinematic path-following design with a Lyapunov-based adaptive dynamic law to estimate uncertain parameters and ensure stability. The approach uses:

-  A **simple kinematic controller** to generate desired linear and angular velocities.
-  A **composite adaptive dynamic controller** that estimates unknown physical parameters (mass and inertia) and simultaneously rejects external disturbances (force and torque).

All derivations are based on Lyapunov stability theory to guarantee bounded tracking errors and parameter estimates.

## Problem Statement

Traditional energy-based controllers that ignore unknown parameters such as mass and inertia often fail to stabilize nonholonomic mobile robots under external disturbances like wind. In such scenarios, adaptive control techniques are necessary to ensure reliable performance.

We consider the control of a nonholonomic mobile robot described by:

- State variables: `s ∈ ℝⁿ`
- Control actions: `a ∈ ℝᵐ`
- Unknown system parameters: `θ ∈ ℝᵖ`

The goal is to follow a desired trajectory accurately, despite uncertainties in the robot's dynamics and environmental disturbances. To achieve this, we design an adaptive control law of the form:





## Kinematic Control Layer

The kinematic model of the unicycle-type robot is given by:

$$
\begin{aligned}
\dot{x} &= v \cos \theta \\
\dot{y} &= v \sin \theta \\
\dot{\theta} &= \omega
\end{aligned}
$$

Where:
* `(x, y)` is the robot's position in the world frame. `\theta` is the robot's orientation (angle with the world X-axis).
*  s = [x,y,\theta]
* `v` is the forward linear velocity.
* `omega` is the angular velocity.
* a = [v,omega]

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


$$ \dot{V} := -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 $$
## 2.Dynamic Model of the Robot


We consider a second-order mechanical system (linear + angular velocity dynamics) given by:

$$
M₂(p)·v̇ = τ + d(t)
$$

Where:
- `v = [v₁, ω]ᵀ` is the velocity vector (linear and angular).
- `v̇` is the acceleration.
- `τ` is the generalized input vector (linear force + angular torque).
- `d(t)` is an unknown, bounded disturbance.
- `M₂(p) = diag(m, I)` is the mass-inertia matrix.

We assume the exact values of the parameters $m$, $I$, and the disturbance $d(t)$ are unknown.


## Regressor Matrix and Parameter Estimation

Define:
- `p = [m, I]ᵀ` as the unknown parameter vector.
- `p̂` as the adaptive estimate of `p`.
- `Δp = p̂ - p` as the estimation error.

Let  `vᵈ = [v₁ᵈ, ωᵈ]ᵀ` be the desired velocity vector from a kinematic controller.

The regressor matrix `Y_c` is defined such that:
$$M₂ · vᵈ = Y_c · p$$



## Lyapunov-Based Adaptive Control Design


### Tracking Error and Lyapunov Candidate

Define the velocity tracking error:


$$
η = v - vᵈ
$$


We choose a Lyapunov candidate function:

$$
V := \frac{1}{2} ηᵀM₂η + \frac{1}{2}ΔpᵀΓₚ⁻¹Δp
$$

where:
- `η` is the tracking error.
- `M₂` is the mass-inertia matrix `diag(m, I)`.
- `Δp = p̂ - p` is the parameter estimation error.
- `Δṗ = ṗ̂` is the time derivative of the parameter estimation error.
- `Γₚ` is a positive-definite adaptation gain matrix.


### Time Derivative of the Lyapunov Function


Taking the time derivative:


The time derivative of \(V\) is:

$$
\dot{V} = \frac{d}{dt} \left[ \frac{1}{2} \eta^T M_2 \eta \right] + \frac{d}{dt} \left[ \frac{1}{2} \Delta p^T \Gamma_p^{-1} \Delta p \right]
$$

### First Term: $\frac{d}{dt} \left( \frac{1}{2} \eta^T M_2 \eta \right)$

Using the product rule for matrices, where $M_2$ is constant (since mass $m$ and inertia $I$ are treated as fixed but unknown parameters):

$$
\frac{d}{dt} (\eta^T M_2 \eta) = \dot{\eta}^T M_2 \eta + \eta^T M_2 \dot{\eta}
$$

Since $M_2$ is symmetric, $\eta^T M_2 \dot{\eta} = (\dot{\eta}^T M_2 \eta)^T$, so:

$$
\frac{d}{dt} \left( \frac{1}{2} \eta^T M_2 \eta \right) = \eta^T M_2 \dot{\eta}
$$

(If $M_2$ were time-varying, an additional term $\frac{1}{2}\eta^T \dot M_2 \eta$ would appear, but we assume $M_2$ is constant, so this term is zero.)

### Second Term: $\frac{d}{dt} \left( \frac{1}{2} \Delta p^T \Gamma_p^{-1} \Delta p \right)$

Since $\Gamma_p$ is constant (a design parameter), and $\Delta p = \hat{p} - p$ with $p$ constant (true parameters are fixed):

$$
\frac{d}{dt} (\Delta p^T \Gamma_p^{-1} \Delta p) = \dot{\Delta p}^T \Gamma_p^{-1} \Delta p + \Delta p^T \Gamma_p^{-1} \dot{\Delta p}
$$

Since $\Gamma_p^{-1}$ is symmetric and $\Delta p^T \Gamma_p^{-1} \dot{\Delta p} = (\dot{\Delta p}^T \Gamma_p^{-1} \Delta p)^T$:

$$
\frac{d}{dt} \left( \frac{1}{2} \Delta p^T \Gamma_p^{-1} \Delta p \right) = \Delta p^T \Gamma_p^{-1} \dot{\Delta p}
$$

But $\dot{\Delta p} = \dot{\hat{p}}$ (since $\dot{p} = 0$), so:

$$
\frac{d}{dt} \left( \frac{1}{2} \Delta p^T \Gamma_p^{-1} \Delta p \right) = \Delta p^T \Gamma_p^{-1} \dot{\hat{p}}
$$

Alternatively, expressing it in terms of the derivative:

$$
\frac{d}{dt} \left[ \Delta p^T \Gamma_p^{-1} \Delta p \right] = \dot{\Delta p}^T \Gamma_p^{-1} \Delta p + \Delta p^T \Gamma_p^{-1} \dot{\Delta p}
$$

Since $\Gamma_p^{-1}$ is symmetric, and using the property:

<p align="center">
  <img
    src="https://latex.codecogs.com/svg.latex?\Delta%20p^T%20\Gamma_p^{-1}%20\dot{\Delta%20p}%20=%20(\dot{\Delta%20p}^T%20\Gamma_p^{-1}%20\Delta%20p)^T"
    alt="\Delta p^T \Gamma_p^{-1} \dot{\Delta p} = (\dot{\Delta p}^T \Gamma_p^{-1} \Delta p)^T" />
</p>

we get:

$$
\frac{d}{dt}\Bigl[\tfrac12\Delta p^T\Gamma_p^{-1}\Delta p\Bigr]
= \Delta p^T\Gamma_p^{-1}\dot{\Delta p}
$$

But since $\dot{\Delta p} = \dot{\hat p}$ (because $\dot p = 0$):


<p align="center">
  <img
    src="https://latex.codecogs.com/svg.latex?\frac{d}{dt}%20\left[%20\frac{1}{2}%20\Delta%20p^T%20\Gamma_p^{-1}%20\Delta%20p%20\right]%20=%20\Delta%20p^T%20\Gamma_p^{-1}%20\dot{\hat{p}}"
    alt="\frac{d}{dt} \left[ \frac{1}{2} \Delta p^T \Gamma_p^{-1} \Delta p \right] = \Delta p^T \Gamma_p^{-1} \dot{\hat{p}}" />
</p>

$$V̇ := ηᵀ M₂ η̇ + Δpᵀ Γₚ⁻¹ ṗ̂ $$
$$:= ηᵀ M₂ (v̇ - v̇ᵈ) + Δpᵀ Γₚ⁻¹ ṗ̂ $$
$$:= ηᵀ (τ + d(t)) - ηᵀ M₂ v̇ᵈ + Δpᵀ Γₚ⁻¹ ṗ̂ $$
$$:= ηᵀ τ + ηᵀ d(t) - ηᵀ Y_c p + Δpᵀ Γₚ⁻¹ ṗ̂ $$


Using `p = ṗ̂ - Δp`:

$$
-ηᵀ Y_c p = -ηᵀ Y_c ṗ̂ + ηᵀ Y_c Δp
$$

Substitute into `V̇`:

$$
V̇ = ηᵀ τ - ηᵀ Y_c p̂ + ηᵀ d(t) + ηᵀ Y_c Δp + ΔpᵀΓₚ⁻¹ ṗ̂
$$



### Control Law Design


Choose the control law:

<p align="center">
  <img
    src="https://latex.codecogs.com/svg.latex?%5Ctau%20%3D%20Y_c%20%5Ccdot%20%5Chat%7Bp%7D%20-%20K_d%20%5Ccdot%20%5Ceta%20-%20u_%7Brobust%7D"
    alt="\tau = Y_c \cdot \hat{p} - K_d \cdot \eta - u_{robust}" />
</p>

Then:

$$
ηᵀ τ = ηᵀ Y_c p̂ - ηᵀ K_d η - ηᵀ u_{robust}
$$


Substitute into the Lyapunov derivative:

<p align="center">
  <img
    src="https://latex.codecogs.com/svg.latex?%5Cdot%7BV%7D%20%3D%20-%5Ceta%5ET%20K_d%20%5Ceta%20-%20%5Ceta%5ET%20u_%7Brobust%7D%20%2B%20%5Ceta%5ET%20d%28t%29%20%2B%20%5Ceta%5ET%20Y_c%20%5CDelta%20p%20%2B%20%5CDelta%20p%5ET%20%5CGamma_p%5E%7B-1%7D%20%5Cdot%7B%5Chat%20p%7D"
    alt="V̇ = -ηᵀ K_d η - ηᵀ u_{robust} + ηᵀ d(t) + ηᵀ Y_c Δp + Δpᵀ Γₚ⁻¹ ṗ̂" />
</p>

### Adaptive Law Design


Choose the adaptive law:

$$
ṗ̂ = -Γₚ Y_cᵀ η
$$
Then:
$$Δpᵀ Γₚ⁻¹ ṗ̂ = -ηᵀ Y_c Δp$$

This cancels the `ηᵀ Y_c Δp` term in the Lyapunov derivative.


### Robust Term Design


Assume:

$$
‖d(t)‖ ≤ d_B
$$


Choose:

$$
u_{\mathrm{robust}} = d_{B} \cdot \tanh\Bigl(\frac{\eta}{\epsilon}\Bigr)
$$

Then the Lyapunov derivative becomes:

$$
V̇ ≤ -ηᵀ K_d η
$$


which ensures global uniform ultimate boundedness of the tracking error.


## Final Control and Adaptive Laws


**Control Law:**

$$
\tau = Y_c \hat{p}-K_d\eta\-d_B\cdot\tanh\bigl(\tfrac{\eta}{\epsilon}\bigr)
$$

**Adaptive Law:**

$$
\dot{\hat{p}} = -\Gamma_p Y_c^{T}\eta
$$


## Implementation Details

### Path Generation

The `generate_path` function in `main.py` can create various geometric paths (Circle, Ellipse, Spiral, Line, etc.). You select the desired path type using the `selected_path_type` variable in `main.py`.

### Simulation and Controller Parameters

The key simulation parameters are set near the top of the `main()` function in `main.py`:

* `k_forward`: Corresponds to $K_x$.
* `k_theta`: Corresponds to $K_\theta$.
* `k_lateral_gain_factor`: Used to determine $K_y$ ($K_y = K_{\theta} \times k_{lateral-gain-factor}$).
* `v_ref`: Corresponds to $v_{ref}$.
* `omega_max`: Maximum limit applied to the calculated chassis angular velocity $\omega$.
* `wheel_radius`, `wheel_width`: Robot physical parameters used for simulation and control calculations.
* `dt`: Simulation time step.
*  wheel_radius 
*  wheel_width
*  mass
*  inertia
*  kd1_factor for mass
*  kd2_factor for inertia
*  Γₚ which is adaptive gain matrix
*  bounded disturbance parameter
*  Kick/disturbance parameter
*  Kick/disturbance duration


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
