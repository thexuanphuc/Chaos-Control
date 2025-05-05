# Mass and Inertia Adaptive Control for Robust Path Following in Nonholonomic Mobile Robots

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


$$ \dot{V} := -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 $$
## 2.Dynamic Model of the Robot


We consider a second-order mechanical system (linear + angular velocity dynamics) given by:

$$
M₂(p) · v̇ = τ + d(t)
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
- `ṗ̂ = p̂ - p` as the estimation error.

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
V := \frac{1}{2} ηᵀ M₂ η + \frac{1}{2} Δpᵀ Γₚ⁻¹ Δp
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

(If $M_2$ were time-varying, an additional term $\frac{1}{2}\eta^T \dot M_2 \eta$ would appear, but the document assumes $M_2$ is constant, so this term is zero.)

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

$$
\Delta p^T\,\Gamma_p^{-1}\,\dot{\Delta p}
=
\bigl(\dot{\Delta p}^T\,\Gamma_p^{-1}\,\Delta p\bigr)^T
$$

we get:

$$
\frac{d}{dt}\!\Bigl[\tfrac12\,\Delta p^T\,\Gamma_p^{-1}\,\Delta p\Bigr]
= \Delta p^T\,\Gamma_p^{-1}\,\dot{\Delta p}
$$

But since $\dot{\Delta p} = \dot{\hat p}$ (because $\dot p = 0$):


$$

\frac{d}{dt} \left[ \frac{1}{2} \Delta p^T \Gamma_p^{-1} \Delta p \right] = \Delta p^T \Gamma_p^{-1} \dot{\hat{p}}
$$

$$V̇ := ηᵀ M₂ η̇ + Δpᵀ Γₚ⁻¹ ṗ̂ $$
$$= ηᵀ M₂ (v̇ - v̇ᵈ) + Δpᵀ Γₚ⁻¹ ṗ̂ $$
$$= ηᵀ (τ + d(t)) - ηᵀ M₂ v̇ᵈ + Δpᵀ Γₚ⁻¹ ṗ̂ $$
$$= ηᵀ τ + ηᵀ d(t) - ηᵀ Y_c p + Δpᵀ Γₚ⁻¹ ṗ̂ $$


Using `p = ṗ̂̂ + p̂`:

$$
-ηᵀ Y_c p = -ηᵀ Y_c ṗ̂ + ηᵀ Y_c Δp
$$

Substitute into `V̇`:

$$
V̇ = ηᵀ τ - ηᵀ Y_c p̂ + ηᵀ d(t) + ηᵀ Y_c Δp + Δpᵀ Γₚ⁻¹ ṗ̂
$$



### Control Law Design


Choose the control law:

$$

τ = Y_c · p̂ - K_d · η - u_{robust}

$$

Then:

$$
ηᵀ τ = ηᵀ Y_c p̂ - ηᵀ K_d η - ηᵀ u_{robust}
$$


Substitute into the Lyapunov derivative:

$$
V̇ = -ηᵀ K_d η - ηᵀ u_robust + ηᵀ d(t) + ηᵀ Y_c Δp + Δpᵀ Γₚ⁻¹ ṗ̂

$$

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
u_{\mathrm{robust}} = d_{B} \,\cdot\, \tanh\!\Bigl(\frac{\eta}{\epsilon}\Bigr)
$$

Then the Lyapunov derivative becomes:

$$
V̇ ≤ -ηᵀ K_d η
$$


which ensures global uniform ultimate boundedness of the tracking error.


## Final Control and Adaptive Laws


**Control Law:**

$$
\tau = Y_c \,\hat{p}\;-\;K_d\,\eta\;-\;d_B\;\cdot\;\tanh\!\bigl(\tfrac{\eta}{\epsilon}\bigr)
$$

**Adaptive Law:**

$$
\dot{\hat{p}} = -\,\Gamma_p\,Y_c^{T}\,\eta
$$


