# Backstepping-Based Adaptive Controller Derivation for a Nonholonomic Mobile Robot  

*May 2025*


<div align="center">
    <img src="media/Heart_path_zeroOffest.gif" alt="Heart Trajectory" width="320">
    <img src="media/Circle_path.gif" alt="Circle Trajectory" width="320">
    <img src="media/ComplexPath_path.gif" alt="Complex Trajectory" width="320">
</div>

<div align="center">
    <img src="media/Lemniscate_path_zeroOffest.gif" alt="Lemniscate Trajectory" width="320">
    <img src="media/SineWave_path_zeroOffest.gif" alt="Sine Wave Trajectory" width="320">
    <img src="media/Line_path.gif" alt="Line Trajectory" width="320">
</div>



## 📑 Table of Contents
- [Introduction](#-introduction)
- [Problem Statement](#-problem-statement)
  - [System Description](#system-description)
- [System Model](#2-system-model)
  - [Kinematic Model](#21-kinematic-model)
  - [Error Definition in Local Coordinates](#error-definition-in-local-coordinates)
  - [Error Dynamics in Local Coordinates](#error-dynamics-in-local-coordinates)
  - [Dynamic Model](#22-dynamic-model)
- [Kinematic Controller Design (First Backstepping Step)](#3-kinematic-controller-design-first-backstepping-step)
  - [Lyapunov Function](#31-lyapunov-function)
  - [Control Law Design](#control-law-design)
- [Dynamic Controller Design (Second Backstepping Step)](#4-dynamic-controller-design-second-backstepping-step)
  - [Velocity Tracking Error](#41-velocity-tracking-error)
  - [Composite Lyapunov Function](#42-composite-lyapunov-function)
  - [Recompute $\dot{V}_1$ with Velocity Errors](#43-recompute-dotv_1-with-velocity-errors)
  - [Compute $\dot{V}$](#44-compute-dotv)
- [Final Controller Design (Third Backstepping Step)](#4-final-controller-design-third-backstepping-step)
  - [Step 1: Lyapunov Function and Components](#step-1-define-the-lyapunov-function-and-its-components)
  - [Step 2: Time Derivative $\dot{V}_3$](#step-2-compute-the-time-derivative--dotv_3-)
  - [Step 3: $\dot{V}$ with Actuator Dynamics](#step-3-compute--dotv--with-actuator-dynamics)
  - [Step 4: Compute $\dot{V}_3$ Explicitly](#step-4-compute--dotv_3--explicitly)
  - [Step 5: Design $a$ to Make $\dot{V}_3 < 0$](#step-5-design--a--to-make--dotv_3--0)
  - [Final Expressions](#final-expressions)

## 🧭 Introduction

This repository presents an adaptive backstepping control strategy for path following in a 3-wheeled nonholonomic mobile robot operating under unknown mass, inertia, and external disturbances (e.g., wind). The proposed controller combines:

- A kinematic-level backstepping controller that computes desired linear and angular velocities based on position and orientation errors.
- A dynamic-level adaptive backstepping controller that ensures the desired velocities are tracked while estimating unknown parameters and rejecting disturbances.

All derivations are based on Lyapunov stability theory to guarantee bounded tracking errors and parameter estimates.
---

## ❓ Problem Statement

Traditional controllers that ignore physical uncertainties (e.g., mass and inertia) or environmental disturbances (e.g., wind) often fail to stabilize nonholonomic robots in realistic scenarios. To overcome this, we design a three-level adaptive backstepping controller.

### System Description

We consider a nonholonomic mobile robot described by:

- State variables:  
  s ∈ ℝⁿ  
- Control action (torques/forces):  
  a ∈ ℝᵐ  
- Unknown physical parameters:  
  θ ∈ ℝᵖ  

The goal is to follow a desired trajectory accurately, despite uncertainties in the robot's dynamics and environmental disturbances.

## 2. System Model  
### 2.1 Kinematic Model  
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
* `\theta` is the robot's orientation (angle with the world X-axis).
* `v` is the forward linear velocity.
* `omega` is the angular velocity.

The `Simulation` class uses this model implicitly when converting wheel velocities (commands from the controller) into chassis motion (`v`, `omega`) and updating the state (`x`, `y`, `theta`).
  
Tracking errors in the robot's body frame:  

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

### 2.2 Dynamic Model  
## Dynamic Model

The dynamic model governs velocity dynamics:

$$
M_2 \dot{v} = \tau + d(t)
$$

where:  
- $v = [v, \omega]^T$: Velocity vector  
- $ Control action: a = \tau = [\tau_v, \tau_\omega]^T$  
- $d(t)$: Bounded disturbance ($|d(t)| \leq d_B$)  
- $M_2 = \text{diag}(m, I)$: Unknown mass-inertia matrix
- Define:
- `p = [m, I]ᵀ` is the unknown parameter vector.
- `p̂` is the adaptive estimate of `p`.
- `Δp = p̂ - p` is the estimation error.

Let  `vᵈ = [v₁ᵈ, ωᵈ]ᵀ` be the desired velocity vector from a kinematic controller.
The regressor matrix is:

$$
Y_c = 
\begin{bmatrix}
\dot{v}^d & 0 \\
0 & \dot{\omega}^d
\end{bmatrix}
$$

$$
\quad M_2 \dot{v}^d = Y_c p
$$

## 3. Kinematic Controller Design (First Backstepping Step)  
### 3.1 Lyapunov Function  
## Lyapunov Function for Kinematic Errors
For developing the kinematic controller, we are assuming $v = v_d$ and $\omega = \omega_d$.
Choose Lyapunov function for kinematic errors:

$$
V_1 := \frac{1}{2} \left( e_x^2 + e_y^2 + \frac{1}{K_y} e_\theta^2 \right)
$$

Taking time derivative and substitute error dynamics:

  $$\dot{V}_1 := e_x (\omega e_y - v + v_r \cos e_\theta) + e_y (-\omega e_x + v_r \sin e_\theta) + \frac{1}{K_y} e_\theta (\omega_r - \omega)$$

### Control Law Design

Virtual control inputs:

$$
\begin{aligned}
v^d &= v_r \cos e_\theta + K_x e_x \\
\omega^d &= \omega_r + K_\theta e_\theta + v_r e_y K_y \frac{\sin e_\theta}{e_\theta}
\end{aligned}
$$

Where:
- K_x > 0, K_theta > 0, K_y > 0 are positive controller gains.
- Based on these control actions $\dot{V}_1 < 0$


![Lyapunov equation](https://latex.codecogs.com/svg.image?\dot{V}_1%20:=%20-K_x%20e_x^2%20-%20\frac{K_\theta}{K_y}%20e_\theta^2)



## 4. Dynamic Controller Design (Second Backstepping Step)  
### 4.1 Velocity Tracking Error  
<!-- Define velocity error:  
$$
\eta = v - v^d = \begin{bmatrix} \eta_1 \\ \eta_2 \end{bmatrix}
$$   -->

+ Error Dynamics Equations

In error dynamics, the velocity $v$ and angular velocity $\omega$ are expressed as:

$$
v = \eta_1 + v_d
$$
$$
\omega = \eta_2 + \omega_d
$$

The updated error dynamics are given by the following differential equations:

$$
\dot{e}_x = (\eta_2 + \omega_d)e_y - (\eta_1 + v_d) + v_r \cos(e_\theta)
$$
$$
\dot{e}_y = -(\eta_2 + \omega_d)e_x + v_r \sin(e_\theta)
$$
$$
\dot{e}_\theta = (\omega_r - \omega_d) - \eta_2
$$

These equations describe the evolution of the error states $e_x$, $e_y$, and $e_\theta$ in the system.

Dynamic model becomes:  
$$
M_2 \dot{\eta} = \tau + d(t) - Y_c \hat{p} + Y_c \Delta p
$$

### 4.2 Composite Lyapunov Function  
Composite Lyapunov function:  
$$
V_2 = V_1 + \frac{1}{2} \eta^T M_2 \eta + \frac{1}{2} \Delta p^T \Gamma_p^{-1} \Delta p
$$

### 4.3 Recompute $\dot{V}_1$ with Velocity Errors  
Updated error dynamics:  
$$
\dot{V}_1 = -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 - e_x \eta_1 - \frac{1}{K_y} e_\theta \eta_2
$$


### 4.4 Compute $\dot{V}$  
Adaptive law and control input:  
$$
\begin{aligned}
\dot{\hat{p}} &= -\Gamma_p Y_c^T \eta \\
\tau &= Y_c \hat{p} - K_d \eta - d_B \tanh\left(\frac{\eta}{\epsilon}\right) + \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix}
\end{aligned}
$$  
Resulting in:  
$$
\dot{V}_2 \leq -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 - \eta^T K_d \eta \leq 0
$$


## 4. Final Controller Design (Third Backstepping Step)  



### Step 1: Define the Lyapunov Function and Its Components
The extended Lyapunov function with actuator dynamics is given by:

$ V_3 = V_2 + \frac{1}{2} \bar{e}_\tau^T \bar{e}_\tau $

where:
- $ V_2 = V_1 + \frac{1}{2} \eta^T M_2 \eta + \frac{1}{2} \Delta p^T \Gamma_p^{-1} \Delta p $ is the composite Lyapunov function from the dynamic controller design,
- $ \bar{e}_\tau = \tau - \tau_{\text{real}} $ is the error between the desired control input $ \tau $ and the actual control input $ \tau_{\text{real}} $,
- $ V_1 = \frac{1}{2} \left( e_x^2 + e_y^2 + \frac{1}{K_y} e_\theta^2 \right) $ is the kinematic Lyapunov function,
- $ \eta = v - v^d $ is the velocity tracking error,
- $ \Delta p = p - \hat{p} $ is the parameter estimation error,
- $ M_2 = \text{diag}(m, I) $ is the unknown mass-inertia matrix,
- $ \Gamma_p $ is a positive definite adaptation gain matrix.

The actuator dynamics are:

$ \dot{\tau}_{\text{real}} = \frac{1}{\gamma} (-\tau_{\text{real}} + a) $

where $ a $ is the control input we can design, and $ \gamma > 0 $ is a time constant.

### Step 2: Compute the Time Derivative $ \dot{V}_3 $
The time derivative of $ V_3 $ is:

$ \dot{V}_3 = \dot{V}_2 + \bar{e}_\tau^T \dot{\bar{e}}_\tau $

Since $ \bar{e}_\tau = \tau - \tau_{\text{real}} $, its derivative is:

$ \dot{\bar{e}}_\tau = \dot{\tau} - \dot{\tau}_{\text{real}} $

Substitute the actuator dynamics:

$ \dot{\bar{e}}_\tau = \dot{\tau} - \frac{1}{\gamma} (-\tau_{\text{real}} + a) $

Thus:

$ \dot{V}_3 = \dot{V}_2 + \bar{e}_\tau^T \left( \dot{\tau} - \frac{1}{\gamma} (-\tau_{\text{real}} + a) \right) $

We need to compute $ \dot{V}_2 $ with $ \tau_{\text{real}} $ as the actual input (since the dynamic model is now $ M_2 \dot{v} = \tau_{\text{real}} + d(t) $), and express $ \dot{\tau} $ explicitly.

---

### Step 3: Compute $ \dot{V}_2 $ with Actuator Dynamics
The composite Lyapunov function is:

$ V_2 = V_1 + \frac{1}{2} \eta^T M_2 \eta + \frac{1}{2} \Delta p^T \Gamma_p^{-1} \Delta p $

Its derivative is:

$ \dot{V}_2 = \dot{V}_1 + \eta^T M_2 \dot{\eta} + \Delta p^T \Gamma_p^{-1} \dot{\Delta p} $

#### 3.1: Compute $ \dot{V}_1 $
From the kinematic controller design:

$ \dot{V}_1 = -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 - e_x \eta_1 - \frac{1}{K_y} e_\theta \eta_2 $

This accounts for the velocity error $ \eta = v - v^d $, where $ v = [v, \omega]^T $ is the actual velocity, and $ v^d = [v^d, \omega^d]^T $ is the desired velocity.

#### 3.2: Compute $ M_2 \dot{\eta} $
The velocity error is $ \eta = v - v^d $, so:

$ \dot{\eta} = \dot{v} - \dot{v}^d $

The dynamic model is:

$ M_2 \dot{v} = \tau_{\text{real}} + d(t) $

$ \dot{v} = M_2^{-1} (\tau_{\text{real}} + d(t)) $

Also, $ M_2 \dot{v}^d = Y_c p $, so:

$ \dot{v}^d = M_2^{-1} Y_c p $

Thus:

$ M_2 \dot{\eta} = M_2 (\dot{v} - \dot{v}^d) = \tau_{\text{real}} + d(t) - Y_c p $

Since $ p = \hat{p} + \Delta p $:

$ M_2 \dot{\eta} = \tau_{\text{real}} + d(t) - Y_c (\hat{p} + \Delta p) = \tau_{\text{real}} + d(t) - Y_c \hat{p} - Y_c \Delta p $

#### 3.3: Compute $ \Delta p^T \Gamma_p^{-1} \dot{\Delta p} $
The parameter estimation error is $ \Delta p = p - \hat{p} $, and $ p $ is constant, so:

$ \dot{\Delta p} = -\dot{\hat{p}} $

The adaptive law is:

$ \dot{\hat{p}} = -\Gamma_p Y_c^T \eta $

$ \dot{\Delta p} = \Gamma_p Y_c^T \eta $

$ \Delta p^T \Gamma_p^{-1} \dot{\Delta p} = \Delta p^T \Gamma_p^{-1} (\Gamma_p Y_c^T \eta) = \Delta p^T Y_c^T \eta $

#### 3.4: Assemble $ \dot{V}_2 $
Substitute into $ \dot{V}_2 $:

$ \dot{V}_2 = \dot{V}_1 + \eta^T (\tau_{\text{real}} + d(t) - Y_c \hat{p} - Y_c \Delta p) + \Delta p^T Y_c^T \eta $

$ \dot{V}_1 = -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 - e_x \eta_1 - \frac{1}{K_y} e_\theta \eta_2 $

Rewrite the error terms:

$ - e_x \eta_1 - \frac{1}{K_y} e_\theta \eta_2 = -\eta^T \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix} $

So:

$ \dot{V}_2 = -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 - \eta^T \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix} + \eta^T (\tau_{\text{real}} + d(t) - Y_c \hat{p}) + \eta^T (-Y_c \Delta p) + \Delta p^T Y_c^T \eta $

Since $ Y_c $ is diagonal ($ Y_c = \begin{bmatrix} \dot{v}^d & 0 \\ 0 & \dot{\omega}^d \end{bmatrix} $), $ Y_c^T = Y_c $, and for vectors $ a $ and $ b $, $ a^T Y_c b = (a^T Y_c b)^T = b^T Y_c^T a $, so:

$ -\eta^T Y_c \Delta p + \Delta p^T Y_c^T \eta = -\eta^T Y_c \Delta p + \Delta p^T Y_c \eta = 0 $

Thus:

$ \dot{V}_2 = -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 + \eta^T \left( \tau_{\text{real}} + d(t) - Y_c \hat{p} - \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix} \right) $

Substitute $ \tau_{\text{real}} = \tau - \bar{e}_\tau $:

$ \dot{V}_2 = -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 + \eta^T \left( \tau - \bar{e}_\tau + d(t) - Y_c \hat{p} - \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix} \right) $

Use the desired control:

$ \tau = Y_c \hat{p} - K_d \eta - d_B \tanh\left(\frac{\eta}{\epsilon}\right) + \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix} $

$ \tau - Y_c \hat{p} - \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix} = -K_d \eta - d_B \tanh\left(\frac{\eta}{\epsilon}\right) $

$ \dot{V}_2 = -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 + \eta^T \left( -K_d \eta - d_B \tanh\left(\frac{\eta}{\epsilon}\right) + d(t) - \bar{e}_\tau \right) $

$ \dot{V}_2 = -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 - \eta^T K_d \eta + \eta^T d(t) - \eta^T d_B \tanh\left(\frac{\eta}{\epsilon}\right) - \eta^T \bar{e}_\tau $

---

### Step 4: Compute $ \dot{V}_3 $ Explicitly
Substitute into $ \dot{V}_3 $:

$ \dot{V}_3 = \dot{V}_2 + \bar{e}_\tau^T \left( \dot{\tau} - \frac{1}{\gamma} (-\tau_{\text{real}} + a) \right) $

$ \dot{V}_3 = -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 - \eta^T K_d \eta + \eta^T d(t) - \eta^T d_B \tanh\left(\frac{\eta}{\epsilon}\right) - \eta^T \bar{e}_\tau + \bar{e}_\tau^T \left( \dot{\tau} - \frac{1}{\gamma} (-\tau_{\text{real}} + a) \right) $

#### 4.1: Compute $ \dot{\tau} $
$ \tau = Y_c \hat{p} - K_d \eta - d_B \tanh\left(\frac{\eta}{\epsilon}\right) + \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix} $

$ \dot{\tau} = \dot{Y}_c \hat{p} + Y_c \dot{\hat{p}} - K_d \dot{\eta} - d_B \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) + \frac{d}{dt} \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix} $

- **$ \dot{Y}_c $**:
  $ Y_c = \begin{bmatrix} \dot{v}^d & 0 \\ 0 & \dot{\omega}^d \end{bmatrix} $
  $ \dot{Y}_c = \begin{bmatrix} \ddot{v}^d & 0 \\ 0 & \ddot{\omega}^d \end{bmatrix} $
  Where $ \ddot{v}^d $ and $ \ddot{\omega}^d $ are the second derivatives of the desired velocities, computable from $ v^d $ and $ \omega^d $.

- **$ Y_c \dot{\hat{p}} $**:
  $ \dot{\hat{p}} = -\Gamma_p Y_c^T \eta $
  $ Y_c \dot{\hat{p}} = -Y_c \Gamma_p Y_c^T \eta $

- **$ -K_d \dot{\eta} $**:
  $ \dot{\eta} = M_2^{-1} (\tau_{\text{real}} + d(t) - Y_c p) $
  $ -K_d \dot{\eta} = -K_d M_2^{-1} (\tau_{\text{real}} + d(t) - Y_c \hat{p} - Y_c \Delta p) $

- **$ -d_B \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) $**:
  $ \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) = \text{diag}\left(1 - \tanh^2\left(\frac{\eta_i}{\epsilon}\right)\right) \cdot \frac{1}{\epsilon} \dot{\eta} $
  $ -d_B \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) = -d_B \cdot \text{diag}\left(1 - \tanh^2\left(\frac{\eta_i}{\epsilon}\right)\right) \cdot \frac{1}{\epsilon} M_2^{-1} (\tau_{\text{real}} + d(t) - Y_c p) $

- **$ \frac{d}{dt} \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix} $**:
  $ \dot{e}_x = \omega e_y - v + v_r \cos e_\theta $
  $ \dot{e}_\theta = \omega_r - \omega $
  $ \frac{d}{dt} \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix} = \begin{bmatrix} \omega e_y - v + v_r \cos e_\theta \\ \frac{1}{K_y} (\omega_r - \omega) \end{bmatrix} $

Combining these, $ \dot{\tau} $ is a complex expression involving states, their derivatives, and $ \tau_{\text{real}} $, but we’ll keep it symbolic for now.

---

### Step 5: Design $ a $ to Make $ \dot{V}_3 < 0 $
We need $ \dot{V}_3 $ to be negative definite. Current form:

$ \dot{V}_3 = -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 - \eta^T K_d \eta + \eta^T d(t) - \eta^T d_B \tanh\left(\frac{\eta}{\epsilon}\right) - \eta^T \bar{e}_\tau + \bar{e}_\tau^T \left( \dot{\tau} - \frac{1}{\gamma} (-\tau_{\text{real}} + a) \right) $

#### 5.1: Handle Indefinite Terms
- **$ \eta^T d(t) - \eta^T d_B \tanh\left(\frac{\eta}{\epsilon}\right) $**:
  Since $ |d(t)| \leq d_B $, this term is bounded, and the $ \tanh $ function helps mitigate it, often resulting in a small positive residual, but dominated by negative terms when gains are large.

- **$ - \eta^T \bar{e}_\tau + \bar{e}_\tau^T \left( \dot{\tau} - \dot{\tau}_{\text{real}} \right) $**:
  Choose $ a $ to kill this term and make $\dot{V_3}$ negative. Set:

  $ a = \tau_{\text{real}} + \gamma \left( \dot{\tau} + K_\tau (\tau - \tau_{\text{real}}) + \eta \right) $


---

### Final Expressions
**$ \dot{V}_3 $**:

$ \dot{V}_3 = -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 - \eta^T K_d \eta + \eta^T d(t) - \eta^T d_B \tanh\left(\frac{\eta}{\epsilon}\right) - \eta^T \bar{e}_\tau + \bar{e}_\tau^T \left( \dot{\tau} - \frac{1}{\gamma} (-\tau_{\text{real}} + a) \right) $

**Control Input $ a $**:

$ a = \tau_{\text{real}} + \gamma \left( \dot{\tau} + K_\tau (\tau - \tau_{\text{real}}) + \eta \right) $


where $ K_\tau > 1 $, and $ \dot{\tau} $ is computed as:

$ \dot{\tau} = \dot{Y}_c \hat{p} + Y_c \dot{\hat{p}} - K_d \dot{\eta} - d_B \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) + \frac{d}{dt} \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix} $

- **$ \dot{Y}_c $**:
  $ Y_c = \begin{bmatrix} \dot{v}^d & 0 \\ 0 & \dot{\omega}^d \end{bmatrix} $
  $ \dot{Y}_c = \begin{bmatrix} \ddot{v}^d & 0 \\ 0 & \ddot{\omega}^d \end{bmatrix} $
  Where $ \ddot{v}^d $ and $ \ddot{\omega}^d $ are the second derivatives of the desired velocities, computable from $ v^d $ and $ \omega^d $.

- **$ Y_c \dot{\hat{p}} $**:
  $ \dot{\hat{p}} = -\Gamma_p Y_c^T \eta $
  $ Y_c \dot{\hat{p}} = -Y_c \Gamma_p Y_c^T \eta $

- **$ -K_d \dot{\eta} $**:
  $ \dot{\eta} = M_2^{-1} (\tau_{\text{real}} + d(t) - Y_c p) $
  $ -K_d \dot{\eta} = -K_d M_2^{-1} (\tau_{\text{real}} + d(t) - Y_c \hat{p} - Y_c \Delta p) $

- **$ -d_B \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) $**:
  $ \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) = \text{diag}\left(1 - \tanh^2\left(\frac{\eta_i}{\epsilon}\right)\right) \cdot \frac{1}{\epsilon} \dot{\eta} $
  $ -d_B \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) = -d_B \cdot \text{diag}\left(1 - \tanh^2\left(\frac{\eta_i}{\epsilon}\right)\right) \cdot \frac{1}{\epsilon} M_2^{-1} (\tau_{\text{real}} + d(t) - Y_c p) $

- **$ \frac{d}{dt} \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix} $**:
  $ \dot{e}_x = \omega e_y - v + v_r \cos e_\theta $
  $ \dot{e}_\theta = \omega_r - \omega $
  $ \frac{d}{dt} \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix} = \begin{bmatrix} \omega e_y - v + v_r \cos e_\theta \\ \frac{1}{K_y} (\omega_r - \omega) \end{bmatrix} $

## Practical Implementation

Adaptive Law and Control Summary

Kinematic Law

The complete controller is:

$$ v^d = v_r \cos e_\theta + K_x e_x $$

$$ \omega^d = \omega_r + K_\theta e_\theta + v_r e_y K_\gamma \frac{\sin e_\theta}{e_\theta} $$

$$ \eta = \begin{bmatrix} v - v^d \ \omega - \omega^d \end{bmatrix} $$

Adaptive Law

$$ \dot{\hat{\rho}} = -\Gamma \rho_c Y_c^T \eta $$

Torque Reference

$$ \tau_d = Y_c \hat{\rho} - K_d \eta - d_B \tanh\left(\frac{\eta}{\epsilon}\right) + \begin{bmatrix} e_x \ K_\theta e_\theta \end{bmatrix} $$

Final Control Input

$$ a = \tau_d + \dot{\gamma} \tau_d - \gamma K_\tau (\tau - \tau_d) $$



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

```plaintext
START TRY
// --- Initialization ---
PROCEDURE Main
  // Simulation Setup
  SET simulation time step (dt)
  SET robot physical parameters (wheel radius, width, mass, inertia)
  SET disturbance parameters (continuous level, kick properties)
  CHOOSE path type (e.g., Circle, Line, Complex)
  GENERATE desired path points (x, y coordinates)
  DEFINE initial robot pose (x, y, theta) and velocity (v1, omega) near path start

  // Controller Setup
  SET kinematic controller parameters (gains, reference velocity, lookahead)
  INITIALIZE KinematicController instance
  SET adaptive controller parameters (initial parameter estimates, adaptation gains, feedback gains, disturbance bound, robust term usage)
  INITIALIZE AdaptiveController instance, passing the KinematicController

  // Simulation and Visualization Setup
  INITIALIZE Simulation instance with robot params, path, initial state, and disturbance settings
  INITIALIZE Visualizer instance with the desired path
  INITIALIZE data storage lists (for states, torques, controller status)

  // --- Simulation Loop ---
  FOR each simulation step:
    GET current robot state (pose, velocity)
    CALCULATE desired kinematic velocities using KinematicController
    CALCULATE control torques using AdaptiveController
    APPLY torques to Simulation
    UPDATE robot state (pose, velocity, disturbance)
    STORE data (state, torques, errors, etc.)
    IF controller signals finish, BREAK loop
  END FOR

  // --- Post-Processing ---
  PROCESS collected data (errors, parameters, torques, disturbances)
  CREATE animation and plots using Visualizer
  DISPLAY or SAVE results
END PROCEDURE (See <attachments> above for file contents. You may not need to search or read the file again.)
