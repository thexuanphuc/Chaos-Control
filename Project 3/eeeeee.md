# Backstepping-Based Adaptive Controller Derivation for a Nonholonomic Mobile Robot  
*May 10, 2025*  

## Abstract  
This document presents a detailed derivation of a backstepping-based adaptive controller for a nonholonomic mobile robot with unknown mass, inertia, and bounded external disturbances. The control design is split into two stages: a kinematic controller to stabilize position and orientation tracking errors, and a dynamic controller to ensure velocity tracking while estimating unknown parameters. Lyapunov stability theory is used to guarantee bounded tracking errors and parameter estimates.

## 1. Introduction  
This paper derives a robust adaptive controller for a 3-wheeled nonholonomic mobile robot under unknown mass, inertia, and external disturbances (e.g., wind). The controller employs backstepping to combine a kinematic path-following law with a dynamic adaptive law, ensuring trajectory tracking despite uncertainties. The derivation uses Lyapunov stability theory to ensure bounded tracking errors and parameter estimates.

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
For developing kinematic controller we are assuming -v = v^d and -\omega = \omega_d 
Choose Lyapunov function for kinematic errors:

$$
V_1 := \frac{1}{2} \left( e_x^2 + e_y^2 + \frac{1}{K_y} e_\theta^2 \right)
$$

Taking time derivative and substitute error dynamics:

<p>
  $$\dot{V}_1 := e_x (\omega e_y - v + v_r \cos e_\theta) + e_y (-\omega e_x + v_r \sin e_\theta) + \frac{1}{K_y} e_\theta (\omega_r - \omega)$$
</p>
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

Define velocity error:

![Velocity error](https://latex.codecogs.com/svg.image?\eta%20=%20v%20-%20v^d%20=%20\begin{bmatrix}%20\eta_1%20\\%20\eta_2%20\end{bmatrix})

Dynamic model becomes:

![Dynamic model](https://latex.codecogs.com/svg.image?M_2%20\dot{\eta}%20=%20\tau%20+%20d(t)%20-%20Y_c%20\hat{p}%20+%20Y_c%20\Delta%20p)

---

### 4.2 Composite Lyapunov Function

Composite Lyapunov function:

![Composite Lyapunov](https://latex.codecogs.com/svg.image?V%20=%20V_1%20+%20\frac{1}{2}%20\eta^T%20M_2%20\eta%20+%20\frac{1}{2}%20\Delta%20p^T%20\Gamma_p^{-1}%20\Delta%20p)

---

### 4.3 Recompute V̇₁ with Velocity Errors

Updated error dynamics:

![V1 dot with velocity errors](https://latex.codecogs.com/svg.image?\dot{V}_1%20=%20-K_x%20e_x^2%20-%20\frac{K_\theta}{K_y}%20e_\theta^2%20-%20e_x%20\eta_1%20-%20\frac{1}{K_y}%20e_\theta%20\eta_2)

---

### 4.4 Compute V̇

Adaptive law and control input:

![Adaptive law](https://latex.codecogs.com/svg.image?\dot{\hat{p}}%20=%20-\Gamma_p%20Y_c^T%20\eta)

![Control input](https://latex.codecogs.com/svg.image?\tau%20=%20Y_c%20\hat{p}%20-%20K_d%20\eta%20-%20d_B%20\tanh\left(\frac{\eta}{\epsilon}\right)%20+%20\begin{bmatrix}%20e_x%20\\%20\frac{1}{K_y}%20e_\theta%20\end{bmatrix})

Resulting in:

![V dot result](https://latex.codecogs.com/svg.image?\dot{V}%20\leq%20-K_x%20e_x^2%20-%20\frac{K_\theta}{K_y}%20e_\theta^2%20-%20\eta^T%20K_d%20\eta%20\leq%200)

## 4. Final Controller Design (Third Backstepping Step)  

### Step 1: Define the Lyapunov Function and Its Components
The extended Lyapunov function with actuator dynamics is given by:

$ V_3 = V + \frac{1}{2} \bar{e}_\tau^T \bar{e}_\tau $

where:
- $ V = V_1 + \frac{1}{2} \eta^T M_2 \eta + \frac{1}{2} \Delta p^T \Gamma_p^{-1} \Delta p $ is the composite Lyapunov function from the dynamic controller design,
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

$ \dot{V}_3 = \dot{V} + \bar{e}_\tau^T \dot{\bar{e}}_\tau $

Since $ \bar{e}_\tau = \tau - \tau_{\text{real}} $, its derivative is:

$ \dot{\bar{e}}_\tau = \dot{\tau} - \dot{\tau}_{\text{real}} $

Substitute the actuator dynamics:

$ \dot{\bar{e}}_\tau = \dot{\tau} - \frac{1}{\gamma} (-\tau_{\text{real}} + a) $

Thus:

$ \dot{V}_3 = \dot{V} + \bar{e}_\tau^T \left( \dot{\tau} - \frac{1}{\gamma} (-\tau_{\text{real}} + a) \right) $

We need to compute $ \dot{V} $ with $ \tau_{\text{real}} $ as the actual input (since the dynamic model is now $ M_2 \dot{v} = \tau_{\text{real}} + d(t) $), and express $ \dot{\tau} $ explicitly.

---

### Step 3: Compute $ \dot{V} $ with Actuator Dynamics
The composite Lyapunov function is:

$ V = V_1 + \frac{1}{2} \eta^T M_2 \eta + \frac{1}{2} \Delta p^T \Gamma_p^{-1} \Delta p $

Its derivative is:

$ \dot{V} = \dot{V}_1 + \eta^T M_2 \dot{\eta} + \Delta p^T \Gamma_p^{-1} \dot{\Delta p} $

#### 3.1: Compute $ \dot{V}_1 $
From the kinematic controller design:

![V1 dot actuator](https://latex.codecogs.com/svg.image?\dot{V}_1%20=%20-K_x%20e_x^2%20-%20\frac{K_\theta}{K_y}%20e_\theta^2%20-%20e_x%20\eta_1%20-%20\frac{1}{K_y}%20e_\theta%20\eta_2)

This accounts for the velocity error ![eta](https://latex.codecogs.com/svg.image?\eta%20=%20v%20-%20v^d), where ![v vector](https://latex.codecogs.com/svg.image?v%20=%20%5Bv,%20\omega%5D^T) is the actual velocity, and ![vd vector](https://latex.codecogs.com/svg.image?v^d%20=%20%5Bv^d,%20\omega^d%5D^T) is the desired velocity.

#### 3.2: Compute ![M2 eta dot](https://latex.codecogs.com/svg.image?M_2%20\dot{\eta})
The velocity error is ![eta](https://latex.codecogs.com/svg.image?\eta%20=%20v%20-%20v^d), so:

![eta dot](https://latex.codecogs.com/svg.image?\dot{\eta}%20=%20\dot{v}%20-%20\dot{v}^d)

The dynamic model is:

![dynamic model actuator](https://latex.codecogs.com/svg.image?M_2%20\dot{v}%20=%20\tau_{\text{real}}%20+%20d(t))

![v dot](https://latex.codecogs.com/svg.image?\dot{v}%20=%20M_2^{-1}%20(\tau_{\text{real}}%20+%20d(t)))

Also, ![vd dynamic](https://latex.codecogs.com/svg.image?M_2%20\dot{v}^d%20=%20Y_c%20p), so:

![vd dot](https://latex.codecogs.com/svg.image?\dot{v}^d%20=%20M_2^{-1}%20Y_c%20p)

Thus:

![M2 eta dot expansion](https://latex.codecogs.com/svg.image?M_2%20\dot{\eta}%20=%20M_2%20(\dot{v}%20-%20\dot{v}^d)%20=%20\tau_{\text{real}}%20+%20d(t)%20-%20Y_c%20p)

Since ![p split](https://latex.codecogs.com/svg.image?p%20=%20\hat{p}%20+%20\Delta%20p):

![M2 eta dot split](https://latex.codecogs.com/svg.image?M_2%20\dot{\eta}%20=%20\tau_{\text{real}}%20+%20d(t)%20-%20Y_c%20(\hat{p}%20+%20\Delta%20p)%20=%20\tau_{\text{real}}%20+%20d(t)%20-%20Y_c%20\hat{p}%20-%20Y_c%20\Delta%20p)

#### 3.3: Compute ![Delta p term](https://latex.codecogs.com/svg.image?\Delta%20p^T%20\Gamma_p^{-1}%20\dot{\Delta%20p})
The parameter estimation error is ![Delta p](https://latex.codecogs.com/svg.image?\Delta%20p%20=%20p%20-%20\hat{p}), and ![p constant](https://latex.codecogs.com/svg.image?p) is constant, so:

![Delta p dot](https://latex.codecogs.com/svg.image?\dot{\Delta%20p}%20=%20-\dot{\hat{p}})

The adaptive law is:

![adaptive law actuator](https://latex.codecogs.com/svg.image?\dot{\hat{p}}%20=%20-\Gamma_p%20Y_c^T%20\eta)

![Delta p dot expansion](https://latex.codecogs.com/svg.image?\dot{\Delta%20p}%20=%20\Gamma_p%20Y_c^T%20\eta)

![Delta p Gamma term](https://latex.codecogs.com/svg.image?\Delta%20p^T%20\Gamma_p^{-1}%20\dot{\Delta%20p}%20=%20\Delta%20p^T%20\Gamma_p^{-1}%20(\Gamma_p%20Y_c^T%20\eta)%20=%20\Delta%20p^T%20Y_c^T%20\eta)

#### 3.4: Assemble ![V dot](https://latex.codecogs.com/svg.image?\dot{V})
Substitute into ![V dot](https://latex.codecogs.com/svg.image?\dot{V}):

![V dot expansion](https://latex.codecogs.com/svg.image?\dot{V}%20=%20\dot{V}_1%20+%20\eta^T%20(\tau_{\text{real}}%20+%20d(t)%20-%20Y_c%20\hat{p}%20-%20Y_c%20\Delta%20p)%20+%20\Delta%20p^T%20Y_c^T%20\eta)

![V1 dot repeat](https://latex.codecogs.com/svg.image?\dot{V}_1%20=%20-K_x%20e_x^2%20-%20\frac{K_\theta}{K_y}%20e_\theta^2%20-%20e_x%20\eta_1%20-%20\frac{1}{K_y}%20e_\theta%20\eta_2)

Rewrite the error terms:

![error terms rewrite](https://latex.codecogs.com/svg.image?-e_x%20\eta_1%20-%20\frac{1}{K_y}%20e_\theta%20\eta_2%20=%20-\eta^T%20\begin{bmatrix}%20e_x%20\\%20\frac{1}{K_y}%20e_\theta%20\end{bmatrix})

So:

![V dot error rewrite](https://latex.codecogs.com/svg.image?\dot{V}%20=%20-K_x%20e_x^2%20-%20\frac{K_\theta}{K_y}%20e_\theta^2%20-%20\eta^T%20\begin{bmatrix}%20e_x%20\\%20\frac{1}{K_y}%20e_\theta%20\end{bmatrix}%20+%20\eta^T%20(\tau_{\text{real}}%20+%20d(t)%20-%20Y_c%20\hat{p})%20+%20\eta^T%20(-Y_c%20\Delta%20p)%20+%20\Delta%20p^T%20Y_c^T%20\eta)

Since ![Yc diagonal](https://latex.codecogs.com/svg.image?Y_c) is diagonal, ![YcT=Yc](https://latex.codecogs.com/svg.image?Y_c^T%20=%20Y_c), and for vectors ![a,b](https://latex.codecogs.com/svg.image?a,b), ![aTYcb](https://latex.codecogs.com/svg.image?a^T%20Y_c%20b%20=%20b^T%20Y_c^T%20a), so:

![Yc cross terms](https://latex.codecogs.com/svg.image?-\eta^T%20Y_c%20\Delta%20p%20+%20\Delta%20p^T%20Y_c^T%20\eta%20=%200)

Thus:

![V dot simplified](https://latex.codecogs.com/svg.image?\dot{V}%20=%20-K_x%20e_x%20-%20\frac{K_\theta}{K_y}%20e_\theta^2%20+%20\eta^T%20\left(%20\tau_{\text{real}}%20+%20d(t)%20-%20Y_c%20\hat{p}%20-%20\begin{bmatrix}%20e_x%20\\%20\frac{1}{K_y}%20e_\theta%20\end{bmatrix}%20\right))

Substitute ![tau real](https://latex.codecogs.com/svg.image?\tau_{\text{real}}%20=%20\tau%20-%20\bar{e}_\tau):

![V dot with tau real](https://latex.codecogs.com/svg.image?\dot{V}%20=%20-K_x%20e_x^2%20-%20\frac{K_\theta}{K_y}%20e_\theta^2%20+%20\eta^T%20\left(%20\tau%20-%20\bar{e}_\tau%20+%20d(t)%20-%20Y_c%20\hat{p}%20-%20\begin{bmatrix}%20e_x%20\\%20\frac{1}{K_y}%20e_\theta%20\end{bmatrix}%20\right))

Use the desired control:

![tau desired](https://latex.codecogs.com/svg.image?\tau%20=%20Y_c%20\hat{p}%20-%20K_d%20\eta%20-%20d_B%20\tanh\left(\frac{\eta}{\epsilon}\right)%20+%20\begin{bmatrix}%20e_x%20\\%20\frac{1}{K_y}%20e_\theta%20\end{bmatrix})

![tau minus terms](https://latex.codecogs.com/svg.image?\tau%20-%20Y_c%20\hat{p}%20-%20\begin{bmatrix}%20e_x%20\\%20\frac{1}{K_y}%20e_\theta%20\end{bmatrix}%20=%20-K_d%20\eta%20-%20d_B%20\tanh\left(\frac{\eta}{\epsilon}\right))

![V dot final](https://latex.codecogs.com/svg.image?\dot{V}%20=%20-K_x%20e_x^2%20-%20\frac{K_\theta}{K_y}%20e_\theta^2%20+%20\eta^T%20\left(%20-K_d%20\eta%20-%20d_B%20\tanh\left(\frac{\eta}{\epsilon}\right)%20+%20d(t)%20-%20\bar{e}_\tau%20\right))

![V dot expanded](https://latex.codecogs.com/svg.image?\dot{V}%20=%20-K_x%20e_x^2%20-%20\frac{K_\theta}{K_y}%20e_\theta^2%20-%20\eta^T%20K_d%20\eta%20+%20\eta^T%20d(t)%20-%20\eta^T%20d_B%20\tanh\left(\frac{\eta}{\epsilon}\right)%20-%20\eta^T%20\bar{e}_\tau)

---

### Step 4: Compute ![V3 dot](https://latex.codecogs.com/svg.image?\dot{V}_3) Explicitly
Substitute into ![V3 dot](https://latex.codecogs.com/svg.image?\dot{V}_3):

![V3 dot expansion](https://latex.codecogs.com/svg.image?\dot{V}_3%20=%20\dot{V}%20+%20\bar{e}_\tau^T%20\left(%20\dot{\tau}%20-%20\frac{1}{\gamma}%20(-\tau_{\text{real}}%20+%20a)%20\right))

![V3 dot full](https://latex.codecogs.com/svg.image?\dot{V}_3%20=%20-K_x%20e_x^2%20-%20\frac{K_\theta}{K_y}%20e_\theta^2%20-%20\eta^T%20K_d%20\eta%20+%20\eta^T%20d(t)%20-%20\eta^T%20d_B%20\tanh\left(\frac{\eta}{\epsilon}\right)%20-%20\eta^T%20\bar{e}_\tau%20+%20\bar{e}_\tau^T%20\left(%20\dot{\tau}%20-%20\frac{1}{\gamma}%20(-\tau_{\text{real}}%20+%20a)%20\right))

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
  Choose $ a $ to make this negative. Set:

  $ \dot{\tau}_{\text{real}} = \dot{\tau} + K_\tau \bar{e}_\tau + \eta $

  $ \frac{1}{\gamma} (-\tau_{\text{real}} + a) = \dot{\tau} + K_\tau \bar{e}_\tau + \eta $

  $ a = \tau_{\text{real}} + \gamma ( \dot{\tau} + K_\tau \bar{e}_\tau + \eta ) $

  Then:

  $ \bar{e}_\tau^T (\dot{\tau} - \dot{\tau}_{\text{real}}) = \bar{e}_\tau^T (\dot{\tau} - (\dot{\tau} + K_\tau \bar{e}_\tau + \eta)) = -\bar{e}_\tau^T K_\tau \bar{e}_\tau - \bar{e}_\tau^T \eta $

  $ - \eta^T \bar{e}_\tau + \bar{e}_\tau^T (\dot{\tau} - \dot{\tau}_{\text{real}}) = -\bar{e}_\tau^T \eta - \bar{e}_\tau^T K_\tau \bar{e}_\tau - \bar{e}_\tau^T \eta = -K_\tau \bar{e}_\tau^T \bar{e}_\tau - 2 \bar{e}_\tau^T \eta $

  Use Young’s inequality: $ -2 \bar{e}_\tau^T \eta \leq ||\bar{e}_\tau||^2 + ||\eta||^2 $:

  $ \dot{V}_3 \leq -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 - \eta^T K_d \eta + \eta^T d(t) - \eta^T d_B \tanh\left(\frac{\eta}{\epsilon}\right) - K_\tau \bar{e}_\tau^T \bar{e}__\tau + ||\eta||^2 + ||\bar{e}_\tau||^2 $

  $ = -K_x e_x^2 - \frac{K_\theta}{K_y} e_\theta^2 - (\lambda_{\text{min}}(K_d) - 1) ||\eta||^2 - (K_\tau - 1) ||\bar{e}_\tau||^2 + \eta^T d(t) - \eta^T d_B \tanh\left(\frac{\eta}{\epsilon}\right) $

Choose $ K_d $ and $ K_\tau $ such that $ \lambda_{\text{min}}(K_d) > 1 $ and $ K_\tau > 1 $, ensuring all quadratic terms are negative definite, and the disturbance term is bounded, making $ \dot{V}_3 < 0 $ outside a small region.

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


