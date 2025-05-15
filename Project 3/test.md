# Backstepping-Based Adaptive Controller Derivation for a Nonholonomic Mobile Robot  
*May 10, 2025*  

## Abstract  
This document presents a detailed derivation of a backstepping-based adaptive controller for a nonholonomic mobile robot with unknown mass, inertia, and bounded external disturbances. The control design is split into two stages: a kinematic controller to stabilize position and orientation tracking errors, and a dynamic controller to ensure velocity tracking while estimating unknown parameters. Lyapunov stability theory is used to guarantee bounded tracking errors and parameter estimates.

## 1. Introduction  
This paper derives a robust adaptive controller for a 3-wheeled nonholonomic mobile robot under unknown mass, inertia, and external disturbances (e.g., wind). The controller employs backstepping to combine a kinematic path-following law with a dynamic adaptive law, ensuring trajectory tracking despite uncertainties. The derivation uses Lyapunov stability theory to ensure bounded tracking errors and parameter estimates.

## 2. System Model  
### 2.1 Kinematic Model  
The kinematic model of a unicycle-type mobile robot is given by:  

$$
\begin{aligned}
\dot{x} &= v \cos \theta \\
\dot{y} &= v \sin \theta \\
\dot{\theta} &= \omega 
\end{aligned}
$$  

where $(x, y) \in \mathbb{R}^2$ is the robot's position in the world frame, $\theta \in \mathbb{R}$ is the orientation, $v \in \mathbb{R}$ is the linear velocity, and $\omega \in \mathbb{R}$ is the angular velocity.  

The reference trajectory to be tracked is:  

$$
\begin{aligned}
\dot{x}_r &= v_r \cos \theta_r \\
\dot{y}_r &= v_r \sin \theta_r \\
\dot{\theta}_r &= \omega_r 
\end{aligned}
$$  

Tracking errors in the robot's body frame:  

$$
\begin{aligned}
e_x &= \cos \theta (x_d - x) + \sin \theta (y_d - y) \\
e_y &= -\sin \theta (x_d - x) + \cos \theta (y_d - y) \\
e_θ &= \theta_d - \theta 
\end{aligned}
$$  

Error dynamics:  

$$
\begin{aligned}
\dot{e_x} &= \omega e_y - v + v_r \cos e_θ \\
\dot{e_y} &= -\omega e_x + v_r \sin e_θ \\
\dot{e_θ} &= \omega_r - \omega 
\end{aligned}
$$  


### 2.2 Dynamic Model  
The dynamic model governs velocity dynamics:  

$$
M_2 \dot{v} = \tau + d(t)
$$  

where:  
- $v = [v, \omega]^T$: Velocity vector  
- $\tau = [\tau_v, \tau_\omega]^T$: Control input  
- $d(t)$: Bounded disturbance ($|d(t)| ≤ d_B$)  
- $M_2 = \text{diag}(m, I)$: Unknown mass-inertia matrix  

Define parameter vector $p = [m, I]^T$, estimate $\hat{p} = [\hat{m}, \hat{I}]^T$, and regressor matrix:  

$$
Y_c = \begin{bmatrix} \dot{v}^d & 0 \\ 0 & \dot{\omega}^d \end{bmatrix}, \quad M_2 \dot{v}^d = Y_c p
$$

## 3. Kinematic Controller Design (First Backstepping Step)  
### 3.1 Lyapunov Function  
Choose Lyapunov function for kinematic errors:  

$$
V_1 = \frac{1}{2} \left( e_x^2 + e_y^2 + \frac{1}{K_y} e_θ^2 \right)
$$  

Time derivative:  

$$
\dot{V_1} = e_x (\omega e_y - v + v_r \cos e_θ) + e_y (-\omega e_x + v_r \sin e_θ) + \frac{1}{K_y} e_θ (\omega_r - \omega)
$$

### 3.2 Control Law Design  
Virtual control inputs:  

$$
\begin{aligned}
v^d &= v_r \cos e_θ + K_x e_x \\
\omega^d &= \omega_r + K_θ e_θ + v_r e_y K_y \frac{\sin e_θ}{e_θ}
\end{aligned}
$$  

Closed-loop error dynamics:  

$$
\begin{aligned}
\dot{e_x} &= \omega^d e_y - K_x e_x \\
\dot{e_y} &= -\omega^d e_x + v_r \sin e_θ \\
\dot{e_θ} &= -K_θ e_θ - v_r e_y K_y \frac{\sin e_θ}{e_θ}
\end{aligned}
$$  

Resulting in:  

$$
\dot{V_1} = -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 ≤ 0
$$

## 4. Dynamic Controller Design (Second Backstepping Step)  
### 4.1 Velocity Tracking Error  
Define velocity error:  

$$
\eta = v - v^d = \begin{bmatrix} \eta_1 \\ \eta_2 \end{bmatrix}
$$  

Dynamic model becomes:  

$$
M_2 \dot{\eta} = \tau + d(t) - Y_c \hat{p} + Y_c \Delta p
$$

### 4.2 Composite Lyapunov Function  
Composite Lyapunov function:  

$$
V_2= V_1 + \frac{1}{2} \eta^T M_2 \eta + \frac{1}{2} \Delta p^T \Gamma_p^{-1} \Delta p
$$

### 4.3 Recompute $\dot{V_1}$ with Velocity Errors  
Updated error dynamics:  

$$
\dot{V_1} = -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 - e_x \eta_1 - \frac{1}{K_y} e_θ \eta_2
$$

### 4.4 Compute $\dot{V}$  
Adaptive law and control input:  

$$
\begin{aligned}
\dot{\hat{p}} &= -\Gamma_p Y_c^T \eta \\
\tau &= Y_c \hat{p} - K_d \eta - d_B \tanh\left(\frac{\eta}{\epsilon}\right) + \begin{bmatrix} e_x \\ \frac{1}{K_y} e_θ \end{bmatrix}
\end{aligned}
$$  

Resulting in:  

$$
\dot{V_2}≤ -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 - \eta^T K_d \eta ≤ 0
$$


## 4. Final Controller Design (Third Backstepping Step)  



### Step 1: Define the Lyapunov Function and Its Components
The extended Lyapunov function with actuator dynamics is given by:

$$ V_3 = V_2+ \frac{1}{2} \bar{e}_τ^T \bar{e}_τ $$

where:
- $ V_2= V_1 + \frac{1}{2} \eta^T M_2 \eta + \frac{1}{2} \Delta p^T \Gamma_p^{-1} \Delta p $ is the composite Lyapunov function from the dynamic controller design,
- $ \bar{e}_τ = \tau - \tau_{\text{real}} $ is the error between the desired control input $ \tau $ and the actual control input $ \tau_{\text{real}} $,
- $ V_1 = \frac{1}{2} \left( e_x^2 + e_y^2 + \frac{1}{K_y} e_θ^2 \right) $ is the kinematic Lyapunov function,
- $ \eta = v - v^d $ is the velocity tracking error,
- $ \Delta p = p - \hat{p} $ is the parameter estimation error,
- $ M_2 = \text{diag}(m, I) $ is the unknown mass-inertia matrix,
- $ \Gamma_p $ is a positive definite adaptation gain matrix.

The actuator dynamics are:

$$ \dot{\tau}_{\text{real}} = \frac{1}{\gamma} (-\tau_{\text{real}} + a) $$

where $ a $ is the control input we can design, and $ \gamma > 0 $ is a time constant.

### Step 2: Compute the Time Derivative $ \dot{V_3} $$
The time derivative of $ V_3 $ is:

$$ \dot{V_3} = \dot{V_2}+ \bar{e}_τ^T \dot{\bar{e}}_τ $$

Since $ \bar{e}_τ = \tau - \tau_{\text{real}} $, its derivative is:

$$ \dot{\bar{e}}_τ = \dot{\tau} - \dot{\tau}_{\text{real}} $$

Substitute the actuator dynamics:

$$ \dot{\bar{e}}_τ = \dot{\tau} - \frac{1}{\gamma} (-\tau_{\text{real}} + a) $$

Thus:

$$ \dot{V_3} = \dot{V_2}+ \bar{e}_τ^T \left( \dot{\tau} - \frac{1}{\gamma} (-\tau_{\text{real}} + a) \right) $$

We need to compute $ \dot{V_2}$ with $ \tau_{\text{real}} $ as the actual input (since the dynamic model is now $ M_2 \dot{v} = \tau_{\text{real}} + d(t) $), and express $ \dot{\tau} $ explicitly.

---

### Step 3: Compute $ \dot{V_2}$ with Actuator Dynamics
The composite Lyapunov function is:

$$ V_2= V_1 + \frac{1}{2} \eta^T M_2 \eta + \frac{1}{2} \Delta p^T \Gamma_p^{-1} \Delta p $$

Its derivative is:

$$ \dot{V_2}= \dot{V_1} + \eta^T M_2 \dot{\eta} + \Delta p^T \Gamma_p^{-1} \dot{\Delta p} $$

#### 3.1: Compute $ \dot{V_1} $$
From the kinematic controller design:

$$ \dot{V_1} = -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 - e_x \eta_1 - \frac{1}{K_y} e_θ \eta_2 $$

This accounts for the velocity error $ \eta = v - v^d $, where $ v = [v, \omega]^T $ is the actual velocity, and $ v^d = [v^d, \omega^d]^T $ is the desired velocity.

#### 3.2: Compute $ M_2 \dot{\eta} $$
The velocity error is $ \eta = v - v^d $, so:

$$ \dot{\eta} = \dot{v} - \dot{v}^d $$

The dynamic model is:

$$ M_2 \dot{v} = \tau_{\text{real}} + d(t) $$

$$ \dot{v} = M_2^{-1} (\tau_{\text{real}} + d(t)) $$

Also, $ M_2 \dot{v}^d = Y_c p $, so:

$$ \dot{v}^d = M_2^{-1} Y_c p $$

Thus:

$$ M_2 \dot{\eta} = M_2 (\dot{v} - \dot{v}^d) = \tau_{\text{real}} + d(t) - Y_c p $$

Since $ p = \hat{p} + \Delta p $:

$$ M_2 \dot{\eta} = \tau_{\text{real}} + d(t) - Y_c (\hat{p} + \Delta p) = \tau_{\text{real}} + d(t) - Y_c \hat{p} - Y_c \Delta p $$

#### 3.3: Compute $ \Delta p^T \Gamma_p^{-1} \dot{\Delta p} $$
The parameter estimation error is $ \Delta p = p - \hat{p} $, and $ p $ is constant, so:

$$ \dot{\Delta p} = -\dot{\hat{p}} $$

The adaptive law is:

$$ \dot{\hat{p}} = -\Gamma_p Y_c^T \eta $$

$$ \dot{\Delta p} = \Gamma_p Y_c^T \eta $$

$$ \Delta p^T \Gamma_p^{-1} \dot{\Delta p} = \Delta p^T \Gamma_p^{-1} (\Gamma_p Y_c^T \eta) = \Delta p^T Y_c^T \eta $$

#### 3.4: Assemble $ \dot{V_2}$$
Substitute into $ \dot{V_2}$:

$$ \dot{V_2}= \dot{V_1} + \eta^T (\tau_{\text{real}} + d(t) - Y_c \hat{p} - Y_c \Delta p) + \Delta p^T Y_c^T \eta $$

$$ \dot{V_1} = -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 - e_x \eta_1 - \frac{1}{K_y} e_θ \eta_2 $$

Rewrite the error terms:

$$ - e_x \eta_1 - \frac{1}{K_y} e_θ \eta_2 = -\eta^T \begin{bmatrix} e_x \\ \frac{1}{K_y} e_θ \end{bmatrix} $$

So:

$$ \dot{V_2}= -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 - \eta^T \begin{bmatrix} e_x \\ \frac{1}{K_y} e_θ \end{bmatrix} + \eta^T (\tau_{\text{real}} + d(t) - Y_c \hat{p}) + \eta^T (-Y_c \Delta p) + \Delta p^T Y_c^T \eta $$

Since $ Y_c $ is diagonal ($ Y_c = \begin{bmatrix} \dot{v}^d & 0 \\ 0 & \dot{\omega}^d \end{bmatrix} $), $ Y_c^T = Y_c $, and for vectors $ a $ and $ b $, $ a^T Y_c b = (a^T Y_c b)^T = b^T Y_c^T a $, so:

$$ -\eta^T Y_c \Delta p + \Delta p^T Y_c^T \eta = -\eta^T Y_c \Delta p + \Delta p^T Y_c \eta = 0 $$

Thus:

$$ \dot{V_2}= -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 + \eta^T \left( \tau_{\text{real}} + d(t) - Y_c \hat{p} - \begin{bmatrix} e_x \\ \frac{1}{K_y} e_θ \end{bmatrix} \right) $$

Substitute $ \tau_{\text{real}} = \tau - \bar{e}_τ $:

$$ \dot{V_2}= -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 + \eta^T \left( \tau - \bar{e}_τ + d(t) - Y_c \hat{p} - \begin{bmatrix} e_x \\ \frac{1}{K_y} e_θ \end{bmatrix} \right) $$

Use the desired control:

$$ \tau = Y_c \hat{p} - K_d \eta - d_B \tanh\left(\frac{\eta}{\epsilon}\right) + \begin{bmatrix} e_x \\ \frac{1}{K_y} e_θ \end{bmatrix} $$

$$ \tau - Y_c \hat{p} - \begin{bmatrix} e_x \\ \frac{1}{K_y} e_θ \end{bmatrix} = -K_d \eta - d_B \tanh\left(\frac{\eta}{\epsilon}\right) $$

$$ \dot{V_2}= -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 + \eta^T \left( -K_d \eta - d_B \tanh\left(\frac{\eta}{\epsilon}\right) + d(t) - \bar{e}_τ \right) $$

$$ \dot{V_2}= -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 - \eta^T K_d \eta + \eta^T d(t) - \eta^T d_B \tanh\left(\frac{\eta}{\epsilon}\right) - \eta^T \bar{e}_τ $$

---

### Step 4: Compute $ \dot{V_3} $ Explicitly
Substitute into $ \dot{V_3} $:

$$ \dot{V_3} = \dot{V_2}+ \bar{e}_τ^T \left( \dot{\tau} - \frac{1}{\gamma} (-\tau_{\text{real}} + a) \right) $$

$$ \dot{V_3} = -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 - \eta^T K_d \eta + \eta^T d(t) - \eta^T d_B \tanh\left(\frac{\eta}{\epsilon}\right) - \eta^T \bar{e}_τ + \bar{e}_τ^T \left( \dot{\tau} - \frac{1}{\gamma} (-\tau_{\text{real}} + a) \right) $$

#### 4.1: Compute $ \dot{\tau} $$
$$ \tau = Y_c \hat{p} - K_d \eta - d_B \tanh\left(\frac{\eta}{\epsilon}\right) + \begin{bmatrix} e_x \\ \frac{1}{K_y} e_θ \end{bmatrix} $$

$$ \dot{\tau} = \dot{Y_c} \hat{p} + Y_c \dot{\hat{p}} - K_d \dot{\eta} - d_B \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) + \frac{d}{dt} \begin{bmatrix} e_x \\ \frac{1}{K_y} e_θ \end{bmatrix} $$

- **$ \dot{Y_c} $**:
  $ Y_c = \begin{bmatrix} \dot{v}^d & 0 \\ 0 & \dot{\omega}^d \end{bmatrix} $$
  $ \dot{Y_c} = \begin{bmatrix} \ddot{v}^d & 0 \\ 0 & \ddot{\omega}^d \end{bmatrix} $$
  Where $ \ddot{v}^d $ and $ \ddot{\omega}^d $ are the second derivatives of the desired velocities, computable from $ v^d $ and $ \omega^d $.

- **$ Y_c \dot{\hat{p}} $**:
  $ \dot{\hat{p}} = -\Gamma_p Y_c^T \eta $$
  $ Y_c \dot{\hat{p}} = -Y_c \Gamma_p Y_c^T \eta $$

- **$ -K_d \dot{\eta} $**:
  $ \dot{\eta} = M_2^{-1} (\tau_{\text{real}} + d(t) - Y_c p) $$
  $ -K_d \dot{\eta} = -K_d M_2^{-1} (\tau_{\text{real}} + d(t) - Y_c \hat{p} - Y_c \Delta p) $$

- **$ -d_B \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) $**:
  $ \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) = \text{diag}\left(1 - \tanh^2\left(\frac{\eta_i}{\epsilon}\right)\right) \cdot \frac{1}{\epsilon} \dot{\eta} $$
  $ -d_B \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) = -d_B \cdot \text{diag}\left(1 - \tanh^2\left(\frac{\eta_i}{\epsilon}\right)\right) \cdot \frac{1}{\epsilon} M_2^{-1} (\tau_{\text{real}} + d(t) - Y_c p) $$

- **$ \frac{d}{dt} \begin{bmatrix} e_x \\ \frac{1}{K_y} e_θ \end{bmatrix} $**:
  $ \dot{e_x} = \omega e_y - v + v_r \cos e_θ $$
  $ \dot{e_θ} = \omega_r - \omega $$
  $ \frac{d}{dt} \begin{bmatrix} e_x \\ \frac{1}{K_y} e_θ \end{bmatrix} = \begin{bmatrix} \omega e_y - v + v_r \cos e_θ \\ \frac{1}{K_y} (\omega_r - \omega) \end{bmatrix} $$

Combining these, $ \dot{\tau} $ is a complex expression involving states, their derivatives, and $ \tau_{\text{real}} $, but we’ll keep it symbolic for now.

---

### Step 5: Design $ a $ to Make $ \dot{V_3} < 0 $$
We need $ \dot{V_3} $ to be negative definite. Current form:

$$ \dot{V_3} = -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 - \eta^T K_d \eta + \eta^T d(t) - \eta^T d_B \tanh\left(\frac{\eta}{\epsilon}\right) - \eta^T \bar{e}_τ + \bar{e}_τ^T \left( \dot{\tau} - \frac{1}{\gamma} (-\tau_{\text{real}} + a) \right) $$

#### 5.1: Handle Indefinite Terms
- **$ \eta^T d(t) - \eta^T d_B \tanh\left(\frac{\eta}{\epsilon}\right) $**:
  Since $ |d(t)| ≤ d_B $, this term is bounded, and the $ \tanh $ function helps mitigate it, often resulting in a small positive residual, but dominated by negative terms when gains are large.

- **$ - \eta^T \bar{e}_τ + \bar{e}_τ^T \left( \dot{\tau} - \dot{\tau}_{\text{real}} \right) $**:
  Choose $ a $ to make this negative. Set:

  $ \dot{\tau}_{\text{real}} = \dot{\tau} + K_τ \bar{e}_τ + \eta $$

  $ \frac{1}{\gamma} (-\tau_{\text{real}} + a) = \dot{\tau} + K_τ \bar{e}_τ + \eta $$

  $ a = \tau_{\text{real}} + \gamma ( \dot{\tau} + K_τ \bar{e}_τ + \eta ) $$

  Then:

  $ \bar{e}_τ^T (\dot{\tau} - \dot{\tau}_{\text{real}}) = \bar{e}_τ^T (\dot{\tau} - (\dot{\tau} + K_τ \bar{e}_τ + \eta)) = -\bar{e}_τ^T K_τ \bar{e}_τ - \bar{e}_τ^T \eta $$

  $ - \eta^T \bar{e}_τ + \bar{e}_τ^T (\dot{\tau} - \dot{\tau}_{\text{real}}) = -\bar{e}_τ^T \eta - \bar{e}_τ^T K_τ \bar{e}_τ - \bar{e}_τ^T \eta = -K_τ \bar{e}_τ^T \bar{e}_τ - 2 \bar{e}_τ^T \eta $$

  Use Young’s inequality: $ -2 \bar{e}_τ^T \eta ≤ ||\bar{e}_τ||^2 + ||\eta||^2 $:

  $ \dot{V_3} ≤ -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 - \eta^T K_d \eta + \eta^T d(t) - \eta^T d_B \tanh\left(\frac{\eta}{\epsilon}\right) - K_τ \bar{e}_τ^T \bar{e}_τ + ||\eta||^2 + ||\bar{e}_τ||^2 $$

  $ = -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 - (\lambda_{\text{min}}(K_d) - 1) ||\eta||^2 - (K_τ - 1) ||\bar{e}_τ||^2 + \eta^T d(t) - \eta^T d_B \tanh\left(\frac{\eta}{\epsilon}\right) $$

Choose $ K_d $ and $ K_τ $ such that $ \lambda_{\text{min}}(K_d) > 1 $ and $ K_τ > 1 $, ensuring all quadratic terms are negative definite, and the disturbance term is bounded, making $ \dot{V_3} < 0 $ outside a small region.

---

### Final Expressions
**$ \dot{V_3} $**:

$$ \dot{V_3} = -K_x e_x^2 - \frac{K_θ}{K_y} e_θ^2 - \eta^T K_d \eta + \eta^T d(t) - \eta^T d_B \tanh\left(\frac{\eta}{\epsilon}\right) - \eta^T \bar{e}_τ + \bar{e}_τ^T \left( \dot{\tau} - \frac{1}{\gamma} (-\tau_{\text{real}} + a) \right) $$

**Control Input $ a $**:

$$ a = \tau_{\text{real}} + \gamma \left( \dot{\tau} + K_τ (\tau - \tau_{\text{real}}) + \eta \right) $$


where $ K_τ > 1 $, and $ \dot{\tau} $ is computed as:

$$ \dot{\tau} = \dot{Y_c} \hat{p} + Y_c \dot{\hat{p}} - K_d \dot{\eta} - d_B \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) + \frac{d}{dt} \begin{bmatrix} e_x \\ \frac{1}{K_y} e_θ \end{bmatrix} $$

- **$ \dot{Y_c} $**:
  $ Y_c = \begin{bmatrix} \dot{v}^d & 0 \\ 0 & \dot{\omega}^d \end{bmatrix} $$
  $ \dot{Y_c} = \begin{bmatrix} \ddot{v}^d & 0 \\ 0 & \ddot{\omega}^d \end{bmatrix} $$
  Where $ \ddot{v}^d $ and $ \ddot{\omega}^d $ are the second derivatives of the desired velocities, computable from $ v^d $ and $ \omega^d $.

- **$ Y_c \dot{\hat{p}} $**:
  $ \dot{\hat{p}} = -\Gamma_p Y_c^T \eta $$
  $ Y_c \dot{\hat{p}} = -Y_c \Gamma_p Y_c^T \eta $$

- **$ -K_d \dot{\eta} $**:
  $ \dot{\eta} = M_2^{-1} (\tau_{\text{real}} + d(t) - Y_c p) $$
  $ -K_d \dot{\eta} = -K_d M_2^{-1} (\tau_{\text{real}} + d(t) - Y_c \hat{p} - Y_c \Delta p) $$

- **$ -d_B \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) $**:
  $ \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) = \text{diag}\left(1 - \tanh^2\left(\frac{\eta_i}{\epsilon}\right)\right) \cdot \frac{1}{\epsilon} \dot{\eta} $$
  $ -d_B \frac{d}{dt} \tanh\left(\frac{\eta}{\epsilon}\right) = -d_B \cdot \text{diag}\left(1 - \tanh^2\left(\frac{\eta_i}{\epsilon}\right)\right) \cdot \frac{1}{\epsilon} M_2^{-1} (\tau_{\text{real}} + d(t) - Y_c p) $$

- **$ \frac{d}{dt} \begin{bmatrix} e_x \\ \frac{1}{K_y} e_θ \end{bmatrix} $**:

  $ \dot{e_x} = \omega e_y - v + v_r \cos e_θ $$

  $ \dot{e_θ} = \omega_r - \omega $$
  
  $ \frac{d}{dt} \begin{bmatrix} e_x \\ \frac{1}{K_y} e_θ \end{bmatrix} = \begin{bmatrix} \omega e_y - v + v_r \cos e_θ \\ \frac{1}{K_y} (\omega_r - \omega) \end{bmatrix} $$


