# Backstepping-Based Adaptive Controller for Nonholonomic Mobile Robots

This document describes the design and derivation of a backstepping-based adaptive controller for a nonholonomic mobile robot with unknown parameters (mass, inertia) and bounded external disturbances. The approach follows a two-stage design process leveraging Lyapunov stability theory.

## System Model

The robot is modeled as a unicycle-type mobile robot.

### Kinematic Model

The robot's pose is $(x, y, \theta)$, linear velocity is $v$, and angular velocity is $\omega$. The kinematic equations are:

$$
\dot{x} = v \cos \theta \\
\dot{y} = v \sin \theta \\
\dot{\theta} = \omega
$$

A reference trajectory is defined by $(x_r, y_r, \theta_r)$ with reference velocities $v_r(t), \omega_r(t)$. Tracking errors in the robot's local frame relative to a desired point $(x_d, y_d, \theta_d)$ are defined as $e_x$ (forward error), $e_y$ (lateral error), and $e_\theta$ (orientation error). The error dynamics are provided.

### Dynamic Model

The velocity dynamics are governed by:

$$
M_2 \dot{v} = \tau + d(t)
$$

where $v = [v, \omega]^T$ is the velocity vector, $\tau = [\tau_v, \tau_\omega]^T$ is the control input (generalized force and torque), and $d(t)$ is an unknown bounded disturbance, $||d(t)|| \le d_B$.

The mass-inertia matrix is $M_2 = \text{diag}(m, I)$, with unknown mass $m$ and inertia $I$. The parameter vector is $p = [m, I]^T$, with estimate $\hat{p} = [\hat{m}, \hat{I}]^T$ and estimation error $\Delta p = \hat{p} - p$.

The regressor matrix $Y_c$ is defined such that $M_2 \dot{v}^d = Y_c p$, with the specific form for this model being:

$$
Y_c = \begin{bmatrix} \dot{v}^d & 0 \\ 0 & \dot{\omega}^d \end{bmatrix}
$$

where $v^d = [v^d, \omega^d]^T$ is the desired velocity vector.

## Control Design (Backstepping Approach)

The controller is designed in two steps using backstepping.

### Step 1: Kinematic Controller Design

This step designs virtual control inputs $v^d$ and $\omega^d$ to stabilize the kinematic tracking errors ($e_x, e_y, e_\theta$). A Lyapunov function $V_1 = \frac{1}{2}(e_x^2 + e_y^2 + \frac{1}{K_y}e_\theta^2)$ is used. The kinematic control laws are chosen as:

$$
v^d = v_r \cos e_\theta + K_x e_x \\
\omega^d = \omega_r + K_\theta e_\theta + v_r e_y K_y \frac{\sin e_\theta}{e_\theta}
$$

where $K_x, K_\theta, K_y > 0$ are controller gains.

### Step 2: Dynamic Controller Design

This step designs the actual torque input $\tau$ to ensure that the actual velocities $v, \omega$ track the desired velocities $v^d, \omega^d$ from the kinematic layer, while estimating the unknown parameters $m, I$. The velocity tracking error is $\eta = v - v^d$.

A composite Lyapunov function is used:

$$
V = V_1 + \frac{1}{2}\eta^T M_2 \eta + \frac{1}{2}\Delta p^T \Gamma_p^{-1} \Delta p
$$

where $\Gamma_p$ is a positive definite adaptation gain matrix.

The adaptive law for parameter estimation is derived as:

$$
\dot{\hat{p}} = -\Gamma_p Y_c^T \eta
$$

The control law for the desired generalized force/torque $\tau_d$ is derived based on the Lyapunov analysis and includes a robust term:

$$
\tau_d = Y_c \hat{p} - K_d \eta - d_B \tanh \left( \frac{\eta}{\epsilon} \right) + \begin{bmatrix} e_x \\ \frac{1}{K_y} e_\theta \end{bmatrix}
$$

where $K_d$ is a positive definite gain matrix for velocity error feedback, $d_B$ is the assumed disturbance bound, and $\epsilon > 0$ is a small constant.

The document also considers actuator dynamics $\dot{\tau} = \frac{1}{\gamma}(-\tau + a)$ and derives a final control input $a = \tau_d + \gamma \dot{\tau}_d - \gamma K_\tau (\tau - \tau_d)$, where $\overline{e}_\tau = \tau - \tau_d$ is the torque tracking error and $K_\tau > 0$.

## Adaptive Law and Control Summary

The key components of the complete controller include:

-   Kinematic laws for $v^d, \omega^d$.
-   Velocity tracking error $\eta$.
-   Adaptive law $\dot{\hat{p}} = -\Gamma_p Y_c^T \eta$.
-   Torque reference (desired generalized force/torque) $\tau_d$.
-   Final control input $a$ (if actuator dynamics are considered).

## Stability Guarantee

Lyapunov stability theory is used throughout the derivation to guarantee the boundedness of all tracking errors ($e_x, e_y, e_\theta, \eta$) and parameter estimation errors ($\Delta p$). Under persistent excitation, Barbalat's Lemma is invoked to ensure that all errors converge to zero.