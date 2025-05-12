# Step-by-Step Backstepping Control Derivation for a Mobile Robot

## I. Introduction

This document details the derivation of a backstepping controller for a differential drive mobile robot, similar to the system architecture presented in the Python code. Backstepping is a systematic method for designing controllers for nonlinear systems in a cascaded (or strict-feedback) form. We will break down the robot control problem into two main steps:
1.  **Kinematic Control (Outer Loop)**: Designing desired linear and angular velocities ($v_{1d}, \omega_d$) to make the robot follow a path.
2.  **Dynamic Control (Inner Loop)**: Designing generalized forces (and subsequently motor torques) to make the robot's actual velocities ($v_1, \omega$) track the desired velocities from the kinematic loop.

For simplicity in this derivation, we will initially ignore parameter adaptation and the robustifying term, focusing on the core backstepping structure. These can be added in later stages of the design.

## II. System Model

### a. Kinematic Model

The robot's pose is $(x, y, \theta)$. The kinematic equations are:
$\dot{x} = v_1 \cos\theta$
$\dot{y} = v_1 \sin\theta$
$\dot{\theta} = \omega$

Here, $v_1$ is the robot's linear velocity along its longitudinal axis, and $\omega$ is its angular velocity. These ($v_1, \omega$) are treated as control inputs for the kinematic subsystem.

### b. Dynamic Model (Simplified)

The robot's dynamics relate the generalized forces to the accelerations:
$m \dot{v}_1 = \tau_{bar1} - d_1$
$I \dot{\omega} = \tau_{bar2} - d_2$

Where:
- $m$ is the mass, $I$ is the moment of inertia.
- $\tau_{bar1}$ is the generalized force along the robot's longitudinal axis.
- $\tau_{bar2}$ is the generalized torque around the robot's vertical axis.
- $d_1, d_2$ represent disturbances and unmodeled dynamics (we will ignore these for the initial derivation, i.e., $d_1=0, d_2=0$).

The actual control inputs are the wheel torques $\tau_L, \tau_R$, which are related to $\tau_{bar1}, \tau_{bar2}$ by the matrix $B_2$:
$$ \begin{bmatrix} \tau_{bar1} \\ \tau_{bar2} \end{bmatrix} = B_2 \begin{bmatrix} \tau_R \\ \tau_L \end{bmatrix} $$
And the controller will compute $\tau_{bar1}, \tau_{bar2}$, which are then converted to $\tau_R, \tau_L$.

## III. Backstepping Controller Design

### Step 1: Kinematic Controller Design (Outer Loop)

**Objective**: Make the robot follow a predefined path. We define errors relative to a target point $(x_t, y_t)$ on the path with a desired orientation $\theta_p$.

**Error Coordinates**: As in `LyapunovKinematicController`:
- $e_f = (x_t - x)\cos\theta + (y_t - y)\sin\theta$ (Forward error)
- $e_l = -(x_t - x)\sin\theta + (y_t - y)\cos\theta$ (Lateral error)
- $e_\theta = \theta_p - \theta$ (Orientation error)

The time derivatives of these errors (assuming $(x_t, y_t, \theta_p)$ change slowly or are quasi-static for this analysis step, which is a common simplification for path following):
$\dot{e}_f = \omega e_l - v_{1,ref} + v_1 \cos e_\theta + (\dot{x}_t - \dot{x}_{ref})\cos\theta + (\dot{y}_t - \dot{y}_{ref})\sin\theta$
$\dot{e}_l = -\omega e_f + v_1 \sin e_\theta - (\dot{x}_t - \dot{x}_{ref})\sin\theta + (\dot{y}_t - \dot{y}_{ref})\cos\theta$
$\dot{e}_\theta = \dot{\theta}_p - \omega$

For path following, often a simplified error dynamic is considered where $(x_t, y_t)$ is a point on the path and $\theta_p$ is the path tangent. If we assume $v_{1,ref}$ is the desired speed along the path, and $\dot{x}_t, \dot{y}_t, \dot{\theta}_p$ are properties of the reference path:
$\dot{e}_f \approx \omega e_l - v_{1,ref} \cos e_\theta + v_1$ (if $e_\theta$ is small, $\cos e_\theta \approx 1$, error in robot frame)
$\dot{e}_l \approx -\omega e_f + v_{1,ref} \sin e_\theta$
$\dot{e}_\theta = \omega_p - \omega$ (where $\omega_p = \dot{\theta}_p$ is the desired path curvature rate)

Let's use the error transformation as in many standard nonholonomic path-following works. Consider a target point $(x_d, y_d)$ on the path and the path angle $\theta_d$. The errors are:
$$ \begin{bmatrix} e_1 \\ e_2 \\ e_3 \end{bmatrix} = \begin{bmatrix} \cos\theta & \sin\theta & 0 \\ -\sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x_d - x \\ y_d - y \\ \theta_d - \theta \end{bmatrix} $$
So $e_1$ is along-axis error, $e_2$ is cross-axis error (lateral), $e_3$ is orientation error.
$\dot{e}_1 = \omega e_2 - v_d \cos e_3 + v_1 + \dot{x}_d \cos\theta + \dot{y}_d \sin\theta$
$\dot{e}_2 = -\omega e_1 + v_d \sin e_3 - \dot{x}_d \sin\theta + \dot{y}_d \cos\theta$
$\dot{e}_3 = \omega_d - \omega$
where $v_d$ and $\omega_d$ are the reference path's linear and angular velocity.

The kinematic controller in the code uses $(e_f, e_l, e_\theta)$ which are slightly different but serve a similar purpose. We'll stick to the code's error definitions for consistency with its output:
$e_f$: forward error to target point in robot frame.
$e_l$: lateral error to target point in robot frame.
$e_\theta$: orientation error wrt path tangent at target point.

The time derivatives are (assuming target point is fixed for a moment, or its velocity is small):
$\dot{e}_f \approx \omega e_l - v_1$ (if $v_1$ is velocity towards target)
$\dot{e}_l \approx -\omega e_f$
$\dot{e}_\theta \approx -\omega$

This is too simplified. Let's use the structure from a common nonholonomic controller (e.g., Kanayama or Samson):
Consider errors $(x_e, y_e, \theta_e)$ in a frame attached to the reference path point.
For path following, it's more common to define errors $e_l$ (lateral) and $e_\theta$ (orientation w.r.t path tangent).
Let $v_{ref}$ be the desired forward speed.
$\dot{e}_l = v_1 \sin e_\theta - v_{ref} \tan(e_\theta) \cdot (\text{if path is straight})$
If we use the errors from the code: $e_f, e_l, e_\theta$.
The control laws are:
$v_{1d} = v_{ref} \cos(e_\theta) + k_f e_f$
$\omega_d = (k_{tt} k_{lat\_factor}) v_{ref} \text{sinc}(e_\theta) e_l + k_{tt} e_\theta$

**Lyapunov Candidate for Kinematic Loop ($V_1$)**:
A common Lyapunov function for path following might involve $e_l$ and $e_\theta$. For instance:
$V_1 = \frac{1}{2} e_l^2 + \frac{1}{2\gamma} (e_\theta + \delta e_l)^2$ (this is just an example, not directly what the code uses but illustrates the principle).
Or more simply, related to the control objective:
$V_1 = \frac{1}{2} e_f^2 + \frac{1}{2} e_l^2 + \frac{1}{k_{tt}}(1 - \cos e_\theta)$ (inspired by some nonholonomic designs)

Let's assume the control laws $v_{1d}$ and $\omega_d$ are "virtual controls" designed such that if $v_1 = v_{1d}$ and $\omega = \omega_d$, then the errors $e_f, e_l, e_\theta$ converge to zero (or a small neighborhood). The `LyapunovKinematicController` name implies such a design. The specific derivation of $v_{1d}$ and $\omega_d$ from a Lyapunov function for the $(e_f, e_l, e_\theta)$ dynamics can be quite involved. The provided code gives these laws directly.

For the backstepping derivation, we *assume* $v_{1d}$ and $\omega_d$ are given by this first stage, and they are designed to stabilize the path following errors. Let these desired velocities be $\alpha_1 = v_{1d}$ and $\alpha_2 = \omega_d$.

### Step 2: Dynamic Controller Design (Inner Loop)

**Objective**: Make the actual velocities $v_1, \omega$ track the desired (virtual control) velocities $v_{1d}, \omega_d$ from Step 1.

**Error Variables (Velocity Tracking Errors)**:
Let $z_1 = v_1 - v_{1d}$ (error in linear velocity)
Let $z_2 = \omega - \omega_d$ (error in angular velocity)
In the code, these are $\eta_1$ and $\eta_2$. So, $\eta = [z_1, z_2]^T$.

**Time Derivatives of Velocity Errors**:
$\dot{z}_1 = \dot{v}_1 - \dot{v}_{1d}$
$\dot{z}_2 = \dot{\omega} - \dot{\omega}_{1d}$

From the dynamic model (ignoring $d_1, d_2$ for now):
$\dot{v}_1 = \frac{1}{m} \tau_{bar1}$
$\dot{\omega} = \frac{1}{I} \tau_{bar2}$

So,
$\dot{z}_1 = \frac{1}{m} \tau_{bar1} - \dot{v}_{1d}$
$\dot{z}_2 = \frac{1}{I} \tau_{bar2} - \dot{\omega}_{1d}$

**Lyapunov Candidate for the Augmented System ($V_2$)**:
We augment the (conceptual) Lyapunov function $V_1$ from the kinematic step with terms for the velocity errors $z_1, z_2$.
$V_2 = V_1 + \frac{1}{2} m z_1^2 + \frac{1}{2} I z_2^2$
The terms $\frac{1}{2} m z_1^2$ and $\frac{1}{2} I z_2^2$ represent the "kinetic energy" associated with the velocity errors, scaled by mass and inertia.

**Time Derivative of $V_2$**:
$\dot{V}_2 = \dot{V}_1 + m z_1 \dot{z}_1 + I z_2 \dot{z}_2$
$\dot{V}_2 = \dot{V}_1 + m z_1 (\frac{1}{m} \tau_{bar1} - \dot{v}_{1d}) + I z_2 (\frac{1}{I} \tau_{bar2} - \dot{\omega}_{1d})$
$\dot{V}_2 = \dot{V}_1 + z_1 (\tau_{bar1} - m \dot{v}_{1d}) + z_2 (\tau_{bar2} - I \dot{\omega}_{1d})$

**Designing Actual Control Inputs ($\tau_{bar1}, \tau_{bar2}$)**:
Our goal is to choose $\tau_{bar1}$ and $\tau_{bar2}$ to make $\dot{V}_2 \le 0$.
The term $\dot{V}_1$ involves $z_1$ and $z_2$ because the actual velocities $v_1 = z_1 + v_{1d}$ and $\omega = z_2 + \omega_d$ feed into the kinematic error dynamics. For example, if $\dot{e}_l = v_1 \sin e_\theta + \dots$, then $\dot{e}_l = (z_1+v_{1d}) \sin e_\theta + \dots$.
The terms from $\dot{V}_1$ that involve $z_1$ and $z_2$ are grouped with the $z_1(\dots)$ and $z_2(\dots)$ terms above.

Let's assume the kinematic controller was designed such that if $z_1=0$ and $z_2=0$ (i.e., $v_1=v_{1d}, \omega=\omega_d$), then $\dot{V}_1 \le -k_1 V_1$ (for some $k_1 > 0$) or at least is negative semi-definite. When $z_1, z_2 \neq 0$, $\dot{V}_1$ will have additional terms. For example, if $\dot{e}_f = \omega e_l - v_1$, then $\dot{V}_1$ might contain $e_f(\omega e_l - v_1) = e_f((z_2+\omega_d)e_l - (z_1+v_{1d}))$. The terms $e_f z_2 e_l$ and $-e_f z_1$ would appear.

The standard backstepping procedure is to choose the control input to cancel out "bad" terms and introduce stabilizing negative terms.
Let's select $\tau_{bar1}$ and $\tau_{bar2}$ as follows:
$\tau_{bar1} - m \dot{v}_{1d} = -k_{d1} z_1 - (\text{terms from } \dot{V}_1 \text{ multiplying } z_1)$
$\tau_{bar2} - I \dot{\omega}_{1d} = -k_{d2} z_2 - (\text{terms from } \dot{V}_1 \text{ multiplying } z_2)$

More simply, ignoring the cross terms from $\dot{V}_1$ for a moment (these are often handled by the kinematic controller's robustness or by ensuring $v_{1d}, \omega_d$ are sufficiently smooth):
Choose:
$\tau_{bar1} = m \dot{v}_{1d} - k_{d1} z_1$
$\tau_{bar2} = I \dot{\omega}_{1d} - k_{d2} z_2$
where $k_{d1} > 0$ and $k_{d2} > 0$ are design gains (these correspond to the diagonal elements of $K_d$ in the code).

Substituting these into the $\dot{V}_2$ expression:
$\dot{V}_2 = \dot{V}_1 (\text{evaluated with } v_1=z_1+v_{1d}, \omega=z_2+\omega_d) + z_1 (-k_{d1} z_1) + z_2 (-k_{d2} z_2)$
$\dot{V}_2 = \dot{V}_1' - k_{d1} z_1^2 - k_{d2} z_2^2$

If $V_1$ was designed such that $\dot{V}_1 \le 0$ when $v_1=v_{1d}$ and $\omega=\omega_d$, then $\dot{V}_1'$ (which now includes the effect of $z_1, z_2$ on kinematic errors) plus the terms $-k_{d1}z_1^2 - k_{d2}z_2^2$ can be made negative semi-definite. The terms $-k_{d1}z_1^2 - k_{d2}z_2^2$ are strictly negative if $z_1, z_2 \neq 0$, which helps stabilize the velocity tracking errors.

The control law in `AdaptiveDynamicController` (ignoring adaptation and robustness for this derivation) is:
$\tau_{bar\_cmd} = \text{Feedforward} - K_d \eta$
Here, $\eta = [z_1, z_2]^T$.
The feedforward term $Y_c \hat{p}$ corresponds to $[m \dot{v}_{1d,est}, I \dot{\omega}_{1d,est}]^T$ if $\hat{p} = [m, I]^T$ (true parameters) and $Y_c = \text{diag}(\dot{v}_{d,est})$.
The term $-K_d \eta$ corresponds to $[-k_{d1}z_1, -k_{d2}z_2]^T$ if $K_d = \text{diag}(k_{d1}, k_{d2})$.

So, the derived control law:
$\tau_{bar1,cmd} = m \dot{v}_{1d,est} - k_{d1} (v_1 - v_{1d})$
$\tau_{bar2,cmd} = I \dot{\omega}_{1d,est} - k_{d2} (\omega - \omega_d)$
matches the structure of the code's controller when true parameters $m, I$ are used instead of estimates $\hat{m}, \hat{I}$, and $\dot{v}_{1d,est}, \dot{\omega}_{1d,est}$ are the estimated derivatives of the virtual controls.

## IV. Incorporating Parameter Adaptation (Conceptual)

If $m$ and $I$ are unknown, they are replaced by their estimates $\hat{m}$ and $\hat{I}$.
The control law becomes:
$\tau_{bar1,cmd} = \hat{m} \dot{v}_{1d,est} - k_{d1} z_1$
$\tau_{bar2,cmd} = \hat{I} \dot{\omega}_{1d,est} - k_{d2} z_2$

The Lyapunov function $V_2$ is augmented with parameter error terms:
$V_{2,adaptive} = V_1 + \frac{1}{2} m z_1^2 + \frac{1}{2} I z_2^2 + \frac{1}{2\gamma_m} (m - \hat{m})^2 + \frac{1}{2\gamma_I} (I - \hat{I})^2$
(assuming $m,I$ are constant, so $\dot{m}=\dot{I}=0$).
$\dot{V}_{2,adaptive} = \dot{V}_1 + m z_1 \dot{z}_1 + I z_2 \dot{z}_2 - \frac{1}{\gamma_m}(m-\hat{m})\dot{\hat{m}} - \frac{1}{\gamma_I}(I-\hat{I})\dot{\hat{I}}$

Substitute $\dot{z}_1 = \frac{1}{m}\tau_{bar1} - \dot{v}_{1d}$ (using true $m$ here, which is tricky for design).
A key step in adaptive backstepping is to rewrite $m \dot{v}_{1d} = \hat{m} \dot{v}_{1d} + (m-\hat{m})\dot{v}_{1d}$.
$\dot{V}_{2,adaptive} = \dot{V}_1 + z_1(\tau_{bar1} - \hat{m}\dot{v}_{1d} - (m-\hat{m})\dot{v}_{1d}) + z_2(\tau_{bar2} - \hat{I}\dot{\omega}_{1d} - (I-\hat{I})\dot{\omega}_{1d}) - \frac{1}{\gamma_m}(m-\hat{m})\dot{\hat{m}} - \frac{1}{\gamma_I}(I-\hat{I})\dot{\hat{I}}$

Now, substitute the control law $\tau_{bar1} = \hat{m} \dot{v}_{1d,est} - k_{d1} z_1$ and $\tau_{bar2} = \hat{I} \dot{\omega}_{1d,est} - k_{d2} z_2$:
$\dot{V}_{2,adaptive} = \dot{V}_1' - k_{d1}z_1^2 - k_{d2}z_2^2 - z_1(m-\hat{m})\dot{v}_{1d} - z_2(I-\hat{I})\dot{\omega}_{1d} - \frac{1}{\gamma_m}(m-\hat{m})\dot{\hat{m}} - \frac{1}{\gamma_I}(I-\hat{I})\dot{\hat{I}}$

Group terms with $(m-\hat{m})$ and $(I-\hat{I})$:
$\dot{V}_{2,adaptive} = \dot{V}_1' - k_{d1}z_1^2 - k_{d2}z_2^2 - (m-\hat{m})(z_1\dot{v}_{1d} + \frac{1}{\gamma_m}\dot{\hat{m}}) - (I-\hat{I})(z_2\dot{\omega}_{1d} + \frac{1}{\gamma_I}\dot{\hat{I}})$

To make the terms involving parameter errors cancel or become helpful, choose adaptation laws:
$z_1\dot{v}_{1d} + \frac{1}{\gamma_m}\dot{\hat{m}} = 0 \implies \dot{\hat{m}} = -\gamma_m z_1 \dot{v}_{1d}$
$z_2\dot{\omega}_{1d} + \frac{1}{\gamma_I}\dot{\hat{I}} = 0 \implies \dot{\hat{I}} = -\gamma_I z_2 \dot{\omega}_{1d}$

This corresponds to the adaptation law $\dot{\hat{p}} = -\Gamma_p Y_c^T \eta$ if:
- $\hat{p} = [\hat{m}, \hat{I}]^T$
- $\Gamma_p = \text{diag}(\gamma_m, \gamma_I)$
- $Y_c = \text{diag}(\dot{v}_{1d}, \dot{\omega}_{1d})$ (the regressor)
- $\eta = [z_1, z_2]^T$

With these adaptation laws, $\dot{V}_{2,adaptive} = \dot{V}_1' - k_{d1}z_1^2 - k_{d2}z_2^2$, leading to stability of the tracking errors and boundedness of parameter errors.

## V. Summary of Control Laws

**Kinematic Controller (Virtual Controls)**:
$v_{1d} = v_{ref} \cos(e_\theta) + k_f e_f$
$\omega_d = (k_{tt} k_{lat\_factor}) v_{ref} \text{sinc}(e_\theta) e_l + k_{tt} e_\theta$

**Dynamic Controller (Actual Generalized Forces, using estimates $\hat{m}, \hat{I}$)**:
Let $z_1 = v_1 - v_{1d}$ and $z_2 = \omega - \omega_d$.
$\tau_{bar1,cmd} = \hat{m} \dot{v}_{1d,est} - k_{d1} z_1$
$\tau_{bar2,cmd} = \hat{I} \dot{\omega}_{1d,est} - k_{d2} z_2$

**Adaptation Laws**:
$\dot{\hat{m}} = -\gamma_m z_1 \dot{v}_{1d,est}$
$\dot{\hat{I}} = -\gamma_I z_2 \dot{\omega}_{1d,est}$

These derived laws match the structure implemented in the Python code's `AdaptiveDynamicController`, where $\dot{v}_{1d,est}$ and $\dot{\omega}_{1d,est}$ are the components of `vd_dot_est`, $z_1, z_2$ are components of `eta`, and $k_{d1}, k_{d2}$ are the diagonal elements of `Kd`. The term $Y_c \hat{p}$ in the code is $[\hat{m}\dot{v}_{1d,est}, \hat{I}\dot{\omega}_{1d,est}]^T$.

This step-by-step derivation shows how the backstepping methodology allows for a systematic construction of the controller, starting from the outermost kinematic objectives and progressively designing controllers for inner dynamic loops, ensuring stability at each stage through Lyapunov analysis.
