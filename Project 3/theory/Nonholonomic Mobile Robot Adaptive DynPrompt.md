# Nonholonomic Mobile Robot Adaptive Dynamic Control Simulation

Develop a comprehensive Python simulation for a nonholonomic mobile robot (differential drive model) using an adaptive dynamic controller for path following, layered on top of a kinematic guidance controller. The simulation should be implemented for potential use in environments like Google Colab.

## Core System Requirements

### Robot Model (Simulation Environment)
- Implement a `Simulation` class modeling a differential drive mobile robot.
- State vector should include pose ($x, y, \theta$) and body-frame velocities (forward $v_1$, angular $\omega$).
- Physical parameters: wheel radius ($r$), wheel width ($W$), robot mass ($m$), robot moment of inertia ($I$).
- **Dynamic Model:** The simulation must use the second-order dynamic model described by $M_2(p)\dot{v} = B_2 \tau_{cmd} - \tau_d$, where:
    - $v = [v_1, \omega]^T$ is the vector of actual body-frame velocities.
    - $\dot{v}$ is the vector of body-frame accelerations.
    - $M_2(p) = \mathrm{diag}(m, I)$ is the diagonal mass-inertia matrix with parameters $p = [m, I]^T$.
    - $\tau_{cmd} = [\tau_R, \tau_L]^T$ is the vector of commanded torques for the right and left wheels.
    - $B_2$ is the matrix transforming wheel torques to generalized force/torque acting on the robot body (based on $r$ and $W$).
    - $\tau_d$ is the vector representing external disturbances acting as equivalent generalized forces/torques.
- **Kinematic Model (for Pose Update):**
    ```latex
    \dot{x} = v_1 \cos(\theta) \\
    \dot{y} = v_1 \sin(\theta) \\
    \dot{\theta} = \omega
    ```
- **Integration:** Use Euler integration with a configurable time step ($\Delta t$) for both dynamic and kinematic updates.
- **Angle Normalization:** Normalize orientation $\theta$ to $[-\pi, \pi]$ after each update.
- **Disturbance Modeling:**
  - Continuous random disturbance bounded by $d_B$.
  - Optional trajectory-based "kick" disturbance applied when within a set radius of a path point.

### Control System (Two-Layered)

#### Layer 1: Kinematic Guidance Controller (`LyapunovKinematicController`)
- Input: Current robot pose ($x, y, \theta$) and desired path.
- Output: Desired body-frame velocities ($v_1^d, \omega^d$).
- Functionality:
  - Find target point on path using lookahead distance.
  - Compute kinematic errors:
    - Forward error: projection of $(target - pose)$ onto robot X-axis.
    - Lateral error: projection onto Y-axis.
    - Orientation error: $\theta_e = \mathrm{normalizeAngle}(\theta_{path} - \theta)$.
  - Calculate:
    ```latex
    v_1^d = v_{ref}\cos(\theta_e) + k_f\cdot e_{forward} \\
    \omega^d = k_y v_{ref} \mathrm{sinc}(\theta_e)\cdot e_{lateral} + k_\theta\theta_e
    ```
  - Clip $\omega^d$ within $[-\omega_{max}, \omega_{max}]$.
  - Set `finished_flag` when near final path point.

#### Layer 2: Adaptive Dynamic Controller (`AdaptiveDynamicController`)
- Input: Current state ($x, y, \theta, v_1, \omega$), desired velocities ($v_1^d, \omega^d$), adaptation gains.
- Output: Wheel torques ($\tau_L, \tau_R$).
- **Lyapunov Candidate Function:**
  ```latex
  V = \frac12\eta^T M_2 \eta + \frac12\tilde{p}^T \Gamma_p^{-1} \tilde{p}
  ```
- **Adaptive Law:**
  ```latex
  \dot{\hat{p}} = -\Gamma_p Y_c^T \eta
  ```
  Where $Y_c = \mathrm{diag}(\dot{v}^d_1,\dot{\omega}^d)$.
- **Control Law:**
  ```latex
  \tau_{bar} = Y_c \hat{p} - K_d\,\eta - u_{robust} \\
  [\tau_R,\tau_L]^T = B_2^{-1}\tau_{bar}
  ```
- **Robust Term (optional):**
  ```latex
  u_{robust} = d_B\,\tanh(\eta/\epsilon)
  ```

### Visualization Requirements

#### Animated Visualization (`Visualizer.create_animation`)
- Display desired vs actual path.
- Show robot marker and orientation arrow.
- Animate time-series plots for:
  - Kinematic errors.
  - Velocity errors ($\eta$).
  - Parameter estimates ($\hat{m},\hat{I}$).
  - Commanded torques.
  - Disturbances.
- Configurable interval and frame step.

#### Static Visualization (`Visualizer.plot_final_results`)
- Multi-panel figure with:
  - Kinematic errors vs time.
  - Velocity errors vs time.
  - Parameter estimates vs time (include true values).
  - Commanded torques vs time.
  - Disturbances vs time.
- Titles, labels, legends, grids.

### Path Generation (`generate_path`)
- Support path types: Circle, Ellipse, Spiral, Line, Lemniscate, Sine Wave, Heart, Square Wave, Parabola, Complex.
- Return NumPy array of $(x, y)$ coordinates.

### Simulation Framework (`mainbackstepping.py`)
- Set parameters ($\Delta t$, max steps, path type, initial pose).
- Initialize `Simulation`, `LyapunovKinematicController`, `AdaptiveDynamicController`, and `Visualizer`.
- Run loop: get state, compute velocities, compute torques, execute command, store data, check termination.
- Post-process data: get simulation data, adjust lengths, create animation, plot results.
- Display or save outputs.

---

## Technical Details
- Python 3.7+ with NumPy, Matplotlib.
- Use type hints and clean class/function docstrings.
- Handle integration, normalization, and disturbances robustly.
- Allow configuration via parameters in scripts.

## Expected Output
- Runnable script in Colab or local environment.
- Animated visualization (HTML5 or GIF).
- Static multi-panel plots of performance metrics.
- Console logs for progress and timing.
