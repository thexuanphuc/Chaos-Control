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
    - $M_2(p) = \text{diag}(m, I)$ is the diagonal mass-inertia matrix with parameters $p = [m, I]^T$.
    - $\tau_{cmd} = [\tau_R, \tau_L]^T$ is the vector of commanded torques for the right and left wheels.
    - $B_2$ is the matrix transforming wheel torques to generalized force/torque acting on the robot body (related to $r$ and $W$).
    - $\tau_d$ is the vector representing external disturbances acting as equivalent generalized forces/torques.
- **Kinematic Model (for Pose Update):** The simulation uses the standard nonholonomic kinematic model for updating pose based on current velocities:
    ```latex
    \dot{x} = v_1 \cos(\theta) \\
    \dot{y} = v_1 \sin(\theta) \\
    \dot{\theta} = \omega
    ```
- **Integration:** Use Euler integration with a configurable step size ($\Delta t$) for both dynamics (to find $v(t+\Delta t)$) and kinematics (to find pose$(t+\Delta t)$).
- **Angle Normalization:** Normalize the orientation angle $\theta$ to the range $[-\pi, \pi]$ after each update.
- **Disturbance Modeling:**
    - Implement optional continuous random disturbance bounded by $d_B$.
    - Implement an optional trajectory-based "kick" disturbance: apply a configured disturbance magnitude for a specific duration when the robot gets within a certain distance of a target point on the desired path.

### Control System (Two-Layered)
- **Layer 1: Kinematic Guidance Controller (`LyapunovKinematicController`)**
    - Input: Current robot pose ($x, y, \theta$), desired path.
    - Output: Desired body-frame velocities ($v_1^d, \omega^d$).
    - Functionality:
        - Find a target point on the path using a lookahead distance.
        - Calculate kinematic errors in the robot frame (forward, lateral, orientation error $\theta_e = \theta_{path} - \theta$).
        - Compute desired velocities using a Lyapunov-based law:
            ```latex
            v_1^d = v_{ref} \cos(\theta_e) + k_f \cdot \text{error}_{\text{forward}} \\
            \omega^d = \omega_{ref} + k_\theta \theta_e + k_y v_{ref} \text{sinc}(\theta_e) \cdot \text{error}_{\text{lateral}}
            ```
            (Note: Check if $\omega_{ref}$ based on path curvature is needed/implemented. The provided code calculates $\omega^d$ using gains $k_f, k_\theta, k_y$ without an explicit path curvature term).
        - Include velocity and acceleration limits if applicable (e.g., $\omega_{max}$).
        - Implement a mechanism to detect path completion (`finished_flag`).

- **Layer 2: Adaptive Dynamic Controller (`AdaptiveDynamicController`)**
    - Input: Current robot state ($x, y, \theta, v_1, \omega$), desired path, desired velocities ($v_1^d, \omega^d$) from Layer 1.
    - Output: Wheel torques ($\tau_L, \tau_R$).
    - **Lyapunov Candidate Function (Basis for derivation):** [cite: 7]
        ```latex
        V = \frac{1}{2}\eta^T M_2 \eta + \frac{1}{2}\tilde{p}^T \Gamma_p^{-1} \tilde{p}
        ```
        where $\eta$ is the velocity tracking error and $\tilde{p} = \hat{p} - p$ is the parameter estimation error.
    - Functionality:
        - Calculate velocity tracking error: $\eta = v - v^d = [v_1 - v_1^d, \omega - \omega^d]^T$.
        - Estimate the derivative of desired velocities $\dot{v}^d$ (e.g., using finite differences).
        - Calculate the regressor matrix $Y_c = \text{diag}(\dot{v}_1^d, \dot{\omega}^d)$ such that $M_2 \dot{v}^d \approx Y_c p$. [cite: 6]
        - Maintain adaptive estimates $\hat{p} = [\hat{m}, \hat{I}]^T$ for the unknown parameters $p = [m, I]^T$.
        - Implement the adaptive law to update estimates: [cite: 9, 11]
            ```latex
            \dot{\hat{p}} = -\Gamma_p Y_c^T \eta
            ```
            where $\Gamma_p$ is a diagonal matrix of positive adaptation gains. Use Euler integration for the update.
        - Apply configurable bounds (min/max) to the parameter estimates $\hat{p}$.
        - Implement the control law to compute wheel torques: [cite: 9, 11]
            ```latex
            \tau_{bar} = Y_c \hat{p} - K_d \eta - u_{robust} \\
            [\tau_R, \tau_L]^T = B_2^{-1} \tau_{bar}
            ```
            where $K_d$ is a positive-definite gain matrix for velocity error feedback, and $u_{robust}$ is an optional robust term.
        - Implement the robust term (optional): [cite: 10, 11]
            ```latex
             u_{robust} = d_B \cdot \tanh(\eta / \epsilon)
            ```
            where $d_B$ is the assumed disturbance bound and $\epsilon$ is a small constant.

## Visualization Requirements

### Animated Visualization (`RobotMovementVisualizer`)
- Create a class for animated visualization suitable for Colab (e.g., using `matplotlib.animation`).
- Support export to GIF.
- **Main Path Plot:**
    - Display the desired path (dashed line).
    - Display the actual robot path (solid line).
    - Show the robot's current position with a marker (e.g., circle).
    - Indicate the robot's orientation (e.g., using an arrow or oriented marker).
    - Show velocity vector (optional, e.g., another arrow).
    - Update plot limits dynamically or based on the entire path extent.
    - Display current simulation time.
- **Time-Series Subplots:** Simultaneously animate plots of:
    - Kinematic Errors (forward, lateral, orientation) vs. time.
    - Velocity Errors ($\eta_1, \eta_\omega$) vs. time.
    - Parameter Estimates ($\hat{m}, \hat{I}$) vs. time, including lines for true values ($m, I$).
    - Commanded Wheel Torques ($\tau_L, \tau_R$) vs. time.
    - Applied Disturbances ($\tau_{d,v1}, \tau_{d,\omega}$) vs. time.
- Make animation speed (frame interval) and frame skipping (data steps per frame) configurable.

### Static Visualization (`plot_final_results`)
- Generate a multi-panel figure showing the final results after the simulation completes.
- Include plots for:
    - Kinematic errors vs. time.
    - Velocity errors ($\eta$) vs. time.
    - Parameter estimates ($\hat{p}$) vs. time (with true values).
    - Commanded torques ($\tau_L, \tau_R$) vs. time.
    - Applied disturbances vs. time.
- Ensure plots have titles, labels, legends, and grids.

## Path Generation and Simulation

### Path Generation (`generate_path`)
- Implement a function to generate various path types based on an index or name. Paths should be represented as NumPy arrays of $(x, y)$ coordinates. Support at least:
    - **Circle (1):** $x = R \cos(t)$, $y = R \sin(t)$
    - **Ellipse (2):** $x = R_x \cos(t)$, $y = R_y \sin(t)$
    - **Spiral (3):** $x = a t \cos(t)$, $y = a t \sin(t)$
    - **Line (4):** $x = t$, $y = m x + b$ (or defined by two points)
    - **Lemniscate (5):** $x = a \cos(t) / (1 + \sin^2(t))$, $y = a \sin(t)\cos(t) / (1 + \sin^2(t))$
    - **Sine Wave (6):** $x = t$, $y = A \sin(\omega x)$
    - **Heart (7):** $x =スケール (16 \sin^3(t))$, $y = スケール (13 \cos(t) - 5 \cos(2t) - 2 \cos(3t) - \cos(4t))$ (adjust scale)
    - **Square Wave (8):** (Defined piecewise or using periodic functions)
    - **Parabola (9):** $x = t$, $y = a x^2$
    - **Complex (10):** $x = t \cos(t) + A \sin(B t)$, $y = t \sin(t) + A \cos(C t)$ (example)

### Simulation Framework (`main.py` structure)
- Implement a main script that:
    - Sets up simulation parameters ($\Delta t$, max steps, path type, initial state/pose offsets).
    - Defines robot physical parameters ($m, I, r, W$).
    - Configures disturbance settings (continuous level, kick parameters).
    - Initializes the desired path using `generate_path`.
    - Initializes the robot `Simulation` environment.
    - Initializes the `LyapunovKinematicController` and `AdaptiveDynamicController` with appropriate gains ($\Gamma_p, K_d, k_f, k_\theta$, etc.) and initial parameter estimates $\hat{p}(0)$.
    - Initializes the `Visualizer`.
    - Runs the simulation loop:
        - Get current state from `Simulation`.
        - Compute desired velocities ($v^d$) using the kinematic controller.
        - Compute wheel torques ($\tau_{cmd}$) using the adaptive controller.
        - Handle controller finish condition (set torques to zero).
        - Execute the command in the `Simulation` environment.
        - Store relevant data (pose, velocities, accelerations, torques, errors, estimates, disturbances) at each step.
        - Check for termination conditions.
    - Collect final data.
    - Generate and display/save the animated visualization.
    - Generate and display the static results plots.
    - Include basic timing and progress reporting.

## Technical Implementation Details

### Required Libraries
- NumPy for numerical operations.
- Matplotlib for visualization.
### Code Quality
- Write clean, well-documented code with clear class and function definitions.
- Include type hinting where appropriate.
- Implement basic error handling (e.g., for path generation, file saving).
### Performance Considerations
- Ensure efficient NumPy usage.
- Allow configuration of animation parameters (`interval`, `step`) for performance tuning on different systems.
- Handle potential numerical issues (e.g., division by zero in `sinc`, overflows/underflows in `tanh`).

## Expected Output
- A Python script runnable in Colab.
- An animated visualization (HTML5/JS inline or GIF) showing the robot tracking the selected path along with key time-series data.
- Static plots clearly displaying the performance metrics (errors, estimates, torques, disturbances) over time after simulation completion.
- Console output indicating simulation progress and completion time.