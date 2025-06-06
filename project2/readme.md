# Project 2 Defense: Adaptive Dynamic Control

![Robot Plant Diagram](photo_5474317598851200123_x.jpg)

## 1. System Definition

### 1.1. Variable Definition
* **State ($s$):** Define the state variables of the system.
    * From `Simulation.py`: Robot pose $(x, y, \theta)$ and velocities $(v_1, \omega)$.
    * So, $s = [x, y, \theta, v_1, \omega]^T$.
* **Control Action ($a$):** Define the control inputs.
    * From `Simulation.py` and `Controller.py`: Left and right wheel torques $(\tau_L, \tau_R)$.
    * Generalized input vector $\tau = [\tau_{F}, \tau_{\theta}]^T$ where $\tau_F$ is effective linear force and $\tau_{\theta}$ is effective angular torque[cite: 1, 3].
    * Relationship: $\tau = B_2 \begin{bmatrix} \tau_R \\ \tau_L \end{bmatrix}$.
    * So, $a = [\tau_L, \tau_R]^T$.
* **Actual Velocity Vector ($v$):**
    * $v = [v_1, \omega]^T$ (linear and angular velocity)[cite: 1].
* **Desired Velocity Vector ($v^d$):**
    * $v^d = [v_1^d, \omega^d]^T$ (from kinematic controller)[cite: 6].

### 1.2. Definition of Unknown Parameters ($\theta$)
* **Parameters ($p$):** Define the parameters assumed to be unknown.
    * From `adaptive_dynamic_controller.pdf`: Robot mass ($m$) and inertia ($I$)[cite: 4].
    * The disturbance $d(t)$ is also unknown but bounded[cite: 3, 4].
    * So, $p = [m, I]^T$.
* **Parameter Estimates ($\hat{p}$):**
    * $\hat{p} = [\hat{m}, \hat{I}]^T$ are the adaptive estimates[cite: 5]. Corresponds to `p_hat` in `Controller.py`.
* **Estimation Error ($\tilde{p}$):**
    * $\tilde{p} = \hat{p} - p$[cite: 5].

### 1.3. Drawing
* Include a drawing of the plant (robot).
* Label variables: $x, y, \theta, v_1, \omega$.
* Show control inputs: $\tau_L, \tau_R$.
* Indicate coordinate frames.
* *(Placeholder: Insert drawing here)*

### 1.4. Equations
* **Kinematic Model:** (How pose changes with velocities)
    * $\dot{x} = v_1 \cos(\theta)$
    * $\dot{y} = v_1 \sin(\theta)$
    * $\dot{\theta} = \omega$
    (Implicit in `Simulation.py` state update)
* **Dynamic Model:** (How velocities change with torques)
    * $M_2(p)\dot{v} = \tau_{gen} + d(t)$ [cite: 1]
    * Where $M_2(p) = \text{diag}(m, I)$ is the mass-inertia matrix[cite: 1, 4].
    * $v = [v_1, \omega]^T$ is the actual velocity vector[cite: 1].
    * $\dot{v}$ is the acceleration vector[cite: 2].
    * $\tau_{gen}$ is the generalized input vector derived from wheel torques $a = [\tau_L, \tau_R]^T$. Corresponds to `tau_bar` in `Controller.py` before conversion to wheel torques. $\tau_{gen} = B_2 a$. The matrix $B_2$ is defined in `Simulation.py`.
    * $d(t)$ is the unknown bounded disturbance vector[cite: 1, 3].

### 1.5. System Behaviour Without Control
* Describe how the system behaves with zero control input ($a=0$).
* Show simulation plots or describe expected behavior (e.g., constant velocity, drift due to initial conditions).
* *(Placeholder: Add description/plots)*

### 1.6. Control Boundaries
* Define any limits on control actions ($a$).
    * e.g., Max/min torque: $|\tau_L| \le \tau_{max}$, $|\tau_R| \le \tau_{max}$. (Mentioned as optional in `Controller.py` comments).
* Define any limits on state variables ($s$) if applicable.
* Define bounds on unknown parameters ($p$) or disturbances $d(t)$.
    * $||d(t)|| \le d_B$ (Assumed disturbance bound)[cite: 10]. Implemented as `disturbance_bound` / `controller_dB`.
    * Parameter estimate bounds `min_params`, `max_params` are used in `Controller.py`.

## 2. Problem Definition

### 2.1. Goal
* State the main control objective.
    * Objective: Make the robot velocity vector $v$ track a desired velocity vector $v^d$ generated by a kinematic controller, such that the robot follows a predefined path, despite unknown parameters $p = [m, I]^T$ and disturbances $d(t)$.
    * Minimize the velocity tracking error $\eta = v - v^d$[cite: 7].

### 2.2. How Previous Methods Fail
* Explain limitations of non-adaptive methods (e.g., fixed-gain controllers) when parameters are unknown or vary, or when disturbances are present.
* Show plots/animations comparing a non-adaptive controller (if available) or theoretical performance degradation.
    * *(Placeholder: Add explanation and supporting plots/animations)*

## 3. Lyapunov Analysis

### 3.1. Candidate Function
* Define the Lyapunov candidate function $V$.
    * $V = \frac{1}{2}\eta^T M_2 \eta + \frac{1}{2}\tilde{p}^T \Gamma_p^{-1} \tilde{p}$ [cite: 7]
    * $\eta = v - v^d$ is the velocity tracking error[cite: 7]. Corresponds to `eta` in `Controller.py`.
    * $M_2 = \text{diag}(m, I)$[cite: 4].
    * $\tilde{p} = \hat{p} - p$ is the parameter estimation error[cite: 5].
    * $\Gamma_p$ is a positive-definite adaptation gain matrix[cite: 7]. Corresponds to `gamma_p` in `Controller.py`.

### 3.2. Proof of Stability (Time Derivative $\dot{V}$)
* Show the steps for calculating the time derivative $\dot{V}$.
    * $\dot{V} = \eta^T M_2 \dot{\eta} + \tilde{p}^T \Gamma_p^{-1} \dot{\tilde{p}}$[cite: 9].
    * $\dot{V} = \eta^T(M_2\dot{v} - M_2\dot{v}^d) + \tilde{p}^T \Gamma_p^{-1} \dot{\hat{p}}$ (since $\dot{\tilde{p}} = \dot{\hat{p}}$ as $p$ is constant)[cite: 9].
    * Substitute dynamics $M_2\dot{v} = \tau_{gen} + d(t)$:
        $\dot{V} = \eta^T(\tau_{gen} + d(t) - M_2\dot{v}^d) + \tilde{p}^T \Gamma_p^{-1} \dot{\hat{p}}$[cite: 9].
    * Define the regressor $Y_c = \text{diag}(\dot{v}_1^d, \dot{\omega}^d)$ such that $M_2 \dot{v}^d = Y_c p$[cite: 6]. Corresponds to `Yc` in `Controller.py`.
    * $\dot{V} = \eta^T\tau_{gen} + \eta^T d(t) - \eta^T Y_c p + \tilde{p}^T \Gamma_p^{-1} \dot{\hat{p}}$[cite: 9].
    * Use $p = \hat{p} - \tilde{p}$: $\eta^T Y_c p = \eta^T Y_c \hat{p} - \eta^T Y_c \tilde{p}$[cite: 9].
    * $\dot{V} = \eta^T\tau_{gen} - \eta^T Y_c \hat{p} + \eta^T d(t) + \eta^T Y_c \tilde{p} + \tilde{p}^T \Gamma_p^{-1} \dot{\hat{p}}$[cite: 9].
* *(Placeholder: Add detailed steps if required)*

### 3.3. Adaptation Derivation
* Derive the adaptation law for $\dot{\hat{p}}$.
    * Choose the adaptive law to cancel terms involving $\tilde{p}$:
        $\tilde{p}^T \Gamma_p^{-1} \dot{\hat{p}} + \eta^T Y_c \tilde{p} = 0$.
    * This leads to $\dot{\hat{p}} = -\Gamma_p Y_c^T \eta$[cite: 11].
    * Implemented in `Controller.py` as `p_hat_dot = -self.Gamma_p @ Yc.T @ eta`.

### 3.4. Control Derivation
* Derive the control law $\tau_{gen}$.
    * Substitute the adaptation law into the $\dot{V}$ equation:
        $\dot{V} = \eta^T\tau_{gen} - \eta^T Y_c \hat{p} + \eta^T d(t)$.
    * Choose the control law $\tau_{gen}$ to make $\dot{V}$ negative semi-definite or bounded.
    * Control Law: $\tau_{gen} = Y_c \hat{p} - K_d \eta - u_{robust}$[cite: 11].
        * $Y_c \hat{p}$: Feedforward term using estimated parameters.
        * $K_d \eta$: Feedback term based on velocity error $\eta$. $K_d$ is a positive-definite gain matrix[cite: 11]. Corresponds to `Kd` in `Controller.py`.
        * $u_{robust}$: Robust term to handle disturbances[cite: 11].
    * Substituting $\tau_{gen}$ into $\dot{V}$:
        $\dot{V} = \eta^T(Y_c \hat{p} - K_d \eta - u_{robust}) - \eta^T Y_c \hat{p} + \eta^T d(t)$
        $\dot{V} = -\eta^T K_d \eta - \eta^T u_{robust} + \eta^T d(t)$.
    * Choose the robust term $u_{robust} = d_B \cdot \tanh(\frac{\eta}{\epsilon})$ where $||d(t)|| \le d_B$[cite: 10]. Corresponds to `u_robust` in `Controller.py`.
    * This leads to $\dot{V} \le -\eta^T K_d \eta$, ensuring stability (Global Uniform Ultimate Boundedness)[cite: 10].
    * Final Control Law (Generalized Torque): $\tau_{gen} = Y_c \hat{p} - K_d \eta - d_B \cdot \tanh(\frac{\eta}{\epsilon})$[cite: 11]. Corresponds to `tau_bar` in `Controller.py`.
    * Actual Wheel Torques ($a = [\tau_L, \tau_R]^T$): Calculated using $\tau_{gen} = B_2 a \implies a = B_2^{-1} \tau_{gen}$. Implemented using `B2_inv` in `Controller.py`.

## 4. Experiments Definition

### 4.1. Initial Conditions
* Robot initial pose $(x_0, y_0, \theta_0)$. (Set in `main.py`, slightly offset from path start).
* Robot initial velocity $(v_{1,0}, \omega_0)$. (Set to $[0, 0]$ in `main.py`).

### 4.2. Reference State / Trajectory
* Specify the desired path.
    * Multiple path types generated by `generate_path` in `main.py` (Circle, Ellipse, Sine Wave, Heart, etc.).
    * Reference velocity $v^d$ is generated online by the `LyapunovKinematicController` aiming for a reference speed `v_ref` along the path.

### 4.3. Initial Estimate ($\hat{p}_0$)
* Specify the initial guess for the unknown parameters $\hat{p}(0) = [\hat{m}_0, \hat{I}_0]^T$.
    * Set via `initial_p_hat` in `main.py`, often chosen as a percentage of true values.

### 4.4. Simulation Time
* Total simulation time or number of steps.
    * Set by `max_steps` and `dt` in `main.py`. Simulation can end early if `finished_flag` is set by the kinematic controller.

### 4.5. Simulation Parameters
* Specify gains: $K_d$, $\Gamma_p$. (Set in `main.py`).
* Specify disturbance bound used in controller: $d_B$. (Set as `controller_dB` in `main.py`).
* Specify actual disturbance characteristics in simulation: $d(t)$. (Set by `disturbance_level` and kick parameters in `main.py`/`Simulation.py`).
* Specify true system parameters: $m, I$. (Set as `robot_mass`, `robot_inertia` in `main.py`).
* Specify kinematic controller parameters: $k_f, k_{\theta}, v_{ref}$, etc. (Set in `main.py`).

## 5. Results

### 5.1. General Code Structure
* Briefly describe the code modules.
    * `main.py`: Sets up parameters, initializes simulation, controllers, visualizer, runs simulation loop, calls plotting/animation.
    * `Simulation.py`: Defines robot dynamics, simulates state evolution given torque commands, applies disturbances.
    * `Controller.py`: Implements `LyapunovKinematicController` (generates $v^d$) and `AdaptiveDynamicController` (calculates torques $a$ using adaptive law).
    * `Visualizer.py`: Creates plots and animations of the simulation results.

### 5.2. Plots
* Provide plots generated by `Visualizer.py`. Ensure all requirements are met:
    * **Requirement:** Each plot named (Titles are set in `Visualizer.py`).
    * **Requirement:** Each axis named (Labels are set in `Visualizer.py`).
    * **Requirement:** Each line named (Legends are generated in `Visualizer.py`).
    * **Plots:**
        * State variables ($x, y, \theta$) / positions and velocities variables ($v_1, \omega$) over time.
            * Path plot shows $(x, y)$ trajectory vs desired path.
            * Velocity plot shows $v_1, \omega$ vs $v_1^d, \omega^d$ over time.
        * Control variables ($a = [\tau_L, \tau_R]^T$) over time.
        * Parameters estimation ($\hat{p} = [\hat{m}, \hat{I}]^T$) over time (compared to true values $m, I$).
        * Velocity errors $\eta = v - v^d$ over time.
        * Kinematic errors (forward, lateral, theta) over time.
        * Disturbances applied over time.
    * *(Placeholder: Insert plots generated from simulation runs)*
    * **Requirement:** Phase plots where applicable.
        * Example: Plot $\omega$ vs $v_1$.
        * *(Placeholder: Add phase plots if relevant/generated)*
    * **Requirement:** Gradient for trajectory line to show time (Not explicitly implemented in `Visualizer.py`, but the animation shows time evolution).

### 5.3. Animations
* Provide animations generated by `Visualizer.py`.
    * Animation shows robot movement, path tracking, and evolution of key variables over time.
    * *(Placeholder: Insert animation (e.g., GIF) or link to it)*

## 6. Conclusion
* Summarize the results.
* Discuss the effectiveness of the adaptive controller.
* Mention any challenges or future work.
## 7. Pseudo Code

### Overview
This pseudo code outlines the steps for a robot path-following simulation, including:
- Parameter initialization
- Path generation
- Kinematic and adaptive controllers
- Error handling
- Data collection
- Visualization of results

### Pseudo Code
```plaintext
START TRY
    // Initialize simulation parameters
    DEFINE simulation_parameters (time_step, max_steps, robot_properties, path_type, controller_gains)

    // Generate path
    GENERATE desired_path based on path_type
    IF path_generation fails THEN
        THROW error "Path generation failed"
    END IF

    // Initialize components
    INITIALIZE simulation_environment with robot_state (position, velocity, orientation) and robot_dynamics
    INITIALIZE kinematic_controller with controller_gains
    INITIALIZE adaptive_controller with controller_gains
    INITIALIZE visualizer with visualization_settings

    // Initialize data storage
    INITIALIZE data_storage for states, torques, errors, timestamps

    // Main simulation loop
    LOOP until max_steps reached OR controller signals completion:
        // Get current state
        GET current_state (position, velocity, orientation) from simulation_environment
        IF state_invalid THEN
            THROW error "Invalid robot state"
        END IF

        // Calculate control commands
        CALCULATE desired_velocities using kinematic_controller based on current_state and desired_path
        CALCULATE control_torques using adaptive_controller based on desired_velocities and current_state

        // Apply controls and update simulation
        APPLY control_torques to simulation_environment
        UPDATE robot_state in simulation_environment considering dynamics and disturbances
        IF simulation_update fails THEN
            THROW error "Simulation update failed"
        END IF

        // Store data
        STORE in data_storage:
            current_state
            control_torques
            tracking_errors
            current_timestamp

        // Check completion
        IF controller signals completion THEN
            BREAK loop
        END IF
    END LOOP

    // Process and visualize results
    PROCESS simulation_data from data_storage
- Data collection
- Visualization of results

The code includes robust error handling and clear steps for simulation setup, execution, and cleanup.

