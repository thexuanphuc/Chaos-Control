import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Simulation Parameters
dt = 0.05  # Time step
sim_time = 20  # Total simulation time
steps = int(sim_time / dt)

# Control Gains
Kx = 1.5  # Forward error gain
Ky = 3    # Lateral error gain
Ktheta = 0.1  # Orientation error gain

# Trajectory Generation Functions
def generate_circular_path(radius=2.0, angular_speed=0.3, total_time=20, dt=0.05):
    t_vals = np.arange(0, total_time, dt)
    phi = angular_speed * t_vals
    x_path = radius * np.cos(phi)
    y_path = radius * np.sin(phi)
    vx = -radius * angular_speed * np.sin(phi)
    vy = radius * angular_speed * np.cos(phi)
    theta_path = np.arctan2(vy, vx)
    path = np.vstack((x_path, y_path, theta_path)).T
    vr = radius * angular_speed
    wr = angular_speed
    return path, vr, wr

def generate_straight_path(speed=1.0, total_time=20, dt=0.05):
    t_vals = np.arange(0, total_time, dt)
    x_path = speed * t_vals
    y_path = np.zeros_like(t_vals)
    theta_path = np.zeros_like(t_vals)
    path = np.vstack((x_path, y_path, theta_path)).T
    vr = speed
    wr = 0
    return path, vr, wr

def generate_figure_eight(a=2.0, omega=0.3, total_time=20, dt=0.05):
    t_vals = np.arange(0, total_time, dt)
    x_path = a * np.sin(omega * t_vals)
    y_path = a * np.sin(2 * omega * t_vals)
    vx = a * omega * np.cos(omega * t_vals)
    vy = 2 * a * omega * np.cos(2 * omega * t_vals)
    theta_path = np.arctan2(vy, vx)
    path = np.vstack((x_path, y_path, theta_path)).T
    vr = np.sqrt(vx**2 + vy**2).mean()  # Average speed
    wr = omega
    return path, vr, wr

def generate_sine_wave(A=1.0, omega=1.0, v=1.0, total_time=20, dt=0.05):
    """Generate a sine wave trajectory."""
    t_vals = np.arange(0, total_time, dt)
    x_path = v * t_vals  # Linear motion in x
    y_path = A * np.sin(omega * t_vals)  # Sinusoidal motion in y
    vx = v * np.ones_like(t_vals)  # Velocity in x
    vy = A * omega * np.cos(omega * t_vals)  # Velocity in y
    theta_path = np.arctan2(vy, vx)  # Desired orientation
    path = np.vstack((x_path, y_path, theta_path)).T
    vr = np.sqrt(vx**2 + vy**2).mean()  # Average linear velocity
    wr = 0  # No constant curvature
    return path, vr, wr

# Control Function
def compute_control(x, y, theta, x_goal, y_goal, theta_goal, vr, wr):
    """Compute control inputs (v, omega) based on position error."""
    dx = x_goal - x
    dy = y_goal - y
    e_theta = (theta_goal - theta + np.pi) % (2 * np.pi) - np.pi  # Angle error
    e_x = np.cos(theta) * dx + np.sin(theta) * dy  # Forward error
    e_y = -np.sin(theta) * dx + np.cos(theta) * dy  # Lateral error
    v = vr * np.cos(e_theta) + Kx * e_x  # Linear velocity
    omega = wr + Ktheta * e_theta + vr * Ky * e_y  # Angular velocity
    return v, omega

# Simulation Function
def simulate_trajectory(path_func, total_time=20, dt=0.05):
    """Simulate the robot following a predefined path."""
    predefined_path, vr, wr = path_func(total_time=total_time, dt=dt)
    steps = len(predefined_path)
    x = np.zeros(steps)
    y = np.zeros(steps)
    theta = np.zeros(steps)
    x[0], y[0], theta[0] = predefined_path[0]  # Initial state
    
    for k in range(steps - 1):
        v, omega = compute_control(x[k], y[k], theta[k], 
                                 predefined_path[k, 0], predefined_path[k, 1], 
                                 predefined_path[k, 2], vr, wr)
        x[k + 1] = x[k] + v * np.cos(theta[k]) * dt
        y[k + 1] = y[k] + v * np.sin(theta[k]) * dt
        theta[k + 1] = theta[k] + omega * dt
    return np.vstack((x, y, theta)).T, predefined_path

# Animation Function
def simple_animation(robot_states, path):
    """Animate the robot's motion along the path."""
    fig, ax = plt.subplots()
    ax.set_xlim(-5, 25)  # Adjusted for sine wave
    ax.set_ylim(-5, 5)
    ax.set_aspect('equal')
    plt.grid('--')
    robot_traj_line, = ax.plot([], [], 'r-', linewidth=2, label='Robot Trajectory')
    robot_body, = ax.plot([], [], 'k-', linewidth=2)
    path_line, = ax.plot(path[:, 0], path[:, 1], 'b--', linewidth=1.5, label='Path')
    plt.legend()

    def init():
        robot_traj_line.set_data([], [])
        robot_body.set_data([], [])
        return robot_traj_line, robot_body, path_line

    def animate(i):
        robot_traj_line.set_data(robot_states[:i, 0], robot_states[:i, 1])
        x_c, y_c, theta_c = robot_states[i]
        L, W = 0.2, 0.1  # Robot size
        p1 = [x_c + L * np.cos(theta_c), y_c + L * np.sin(theta_c)]
        p2 = [x_c + W * np.cos(theta_c + 2 * np.pi / 3), y_c + W * np.sin(theta_c + 2 * np.pi / 3)]
        p3 = [x_c + W * np.cos(theta_c - 2 * np.pi / 3), y_c + W * np.sin(theta_c - 2 * np.pi / 3)]
        robot_shape_x = [p1[0], p2[0], p3[0], p1[0]]
        robot_shape_y = [p1[1], p2[1], p3[1], p1[1]]
        robot_body.set_data(robot_shape_x, robot_shape_y)
        return robot_traj_line, robot_body, path_line

    ani = animation.FuncAnimation(fig, animate, frames=range(0, len(robot_states), 2), 
                                  init_func=init, interval=50, blit=True, repeat=False)
    plt.show()

# Test Different Trajectories
trajectories = [
    ("Circular Path", generate_circular_path),
    ("Straight Line", generate_straight_path),
    ("Figure Eight", generate_figure_eight),
    ("Sine Wave", generate_sine_wave)
]

for name, path_func in trajectories:
    print(f"Simulating {name}")
    robot_states, predefined_path = simulate_trajectory(path_func)
    simple_animation(robot_states, predefined_path)