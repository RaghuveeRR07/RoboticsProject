# ============================================================
# Cooperative Robot Calibration Simulation (Advanced Version)
# ------------------------------------------------------------
# This script demonstrates calibration of a robot model by
# minimizing error between a true robot and an estimated robot
# using multiple configurations and optimization.
# ============================================================

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from IPython.display import HTML

# ============================================================
# 1. PARAMETERS
# ============================================================

# True robot parameters (ground truth)
L_TRUE = np.array([1.0, 1.0, 1.0])

# Initial incorrect parameters
L_EST = np.array([1.3, 0.7, 1.2])

# Learning rate
ALPHA = 0.02

# Number of random configurations
NUM_SAMPLES = 20

# Generate random joint configurations
Q_SET = np.random.uniform(low=-1.0, high=1.0, size=(NUM_SAMPLES, 3))


# ============================================================
# 2. FORWARD KINEMATICS
# ============================================================

def forward_kinematics(q, l):
    """
    Returns end-effector position for given joint angles and link lengths.
    """
    x1 = l[0] * np.cos(q[0])
    y1 = l[0] * np.sin(q[0])

    x2 = x1 + l[1] * np.cos(q[0] + q[1])
    y2 = y1 + l[1] * np.sin(q[0] + q[1])

    x3 = x2 + l[2] * np.cos(q[0] + q[1] + q[2])
    y3 = y2 + l[2] * np.sin(q[0] + q[1] + q[2])

    return np.array([x3, y3, 0])


def forward_full(q, l):
    """
    Returns all joint positions for visualization.
    """
    x1 = l[0] * np.cos(q[0])
    y1 = l[0] * np.sin(q[0])

    x2 = x1 + l[1] * np.cos(q[0] + q[1])
    y2 = y1 + l[1] * np.sin(q[0] + q[1])

    x3 = x2 + l[2] * np.cos(q[0] + q[1] + q[2])
    y3 = y2 + l[2] * np.sin(q[0] + q[1] + q[2])

    return np.array([
        [0, 0, 0],
        [x1, y1, 0],
        [x2, y2, 0],
        [x3, y3, 0]
    ])


# ============================================================
# 3. ERROR FUNCTION
# ============================================================

def compute_total_error(l_est):
    """
    Computes total error across multiple configurations.
    """
    total_error = 0

    for q in Q_SET:
        p_true = forward_kinematics(q, L_TRUE)
        p_est  = forward_kinematics(q, l_est)

        total_error += np.linalg.norm(p_true - p_est) ** 2

    return total_error


# ============================================================
# 4. PARAMETER UPDATE (GRADIENT DESCENT)
# ============================================================

def update_parameters(l_est):
    """
    Updates estimated parameters using numerical gradient descent.
    """
    grad = np.zeros_like(l_est)

    for i in range(len(l_est)):
        delta = np.zeros_like(l_est)
        delta[i] = 1e-4

        err1 = compute_total_error(l_est + delta)
        err2 = compute_total_error(l_est - delta)

        grad[i] = (err1 - err2) / (2e-4)

    return l_est - ALPHA * grad


# ============================================================
# 5. VISUALIZATION
# ============================================================

def plot_robot(ax, points, color, label):
    """
    Plots robot links in 3D.
    """
    ax.plot(points[:, 0], points[:, 1], points[:, 2],
            '-o', color=color, label=label)


# ============================================================
# 6. ANIMATION SETUP
# ============================================================

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Select one configuration to visualize
q_vis = Q_SET[0]

# Store error history
error_history = []


def update(frame):
    """
    Animation update function.
    """
    global L_EST

    ax.cla()

    # Update parameters
    L_EST = update_parameters(L_EST)

    # Compute error
    err = compute_total_error(L_EST)
    error_history.append(err)

    # Compute robot positions
    p_true = forward_full(q_vis, L_TRUE)
    p_est  = forward_full(q_vis, L_EST)

    # Plot robots
    plot_robot(ax, p_true, 'blue', 'True Robot')
    plot_robot(ax, p_est, 'red', 'Estimated Robot')

    # Plot end-effectors
    ax.scatter(*p_true[-1], color='blue', s=60)
    ax.scatter(*p_est[-1], color='red', s=60)

    # Axis limits
    ax.set_xlim([-3, 3])
    ax.set_ylim([-3, 3])
    ax.set_zlim([-1, 1])

    ax.set_title(f"Calibration | Error: {err:.4f}")
    ax.legend()


# Create animation
anim = FuncAnimation(fig, update, frames=150, interval=100)

# Display animation (Colab/Jupyter)
HTML(anim.to_jshtml())


# ============================================================
# 7. ERROR PLOT (OPTIONAL BUT RECOMMENDED)
# ============================================================

plt.figure()
plt.plot(error_history)
plt.title("Error Convergence During Calibration")
plt.xlabel("Iteration")
plt.ylabel("Total Error")
plt.grid()
plt.show()