import numpy as np
import matplotlib.pyplot as plt

# ---------------- PARAMETERS ----------------
ZETA = 1.0       # Attractive gain
ETA = 0.1        # Repulsive gain
RHO_0 = 0.5      # Obstacle influence distance
ALPHA = 0.02     # Step size
EPSILON = 0.05   # Convergence threshold
MAX_ITER = 5000  # Maximum iterations

# ---------------- INITIAL CONFIG ----------------
q_init = np.array([0.0, 0.5])              # [theta, d]
q_goal = np.array([2*np.pi/3, 1.0])        # 120 degrees

# Obstacle position
obstacle = np.array([0.4, -0.3])


# ---------------- FUNCTIONS ----------------
def forward_kinematics(q):
    """
    Computes end-effector position o2(q)
    """
    theta, d = q
    x = d * np.sin(theta)
    y = -d * np.cos(theta)
    return np.array([x, y])


def compute_jacobian(q):
    """
    Computes Jacobian matrix for o2
    """
    theta, d = q
    return np.array([
        [d * np.cos(theta), np.sin(theta)],
        [d * np.sin(theta), -np.cos(theta)]
    ])


def attractive_force(pos, goal):
    """
    Attractive force toward goal
    """
    return -ZETA * (pos - goal)


def repulsive_force(pos):
    """
    Repulsive force from obstacle
    """
    diff = pos - obstacle
    rho = np.linalg.norm(diff)

    if rho <= RHO_0 and rho > 1e-6:
        grad = diff / rho
        force = ETA * (1/rho - 1/RHO_0) * (1/(rho**2)) * grad
        return force
    else:
        return np.zeros(2)


def simulate():
    """
    Runs the potential field simulation
    """
    q = q_init.copy()
    trajectory = []

    for i in range(MAX_ITER):

        if np.linalg.norm(q - q_goal) < EPSILON:
            print(f"Converged in {i} iterations")
            break

        pos = forward_kinematics(q)
        goal = forward_kinematics(q_goal)

        # Forces
        f_att = attractive_force(pos, goal)
        f_rep = repulsive_force(pos)
        f_total = f_att + f_rep

        # Jacobian and torque
        J = compute_jacobian(q)
        tau = J.T @ f_total

        # Stability check
        tau_norm = np.linalg.norm(tau)
        if tau_norm < 1e-6:
            print("Stopped due to very small torque")
            break

        # Update configuration
        q = q + ALPHA * tau / tau_norm

        trajectory.append(pos)

    return np.array(trajectory), q


def plot_results(trajectory):
    """
    Plots the trajectory
    """
    plt.figure()

    # Path
    plt.plot(trajectory[:, 0], trajectory[:, 1], linewidth=2, label="Path")

    # Obstacle
    plt.scatter(obstacle[0], obstacle[1], label="Obstacle")

    # Goal
    goal_pos = forward_kinematics(q_goal)
    plt.scatter(goal_pos[0], goal_pos[1], label="Goal")

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("End-Effector Path (Potential Field)")
    plt.legend()
    plt.grid()

    plt.savefig("../results/trajectory.png")
    plt.show()


if __name__ == "__main__":
    trajectory, final_q = simulate()
    print("Final configuration:", final_q)
    plot_results(trajectory)