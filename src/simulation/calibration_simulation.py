import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from IPython.display import HTML

# ---------------- PARAMETERS ----------------
L_TRUE = np.array([1.0, 1.0, 1.0])
L_EST = np.array([1.3, 0.7, 1.2]) # Initial guess

ALPHA = 0.05  # Increased learning rate for faster visual convergence
NUM_SAMPLES = 40
NUM_FRAMES = 100

# FIXED samples (for a stable loss landscape)
np.random.seed(42) # For reproducibility
Q_SET = np.random.uniform(-np.pi/2, np.pi/2, (NUM_SAMPLES, 3))

# ---------------- FORWARD KINEMATICS ----------------
def fk(q, l):
    """Returns the (x, y) end-effector position."""
    t1, t2, t3 = q
    x = (l[0]*np.cos(t1) +
         l[1]*np.cos(t1+t2) +
         l[2]*np.cos(t1+t2+t3))
    y = (l[0]*np.sin(t1) +
         l[1]*np.sin(t1+t2) +
         l[2]*np.sin(t1+t2+t3))
    return np.array([x, y])

def fk_full(q, l):
    """Returns the coordinates of all joints for plotting."""
    t1, t2, t3 = q
    x1, y1 = l[0]*np.cos(t1), l[0]*np.sin(t1)
    x2, y2 = x1 + l[1]*np.cos(t1+t2), y1 + l[1]*np.sin(t1+t2)
    x3, y3 = x2 + l[2]*np.cos(t1+t2+t3), y2 + l[2]*np.sin(t1+t2+t3)
    
    return np.array([
        [0, 0, 0],
        [x1, y1, 0],
        [x2, y2, 0],
        [x3, y3, 0]
    ])

# ---------------- ERROR + GRADIENT ----------------
def compute_error_and_grad(l_curr):
    grad = np.zeros_like(l_curr)
    total_error = 0

    for q in Q_SET:
        p_true = fk(q, L_TRUE)
        p_est  = fk(q, l_curr)
        diff = p_est - p_true
        total_error += np.sum(diff**2)

        t1, t2, t3 = q
        # Analytical gradients: d(Error)/dl_i
        grad[0] += 2 * diff @ np.array([np.cos(t1), np.sin(t1)])
        grad[1] += 2 * diff @ np.array([np.cos(t1+t2), np.sin(t1+t2)])
        grad[2] += 2 * diff @ np.array([np.cos(t1+t2+t3), np.sin(t1+t2+t3)])

    return total_error / NUM_SAMPLES, grad / NUM_SAMPLES

# ---------------- ANIMATION SETUP ----------------
fig = plt.figure(figsize=(10, 5))
ax = fig.add_subplot(121, projection='3d') # 3D Robot view
ax2 = fig.add_subplot(122)                 # 2D Error plot

q_vis = np.array([0.5, 0.8, -0.3]) # Fixed pose for visualization
error_history = []
l_evolution = L_EST.copy()

def update(frame):
    global l_evolution
    ax.cla()
    
    err, grad = compute_error_and_grad(l_evolution)
    l_evolution = l_evolution - ALPHA * grad
    error_history.append(err)

    # Robot Plot
    p_true = fk_full(q_vis, L_TRUE)
    p_est  = fk_full(q_vis, l_evolution)

    ax.plot(p_true[:,0], p_true[:,1], p_true[:,2], 'g-o', linewidth=3, label="True (Goal)")
    ax.plot(p_est[:,0], p_est[:,1], p_est[:,2], 'r--o', label="Calibrating...")
    
    ax.set_xlim([-3, 3]); ax.set_ylim([-3, 3]); ax.set_zlim([-1, 1])
    ax.set_title(f"Iter: {frame} | Error: {err:.4f}")
    ax.legend()

    # Live Error Plot
    ax2.cla()
    ax2.plot(error_history, color='blue')
    ax2.set_yscale('log') # Log scale helps see convergence
    ax2.set_title("Mean Squared Error (Log Scale)")
    ax2.set_xlabel("Iteration")
    ax2.grid(True)

anim = FuncAnimation(fig, update, frames=NUM_FRAMES, interval=50, repeat=False)
plt.close() # Prevents duplicate static plot
HTML(anim.to_jshtml())
