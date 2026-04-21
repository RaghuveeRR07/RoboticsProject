# Identification of DH Parameters for Calibration of Cooperative Robots

## 👨‍💻 Author
**Raghuveer Kulkarni**  
Roll Number: **B23ES1020**  
Indian Institute of Technology Jodhpur  

---

## 📌 Project Overview

This project focuses on the calibration of cooperative robotic systems by identifying accurate Denavit-Hartenberg (DH) parameters.

In multi-robot systems, even small inaccuracies in internal parameters (such as link lengths or joint offsets) can lead to inconsistencies in spatial understanding. As a result, two robots attempting to reach the same point may not actually coincide in physical space.

The objective of this project is to develop a simulation-based approach to **calibrate a robot model by minimizing the error between an incorrect model and a reference model**.

---

## 🧠 Key Idea

The core idea behind this project is to treat calibration as a **parameter estimation problem**.

- A **true robot** represents the correct physical system  
- An **estimated robot** represents the internal model with incorrect parameters  
- The difference between their end-effector positions is defined as **error**  
- This error is minimized using **optimization techniques**

Over time, the estimated robot updates its parameters and aligns with the true robot.

---

## ⚙️ Methodology

### 1. Kinematic Modeling
- The robot is modeled using DH parameters (simplified as link lengths in this implementation)
- Forward kinematics is used to compute the end-effector position

---

### 2. Multi-Configuration Sampling
- Multiple random joint configurations are generated
- This ensures that calibration is valid across different robot poses
- Prevents overfitting to a single configuration

---

### 3. Error Formulation

The calibration error is defined as:

\[
E = \sum_{i=1}^{N} \|o_{\text{true}}(q_i) - o_{\text{est}}(q_i)\|^2
\]

Where:
- \(o_{\text{true}}\) = true robot position  
- \(o_{\text{est}}\) = estimated robot position  

---

### 4. Optimization

- Gradient-based optimization is used to minimize error
- Parameters are iteratively updated
- Convergence is achieved when error becomes minimal

---

### 5. Visualization

A 3D simulation is implemented:

- 🔵 **Blue Robot** → True (reference) model  
- 🔴 **Red Robot** → Estimated model  

The animation shows how the estimated robot gradually aligns with the true robot.

---

## 📊 Results

- The estimated robot initially shows significant deviation from the true robot
- Over iterations, the alignment improves
- The error decreases steadily (as shown in the convergence graph)
- Final result shows near-overlap between both robots

---

## 📷 Sample Output

![Calibration Result](results/calibration.png)

> The figure shows alignment between the true robot and the calibrated robot after optimization.

---

## 🌍 Real-Life Applications

This project represents a simplified version of real-world robot calibration problems, which are critical in several domains:

---

### 🤖 1. Industrial Automation (Assembly Lines)

In manufacturing industries, multiple robotic arms work together to assemble products.

**Example:**
- One robot holds a car component  
- Another performs welding or drilling  

👉 If robots are not calibrated:
- Misalignment occurs  
- Parts may not fit properly  
- Product quality degrades  

✔ Calibration ensures both robots agree on the same position.

---

### 🏭 2. Dual-Arm Manipulation Systems

Used in:
- Electronics assembly  
- Packaging systems  

**Example:**
- One robot stabilizes an object  
- Another performs precise operations  

👉 Requires:
- High precision  
- Consistent spatial understanding  

---

### 🏥 3. Surgical Robotics

In robotic-assisted surgeries:
- Multiple robotic arms operate simultaneously  

👉 Calibration is critical because:
- Even small errors can cause serious consequences  
- Precision must be extremely high  

---

### 🛰 4. Space Robotics

Used in:
- Satellite servicing  
- Space station robotic arms  

👉 Challenges:
- No human intervention  
- High precision required  

✔ Calibration ensures reliable operation in extreme environments.

---

### 📦 5. Autonomous Warehousing

Used in:
- Amazon-style warehouse systems  

Multiple robots:
- Pick, place, and transfer objects  

👉 Requires:
- Consistent coordinate systems  
- Accurate object handling  

---

### 🧪 6. Research and Multi-Robot Systems

Used in:
- Swarm robotics  
- Cooperative manipulation  

👉 Calibration enables:
- Coordination  
- Task sharing  
- System scalability  

---

## 🔍 Relation to Real Systems

This project is a **simplified simulation** of real-world calibration methods.

| Project | Real World |
|--------|------------|
| True robot | Physical robot / sensor measurements |
| Estimated robot | Robot’s internal model |
| Error minimization | Calibration algorithms |
| Simulation | Real hardware + sensors |

In practice:
- Cameras, sensors, or measurement systems are used  
- Full DH parameters are optimized  
- Noise and uncertainty are handled  

---

## ⚠️ Limitations

- Only link lengths are optimized (not full DH parameters)
- Planar 3R robot model is used
- No real sensor data or noise considered
- Simplified simulation environment

---

## 🔮 Future Work

- Extend to full DH parameter estimation (a, d, α, θ)
- Implement explicit two-robot calibration (handclasp simulation)
- Integrate with physics engines (MuJoCo / PyBullet)
- Add noise and real-world uncertainties
- Apply to 6-DOF industrial robots


