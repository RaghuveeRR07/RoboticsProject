# RoboticsProject
# Path Planning using Potential Field (RP Robot)

## 📌 Overview

This project implements path planning for a 2-link RP robot using the potential field method. The objective is to move the robot from an initial configuration to a final configuration while avoiding an obstacle.

---

## ⚙️ Methodology

The approach is based on artificial potential fields:

- **Attractive Force**: Pulls the robot toward the goal
- **Repulsive Force**: Pushes the robot away from obstacles
- **Jacobian Transpose**: Converts workspace forces to joint torques
- **Gradient Descent**: Updates joint configuration iteratively

---

## 📐 Mathematical Model

End-effector position:

o2(q) = [ d*sin(theta), -d*cos(theta) ]

Total force:

f = f_att + f_rep

Joint torque:

tau = J^T * f

Update rule:

q_next = q + alpha * tau / ||tau||

---

## 🚀 How to Run

1. Install dependencies:

```bash
pip install -r requirements.txt