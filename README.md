# Identification of DH Parameters for Calibration of Cooperative Robots

## 👨‍💻 Author
**Raghuveer Kulkarni**  
Roll Number: **B23ES1020**  
Indian Institute of Technology Jodhpur  

---

## 📌 Overview

This project focuses on the calibration of cooperative robotic systems by identifying accurate Denavit-Hartenberg (DH) parameters. When multiple robots work together, inconsistencies in their internal models can lead to errors in positioning and coordination.

The goal of this project is to develop a method that aligns the coordinate frames of two robots by minimizing the difference in their computed end-effector positions.

---

## 🧠 Key Idea

The project is based on the concept of **model correction through optimization**.

- A **true robot** represents the correct kinematic model  
- An **estimated robot** starts with incorrect parameters  
- The difference between their end-effector positions is treated as **error**  
- This error is minimized by updating the model parameters  

Over time, the estimated robot aligns with the true robot, achieving calibration.

---

## ⚙️ Methodology

### 1. Kinematic Modeling
- Robots are modeled using DH parameters (simplified as link lengths in this implementation)
- Forward kinematics is used to compute end-effector positions

### 2. Multi-Configuration Sampling
- Multiple random joint configurations are generated
- This ensures calibration is valid across different poses

### 3. Error Formulation
\[
E = \sum \|o_{true}(q) - o_{est}(q)\|^2
\]

- Error is defined as the difference between true and estimated end-effector positions

### 4. Optimization
- Numerical gradient descent is used
- Parameters are updated iteratively to reduce error

---

## 🧪 Simulation

A 3D simulation is implemented using Python and Matplotlib:

- 🔵 **Blue Robot** → True model  
- 🔴 **Red Robot** → Estimated model  

### What happens:
- Initially, the red robot is misaligned  
- Over iterations, it updates its parameters  
- Finally, both robots align  

---

## 📊 Results

- The error decreases over time (shown using convergence plot)
- The estimated robot gradually matches the true robot
- Demonstrates successful calibration using optimization

---

## 📁 Project Structure
