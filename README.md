# 🤖 Symbolic Robotics Engine (Python)

This package provides symbolic tools for modeling, analyzing, and visualizing robotic systems using Denavit-Hartenberg (DH) parameters and symbolic math via `sympy`.

---

## 📦 Features

- DH parameter modeling (`DHParams`)
- Symbolic forward kinematics
- Recursive Newton-Euler dynamics:
  - Angular/linear velocity
  - Angular/linear acceleration
- Inverse kinematics:
  - ✅ Analytic for 2R planar robot
  - ✅ Numerical for 3R spherical robot
- Visual GUI apps for 2R and 3R using Streamlit
- Plotting in 2D and 3D

---

## 📘 Mathematical Foundation

All formulas follow the conventions in the textbook:

**John J. Craig – Introduction to Robotics: Mechanics and Control**  
Free online version:  
[https://marsuniversity.github.io/ece387/Introduction-to-Robotics-Craig.pdf](https://marsuniversity.github.io/ece387/Introduction-to-Robotics-Craig.pdf)

---

## 🧠 Key Equations

### 1. Denavit-Hartenberg (DH) Transformation

\[
T_i =
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

**Code**:
```python
T = compute_transformation_matrix(DHParams(...))
```

---

### 2. Forward Kinematics

\[
T_0^n = T_1 \cdot T_2 \cdot \dots \cdot T_n
\]

**Code**:
```python
T_all = compute_forward_kinematics(robot)
```

---

### 3. Inverse Kinematics

- **2R Planar**: Closed-form solution using trigonometry
```python
theta1, theta2 = inverse_kinematics_2R(x, y, a1, a2)
```

- **3R Spherical**: Numerical optimization to match target position
```python
theta_vals = inverse_kinematics_3R_numeric(robot, target_pos)
```

---

### 4. Velocity & Acceleration

Recursive Newton-Euler method:
```python
w, v = compute_velocities(robot)
wd, vd = compute_accelerations(robot, w, v)
```

---

## 📁 Project Structure

```
symbolic_robotics/
├── src/
│   ├── base_robot.py
│   ├── dh_parameters.py
│   ├── kinematics.py
│   ├── dynamics.py
│   ├── inverse_kinematics.py
│   ├── utils.py
│   └── __init__.py
├── examples/
│   ├── planar_2R_example.ipynb
│   ├── spherical_3R_example.ipynb
├── app_2R.py
├── app_3R.py
└── README.md
```

---

## 🚀 Run the GUI

```bash
streamlit run app_2R.py     # 2R planar arm
streamlit run app_3R.py     # 3R spherical arm
```

---

## ✍️ Write Your Own Example

```python
from symbolic_robotics import DHParams, RobotModel
dh1 = DHParams(a=1, alpha=0, d=0, theta=theta1)
robot = RobotModel(n_joints=1, time_var=t, dh_params=[dh1])
```

---

## 📜 License

MIT License

---

## 🙌 Acknowledgment

Based on the formulations and diagrams from:
**Craig, John J. — Introduction to Robotics: Mechanics and Control**