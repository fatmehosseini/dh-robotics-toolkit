
# 🤖 Symbolic Robotics Engine (Python)

This package provides symbolic tools for modeling, analyzing, and simulating robotic systems using Denavit-Hartenberg (DH) parameters and symbolic math via `sympy`. It supports symbolic forward kinematics, velocities, and accelerations for robotic manipulators.

---

## 📦 Features

- Denavit-Hartenberg modeling (`DHParams`)
- Symbolic forward kinematics (`compute_forward_kinematics`)
- Recursive Newton-Euler dynamics:
  - Angular/linear velocity (`compute_velocities`)
  - Angular/linear acceleration (`compute_accelerations`)
- Plug-and-play structure (`RobotModel`)
- Visualization-ready for plotting robot configurations
- Fully symbolic using `sympy`

---

## 🧠 Mathematical Foundation

We follow **Craig’s formulation** of robotic kinematics and dynamics:

### 1. Denavit-Hartenberg Transformation

For each link \( i \), the transformation from frame \( i-1 \) to \( i \) is:

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
T_i = compute_transformation_matrix(dh_params[i])
```

---

### 2. Forward Kinematics

Chain all transformations:

\[
T_0^n = T_1 \cdot T_2 \cdots T_n
\]

**Code**:
```python
T_all = compute_forward_kinematics(robot)
T_end = get_end_effector_pose(robot)
```

---

### 3. Angular Velocity (Recursive)

\[
\omega_i = R_i^T (\omega_{i-1} + z_{i-1}\dot{\theta}_i)
\]

**Code**:
```python
w, v = compute_velocities(robot)
```

---

### 4. Angular Acceleration (Recursive)

\[
\dot{\omega}_i = R_i^T (\dot{\omega}_{i-1} + z_{i-1}\ddot{\theta}_i + \omega_{i-1} \times z_{i-1} \dot{\theta}_i)
\]

**Code**:
```python
wd, vd = compute_accelerations(robot, w, v)
```

---

## 📁 Folder Structure

```
symbolic_robotics/
├── src/
│   ├── base_robot.py
│   ├── dh_parameters.py
│   ├── kinematics.py
│   ├── dynamics.py
│   ├── utils.py
│   └── __init__.py
├── examples/
│   ├── planar_2R_example.ipynb
│   └── spherical_3R_example.ipynb
```

---

## ✍️ How to Write Your Own Example

1. **Define time variable and symbolic joint functions**:
```python
from sympy import symbols, Function
t = symbols('t')
theta1 = Function('theta1')(t)
```

2. **Define your DH parameters** using `DHParams`:
```python
from symbolic_robotics import DHParams
dh1 = DHParams(a=1, alpha=0, d=0, theta=theta1)
```

3. **Create your robot**:
```python
from symbolic_robotics import RobotModel
robot = RobotModel(n_joints=1, time_var=t, dh_params=[dh1])
```

4. **Compute transformations**:
```python
from symbolic_robotics import compute_forward_kinematics
T = compute_forward_kinematics(robot)
```

---

## 📜 License

MIT License

---

## 📬 Contributions

Feel free to open issues or submit pull requests!

