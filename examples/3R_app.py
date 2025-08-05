import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
from sympy import symbols, Function, pi, latex
import sys
sys.path.append("../src")  # or the relative path to your src folder
from sympy import symbols, Function, pi
from base_robot import RobotModel
from dh_parameters import DHParams
from kinematics import compute_forward_kinematics, get_end_effector_pose
# Streamlit settings
st.set_page_config(page_title="3R Spherical Robot", layout="centered")
st.title("🧭 3R Spherical Robot Kinematics Explorer")

st.markdown("""
This tool lets you explore symbolic and visual kinematics of a **3R spherical robot**.
- Adjust joint angles
- See symbolic end-effector pose
- View 3D configuration
""")

# Define symbolic variables
t = symbols('t')
theta1_sym = Function('theta1')(t)
theta2_sym = Function('theta2')(t)
theta3_sym = Function('theta3')(t)

# Joint sliders
theta1_deg = st.slider("θ₁ (deg)", 0, 180, 45)
theta2_deg = st.slider("θ₂ (deg)", 0, 180, 90)
theta3_deg = st.slider("θ₃ (deg)", 0, 180, 60)

theta1 = np.deg2rad(theta1_deg)
theta2 = np.deg2rad(theta2_deg)
theta3 = np.deg2rad(theta3_deg)

# DH parameters (standard)
# a, alpha, d, theta
a1 = 0
a2 = 1
a3 = 1
d1 = 1

dh1 = DHParams(a=a1, alpha=pi/2, d=d1, theta=theta1_sym)
dh2 = DHParams(a=a2, alpha=0,    d=0,  theta=theta2_sym)
dh3 = DHParams(a=a3, alpha=0,    d=0,  theta=theta3_sym)

robot = RobotModel(
    n_joints=3,
    time_var=t,
    dh_params=[dh1, dh2, dh3]
)

# FK computations
T_all = compute_forward_kinematics(robot)
T_end = get_end_effector_pose(robot)

with st.expander("📐 Symbolic End-Effector Pose (T₀₃)"):
    st.latex(latex(T_end))

# Numerical substitution
T1 = T_all[0].subs({theta1_sym: theta1})
T2 = T_all[1].subs({theta1_sym: theta1, theta2_sym: theta2})
T3 = T_all[2].subs({theta1_sym: theta1, theta2_sym: theta2, theta3_sym: theta3})

# Extract positions
O0 = np.array([0, 0, 0])
O1 = np.array([T1[0, 3], T1[1, 3], T1[2, 3]]).astype(float)
O2 = np.array([T2[0, 3], T2[1, 3], T2[2, 3]]).astype(float)
O3 = np.array([T3[0, 3], T3[1, 3], T3[2, 3]]).astype(float)

# Plot 3D robot
fig = plt.figure(figsize=(7, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot([O0[0], O1[0], O2[0], O3[0]],
        [O0[1], O1[1], O2[1], O3[1]],
        [O0[2], O1[2], O2[2], O3[2]], '-o', linewidth=3, markersize=6)

ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(0, 3)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("3R Spherical Robot Configuration")
ax.view_init(elev=30, azim=135)
st.pyplot(fig)