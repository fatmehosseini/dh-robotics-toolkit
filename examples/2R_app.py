import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
from sympy import symbols, Function, latex
import sys
sys.path.append("../src")  # or the relative path to your src folder
from inverse_kinematics import inverse_kinematics_2R
from sympy import symbols, Function, pi
from base_robot import RobotModel
from dh_parameters import DHParams
from kinematics import compute_forward_kinematics, get_end_effector_pose

# Streamlit GUI setup
st.set_page_config(page_title="2R Planar Robot", layout="centered")
st.title("🦿 2R Planar Robot Kinematics Explorer")

st.markdown("""
This app allows you to:
- Select joint angles
- Visualize the 2R robot configuration
- See symbolic forward kinematics
- View LaTeX output of the end-effector pose
""")

# Time symbol and symbolic functions
t = symbols('t')
theta1_sym = Function('theta1')(t)
theta2_sym = Function('theta2')(t)

# Sliders for joint angles
theta1_deg = st.slider("θ₁ (degrees)", 0, 180, 45)
theta2_deg = st.slider("θ₂ (degrees)", 0, 180, 60)
theta1 = np.deg2rad(theta1_deg)
theta2 = np.deg2rad(theta2_deg)

# Define DH parameters for 2R planar
dh1 = DHParams(a=1, alpha=0, d=0, theta=theta1_sym)
dh2 = DHParams(a=1, alpha=0, d=0, theta=theta2_sym)
robot = RobotModel(n_joints=2, time_var=t, dh_params=[dh1, dh2])

# Compute forward kinematics
T_all = compute_forward_kinematics(robot)
T_end = get_end_effector_pose(robot)

# Show symbolic matrix as LaTeX
with st.expander("📐 Symbolic End-Effector Pose (T₀₂)"):
    st.latex(latex(T_end))

# Numerical substitution
T1 = T_all[0].subs({theta1_sym: theta1})
T2 = T_all[1].subs({theta1_sym: theta1, theta2_sym: theta2})

# Joint positions
O0 = np.array([0, 0])
O1 = np.array([T1[0, 3], T1[1, 3]]).astype(np.float64)
O2 = np.array([T2[0, 3], T2[1, 3]]).astype(np.float64)

# Plot robot
fig, ax = plt.subplots()
ax.plot([O0[0], O1[0], O2[0]], [O0[1], O1[1], O2[1]], '-o', linewidth=3)
ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.5)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title("2R Planar Robot Configuration")

# Inverse Kinematics section
with st.expander("🎯 Inverse Kinematics (Solve for θ₁, θ₂ from x, y)"):
    x_input = st.number_input("Target x", value=1.0)
    y_input = st.number_input("Target y", value=1.0)
    if st.button("Solve IK"):
        result = inverse_kinematics_2R(x_input, y_input, a1=1.0, a2=1.0)
        if result:
            th1, th2 = result
            st.success(f"θ₁ = {np.rad2deg(th1):.2f}°, θ₂ = {np.rad2deg(th2):.2f}°")
        else:
            st.error("No solution: target is outside reachable workspace.")


st.pyplot(fig)