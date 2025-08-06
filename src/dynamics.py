import sympy as sp
import numpy as np
from typing import List

from .base_robot import RobotModel
from .dh_parameters import compute_transformation_matrix
from .utils import skew


def compute_velocities(robot: RobotModel):
    """
    Recursively computes angular and linear velocities of each link using symbolic math.

    Returns:
        List[Tuple[sp.Matrix, sp.Matrix]]: List of (angular_velocity, linear_velocity) per link
    """
    w = [sp.Matrix(robot.w0)]      # angular velocity
    v = [sp.Matrix(robot.v0)]      # linear velocity

    for i in range(robot.n_joints):
        dh = robot.dh_params[i]
        T_i = compute_transformation_matrix(dh)
        R = T_i[:3, :3]
        P = T_i[:3, 3]

        # z axis in local frame (joint axis)
        z = sp.Matrix([0, 0, 1])

        # symbolic derivative of theta or d wrt time
        if dh.theta.has(robot.time_var):
            joint_var_dot = sp.diff(dh.theta, robot.time_var)
            w_i = R.T * (w[-1] + z * joint_var_dot)
            v_i = R.T * (v[-1] + w[-1].cross(P))
        else:
            joint_var_dot = sp.diff(dh.d, robot.time_var)
            w_i = R.T * w[-1]
            v_i = R.T * (v[-1] + w[-1].cross(P) + z * joint_var_dot)

        w.append(w_i)
        v.append(v_i)

    return w[1:], v[1:]  # drop base link


def compute_accelerations(robot: RobotModel, w: List[sp.Matrix], v: List[sp.Matrix]):
    """
    Recursively computes angular and linear accelerations using symbolic math.

    Args:
        w: Angular velocities from compute_velocities
        v: Linear velocities from compute_velocities

    Returns:
        List[Tuple[sp.Matrix, sp.Matrix]]: List of (angular_acceleration, linear_acceleration) per link
    """
    wd = [sp.Matrix(robot.wdot0)]   # angular acceleration
    vd = [sp.Matrix(robot.vdot0)]   # linear acceleration

    for i in range(robot.n_joints):
        dh = robot.dh_params[i]
        T_i = compute_transformation_matrix(dh)
        R = T_i[:3, :3]
        P = T_i[:3, 3]

        z = sp.Matrix([0, 0, 1])

        if dh.theta.has(robot.time_var):
            joint_var_dot = sp.diff(dh.theta, robot.time_var)
            joint_var_ddot = sp.diff(joint_var_dot, robot.time_var)

            wd_i = R.T * (wd[-1] + z * joint_var_ddot + w[-1].cross(z * joint_var_dot))
            vd_i = R.T * (vd[-1] + wd[-1].cross(P) + w[-1].cross(w[-1].cross(P)) +
                          2 * w[-1].cross(z * joint_var_dot) + z * joint_var_ddot)
        else:
            joint_var_dot = sp.diff(dh.d, robot.time_var)
            joint_var_ddot = sp.diff(joint_var_dot, robot.time_var)

            wd_i = R.T * wd[-1]
            vd_i = R.T * (vd[-1] + wd[-1].cross(P) + w[-1].cross(w[-1].cross(P)) +
                          z * joint_var_ddot + 2 * w[-1].cross(z * joint_var_dot))

        wd.append(wd_i)
        vd.append(vd_i)

    return wd[1:], vd[1:]  # drop base link
