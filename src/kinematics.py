# src/kinematics.py

import sympy as sp
from typing import List

from base_robot import RobotModel
from dh_parameters import compute_transformation_matrix


def compute_forward_kinematics(robot: RobotModel) -> List[sp.Matrix]:
    """
    Computes forward kinematics: list of transformation matrices from base to each link.

    Args:
        robot (RobotModel): Robot model with DH parameters

    Returns:
        List[sp.Matrix]: List of 4x4 transformation matrices T_0i (i=1...n)
    """
    transformations = []
    T_total = sp.eye(4)

    for dh in robot.dh_params:
        T_i = compute_transformation_matrix(dh)
        T_total = T_total * T_i
        transformations.append(T_total)

    return transformations


def get_end_effector_pose(robot: RobotModel) -> sp.Matrix:
    """
    Returns the symbolic 4x4 transformation matrix from base to end-effector.

    Args:
        robot (RobotModel): Robot model

    Returns:
        sp.Matrix: Homogeneous transformation matrix of end-effector (T_0n)
    """
    T = compute_forward_kinematics(robot)
    return T[-1] if T else sp.eye(4)
