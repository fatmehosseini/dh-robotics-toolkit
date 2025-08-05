"""
Symbolic Robotics Package
--------------------------
This package provides symbolic tools for modeling, analyzing, and simulating robotic systems
using Denavit-Hartenberg parameters and symbolic math with sympy.

Modules:
- base_robot: Main RobotModel class
- dh_parameters: DH parameter representation and transformation matrix computation
- kinematics: Forward kinematics computation
- dynamics: Symbolic velocity and acceleration calculation
- utils: Miscellaneous symbolic math tools
"""

from .base_robot import RobotModel
from .dh_parameters import DHParams, compute_transformation_matrix
from .kinematics import compute_forward_kinematics
from .dynamics import compute_velocities, compute_accelerations
from . import utils

__all__ = [
    "RobotModel",
    "DHParams",
    "compute_transformation_matrix",
    "compute_forward_kinematics",
    "compute_velocities",
    "compute_accelerations",
    "utils"
]
