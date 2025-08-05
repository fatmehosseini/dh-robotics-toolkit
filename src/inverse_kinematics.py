# src/inverse_kinematics.py

import numpy as np
from typing import Tuple, Optional


def inverse_kinematics_2R(x: float, y: float, a1: float, a2: float) -> Optional[Tuple[float, float]]:
    """
    Solves inverse kinematics for a 2R planar robot arm using analytic geometry.

    Args:
        x (float): Desired x position of end-effector
        y (float): Desired y position of end-effector
        a1 (float): Length of link 1
        a2 (float): Length of link 2

    Returns:
        Tuple[float, float] or None: (theta1, theta2) in radians, or None if no solution
    """
    D = (x**2 + y**2 - a1**2 - a2**2) / (2 * a1 * a2)

    if abs(D) > 1:
        # No solution (target unreachable)
        return None

    # Two possible solutions for theta2
    theta2 = np.arctan2(np.sqrt(1 - D**2), D)  # Elbow-down solution

    # Compute theta1 based on theta2
    k1 = a1 + a2 * np.cos(theta2)
    k2 = a2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return (theta1, theta2)
