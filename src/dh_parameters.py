# src/dh_parameters.py

import sympy as sp
from dataclasses import dataclass


@dataclass
class DHParams:
    """
    Represents Denavit-Hartenberg parameters for one joint/link.
    
    Attributes:
        a (float): Link length
        alpha (float): Link twist (in radians)
        d (sp.Expr): Link offset (can be symbolic or numeric)
        theta (sp.Expr): Joint angle (can be symbolic or numeric)
    """
    a: float
    alpha: float
    d: sp.Expr
    theta: sp.Expr


def compute_transformation_matrix(dh: DHParams) -> sp.Matrix:
    """
    Computes the symbolic homogeneous transformation matrix using standard DH convention.

    Args:
        dh (DHParams): Denavit-Hartenberg parameters for a link

    Returns:
        sp.Matrix: 4x4 symbolic transformation matrix
    """
    a, alpha, d, theta = dh.a, dh.alpha, dh.d, dh.theta

    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,              sp.sin(alpha),                sp.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])
