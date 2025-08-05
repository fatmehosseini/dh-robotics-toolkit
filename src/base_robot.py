# src/base_robot.py

import numpy as np
import sympy as sp
from dataclasses import dataclass
from typing import List, Union

from dh_parameters import DHParams


class RobotModel:
    """
    Represents a symbolic robot model based on Denavit-Hartenberg (DH) parameters.

    Attributes:
        n_joints (int): Number of joints
        time_var (sp.Symbol): Time variable (e.g., t)
        dh_params (List[DHParams]): List of DH parameters for each link
        base_linear_velocity (np.ndarray): Initial linear velocity of base
        base_angular_velocity (np.ndarray): Initial angular velocity of base
        base_linear_acceleration (np.ndarray): Initial linear acceleration of base
        base_angular_acceleration (np.ndarray): Initial angular acceleration of base
    """

    def __init__(
        self,
        n_joints: int,
        time_var: sp.Symbol,
        dh_params: List[DHParams],
        base_linear_velocity: np.ndarray = np.zeros((3, 1)),
        base_angular_velocity: np.ndarray = np.zeros((3, 1)),
        base_linear_acceleration: np.ndarray = np.zeros((3, 1)),
        base_angular_acceleration: np.ndarray = np.zeros((3, 1)),
    ):
        self.n_joints = n_joints
        self.time_var = time_var
        self.dh_params = dh_params

        # Base (initial) conditions
        self.v0 = base_linear_velocity
        self.w0 = base_angular_velocity
        self.vdot0 = base_linear_acceleration
        self.wdot0 = base_angular_acceleration

        self._validate()

    def _validate(self):
        if len(self.dh_params) != self.n_joints:
            raise ValueError(f"Expected {self.n_joints} DH parameter sets, got {len(self.dh_params)}")

    def describe(self):
        print(f"Robot with {self.n_joints} joints.")
        for i, dh in enumerate(self.dh_params):
            print(f"  Joint {i+1}: a={dh.a}, alpha={dh.alpha}, d={dh.d}, theta={dh.theta}")
