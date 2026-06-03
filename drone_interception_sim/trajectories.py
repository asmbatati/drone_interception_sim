"""Parametric 3D trajectory generators (circle and figure-eight).

Re-homed from uav_gz_sim so this package is self-contained.
"""
import numpy as np


def _basis_from_normal(normal_vector):
    n = normal_vector / np.linalg.norm(normal_vector)
    not_parallel = np.array([1, 0, 0]) if n[0] == 0 else np.array([0, 1, 0])
    v1 = np.cross(n, not_parallel)
    v1 = v1 / np.linalg.norm(v1)
    v2 = np.cross(n, v1)
    v2 = v2 / np.linalg.norm(v2)
    return n, v1, v2


class Circle3D:
    """Constant-speed circular trajectory in an arbitrary plane."""

    def __init__(self, normal_vector, center_vector, radius=1, omega=1):
        self.center_vector = center_vector
        self.radius = radius
        self.omega = omega
        self.normal_vector, self.v1, self.v2 = _basis_from_normal(normal_vector)

    def generate_trajectory_setpoint(self, time):
        """Return the (x, y, z) setpoint at the given time (seconds)."""
        t = self.omega * time
        return self.center_vector + self.radius * (np.cos(t) * self.v1 + np.sin(t) * self.v2)

    def timeToCompleteFullTrajectory(self):
        """Return the period (seconds) of one full loop."""
        return 2 * np.pi / self.omega


class Infinity3D:
    """Figure-eight (lemniscate) trajectory in an arbitrary plane."""

    def __init__(self, normal_vector, center_vector, radius=1, omega=1):
        self.center_vector = center_vector
        self.radius = radius
        self.omega = omega
        self.normal_vector, self.v1, self.v2 = _basis_from_normal(normal_vector)

    def generate_trajectory_setpoint(self, time):
        """Return the (x, y, z) setpoint at the given time (seconds)."""
        t = self.omega * time
        return self.center_vector + self.radius * (np.cos(t) * self.v1 + np.sin(2 * t) * self.v2)

    def timeToCompleteFullTrajectory(self):
        """Return the period (seconds) of one full figure-eight."""
        return 2 * np.pi / self.omega
