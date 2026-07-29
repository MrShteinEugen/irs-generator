"""Rotation utilities and numerically stable attitude propagation."""

from __future__ import annotations

from math import cos, pi, sin
from typing import Protocol, Self, cast

import numpy as np
from numpy.typing import NDArray

from irs_generator.navigation_model.orientation import EulerAngles
from irs_generator.utils.math import VectorLike, skew, vector3

type Matrix3 = NDArray[np.float64]

__all__ = [
    "AttitudeIntegrator",
    "LieGroupAttitudeIntegrator",
    "Matrix3",
    "dcm_body_to_nav_to_euler",
    "euler_to_dcm_body_to_nav",
    "project_to_rotation_matrix",
]


def euler_to_dcm_body_to_nav(attitude: EulerAngles) -> Matrix3:
    """Build the body-to-navigation DCM used by the original project."""

    pitch = attitude.pitch_rad
    roll = attitude.roll_rad
    heading = attitude.heading_rad

    c_pitch, s_pitch = cos(pitch), sin(pitch)
    c_roll, s_roll = cos(roll), sin(roll)
    c_heading, s_heading = cos(heading), sin(heading)

    return cast(
        Matrix3,
        np.array(
            (
                (
                    c_roll * c_heading + s_roll * s_pitch * s_heading,
                    c_pitch * s_heading,
                    s_roll * c_heading - s_pitch * c_roll * s_heading,
                ),
                (
                    -c_roll * s_heading + s_roll * s_pitch * c_heading,
                    c_pitch * c_heading,
                    -s_roll * s_heading - s_pitch * c_roll * c_heading,
                ),
                (-c_pitch * s_roll, s_pitch, c_pitch * c_roll),
            ),
            dtype=np.float64,
        ),
    )


def dcm_body_to_nav_to_euler(dcm: Matrix3) -> EulerAngles:
    matrix = np.asarray(dcm, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError(f"dcm must have shape (3, 3), got {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("dcm must contain only finite values")

    heading = float(np.arctan2(matrix[0, 1], matrix[1, 1]) % (2.0 * pi))
    roll = float(-np.arctan2(matrix[2, 0], matrix[2, 2]))
    pitch = float(
        np.arctan2(
            matrix[2, 1],
            np.hypot(matrix[2, 0], matrix[2, 2]),
        )
    )
    return EulerAngles(pitch, roll, heading)


def _rotation_exponential(rotation_vector_rad: VectorLike) -> Matrix3:
    """SO(3) exponential map using Rodrigues' formula."""

    rotation_vector = vector3(rotation_vector_rad, name="rotation_vector_rad")
    angle = float(np.linalg.norm(rotation_vector))
    identity = np.eye(3, dtype=np.float64)
    omega = skew(rotation_vector)

    if angle < 1e-8:
        return identity + omega + 0.5 * (omega @ omega)

    return (
        identity
        + (sin(angle) / angle) * omega
        + ((1.0 - cos(angle)) / (angle * angle)) * (omega @ omega)
    )


def project_to_rotation_matrix(matrix: Matrix3) -> Matrix3:
    """Return the closest proper orthogonal matrix in Frobenius norm."""

    u, _, vt = np.linalg.svd(np.asarray(matrix, dtype=np.float64))
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    return cast(Matrix3, rotation)


class AttitudeIntegrator(Protocol):
    def fork(self) -> Self:
        """Return an independent integrator with the same configuration."""

    def propagate(
        self,
        body_to_nav_dcm: Matrix3,
        body_rate_rad_s: VectorLike,
        navigation_rate_rad_s: VectorLike,
        dt_s: float,
    ) -> Matrix3:
        """Propagate a body-to-navigation DCM by one time step."""


class LieGroupAttitudeIntegrator:
    """Split SO(3) integrator for the Poisson kinematic equation.

    For constant rates over a step it applies
    ``exp(-Ω_nav dt) C exp(Ω_body dt)``. Unlike forward Euler integration,
    the result remains a valid rotation matrix up to round-off.
    """

    __slots__ = ("_projection_threshold",)

    def __init__(self, projection_threshold: float = 1e-12) -> None:
        threshold = float(projection_threshold)
        if not np.isfinite(threshold) or threshold <= 0.0:
            raise ValueError("projection_threshold must be finite and > 0")
        self._projection_threshold = threshold

    def fork(self) -> Self:
        return type(self)(self._projection_threshold)

    def propagate(
        self,
        body_to_nav_dcm: Matrix3,
        body_rate_rad_s: VectorLike,
        navigation_rate_rad_s: VectorLike,
        dt_s: float,
    ) -> Matrix3:
        dt = float(dt_s)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"dt_s must be finite and > 0, got {dt_s!r}")

        dcm = np.asarray(body_to_nav_dcm, dtype=np.float64)
        if dcm.shape != (3, 3):
            raise ValueError(f"body_to_nav_dcm must have shape (3, 3), got {dcm.shape}")
        if not np.all(np.isfinite(dcm)):
            raise ValueError("body_to_nav_dcm must contain only finite values")

        nav_increment = -vector3(navigation_rate_rad_s) * dt
        body_increment = vector3(body_rate_rad_s) * dt
        propagated = (
            _rotation_exponential(nav_increment)
            @ dcm
            @ _rotation_exponential(body_increment)
        )
        orthogonality_error = float(
            np.linalg.norm(propagated.T @ propagated - np.eye(3), ord="fro")
        )
        determinant = float(np.linalg.det(propagated))
        if orthogonality_error > self._projection_threshold or determinant <= 0.0:
            return project_to_rotation_matrix(propagated)
        return propagated
