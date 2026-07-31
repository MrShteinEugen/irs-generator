from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

import numpy as np

from irs_generator.utils._validation import (
    _finite_scalar,
    _positive_scalar,
    _validated_latitude,
)
from irs_generator.utils.math import Scalar

from .geometry import ReferenceEllipsoid
from .rotation import RotationParameters

__all__ = [
    "ConstantGravity",
    "GravityModel",
    "InverseSquareGravity",
    "SomiglianaNormalGravity",
]


@runtime_checkable
class GravityModel(Protocol):
    """Strategy interface for scalar gravity calculations."""

    def gravity_m_s2(
        self,
        latitude_rad: Scalar,
        height_m: Scalar = 0.0,
    ) -> Scalar:
        """Return gravity magnitude in m/s²."""


@dataclass(frozen=True, slots=True)
class SomiglianaNormalGravity:
    """Normal gravity for a rotating reference ellipsoid.

    Parameters
    ----------
    ellipsoid
        Reference ellipsoid used for geometry and eccentricity.
    rotation
        Earth rotation parameters.
    gravitational_parameter_m3_s2
        Geocentric gravitational constant ``GM`` in m³/s².

    Notes
    -----
    Surface gravity is calculated with Somigliana's formula. Height correction
    uses the standard second-order near-surface expansion.
    """

    ellipsoid: ReferenceEllipsoid
    rotation: RotationParameters
    gravitational_parameter_m3_s2: Scalar

    equatorial_gravity_m_s2: np.longdouble = field(init=False)
    polar_gravity_m_s2: np.longdouble = field(init=False)
    somigliana_k: np.longdouble = field(init=False)
    rotational_parameter_m: np.longdouble = field(init=False)

    def __post_init__(self) -> None:
        mu = _positive_scalar(
            self.gravitational_parameter_m3_s2,
            name="gravitational_parameter_m3_s2",
        )
        a = self.ellipsoid.semi_major_axis_m
        b = self.ellipsoid.semi_minor_axis_m
        omega = self.rotation.angular_velocity_rad_s

        second_eccentricity = np.sqrt(self.ellipsoid.second_eccentricity_squared)
        ep2 = second_eccentricity * second_eccentricity

        q0 = np.longdouble(0.5) * (
            (np.longdouble(1.0) + np.longdouble(3.0) / ep2)
            * np.arctan(second_eccentricity)
            - np.longdouble(3.0) / second_eccentricity
        )
        q0_prime = (
            np.longdouble(3.0)
            * (np.longdouble(1.0) + np.longdouble(1.0) / ep2)
            * (
                np.longdouble(1.0)
                - np.arctan(second_eccentricity) / second_eccentricity
            )
            - np.longdouble(1.0)
        )
        rotational_m = omega * omega * a * a * b / mu

        gamma_e = (
            mu
            / (a * b)
            * (
                np.longdouble(1.0)
                - rotational_m
                - rotational_m
                * second_eccentricity
                * q0_prime
                / (np.longdouble(6.0) * q0)
            )
        )
        gamma_p = (
            mu
            / (a * a)
            * (
                np.longdouble(1.0)
                + rotational_m
                * second_eccentricity
                * q0_prime
                / (np.longdouble(3.0) * q0)
            )
        )
        somigliana_k = b * gamma_p / (a * gamma_e) - np.longdouble(1.0)

        object.__setattr__(self, "gravitational_parameter_m3_s2", mu)
        object.__setattr__(self, "equatorial_gravity_m_s2", gamma_e)
        object.__setattr__(self, "polar_gravity_m_s2", gamma_p)
        object.__setattr__(self, "somigliana_k", somigliana_k)
        object.__setattr__(self, "rotational_parameter_m", rotational_m)

    def surface_gravity_m_s2(self, latitude_rad: Scalar) -> np.longdouble:
        """Return normal gravity on the reference ellipsoid.

        Parameters
        ----------
        latitude_rad
            Geodetic latitude in radians.

        Returns
        -------
        numpy.longdouble
            Gravity magnitude in m/s².
        """

        latitude = _validated_latitude(latitude_rad)
        sin_lat = np.sin(latitude)
        sin_lat_squared = sin_lat * sin_lat
        e2 = self.ellipsoid.first_eccentricity_squared

        return np.longdouble(
            self.equatorial_gravity_m_s2
            * (np.longdouble(1.0) + self.somigliana_k * sin_lat_squared)
            / np.sqrt(np.longdouble(1.0) - e2 * sin_lat_squared)
        )

    def gravity_m_s2(
        self,
        latitude_rad: Scalar,
        height_m: Scalar = 0.0,
    ) -> np.longdouble:
        """Return normal gravity at latitude and ellipsoidal height.

        Parameters
        ----------
        latitude_rad
            Geodetic latitude in radians.
        height_m
            Ellipsoidal height in metres.

        Returns
        -------
        numpy.longdouble
            Gravity magnitude in m/s².
        """

        latitude = _validated_latitude(latitude_rad)
        height = _finite_scalar(height_m, name="height_m")
        a = self.ellipsoid.semi_major_axis_m
        if height <= -a:
            raise ValueError(
                "height_m places the point at or beyond the model centre, "
                f"got {height!r}"
            )

        sin_lat = np.sin(latitude)
        sin_lat_squared = sin_lat * sin_lat
        surface_gravity = self.surface_gravity_m_s2(latitude)
        f = self.ellipsoid.flattening
        m = self.rotational_parameter_m
        height_ratio = height / a

        correction = (
            np.longdouble(1.0)
            - np.longdouble(2.0)
            * (
                np.longdouble(1.0)
                + f
                + m
                - np.longdouble(2.0) * f * sin_lat_squared
            )
            * height_ratio
            + np.longdouble(3.0) * height_ratio * height_ratio
        )
        return np.longdouble(surface_gravity * correction)


@dataclass(frozen=True, slots=True)
class InverseSquareGravity:
    """Spherical central-gravity approximation ``mu / r**2``.

    Parameters
    ----------
    gravitational_parameter_m3_s2
        Geocentric gravitational constant ``GM`` in m³/s².
    reference_radius_m
        Reference spherical radius in metres.

    Notes
    -----
    Rotation and latitude-dependent centrifugal acceleration are not included.
    """

    gravitational_parameter_m3_s2: Scalar
    reference_radius_m: Scalar

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "gravitational_parameter_m3_s2",
            _positive_scalar(
                self.gravitational_parameter_m3_s2,
                name="gravitational_parameter_m3_s2",
            ),
        )
        object.__setattr__(
            self,
            "reference_radius_m",
            _positive_scalar(self.reference_radius_m, name="reference_radius_m"),
        )

    def gravity_m_s2(
        self,
        latitude_rad: Scalar,
        height_m: Scalar = 0.0,
    ) -> np.longdouble:
        """Return gravity at ``reference_radius_m + height_m``.

        Parameters
        ----------
        latitude_rad
            Geodetic latitude in radians. Validated for API consistency but not
            used by the spherical formula.
        height_m
            Height above the reference sphere in metres.

        Returns
        -------
        numpy.longdouble
            Gravity magnitude in m/s².
        """

        _validated_latitude(latitude_rad)
        height = _finite_scalar(height_m, name="height_m")
        radius = self.reference_radius_m + height
        if radius <= 0.0:
            raise ValueError(
                f"reference_radius_m + height_m must be > 0, got {radius!r}"
            )
        return np.longdouble(self.gravitational_parameter_m3_s2 / (radius * radius))


@dataclass(frozen=True, slots=True)
class ConstantGravity:
    """Constant engineering gravity approximation.

    Parameters
    ----------
    value_m_s2
        Constant gravity magnitude in m/s². Must be positive.
    """

    value_m_s2: Scalar = 9.80665

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "value_m_s2",
            _positive_scalar(self.value_m_s2, name="value_m_s2"),
        )

    def gravity_m_s2(
        self,
        latitude_rad: Scalar,
        height_m: Scalar = 0.0,
    ) -> np.longdouble:
        """Return the configured constant gravity value.

        Parameters
        ----------
        latitude_rad
            Geodetic latitude in radians. Validated but not used.
        height_m
            Height in metres. Validated but not used.

        Returns
        -------
        numpy.longdouble
            Gravity magnitude in m/s².
        """

        _validated_latitude(latitude_rad)
        _finite_scalar(height_m, name="height_m")
        return np.longdouble(self.value_m_s2)
