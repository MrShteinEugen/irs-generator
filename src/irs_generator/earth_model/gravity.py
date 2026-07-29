from dataclasses import dataclass, field
from math import atan, sin, sqrt
from typing import Protocol, runtime_checkable

from irs_generator.utils._validation import (
    _finite_float,
    _positive_float,
    _validated_latitude,
)

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
        latitude_rad: float,
        height_m: float = 0.0,
    ) -> float:
        """Return gravity magnitude in m/s²."""


@dataclass(frozen=True, slots=True)
class SomiglianaNormalGravity:
    """Normal gravity for a rotating reference ellipsoid.

    Surface gravity is calculated with Somigliana's formula. Gravity above or
    below the ellipsoid uses the standard second-order ellipsoidal height
    expansion. This is a near-surface normal-gravity model, not a local measured
    gravity model and not an orbital-force model.
    """

    ellipsoid: ReferenceEllipsoid
    rotation: RotationParameters
    gravitational_parameter_m3_s2: float

    equatorial_gravity_m_s2: float = field(init=False)
    polar_gravity_m_s2: float = field(init=False)
    somigliana_k: float = field(init=False)
    rotational_parameter_m: float = field(init=False)

    def __post_init__(self) -> None:
        mu = _positive_float(
            self.gravitational_parameter_m3_s2,
            name="gravitational_parameter_m3_s2",
        )
        a = self.ellipsoid.semi_major_axis_m
        b = self.ellipsoid.semi_minor_axis_m
        omega = self.rotation.angular_velocity_rad_s

        second_eccentricity = sqrt(self.ellipsoid.second_eccentricity_squared)
        ep2 = second_eccentricity * second_eccentricity

        q0 = 0.5 * (
            (1.0 + 3.0 / ep2) * atan(second_eccentricity) - 3.0 / second_eccentricity
        )
        q0_prime = (
            3.0
            * (1.0 + 1.0 / ep2)
            * (1.0 - atan(second_eccentricity) / second_eccentricity)
            - 1.0
        )
        rotational_m = omega * omega * a * a * b / mu

        gamma_e = (
            mu
            / (a * b)
            * (
                1.0
                - rotational_m
                - rotational_m * second_eccentricity * q0_prime / (6.0 * q0)
            )
        )
        gamma_p = (
            mu
            / (a * a)
            * (1.0 + rotational_m * second_eccentricity * q0_prime / (3.0 * q0))
        )
        somigliana_k = b * gamma_p / (a * gamma_e) - 1.0

        object.__setattr__(self, "gravitational_parameter_m3_s2", mu)
        object.__setattr__(self, "equatorial_gravity_m_s2", gamma_e)
        object.__setattr__(self, "polar_gravity_m_s2", gamma_p)
        object.__setattr__(self, "somigliana_k", somigliana_k)
        object.__setattr__(self, "rotational_parameter_m", rotational_m)

    def surface_gravity_m_s2(self, latitude_rad: float) -> float:
        """Normal gravity on the reference ellipsoid."""

        latitude = _validated_latitude(latitude_rad)
        sin_lat = sin(latitude)
        sin_lat_squared = sin_lat * sin_lat
        e2 = self.ellipsoid.first_eccentricity_squared

        return (
            self.equatorial_gravity_m_s2
            * (1.0 + self.somigliana_k * sin_lat_squared)
            / sqrt(1.0 - e2 * sin_lat_squared)
        )

    def gravity_m_s2(
        self,
        latitude_rad: float,
        height_m: float = 0.0,
    ) -> float:
        """Normal gravity at geodetic latitude and ellipsoidal height."""

        latitude = _validated_latitude(latitude_rad)
        height = _finite_float(height_m, name="height_m")
        a = self.ellipsoid.semi_major_axis_m
        if height <= -a:
            raise ValueError(
                "height_m places the point at or beyond the model centre, "
                f"got {height!r}"
            )

        sin_lat = sin(latitude)
        sin_lat_squared = sin_lat * sin_lat
        surface_gravity = self.surface_gravity_m_s2(latitude)
        f = self.ellipsoid.flattening
        m = self.rotational_parameter_m
        height_ratio = height / a

        correction = (
            1.0
            - 2.0 * (1.0 + f + m - 2.0 * f * sin_lat_squared) * height_ratio
            + 3.0 * height_ratio * height_ratio
        )
        return surface_gravity * correction


@dataclass(frozen=True, slots=True)
class InverseSquareGravity:
    """Spherical central-gravity approximation μ / r².

    Rotation and latitude-dependent centrifugal acceleration are intentionally
    excluded. This model is useful when a simple central field is required.
    """

    gravitational_parameter_m3_s2: float
    reference_radius_m: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "gravitational_parameter_m3_s2",
            _positive_float(
                self.gravitational_parameter_m3_s2,
                name="gravitational_parameter_m3_s2",
            ),
        )
        object.__setattr__(
            self,
            "reference_radius_m",
            _positive_float(self.reference_radius_m, name="reference_radius_m"),
        )

    def gravity_m_s2(
        self,
        latitude_rad: float,
        height_m: float = 0.0,
    ) -> float:
        _validated_latitude(latitude_rad)
        height = _finite_float(height_m, name="height_m")
        radius = self.reference_radius_m + height
        if radius <= 0.0:
            raise ValueError(
                f"reference_radius_m + height_m must be > 0, got {radius!r}"
            )
        return self.gravitational_parameter_m3_s2 / (radius * radius)


@dataclass(frozen=True, slots=True)
class ConstantGravity:
    """Constant engineering gravity approximation."""

    value_m_s2: float = 9.80665

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "value_m_s2",
            _positive_float(self.value_m_s2, name="value_m_s2"),
        )

    def gravity_m_s2(
        self,
        latitude_rad: float,
        height_m: float = 0.0,
    ) -> float:
        _validated_latitude(latitude_rad)
        _finite_float(height_m, name="height_m")
        return self.value_m_s2
