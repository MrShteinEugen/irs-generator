from dataclasses import dataclass, field

import numpy as np

from irs_generator.utils._validation import (
    _finite_scalar,
    _positive_scalar,
    _validated_latitude,
)
from irs_generator.utils.math import Scalar

__all__ = ["ReferenceEllipsoid"]


@dataclass(frozen=True, slots=True)
class ReferenceEllipsoid:
    """Oblate reference ellipsoid.

    Parameters
    ----------
    semi_major_axis_m
        Semi-major axis ``a`` in metres. Must be positive.
    inverse_flattening
        Inverse flattening ``1/f``. Must be greater than one.

    Attributes
    ----------
    flattening
        Flattening ``f``.
    semi_minor_axis_m
        Semi-minor axis ``b`` in metres.
    first_eccentricity_squared
        First eccentricity squared, ``e²``.
    second_eccentricity_squared
        Second eccentricity squared, ``e'²``.
    """

    semi_major_axis_m: Scalar
    inverse_flattening: Scalar

    flattening: np.longdouble = field(init=False)
    semi_minor_axis_m: np.longdouble = field(init=False)
    first_eccentricity_squared: np.longdouble = field(init=False)
    second_eccentricity_squared: np.longdouble = field(init=False)

    def __post_init__(self) -> None:
        a = _positive_scalar(self.semi_major_axis_m, name="semi_major_axis_m")
        inverse_f = _finite_scalar(
            self.inverse_flattening,
            name="inverse_flattening",
        )
        if inverse_f <= 1.0:
            raise ValueError(
                "inverse_flattening must be > 1 for an oblate ellipsoid, "
                f"got {inverse_f!r}"
            )

        flattening = np.longdouble(1.0) / inverse_f
        b = a * (np.longdouble(1.0) - flattening)
        e2 = flattening * (np.longdouble(2.0) - flattening)
        ep2 = e2 / (np.longdouble(1.0) - e2)

        object.__setattr__(self, "semi_major_axis_m", a)
        object.__setattr__(self, "inverse_flattening", inverse_f)
        object.__setattr__(self, "flattening", flattening)
        object.__setattr__(self, "semi_minor_axis_m", b)
        object.__setattr__(self, "first_eccentricity_squared", e2)
        object.__setattr__(self, "second_eccentricity_squared", ep2)

    @property
    def mean_radius_m(self) -> np.longdouble:
        """Arithmetic mean radius ``R1 = (2a + b) / 3``."""

        return (
            np.longdouble(2.0) * self.semi_major_axis_m + self.semi_minor_axis_m
        ) / np.longdouble(3.0)

    @property
    def volumetric_radius_m(self) -> np.longdouble:
        """Radius of a sphere with the same volume as the ellipsoid."""

        a = self.semi_major_axis_m
        b = self.semi_minor_axis_m
        return np.longdouble((a * a * b) ** (np.longdouble(1.0) / np.longdouble(3.0)))

    def meridional_radius_m(self, latitude_rad: Scalar) -> np.longdouble:
        """Return the meridional radius of curvature.

        Parameters
        ----------
        latitude_rad
            Geodetic latitude in radians.

        Returns
        -------
        numpy.longdouble
            Radius ``M`` in metres.
        """

        latitude = _validated_latitude(latitude_rad)
        sin_lat = np.sin(latitude)
        denominator = (
            np.longdouble(1.0) - self.first_eccentricity_squared * sin_lat * sin_lat
        )
        return np.longdouble(
            self.semi_major_axis_m
            * (np.longdouble(1.0) - self.first_eccentricity_squared)
            / (denominator * np.sqrt(denominator))
        )

    def prime_vertical_radius_m(self, latitude_rad: Scalar) -> np.longdouble:
        """Return the prime-vertical radius of curvature.

        Parameters
        ----------
        latitude_rad
            Geodetic latitude in radians.

        Returns
        -------
        numpy.longdouble
            Radius ``N`` in metres.
        """

        latitude = _validated_latitude(latitude_rad)
        sin_lat = np.sin(latitude)
        denominator = (
            np.longdouble(1.0) - self.first_eccentricity_squared * sin_lat * sin_lat
        )
        return np.longdouble(self.semi_major_axis_m / np.sqrt(denominator))

    def geocentric_surface_radius_m(self, latitude_rad: Scalar) -> np.longdouble:
        """Return the distance from ellipsoid centre to surface.

        Parameters
        ----------
        latitude_rad
            Geodetic latitude in radians, not geocentric latitude.

        Returns
        -------
        numpy.longdouble
            Surface radius in metres.
        """

        latitude = _validated_latitude(latitude_rad)
        sin_lat = np.sin(latitude)
        cos_lat = np.cos(latitude)
        a = self.semi_major_axis_m
        b = self.semi_minor_axis_m

        numerator = (a * a * cos_lat) ** 2 + (b * b * sin_lat) ** 2
        denominator = (a * cos_lat) ** 2 + (b * sin_lat) ** 2
        return np.longdouble(np.sqrt(numerator / denominator))
