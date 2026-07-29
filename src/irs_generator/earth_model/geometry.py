from dataclasses import dataclass, field
from math import sin, sqrt, cos

from irs_generator.utils._validation import _finite_float, _positive_float, _validated_latitude

__all__ = ["ReferenceEllipsoid"]


@dataclass(frozen=True, slots=True)
class ReferenceEllipsoid:
    """Immutable oblate reference ellipsoid.

    The ellipsoid is defined by the semi-major axis ``a`` and inverse
    flattening ``1/f``. All other geometric values are derived once during
    construction, preventing inconsistent combinations of ``a``, ``b`` and
    eccentricity.
    """

    semi_major_axis_m: float
    inverse_flattening: float

    flattening: float = field(init=False)
    semi_minor_axis_m: float = field(init=False)
    first_eccentricity_squared: float = field(init=False)
    second_eccentricity_squared: float = field(init=False)

    def __post_init__(self) -> None:
        a = _positive_float(self.semi_major_axis_m, name="semi_major_axis_m")
        inverse_f = _finite_float(
            self.inverse_flattening,
            name="inverse_flattening",
        )
        if inverse_f <= 1.0:
            raise ValueError(
                "inverse_flattening must be > 1 for an oblate ellipsoid, "
                f"got {inverse_f!r}"
            )

        flattening = 1.0 / inverse_f
        b = a * (1.0 - flattening)
        e2 = flattening * (2.0 - flattening)
        ep2 = e2 / (1.0 - e2)

        object.__setattr__(self, "semi_major_axis_m", a)
        object.__setattr__(self, "inverse_flattening", inverse_f)
        object.__setattr__(self, "flattening", flattening)
        object.__setattr__(self, "semi_minor_axis_m", b)
        object.__setattr__(self, "first_eccentricity_squared", e2)
        object.__setattr__(self, "second_eccentricity_squared", ep2)

    @property
    def mean_radius_m(self) -> float:
        """Arithmetic mean radius R₁ = (2a + b) / 3."""

        return (2.0 * self.semi_major_axis_m + self.semi_minor_axis_m) / 3.0

    @property
    def volumetric_radius_m(self) -> float:
        """Radius of a sphere having the same volume as the ellipsoid."""

        a = self.semi_major_axis_m
        b = self.semi_minor_axis_m
        return (a * a * b) ** (1.0 / 3.0)

    def meridional_radius_m(self, latitude_rad: float) -> float:
        """Meridional radius of curvature M at geodetic latitude."""

        latitude = _validated_latitude(latitude_rad)
        sin_lat = sin(latitude)
        denominator = 1.0 - self.first_eccentricity_squared * sin_lat * sin_lat
        return (
                self.semi_major_axis_m
                * (1.0 - self.first_eccentricity_squared)
                / (denominator * sqrt(denominator))
        )

    def prime_vertical_radius_m(self, latitude_rad: float) -> float:
        """Prime-vertical radius of curvature N at geodetic latitude."""

        latitude = _validated_latitude(latitude_rad)
        sin_lat = sin(latitude)
        denominator = 1.0 - self.first_eccentricity_squared * sin_lat * sin_lat
        return self.semi_major_axis_m / sqrt(denominator)

    def geocentric_surface_radius_m(self, latitude_rad: float) -> float:
        """Distance from the ellipsoid centre to its surface.

        ``latitude_rad`` is geodetic latitude, not geocentric latitude.
        """

        latitude = _validated_latitude(latitude_rad)
        sin_lat = sin(latitude)
        cos_lat = cos(latitude)
        a = self.semi_major_axis_m
        b = self.semi_minor_axis_m

        numerator = (a * a * cos_lat) ** 2 + (b * b * sin_lat) ** 2
        denominator = (a * cos_lat) ** 2 + (b * sin_lat) ** 2
        return sqrt(numerator / denominator)
