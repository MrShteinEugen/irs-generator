from dataclasses import dataclass
from typing import ClassVar, Protocol, runtime_checkable

import numpy as np

from irs_generator.utils._validation import _positive_scalar, _validated_latitude
from irs_generator.utils.math import Scalar

from .geometry import ReferenceEllipsoid
from .gravity import GravityModel, InverseSquareGravity, SomiglianaNormalGravity
from .rotation import RotationParameters

__all__ = [
    "EarthModel",
    "EllipsoidalEarthModel",
    "GRS80EarthModel",
    "SphericalEarthModel",
    "WGS84EarthModel",
]


@runtime_checkable
class EarthModel(Protocol):
    """Structural interface shared by all Earth models."""

    @property
    def name(self) -> str:
        """Return a human-readable model name."""

    @property
    def rotation(self) -> RotationParameters:
        """Return Earth rotation parameters."""

    @property
    def gravity_model(self) -> GravityModel:
        """Return the gravity strategy."""

    @property
    def mean_radius_m(self) -> Scalar:
        """Return the model's representative mean radius in metres."""

    def meridional_radius_m(self, latitude_rad: Scalar) -> Scalar:
        """Return the meridional radius of curvature M in metres."""

    def prime_vertical_radius_m(self, latitude_rad: Scalar) -> Scalar:
        """Return the prime-vertical radius of curvature N in metres."""

    def gravity_m_s2(
        self,
        latitude_rad: Scalar,
        height_m: Scalar = 0.0,
    ) -> Scalar:
        """Return gravity according to the configured gravity strategy."""


@dataclass(frozen=True, slots=True)
class EllipsoidalEarthModel:
    """Earth model composed from ellipsoid, rotation and gravity strategy.

    Parameters
    ----------
    name
        Human-readable model name.
    ellipsoid
        Reference ellipsoid used for curvature radii.
    rotation
        Earth rotation parameters.
    gravitational_parameter_m3_s2
        Geocentric gravitational constant ``GM`` in m³/s².
    gravity_model
        Gravity strategy used by :meth:`gravity_m_s2`.
    """

    name: str
    ellipsoid: ReferenceEllipsoid
    rotation: RotationParameters
    gravitational_parameter_m3_s2: Scalar
    gravity_model: GravityModel

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("name must not be empty")
        object.__setattr__(
            self,
            "gravitational_parameter_m3_s2",
            _positive_scalar(
                self.gravitational_parameter_m3_s2,
                name="gravitational_parameter_m3_s2",
            ),
        )
        if not isinstance(self.gravity_model, GravityModel):
            raise TypeError("gravity_model must implement GravityModel")

    @property
    def mean_radius_m(self) -> np.longdouble:
        return self.ellipsoid.mean_radius_m

    def meridional_radius_m(self, latitude_rad: Scalar) -> np.longdouble:
        return self.ellipsoid.meridional_radius_m(latitude_rad)

    def prime_vertical_radius_m(self, latitude_rad: Scalar) -> np.longdouble:
        return self.ellipsoid.prime_vertical_radius_m(latitude_rad)

    def gravity_m_s2(
        self,
        latitude_rad: Scalar,
        height_m: Scalar = 0.0,
    ) -> np.longdouble:
        return np.longdouble(self.gravity_model.gravity_m_s2(latitude_rad, height_m))


class WGS84EarthModel(EllipsoidalEarthModel):
    """WGS 84 ellipsoidal Earth model with normal gravity."""

    __slots__ = ()

    SEMI_MAJOR_AXIS_M: ClassVar[Scalar] = 6_378_137.0
    INVERSE_FLATTENING: ClassVar[Scalar] = 298.257_223_563
    GRAVITATIONAL_PARAMETER_M3_S2: ClassVar[Scalar] = 3.986_004_418e14
    ANGULAR_VELOCITY_RAD_S: ClassVar[Scalar] = 7.292_115e-5

    def __init__(self) -> None:
        ellipsoid = ReferenceEllipsoid(
            semi_major_axis_m=self.SEMI_MAJOR_AXIS_M,
            inverse_flattening=self.INVERSE_FLATTENING,
        )
        rotation = RotationParameters(self.ANGULAR_VELOCITY_RAD_S)
        gravity = SomiglianaNormalGravity(
            ellipsoid=ellipsoid,
            rotation=rotation,
            gravitational_parameter_m3_s2=self.GRAVITATIONAL_PARAMETER_M3_S2,
        )
        super().__init__(
            name="WGS 84",
            ellipsoid=ellipsoid,
            rotation=rotation,
            gravitational_parameter_m3_s2=self.GRAVITATIONAL_PARAMETER_M3_S2,
            gravity_model=gravity,
        )


class GRS80EarthModel(EllipsoidalEarthModel):
    """GRS 80 ellipsoidal Earth model with normal gravity."""

    __slots__ = ()

    SEMI_MAJOR_AXIS_M: ClassVar[Scalar] = 6_378_137.0
    INVERSE_FLATTENING: ClassVar[Scalar] = 298.257_222_101
    GRAVITATIONAL_PARAMETER_M3_S2: ClassVar[Scalar] = 3.986_005e14
    ANGULAR_VELOCITY_RAD_S: ClassVar[Scalar] = 7.292_115e-5

    def __init__(self) -> None:
        ellipsoid = ReferenceEllipsoid(
            semi_major_axis_m=self.SEMI_MAJOR_AXIS_M,
            inverse_flattening=self.INVERSE_FLATTENING,
        )
        rotation = RotationParameters(self.ANGULAR_VELOCITY_RAD_S)
        gravity = SomiglianaNormalGravity(
            ellipsoid=ellipsoid,
            rotation=rotation,
            gravitational_parameter_m3_s2=self.GRAVITATIONAL_PARAMETER_M3_S2,
        )
        super().__init__(
            name="GRS 80",
            ellipsoid=ellipsoid,
            rotation=rotation,
            gravitational_parameter_m3_s2=self.GRAVITATIONAL_PARAMETER_M3_S2,
            gravity_model=gravity,
        )


@dataclass(frozen=True, slots=True, init=False)
class SphericalEarthModel:
    """Configurable spherical Earth approximation.

    Parameters
    ----------
    name
        Human-readable model name.
    radius_m
        Spherical Earth radius in metres.
    gravitational_parameter_m3_s2
        Geocentric gravitational constant ``GM`` in m³/s².
    angular_velocity_rad_s
        Earth angular velocity in radians per second.
    gravity_model
        Optional gravity strategy. If omitted, inverse-square gravity is used.
    """

    DEFAULT_RADIUS_M: ClassVar[Scalar] = 6_371_008.8
    DEFAULT_GRAVITATIONAL_PARAMETER_M3_S2: ClassVar[Scalar] = 3.986_004_418e14
    DEFAULT_ANGULAR_VELOCITY_RAD_S: ClassVar[Scalar] = 7.292_115e-5

    name: str
    radius_m: Scalar
    rotation: RotationParameters
    gravitational_parameter_m3_s2: Scalar
    gravity_model: GravityModel

    def __init__(
        self,
        *,
        name: str = "Mean spherical Earth",
        radius_m: Scalar = DEFAULT_RADIUS_M,
        gravitational_parameter_m3_s2: Scalar = (DEFAULT_GRAVITATIONAL_PARAMETER_M3_S2),
        angular_velocity_rad_s: Scalar = DEFAULT_ANGULAR_VELOCITY_RAD_S,
        gravity_model: GravityModel | None = None,
    ) -> None:
        if not name.strip():
            raise ValueError("name must not be empty")

        radius = _positive_scalar(radius_m, name="radius_m")
        mu = _positive_scalar(
            gravitational_parameter_m3_s2,
            name="gravitational_parameter_m3_s2",
        )
        rotation = RotationParameters(angular_velocity_rad_s)
        selected_gravity_model = (
            InverseSquareGravity(
                gravitational_parameter_m3_s2=mu,
                reference_radius_m=radius,
            )
            if gravity_model is None
            else gravity_model
        )
        if not isinstance(selected_gravity_model, GravityModel):
            raise TypeError("gravity_model must implement GravityModel")

        object.__setattr__(self, "name", name)
        object.__setattr__(self, "radius_m", radius)
        object.__setattr__(self, "rotation", rotation)
        object.__setattr__(self, "gravitational_parameter_m3_s2", mu)
        object.__setattr__(self, "gravity_model", selected_gravity_model)

    @property
    def mean_radius_m(self) -> np.longdouble:
        return np.longdouble(self.radius_m)

    def meridional_radius_m(self, latitude_rad: Scalar) -> np.longdouble:
        _validated_latitude(latitude_rad)
        return np.longdouble(self.radius_m)

    def prime_vertical_radius_m(self, latitude_rad: Scalar) -> np.longdouble:
        _validated_latitude(latitude_rad)
        return np.longdouble(self.radius_m)

    def gravity_m_s2(
        self,
        latitude_rad: Scalar,
        height_m: Scalar = 0.0,
    ) -> np.longdouble:
        return np.longdouble(self.gravity_model.gravity_m_s2(latitude_rad, height_m))
