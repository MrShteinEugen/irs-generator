"""Composable GNSS-aiding strategies for the INS mechanization."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, isfinite, radians
from typing import Protocol, Sequence

import numpy as np

from irs_generator.earth_model import EarthModel
from irs_generator.gps_model.gnss import GnssSample
from irs_generator.irs_model.imu import ImuSample
from irs_generator.irs_model.rotation import Matrix3
from irs_generator.navigation_model.navigation import NavigationState
from irs_generator.utils._validation import _non_negative
from irs_generator.utils.math import Vector3, angle_difference_rad

__all__ = [
    "CompositeCorrection",
    "CorrectionContext",
    "CorrectionOutput",
    "CorrectionStrategy",
    "HeightAidingConfig",
    "HeightAidingCorrection",
    "NoCorrection",
    "PositionAidingConfig",
    "PositionAidingCorrection",
    "RadialAttitudeConfig",
    "RadialAttitudeCorrection",
    "VelocityAidingConfig",
    "VelocityAidingCorrection",
]


@dataclass(frozen=True, slots=True)
class CorrectionContext:
    imu: ImuSample
    ins_state: NavigationState
    gnss_sample: GnssSample | None
    body_to_nav_dcm: Matrix3
    specific_force_nav_m_s2: Vector3
    base_navigation_rate_rad_s: Vector3
    earth_model: EarthModel
    dt_s: float

    def __post_init__(self) -> None:
        dt = float(self.dt_s)
        if not isfinite(dt) or dt <= 0.0:
            raise ValueError("dt_s must be finite and > 0")
        matrix = np.array(self.body_to_nav_dcm, dtype=np.float64, copy=True)
        if matrix.shape != (3, 3):
            raise ValueError("body_to_nav_dcm must have shape (3, 3)")
        if not np.all(np.isfinite(matrix)):
            raise ValueError("body_to_nav_dcm must contain only finite values")
        matrix.setflags(write=False)
        object.__setattr__(self, "dt_s", dt)
        object.__setattr__(self, "body_to_nav_dcm", matrix)


@dataclass(frozen=True, slots=True)
class CorrectionOutput:
    """Additive corrections applied during one mechanization step."""

    navigation_rate_rad_s: Vector3 = field(default_factory=Vector3.zero)
    attitude_only_rate_rad_s: Vector3 = field(default_factory=Vector3.zero)
    acceleration_nav_m_s2: Vector3 = field(default_factory=Vector3.zero)
    velocity_delta_nav_m_s: Vector3 = field(default_factory=Vector3.zero)
    position_rate_lon_lat_height: Vector3 = field(default_factory=Vector3.zero)
    position_delta_lon_lat_height: Vector3 = field(default_factory=Vector3.zero)
    applied: bool = False

    def __add__(self, other: "CorrectionOutput") -> "CorrectionOutput":
        if not isinstance(other, CorrectionOutput):
            return NotImplemented
        return CorrectionOutput(
            navigation_rate_rad_s=(
                self.navigation_rate_rad_s + other.navigation_rate_rad_s
            ),
            attitude_only_rate_rad_s=(
                self.attitude_only_rate_rad_s + other.attitude_only_rate_rad_s
            ),
            acceleration_nav_m_s2=(
                self.acceleration_nav_m_s2 + other.acceleration_nav_m_s2
            ),
            velocity_delta_nav_m_s=(
                self.velocity_delta_nav_m_s + other.velocity_delta_nav_m_s
            ),
            position_rate_lon_lat_height=(
                self.position_rate_lon_lat_height
                + other.position_rate_lon_lat_height
            ),
            position_delta_lon_lat_height=(
                self.position_delta_lon_lat_height
                + other.position_delta_lon_lat_height
            ),
            applied=self.applied or other.applied,
        )


class CorrectionStrategy(Protocol):
    def compute(self, context: CorrectionContext) -> CorrectionOutput:
        """Compute corrections for the current step."""

    def reset(self) -> None:
        """Reset state retained by the strategy."""


class NoCorrection:
    __slots__ = ()

    @staticmethod
    def compute(context: CorrectionContext) -> CorrectionOutput:
        del context
        return CorrectionOutput()

    @staticmethod
    def reset() -> None:
        return None


@dataclass(slots=True)
class CompositeCorrection:
    strategies: Sequence[CorrectionStrategy]

    def __post_init__(self) -> None:
        self.strategies = tuple(self.strategies)
        for strategy in self.strategies:
            if not callable(getattr(strategy, "compute", None)):
                raise TypeError("every strategy must provide compute(context)")
            if not callable(getattr(strategy, "reset", None)):
                raise TypeError("every strategy must provide reset()")

    def compute(self, context: CorrectionContext) -> CorrectionOutput:
        result = CorrectionOutput()
        for strategy in self.strategies:
            result = result + strategy.compute(context)
        return result

    def reset(self) -> None:
        for strategy in self.strategies:
            strategy.reset()


@dataclass(frozen=True, slots=True)
class VelocityAidingConfig:
    velocity_error_gain_per_s: float = 0.9
    transport_rate_gain: float = 650.0

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "velocity_error_gain_per_s",
            _non_negative(self.velocity_error_gain_per_s, "velocity_error_gain_per_s"),
        )
        object.__setattr__(
            self,
            "transport_rate_gain",
            _non_negative(self.transport_rate_gain, "transport_rate_gain"),
        )


@dataclass(slots=True)
class VelocityAidingCorrection:
    """Horizontal GNSS velocity aiding.

    The refactor treats both horizontal feedback terms as accelerations, so
    both are integrated consistently with ``dt``. This fixes the asymmetric
    time scaling in the original implementation.
    """

    config: VelocityAidingConfig = field(default_factory=VelocityAidingConfig)

    def compute(self, context: CorrectionContext) -> CorrectionOutput:
        gnss = context.gnss_sample
        if gnss is None or not gnss.valid:
            return CorrectionOutput()

        ins_v = context.ins_state.velocity
        gnss_v = gnss.velocity
        latitude = context.ins_state.position.latitude_rad
        height = gnss.position.height_m

        meridional_radius = (
            context.earth_model.meridional_radius_m(latitude) + height
        )
        prime_vertical_radius = (
            context.earth_model.prime_vertical_radius_m(latitude) + height
        )
        error_east = ins_v.east_m_s - gnss_v.east_m_s
        error_north = ins_v.north_m_s - gnss_v.north_m_s

        return CorrectionOutput(
            navigation_rate_rad_s=Vector3(
                -self.config.transport_rate_gain * error_north / meridional_radius,
                self.config.transport_rate_gain * error_east / prime_vertical_radius,
                0.0,
            ),
            acceleration_nav_m_s2=Vector3(
                -self.config.velocity_error_gain_per_s * error_east,
                -self.config.velocity_error_gain_per_s * error_north,
                0.0,
            ),
            applied=True,
        )

    @staticmethod
    def reset() -> None:
        return None


@dataclass(frozen=True, slots=True)
class PositionAidingConfig:
    position_error_gain_per_step: float = 0.032_554_663_661_856_17
    horizontal_feedback_gain: float = 482.75
    navigation_rate_gain_per_s: float = 4.153_044_950_005_365

    def __post_init__(self) -> None:
        for name in (
            "position_error_gain_per_step",
            "horizontal_feedback_gain",
            "navigation_rate_gain_per_s",
        ):
            object.__setattr__(self, name, _non_negative(getattr(self, name), name))


@dataclass(slots=True)
class PositionAidingCorrection:
    """Horizontal position feedback preserving the source equations."""

    config: PositionAidingConfig = field(default_factory=PositionAidingConfig)

    def compute(self, context: CorrectionContext) -> CorrectionOutput:
        gnss = context.gnss_sample
        if gnss is None or not gnss.valid:
            return CorrectionOutput()

        ins_position = context.ins_state.position
        gnss_position = gnss.position
        latitude = ins_position.latitude_rad
        cos_latitude = cos(latitude)
        longitude_error = angle_difference_rad(
            ins_position.longitude_rad,
            gnss_position.longitude_rad,
        )
        latitude_error = ins_position.latitude_rad - gnss_position.latitude_rad
        gravity = context.earth_model.gravity_m_s2(
            latitude,
            ins_position.height_m,
        )

        return CorrectionOutput(
            navigation_rate_rad_s=Vector3(
                -self.config.navigation_rate_gain_per_s * latitude_error,
                self.config.navigation_rate_gain_per_s
                * latitude_error
                * cos_latitude,
                0.0,
            ),
            acceleration_nav_m_s2=Vector3(
                -self.config.horizontal_feedback_gain
                * longitude_error
                * gravity
                * cos_latitude,
                -self.config.horizontal_feedback_gain
                * latitude_error
                * gravity
                * cos_latitude,
                0.0,
            ),
            position_delta_lon_lat_height=Vector3(
                -self.config.position_error_gain_per_step * longitude_error,
                -self.config.position_error_gain_per_step * latitude_error,
                0.0,
            ),
            applied=True,
        )

    @staticmethod
    def reset() -> None:
        return None


@dataclass(frozen=True, slots=True)
class HeightAidingConfig:
    vertical_acceleration_gain_per_s2: float = 0.217_031_091_079_041_12
    height_error_gain_per_step: float = 1.024_034_961_027_680_8

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "vertical_acceleration_gain_per_s2",
            _non_negative(
                self.vertical_acceleration_gain_per_s2,
                "vertical_acceleration_gain_per_s2",
            ),
        )
        object.__setattr__(
            self,
            "height_error_gain_per_step",
            _non_negative(
                self.height_error_gain_per_step,
                "height_error_gain_per_step",
            ),
        )


@dataclass(slots=True)
class HeightAidingCorrection:
    config: HeightAidingConfig = field(default_factory=HeightAidingConfig)

    def compute(self, context: CorrectionContext) -> CorrectionOutput:
        gnss = context.gnss_sample
        if gnss is None or not gnss.valid:
            return CorrectionOutput()

        height_error = (
            context.ins_state.position.height_m - gnss.position.height_m
        )
        return CorrectionOutput(
            acceleration_nav_m_s2=Vector3(
                0.0,
                0.0,
                -self.config.vertical_acceleration_gain_per_s2 * height_error,
            ),
            position_delta_lon_lat_height=Vector3(
                0.0,
                0.0,
                -self.config.height_error_gain_per_step * height_error,
            ),
            applied=True,
        )

    @staticmethod
    def reset() -> None:
        return None


@dataclass(frozen=True, slots=True)
class RadialAttitudeConfig:
    radial_gain_s_per_m: float = 0.02
    heading_gain_per_s: float = 0.5
    max_yaw_rate_rad_s: float = radians(15.0)
    max_horizontal_specific_force_m_s2: float = 0.05**0.5
    min_horizontal_speed_m_s: float = 7.0

    def __post_init__(self) -> None:
        for name in (
            "radial_gain_s_per_m",
            "heading_gain_per_s",
            "max_yaw_rate_rad_s",
            "max_horizontal_specific_force_m_s2",
            "min_horizontal_speed_m_s",
        ):
            object.__setattr__(self, name, _non_negative(getattr(self, name), name))


@dataclass(slots=True)
class RadialAttitudeCorrection:
    """Radial vertical and GNSS course aiding.

    The first valid GNSS sample initializes the differentiator instead of being
    differentiated against an artificial zero velocity. Heading errors are
    wrapped, preventing a 2π discontinuity.
    """

    config: RadialAttitudeConfig = field(default_factory=RadialAttitudeConfig)
    _previous_gnss_velocity: Vector3 | None = field(default=None, init=False)

    def compute(self, context: CorrectionContext) -> CorrectionOutput:
        gnss = context.gnss_sample
        if gnss is None or not gnss.valid:
            self._previous_gnss_velocity = None
            return CorrectionOutput()

        imu_rate = context.imu.angular_rate_body_rad_s
        specific_force = context.specific_force_nav_m_s2
        gnss_velocity = gnss.velocity
        horizontal_force = float(np.hypot(specific_force.x, specific_force.y))
        horizontal_speed = float(
            np.hypot(gnss_velocity.east_m_s, gnss_velocity.north_m_s)
        )

        allowed = (
            abs(imu_rate.z) <= self.config.max_yaw_rate_rad_s
            and horizontal_force
            <= self.config.max_horizontal_specific_force_m_s2
            and horizontal_speed >= self.config.min_horizontal_speed_m_s
        )
        if not allowed:
            self._previous_gnss_velocity = Vector3(
                gnss_velocity.east_m_s,
                gnss_velocity.north_m_s,
                gnss_velocity.up_m_s,
            )
            return CorrectionOutput()

        current_gnss_velocity = Vector3(
            gnss_velocity.east_m_s,
            gnss_velocity.north_m_s,
            gnss_velocity.up_m_s,
        )
        if self._previous_gnss_velocity is None:
            gnss_acceleration = Vector3.zero()
        else:
            gnss_acceleration = Vector3(
                (
                    current_gnss_velocity.x
                    - self._previous_gnss_velocity.x
                )
                / context.dt_s,
                (
                    current_gnss_velocity.y
                    - self._previous_gnss_velocity.y
                )
                / context.dt_s,
                (
                    current_gnss_velocity.z
                    - self._previous_gnss_velocity.z
                )
                / context.dt_s,
            )
        self._previous_gnss_velocity = current_gnss_velocity

        gnss_heading = float(
            np.arctan2(gnss_velocity.east_m_s, gnss_velocity.north_m_s)
            % (2.0 * np.pi)
        )
        heading_error = angle_difference_rad(
            context.ins_state.attitude.heading_rad,
            gnss_heading,
        )

        return CorrectionOutput(
            attitude_only_rate_rad_s=Vector3(
                -self.config.radial_gain_s_per_m
                * (specific_force.y - gnss_acceleration.y),
                self.config.radial_gain_s_per_m
                * (specific_force.x - gnss_acceleration.x),
                -self.config.heading_gain_per_s * heading_error,
            ),
            applied=True,
        )

    def reset(self) -> None:
        self._previous_gnss_velocity = None
