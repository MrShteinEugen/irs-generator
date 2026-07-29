"""Core strapdown inertial-navigation mechanization."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, isfinite, sin, tan

from .corrections import (
    CorrectionContext,
    CorrectionStrategy,
    NoCorrection,
)
from irs_generator.earth_model import EarthModel, GeodeticPosition, WGS84EarthModel
from irs_generator.gps_model import GnssSample
from irs_generator.navigation_model import (
    NavigationState,
    NavigationVelocity,
)
from irs_generator.utils.math import Vector3, _normalize_longitude
from .imu import ImuSample
from .rotation import (
    AttitudeIntegrator,
    LieGroupAttitudeIntegrator,
    Matrix3,
    dcm_body_to_nav_to_euler,
    euler_to_dcm_body_to_nav,
)

__all__ = ["MechanizationConfig", "StrapdownINS"]


@dataclass(frozen=True, slots=True)
class MechanizationConfig:
    minimum_absolute_cos_latitude: float = 1e-8
    normalize_longitude: bool = True

    def __post_init__(self) -> None:
        threshold = float(self.minimum_absolute_cos_latitude)
        if not isfinite(threshold) or not 0.0 < threshold < 1.0:
            raise ValueError(
                "minimum_absolute_cos_latitude must be finite and in (0, 1)"
            )
        object.__setattr__(self, "minimum_absolute_cos_latitude", threshold)





class StrapdownINS:
    """Stateful ENU strapdown INS with replaceable Earth and aiding models."""

    __slots__ = (
        "_attitude_integrator",
        "_body_to_nav_dcm",
        "_config",
        "_correction",
        "_earth_model",
        "_state",
    )

    def __init__(
        self,
        initial_state: NavigationState,
        *,
        earth_model: EarthModel | None = None,
        correction: CorrectionStrategy | None = None,
        attitude_integrator: AttitudeIntegrator | None = None,
        config: MechanizationConfig | None = None,
    ) -> None:
        self._earth_model = earth_model if earth_model is not None else WGS84EarthModel()
        self._correction = correction if correction is not None else NoCorrection()
        self._attitude_integrator = (
            attitude_integrator
            if attitude_integrator is not None
            else LieGroupAttitudeIntegrator()
        )
        self._config = config if config is not None else MechanizationConfig()
        self._state = initial_state
        self._body_to_nav_dcm = euler_to_dcm_body_to_nav(initial_state.attitude)

    @property
    def state(self) -> NavigationState:
        return self._state

    @property
    def earth_model(self) -> EarthModel:
        return self._earth_model

    @property
    def body_to_nav_dcm(self) -> Matrix3:
        return self._body_to_nav_dcm.copy()

    def reset(self, initial_state: NavigationState) -> None:
        self._state = initial_state
        self._body_to_nav_dcm = euler_to_dcm_body_to_nav(initial_state.attitude)
        self._correction.reset()

    def step(
        self,
        imu_sample: ImuSample,
        dt_s: float,
        gnss_sample: GnssSample | None = None,
    ) -> NavigationState:
        dt = float(dt_s)
        if not isfinite(dt) or dt <= 0.0:
            raise ValueError(f"dt_s must be finite and > 0, got {dt_s!r}")

        old_state = self._state
        specific_force_nav_array = (
            self._body_to_nav_dcm
            @ imu_sample.specific_force_body_m_s2.as_array()
        )
        specific_force_nav = Vector3.from_iterable(specific_force_nav_array)
        base_navigation_rate = self.navigation_rate_rad_s(old_state)

        context = CorrectionContext(
            imu=imu_sample,
            ins_state=old_state,
            gnss_sample=gnss_sample,
            body_to_nav_dcm=self._body_to_nav_dcm.copy(),
            specific_force_nav_m_s2=specific_force_nav,
            base_navigation_rate_rad_s=base_navigation_rate,
            earth_model=self._earth_model,
            dt_s=dt,
        )
        correction = self._correction.compute(context)

        navigation_rate_for_velocity = (
            base_navigation_rate + correction.navigation_rate_rad_s
        )
        navigation_rate_for_attitude = (
            navigation_rate_for_velocity + correction.attitude_only_rate_rad_s
        )

        new_dcm = self._attitude_integrator.propagate(
            self._body_to_nav_dcm,
            imu_sample.angular_rate_body_rad_s.as_array(),
            navigation_rate_for_attitude.as_array(),
            dt,
        )
        new_velocity = self._propagate_velocity(
            old_state,
            specific_force_nav,
            navigation_rate_for_velocity,
            correction.acceleration_nav_m_s2,
            correction.velocity_delta_nav_m_s,
            dt,
        )
        new_position = self._propagate_position(
            old_state,
            correction.position_rate_lon_lat_height,
            correction.position_delta_lon_lat_height,
            dt,
        )
        new_attitude = dcm_body_to_nav_to_euler(new_dcm)

        new_state = NavigationState(
            velocity=new_velocity,
            position=new_position,
            attitude=new_attitude,
            correction_applied=correction.applied,
        )
        self._state = new_state
        self._body_to_nav_dcm = new_dcm
        return new_state

    def navigation_rate_rad_s(self, state: NavigationState) -> Vector3:
        """Earth plus transport rate of the ENU navigation frame."""

        velocity = state.velocity
        position = state.position
        latitude = position.latitude_rad
        meridional_radius = (
            self._earth_model.meridional_radius_m(latitude) + position.height_m
        )
        prime_vertical_radius = (
            self._earth_model.prime_vertical_radius_m(latitude)
            + position.height_m
        )
        if meridional_radius <= 0.0 or prime_vertical_radius <= 0.0:
            raise ValueError("Earth curvature radius plus height must be > 0")

        return Vector3(
            -velocity.north_m_s / meridional_radius,
            velocity.east_m_s / prime_vertical_radius
            + self._earth_model.rotation.angular_velocity_rad_s * cos(latitude),
            velocity.east_m_s / prime_vertical_radius * tan(latitude)
            + self._earth_model.rotation.angular_velocity_rad_s * sin(latitude),
        )

    def _propagate_velocity(
        self,
        state: NavigationState,
        specific_force_nav: Vector3,
        navigation_rate: Vector3,
        acceleration_correction: Vector3,
        direct_velocity_delta: Vector3,
        dt_s: float,
    ) -> NavigationVelocity:
        velocity = state.velocity
        latitude = state.position.latitude_rad
        earth_rate = self._earth_model.rotation.angular_velocity_rad_s
        sin_latitude = sin(latitude)
        cos_latitude = cos(latitude)
        gravity = self._earth_model.gravity_m_s2(
            latitude,
            state.position.height_m,
        )

        acceleration_east = (
            specific_force_nav.x
            + velocity.north_m_s
            * (navigation_rate.z + earth_rate * sin_latitude)
            - velocity.up_m_s
            * (earth_rate * cos_latitude + navigation_rate.y)
            + acceleration_correction.x
        )
        acceleration_north = (
            specific_force_nav.y
            - velocity.east_m_s
            * (navigation_rate.z + earth_rate * sin_latitude)
            - velocity.up_m_s * navigation_rate.x
            + acceleration_correction.y
        )
        acceleration_up = (
            specific_force_nav.z
            + velocity.east_m_s
            * (navigation_rate.y + earth_rate * cos_latitude)
            - velocity.north_m_s * navigation_rate.x
            - gravity
            + acceleration_correction.z
        )

        return NavigationVelocity(
            velocity.east_m_s
            + dt_s * acceleration_east
            + direct_velocity_delta.x,
            velocity.north_m_s
            + dt_s * acceleration_north
            + direct_velocity_delta.y,
            velocity.up_m_s
            + dt_s * acceleration_up
            + direct_velocity_delta.z,
        )

    def _propagate_position(
        self,
        state: NavigationState,
        rate_correction: Vector3,
        direct_position_delta: Vector3,
        dt_s: float,
    ) -> GeodeticPosition:
        velocity = state.velocity
        position = state.position
        meridional_radius = (
            self._earth_model.meridional_radius_m(position.latitude_rad)
            + position.height_m
        )
        prime_vertical_radius = (
            self._earth_model.prime_vertical_radius_m(position.latitude_rad)
            + position.height_m
        )

        latitude_rate = velocity.north_m_s / meridional_radius
        predicted_latitude = (
            position.latitude_rad
            + dt_s * (latitude_rate + rate_correction.y)
            + direct_position_delta.y
        )
        cos_latitude = cos(predicted_latitude)
        if abs(cos_latitude) < self._config.minimum_absolute_cos_latitude:
            raise ValueError(
                "longitude propagation is singular too close to a pole"
            )

        longitude_rate = velocity.east_m_s / (
            prime_vertical_radius * cos_latitude
        )
        longitude = (
            position.longitude_rad
            + dt_s * (longitude_rate + rate_correction.x)
            + direct_position_delta.x
        )
        if self._config.normalize_longitude:
            longitude = _normalize_longitude(longitude)

        height = (
            position.height_m
            + dt_s * (velocity.up_m_s + rate_correction.z)
            + direct_position_delta.z
        )
        return GeodeticPosition(longitude, predicted_latitude, height)
