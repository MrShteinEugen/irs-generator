"""Initial analytic alignment."""

from __future__ import annotations

import numpy as np

from irs_generator.earth_model import EarthModel
from irs_generator.irs_model.imu import ImuSample
from irs_generator.navigation_model.navigation import NavigationState
from irs_generator.navigation_model.orientation import EulerAngles

__all__ = ["AnalyticAlignment"]


class AnalyticAlignment:
    __slots__ = ()

    @staticmethod
    def estimate(
        imu_sample: ImuSample,
        reference_state: NavigationState,
        earth_model: EarthModel,
    ) -> EulerAngles:
        """Estimate pitch, roll and heading from stationary IMU data."""

        acceleration = imu_sample.specific_force_body_m_s2
        angular_rate = imu_sample.angular_rate_body_rad_s
        position = reference_state.position
        gravity = earth_model.gravity_m_s2(
            position.latitude_rad,
            position.height_m,
        )

        pitch = float(
            np.arctan2(
                acceleration.y,
                np.hypot(acceleration.x, acceleration.z),
            )
        )
        roll = float(-np.arctan2(acceleration.x, acceleration.z))
        heading = float(
            np.arctan2(
                acceleration.x * angular_rate.z - acceleration.z * angular_rate.x,
                gravity * angular_rate.y
                - acceleration.y
                * earth_model.rotation.angular_velocity_rad_s
                * np.sin(position.latitude_rad),
            )
        )
        return EulerAngles(pitch, roll, heading)
