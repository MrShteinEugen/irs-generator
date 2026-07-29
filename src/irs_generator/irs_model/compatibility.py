"""Structural adapters for migration from the original record classes."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from irs_generator.gps_model.gnss import GnssSample
from irs_generator.navigation_model.navigation import NavigationState

from .imu import ImuSample

__all__ = [
    "gnss_sample_from_legacy",
    "imu_sample_from_legacy",
    "navigation_state_from_legacy",
    "navigation_state_to_legacy",
]


def navigation_state_from_legacy(record: Any) -> NavigationState:
    """Convert an object exposing the original NavigationRecord attributes."""

    return NavigationState.from_components(
        record.velocity_east_x_m_per_s,
        record.velocity_north_y_m_per_s,
        record.velocity_vertical_z_m_per_s,
        record.coordinate_east_x_rad,
        record.coordinate_north_y_rad,
        record.coordinate_vertical_z_m,
        record.pitch_rad,
        record.roll_rad,
        record.heading_rad,
        bool(record.correction_available),
    )


def gnss_sample_from_legacy(record: Any) -> GnssSample:
    """Convert an original GPS NavigationRecord into a GNSS sample."""

    return GnssSample.from_components(
        record.velocity_east_x_m_per_s,
        record.velocity_north_y_m_per_s,
        record.velocity_vertical_z_m_per_s,
        record.coordinate_east_x_rad,
        record.coordinate_north_y_rad,
        record.coordinate_vertical_z_m,
        bool(record.correction_available),
    )


def imu_sample_from_legacy(record: Any) -> ImuSample:
    """Convert an object exposing the original IMURecord attributes."""

    return ImuSample.from_components(
        record.acc_body_x,
        record.acc_body_y,
        record.acc_body_z,
        record.gyr_body_x,
        record.gyr_body_y,
        record.gyr_body_z,
    )


def navigation_state_to_legacy(
    state: NavigationState,
    record_factory: Callable[..., Any],
) -> Any:
    """Create an old-style record without importing the legacy package."""

    return record_factory(
        state.velocity.east_m_s,
        state.velocity.north_m_s,
        state.velocity.up_m_s,
        state.position.longitude_rad,
        state.position.latitude_rad,
        state.position.height_m,
        state.attitude.pitch_rad,
        state.attitude.roll_rad,
        state.attitude.heading_rad,
        state.correction_applied,
    )
