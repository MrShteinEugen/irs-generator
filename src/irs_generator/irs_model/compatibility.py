"""Structural adapters for external record classes."""

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
    """Convert a navigation record object to :class:`NavigationState`.

    Parameters
    ----------
    record
        Object exposing velocity, coordinate, attitude and correction fields
        with the expected external attribute names.

    Returns
    -------
    NavigationState
        Converted navigation state.
    """

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
    """Convert a navigation record object to :class:`GnssSample`.

    Parameters
    ----------
    record
        Object exposing velocity, coordinate and validity fields with the
        expected external attribute names.

    Returns
    -------
    GnssSample
        Converted GNSS sample.
    """

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
    """Convert an IMU record object to :class:`ImuSample`.

    Parameters
    ----------
    record
        Object exposing body-frame accelerometer and gyroscope fields with the
        expected external attribute names.

    Returns
    -------
    ImuSample
        Converted IMU sample.
    """

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
    """Create an external navigation record from a state.

    Parameters
    ----------
    state
        Navigation state to convert.
    record_factory
        Callable receiving velocity, position, attitude and correction flag
        components.

    Returns
    -------
    Any
        Object returned by ``record_factory``.
    """

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
