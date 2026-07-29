"""Contracts for interchangeable inertial-navigation algorithms."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from irs_generator.gps_model import GnssSample
from irs_generator.navigation_model import NavigationState

from .imu import ImuSample

__all__ = ["NavigationAlgorithm"]


@runtime_checkable
class NavigationAlgorithm(Protocol):
    """Stateful algorithm that integrates IMU samples into navigation state."""

    @property
    def state(self) -> NavigationState:
        """Return the most recently computed navigation state."""

    def reset(self, initial_state: NavigationState) -> None:
        """Reset the algorithm to a known initial navigation state."""

    def step(
        self,
        imu_sample: ImuSample,
        dt_s: float,
        gnss_sample: GnssSample | None = None,
    ) -> NavigationState:
        """Integrate one modeled IMU sample, optionally with GNSS aiding."""
