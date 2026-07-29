"""Contracts for interchangeable inertial-navigation algorithms."""

from __future__ import annotations

from typing import Protocol, Self, runtime_checkable

from irs_generator.gps_model import GnssSample
from irs_generator.navigation_model import NavigationState

from .imu import ImuSample

__all__ = ["InertialNavigationAlgorithm", "NavigationAlgorithm"]


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


@runtime_checkable
class InertialNavigationAlgorithm(NavigationAlgorithm, Protocol):
    """Navigation algorithm that can be safely trialed by the data generator."""

    def fork(self) -> Self:
        """Return an independent copy at the current algorithm state."""
