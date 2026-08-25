"""Contracts for interchangeable inertial-navigation algorithms."""

from __future__ import annotations

from typing import Protocol, Self, runtime_checkable

from irs_generator.gps_model import GnssSample
from irs_generator.navigation_model import NavigationState
from irs_generator.utils.math import Scalar

from .imu import ImuSample

__all__ = ["InertialNavigationAlgorithm", "NavigationAlgorithm"]


@runtime_checkable
class NavigationAlgorithm(Protocol):
    """Stateful algorithm that integrates IMU samples into navigation state.

    Notes
    -----
    Implement this protocol for algorithms that are used by
    :class:`~irs_generator.irs_model.InertialReferenceSystem`.
    """

    @property
    def state(self) -> NavigationState:
        """Return the most recently computed navigation state."""

    def reset(self, initial_state: NavigationState) -> None:
        """Reset the algorithm to a known state.

        Parameters
        ----------
        initial_state
            State that becomes the current algorithm state.
        """

    def step(
        self,
        imu_sample: ImuSample,
        dt_s: Scalar,
        gnss_sample: GnssSample | None = None,
    ) -> NavigationState:
        """Integrate one IMU sample.

        Parameters
        ----------
        imu_sample
            IMU sample to integrate.
        dt_s
            Positive integration step in seconds.
        gnss_sample
            Optional GNSS sample for aiding.

        Returns
        -------
        NavigationState
            Updated navigation state.
        """


@runtime_checkable
class InertialNavigationAlgorithm(NavigationAlgorithm, Protocol):
    """Navigation algorithm that can be safely used by inverse generators."""

    def fork(self) -> Self:
        """Return an independent copy at the current state.

        Returns
        -------
        InertialNavigationAlgorithm
            Copy that can be advanced without mutating the original instance.
        """
