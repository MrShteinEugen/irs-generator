"""Immutable values exchanged by the streaming generation pipeline."""

from __future__ import annotations

from dataclasses import dataclass

from irs_generator.earth_model import GeodeticPosition
from irs_generator.irs_model import ImuSample
from irs_generator.navigation_model import (
    EulerAngles,
    NavigationState,
    NavigationVelocity,
)
from irs_generator.utils._validation import _finite_scalar
from irs_generator.utils.math import Scalar

__all__ = ["GeneratedStep", "GenerationDiagnostics", "TargetTrajectoryPoint"]


@dataclass(frozen=True, slots=True)
class TargetTrajectoryPoint:
    """Prepared truth point consumed by inverse generators.

    Parameters
    ----------
    time_s
        Timestamp in seconds.
    velocity
        Target ENU velocity.
    attitude
        Target attitude.
    position
        Optional target geodetic position. Required for the first point passed
        to :class:`SyntheticDataGenerator`.
    """

    time_s: Scalar
    velocity: NavigationVelocity
    attitude: EulerAngles
    position: GeodeticPosition | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "time_s", _finite_scalar(self.time_s, "time_s"))


@dataclass(frozen=True, slots=True)
class GenerationDiagnostics:
    """Health information for one generated step.

    Parameters
    ----------
    iteration_count
        Number of solver iterations used for the step.
    residual_norm
        Infinity norm of the normalized residual.
    converged
        Whether the solver reached its configured tolerance.
    """

    iteration_count: int
    residual_norm: Scalar
    converged: bool

    def __post_init__(self) -> None:
        if self.iteration_count < 0:
            raise ValueError("iteration_count must be >= 0")
        object.__setattr__(
            self,
            "residual_norm",
            _finite_scalar(self.residual_norm, "residual_norm"),
        )


@dataclass(frozen=True, slots=True)
class GeneratedStep:
    """One generated IMU sample and aligned navigation state.

    Parameters
    ----------
    time_s
        Timestamp in seconds.
    imu_sample
        Generated ideal IMU sample.
    navigation_state
        Navigation state aligned with this output row.
    diagnostics
        Solver or generator diagnostics for the row.
    """

    time_s: Scalar
    imu_sample: ImuSample
    navigation_state: NavigationState
    diagnostics: GenerationDiagnostics

    def __post_init__(self) -> None:
        object.__setattr__(self, "time_s", _finite_scalar(self.time_s, "time_s"))
