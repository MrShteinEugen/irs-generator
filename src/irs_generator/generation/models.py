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
from irs_generator.utils._validation import _finite_float

__all__ = ["GeneratedStep", "GenerationDiagnostics", "TargetTrajectoryPoint"]


@dataclass(frozen=True, slots=True)
class TargetTrajectoryPoint:
    """One prepared truth point consumed by the generator.

    Position is required only for the first point. Later positions may be
    omitted because the generator propagates self-consistent coordinates using
    the selected inertial-navigation algorithm.
    """

    time_s: float
    velocity: NavigationVelocity
    attitude: EulerAngles
    position: GeodeticPosition | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "time_s", _finite_float(self.time_s, "time_s"))


@dataclass(frozen=True, slots=True)
class GenerationDiagnostics:
    """Health information for one inverse-synthesis step."""

    iteration_count: int
    residual_norm: float
    converged: bool

    def __post_init__(self) -> None:
        if self.iteration_count < 0:
            raise ValueError("iteration_count must be >= 0")
        object.__setattr__(
            self,
            "residual_norm",
            _finite_float(self.residual_norm, "residual_norm"),
        )


@dataclass(frozen=True, slots=True)
class GeneratedStep:
    """One output row aligned with the legacy IMU and GPS file convention."""

    time_s: float
    imu_sample: ImuSample
    navigation_state: NavigationState
    diagnostics: GenerationDiagnostics

    def __post_init__(self) -> None:
        object.__setattr__(self, "time_s", _finite_float(self.time_s, "time_s"))
