"""Validation and unit contracts for canonical streamed trajectories."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
from typing import Literal

import numpy as np

from irs_generator.utils.math import Scalar

from .exceptions import InvalidTrajectoryError
from .models import Trajectory, TrajectoryPoint

__all__ = [
    "TrajectoryUnits",
    "TrajectoryValidationConfig",
    "TrajectoryValidator",
]


@dataclass(frozen=True, slots=True)
class TrajectoryUnits:
    """Units required by canonical trajectory points.

    The values identify the units accepted by :class:`TrajectoryPoint`; they do
    not request a conversion. Provider adapters must convert external values before
    yielding canonical points.
    """

    time: Literal["s"] = "s"
    latitude: Literal["rad"] = "rad"
    longitude: Literal["rad"] = "rad"
    height: Literal["m"] = "m"
    velocity: Literal["m/s"] = "m/s"
    attitude: Literal["rad"] = "rad"


@dataclass(frozen=True, slots=True)
class TrajectoryValidationConfig:
    """Policy for validating a canonical trajectory stream.

    Parameters
    ----------
    require_uniform_time_step
        Require all adjacent points to use the first observed time step.
    time_step_absolute_tolerance_s, time_step_relative_tolerance
        Absolute and relative tolerances for time-step comparison.
    """

    require_uniform_time_step: bool = True
    time_step_absolute_tolerance_s: Scalar = 1e-9
    time_step_relative_tolerance: Scalar = 1e-6

    def __post_init__(self) -> None:
        if not isinstance(self.require_uniform_time_step, bool):
            raise TypeError("require_uniform_time_step must be a bool")
        for name in (
            "time_step_absolute_tolerance_s",
            "time_step_relative_tolerance",
        ):
            value = np.longdouble(getattr(self, name))
            if not bool(np.isfinite(value)) or value < 0.0:
                raise ValueError(f"{name} must be finite and >= 0")
            object.__setattr__(self, name, value)


class TrajectoryValidator:
    """Validate a canonical trajectory without materializing its points."""

    units = TrajectoryUnits()

    def __init__(self, config: TrajectoryValidationConfig | None = None) -> None:
        self._config = (
            config if config is not None else TrajectoryValidationConfig()
        )
        if not isinstance(self._config, TrajectoryValidationConfig):
            raise TypeError("config must be a TrajectoryValidationConfig")

    def validate(self, trajectory: Trajectory) -> Iterator[TrajectoryPoint]:
        """Yield validated points from ``trajectory``.

        Raises
        ------
        InvalidTrajectoryError
            If the first point has no position, timestamps are not strictly
            increasing, or the time grid violates the configured policy.
        TypeError
            If the stream yields a value other than :class:`TrajectoryPoint`.
        """

        previous_time_s: np.longdouble | None = None
        reference_time_step_s: np.longdouble | None = None
        for point_index, point in enumerate(trajectory):
            if not isinstance(point, TrajectoryPoint):
                raise TypeError("trajectory must yield TrajectoryPoint instances")
            if point_index == 0 and point.position is None:
                raise InvalidTrajectoryError(
                    "the first trajectory point must define a position"
                )
            if previous_time_s is not None:
                time_step_s = np.longdouble(point.time_s - previous_time_s)
                if time_step_s <= 0.0:
                    raise InvalidTrajectoryError(
                        "trajectory time must be strictly increasing; "
                        f"point {point_index} is invalid"
                    )
                if reference_time_step_s is None:
                    reference_time_step_s = time_step_s
                elif self._config.require_uniform_time_step and not bool(
                    np.isclose(
                        time_step_s,
                        reference_time_step_s,
                        rtol=self._config.time_step_relative_tolerance,
                        atol=self._config.time_step_absolute_tolerance_s,
                    )
                ):
                    raise InvalidTrajectoryError(
                        "trajectory time grid must be uniform; "
                        f"point {point_index} has dt={time_step_s:.12g}s, "
                        f"expected {reference_time_step_s:.12g}s"
                    )
            previous_time_s = np.longdouble(point.time_s)
            yield point
