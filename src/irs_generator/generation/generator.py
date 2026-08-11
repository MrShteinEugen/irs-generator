"""Memory-bounded orchestration of streaming inertial-data synthesis."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, field

from irs_generator.irs_model import ImuSample, InertialNavigationAlgorithm
from irs_generator.navigation_model import NavigationState
from irs_generator.utils.math import Scalar

from .models import GeneratedStep, GenerationDiagnostics, Trajectory
from .solver import StepSolverConfig, solve_imu_step

__all__ = ["GenerationConfig", "SyntheticDataGenerator"]


@dataclass(frozen=True, slots=True)
class GenerationConfig:
    """Policy controlling streaming inverse synthesis.

    Parameters
    ----------
    solver
        Numerical settings for one-step inverse IMU solving.
    fail_on_nonconvergence
        Raise ``RuntimeError`` when the solver does not converge if ``True``.
    """

    solver: StepSolverConfig = field(default_factory=StepSolverConfig)
    fail_on_nonconvergence: bool = True


class SyntheticDataGenerator:
    """Generate ideal IMU and self-consistent GNSS truth.

    Parameters
    ----------
    algorithm
        Inertial navigation algorithm used inside the inverse synthesis loop.
        It must support ``fork()``.
    config
        Optional generation and solver policy.
    """

    def __init__(
        self,
        algorithm: InertialNavigationAlgorithm,
        *,
        config: GenerationConfig | None = None,
    ) -> None:
        if not isinstance(algorithm, InertialNavigationAlgorithm):
            raise TypeError("algorithm must implement InertialNavigationAlgorithm")
        self._algorithm = algorithm
        self._config = config if config is not None else GenerationConfig()

    def generate(
        self,
        points: Trajectory,
    ) -> Iterator[GeneratedStep]:
        """Generate rows from a target trajectory.

        Parameters
        ----------
        points
            Iterable of target trajectory points. The first point must include
            position and initializes the navigation algorithm.

        Yields
        ------
        GeneratedStep
            Generated IMU sample and the aligned navigation state.

        Raises
        ------
        ValueError
            If fewer than two points are provided or the first point has no
            position.
        RuntimeError
            If a step does not converge and ``fail_on_nonconvergence`` is
            enabled.
        """

        iterator = iter(points)
        initial = next(iterator, None)
        if initial is None:
            raise ValueError("target trajectory must contain at least two points")
        if initial.position is None:
            raise ValueError("the first target point must define a position")

        initial_state = NavigationState(
            velocity=initial.velocity,
            position=initial.position,
            attitude=initial.attitude,
        )
        self._algorithm.reset(initial_state)

        next_point = next(iterator, None)
        if next_point is None:
            raise ValueError("target trajectory must contain at least two points")

        previous_point = initial
        previous_guess: ImuSample | None = None
        first_step = True
        while True:
            dt_s: Scalar = next_point.time_s - previous_point.time_s
            solution = solve_imu_step(
                self._algorithm,
                next_point,
                dt_s,
                initial_guess=previous_guess,
                config=self._config.solver,
            )
            if (
                not solution.diagnostics.converged
                and self._config.fail_on_nonconvergence
            ):
                raise RuntimeError(
                    "inverse IMU solver did not converge at "
                    f"t={next_point.time_s:.12g}s; residual="
                    f"{solution.diagnostics.residual_norm:.3e}"
                )

            if first_step:
                yield GeneratedStep(
                    time_s=initial.time_s,
                    imu_sample=solution.imu_sample,
                    navigation_state=self._algorithm.state,
                    diagnostics=GenerationDiagnostics(0, 0.0, True),
                )
                first_step = False

            state = self._algorithm.step(
                solution.imu_sample,
                dt_s,
                gnss_sample=None,
            )
            yield GeneratedStep(
                time_s=next_point.time_s,
                imu_sample=solution.imu_sample,
                navigation_state=state,
                diagnostics=solution.diagnostics,
            )

            previous_guess = solution.imu_sample
            previous_point = next_point
            next_point = next(iterator, None)
            if next_point is None:
                return
