"""Memory-bounded orchestration of streaming inertial-data synthesis."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, field

from irs_generator.irs_model import ImuSample, InertialNavigationAlgorithm
from irs_generator.navigation_model import NavigationState
from irs_generator.utils.math import Scalar

from .exceptions import GenerationConvergenceError, InvalidTrajectoryError
from .models import GeneratedStep, GenerationDiagnostics, Trajectory
from .solver import StepSolverConfig, solve_imu_step
from .trajectory import TrajectoryValidationConfig, TrajectoryValidator

__all__ = ["GenerationConfig", "GenerationMetadata", "SyntheticDataGenerator"]


@dataclass(frozen=True, slots=True)
class GenerationConfig:
    """Policy controlling streaming inverse synthesis.

    Parameters
    ----------
    solver
        Numerical settings for one-step inverse IMU solving.
    fail_on_nonconvergence
        Raise ``RuntimeError`` when the solver does not converge if ``True``.
    trajectory_validation
        Policy for validating the canonical trajectory stream before solving.
    """

    solver: StepSolverConfig = field(default_factory=StepSolverConfig)
    fail_on_nonconvergence: bool = True
    trajectory_validation: TrajectoryValidationConfig = field(
        default_factory=TrajectoryValidationConfig
    )

    def __post_init__(self) -> None:
        if not isinstance(self.solver, StepSolverConfig):
            raise TypeError("solver must be a StepSolverConfig")
        if not isinstance(self.fail_on_nonconvergence, bool):
            raise TypeError("fail_on_nonconvergence must be a bool")
        if not isinstance(self.trajectory_validation, TrajectoryValidationConfig):
            raise TypeError(
                "trajectory_validation must be a TrajectoryValidationConfig"
            )


@dataclass(frozen=True, slots=True)
class GenerationMetadata:
    """Static metadata for a synthetic-data generation run.

    Parameters
    ----------
    algorithm_name
        Fully qualified class name of the selected navigation algorithm.
    config
        Immutable configuration used by the generator.
    """

    algorithm_name: str
    config: GenerationConfig

    def __post_init__(self) -> None:
        if not self.algorithm_name:
            raise ValueError("algorithm_name must not be empty")
        if not isinstance(self.config, GenerationConfig):
            raise TypeError("config must be a GenerationConfig")


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

    @property
    def metadata(self) -> GenerationMetadata:
        """Return immutable metadata describing this generator instance."""

        algorithm_type = type(self._algorithm)
        return GenerationMetadata(
            algorithm_name=f"{algorithm_type.__module__}.{algorithm_type.__qualname__}",
            config=self._config,
        )

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
        InvalidTrajectoryError
            If fewer than two points are provided or the first point has no
            position.
        GenerationConvergenceError
            If a step does not converge and ``fail_on_nonconvergence`` is
            enabled.
        """

        validator = TrajectoryValidator(self._config.trajectory_validation)
        iterator = iter(validator.validate(points))
        initial = next(iterator, None)
        if initial is None:
            raise InvalidTrajectoryError(
                "target trajectory must contain at least two points"
            )
        if initial.position is None:
            raise InvalidTrajectoryError(
                "the first trajectory point must define a position"
            )
        initial_state = NavigationState(
            velocity=initial.velocity,
            position=initial.position,
            attitude=initial.attitude,
        )
        self._algorithm.reset(initial_state)

        next_point = next(iterator, None)
        if next_point is None:
            raise InvalidTrajectoryError(
                "target trajectory must contain at least two points"
            )

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
                raise GenerationConvergenceError(
                    next_point.time_s,
                    solution.diagnostics,
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
