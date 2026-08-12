"""Tests for generation configuration, metadata, and public exceptions."""

from dataclasses import dataclass
from typing import Self, cast

import pytest

from irs_generator.earth_model import GeodeticPosition
from irs_generator.generation import (
    GenerationConfig,
    GenerationConvergenceError,
    InvalidTrajectoryError,
    SyntheticDataGenerator,
    TrajectoryPoint,
)
from irs_generator.generation.solver import StepSolverConfig
from irs_generator.irs_model import ImuSample
from irs_generator.navigation_model import (
    EulerAngles,
    NavigationState,
    NavigationVelocity,
)
from irs_generator.utils.math import Scalar


@dataclass
class FixedNavigationAlgorithm:
    """Test double that ignores IMU samples during propagation."""

    state: NavigationState

    def reset(self, initial_state: NavigationState) -> None:
        self.state = initial_state

    def fork(self) -> Self:
        return type(self)(self.state)

    def step(
        self,
        imu_sample: ImuSample,
        dt_s: Scalar,
        gnss_sample: object | None = None,
    ) -> NavigationState:
        del imu_sample, dt_s, gnss_sample
        return self.state


def _algorithm() -> FixedNavigationAlgorithm:
    state = NavigationState(
        velocity=NavigationVelocity(0.0, 0.0, 0.0),
        position=GeodeticPosition(0.5, 0.25, 100.0),
        attitude=EulerAngles(0.0, 0.0, 0.0),
    )
    return FixedNavigationAlgorithm(state)


def _initial_point() -> TrajectoryPoint:
    return TrajectoryPoint(
        time_s=0.0,
        velocity=NavigationVelocity(0.0, 0.0, 0.0),
        attitude=EulerAngles(0.0, 0.0, 0.0),
        position=GeodeticPosition(0.5, 0.25, 100.0),
    )


def test_metadata_exposes_algorithm_identity_and_configuration() -> None:
    config = GenerationConfig(fail_on_nonconvergence=False)
    generator = SyntheticDataGenerator(_algorithm(), config=config)

    metadata = generator.metadata

    assert metadata.config is config
    assert metadata.algorithm_name.endswith(".FixedNavigationAlgorithm")


def test_generation_config_rejects_invalid_policy_values() -> None:
    with pytest.raises(TypeError, match="solver must be a StepSolverConfig"):
        GenerationConfig(solver=cast(StepSolverConfig, object()))
    with pytest.raises(TypeError, match="fail_on_nonconvergence must be a bool"):
        GenerationConfig(fail_on_nonconvergence=cast(bool, 1))


def test_generator_raises_a_trajectory_error_for_an_empty_stream() -> None:
    generator = SyntheticDataGenerator(_algorithm())

    with pytest.raises(InvalidTrajectoryError, match="at least two points"):
        list(generator.generate(()))


def test_generator_raises_a_convergence_error_with_diagnostics() -> None:
    target = TrajectoryPoint(
        time_s=1.0,
        velocity=NavigationVelocity(1.0, 0.0, 0.0),
        attitude=EulerAngles(0.0, 0.0, 0.0),
    )
    config = GenerationConfig(solver=StepSolverConfig(max_iterations=1))
    generator = SyntheticDataGenerator(_algorithm(), config=config)

    with pytest.raises(GenerationConvergenceError) as error:
        list(generator.generate((_initial_point(), target)))

    assert error.value.time_s == 1.0
    assert not error.value.diagnostics.converged
