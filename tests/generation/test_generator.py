from dataclasses import dataclass

import pytest

from irs_generator.earth_model import GeodeticPosition
from irs_generator.generation import (
    SyntheticDataGenerator,
    TargetTrajectoryPoint,
)
from irs_generator.generation.solver import solve_imu_step
from irs_generator.irs_model import ImuSample
from irs_generator.navigation_model import (
    EulerAngles,
    NavigationState,
    NavigationVelocity,
)
from irs_generator.utils.math import Scalar


@dataclass
class LinearNavigationAlgorithm:
    """A deterministic test double with six independently controllable outputs."""

    state: NavigationState

    def reset(self, initial_state: NavigationState) -> None:
        self.state = initial_state

    def fork(self) -> "LinearNavigationAlgorithm":
        return type(self)(self.state)

    def step(
        self,
        imu_sample: ImuSample,
        dt_s: Scalar,
        gnss_sample: object | None = None,
    ) -> NavigationState:
        del dt_s, gnss_sample
        self.state = NavigationState(
            velocity=NavigationVelocity(*imu_sample.a_3d),
            position=self.state.position,
            attitude=EulerAngles(*imu_sample.w_3d),
        )
        return self.state


def test_generator_uses_the_selected_algorithm_inside_inverse_loop() -> None:
    initial = TargetTrajectoryPoint(
        time_s=0.0,
        position=GeodeticPosition(0.0, 0.5, 100.0),
        velocity=NavigationVelocity(0.0, 0.0, 0.0),
        attitude=EulerAngles(0.0, 0.0, 0.0),
    )
    target = TargetTrajectoryPoint(
        time_s=1.0,
        velocity=NavigationVelocity(4.0, 5.0, 6.0),
        attitude=EulerAngles(0.1, 0.2, 0.3),
    )
    assert initial.position is not None
    algorithm = LinearNavigationAlgorithm(
        NavigationState(initial.velocity, initial.position, initial.attitude)
    )

    output = list(SyntheticDataGenerator(algorithm).generate((initial, target)))

    assert len(output) == 2
    assert output[0].imu_sample == output[1].imu_sample
    assert output[1].navigation_state.velocity.as_array() == pytest.approx(
        target.velocity.as_array()
    )
    assert output[1].navigation_state.attitude.as_array() == pytest.approx(
        target.attitude.as_array()
    )
    assert output[1].diagnostics.converged


def test_inverse_step_trials_do_not_mutate_the_selected_algorithm() -> None:
    initial = TargetTrajectoryPoint(
        time_s=0.0,
        position=GeodeticPosition(0.0, 0.5, 100.0),
        velocity=NavigationVelocity(1.0, 2.0, 3.0),
        attitude=EulerAngles(0.0, 0.0, 0.0),
    )
    target = TargetTrajectoryPoint(
        time_s=1.0,
        velocity=NavigationVelocity(4.0, 5.0, 6.0),
        attitude=EulerAngles(0.1, 0.2, 0.3),
    )
    assert initial.position is not None
    original_state = NavigationState(
        initial.velocity, initial.position, initial.attitude
    )
    algorithm = LinearNavigationAlgorithm(original_state)

    solution = solve_imu_step(algorithm, target, 1.0)

    assert solution.diagnostics.converged
    assert algorithm.state == original_state
