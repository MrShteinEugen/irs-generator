from dataclasses import dataclass
from math import inf, nan

import pytest

from irs_generator.earth_model import GeodeticPosition
from irs_generator.irs_model import (
    BiasImuErrorModel,
    DcmStrapdownINS,
    ImuSample,
    InertialReferenceSystem,
    StrapdownINS,
)
from irs_generator.navigation_model import (
    EulerAngles,
    NavigationState,
    NavigationVelocity,
)
from irs_generator.utils.math import Scalar, Vector3


def _initial_state() -> NavigationState:
    return NavigationState(
        velocity=NavigationVelocity(0.0, 0.0, 0.0),
        position=GeodeticPosition(0.0, 0.5, 100.0),
        attitude=EulerAngles(0.0, 0.0, 0.0),
    )


@dataclass
class RecordingAlgorithm:
    state: NavigationState
    received_sample: ImuSample | None = None
    received_dt_s: Scalar | None = None
    reset_calls: int = 0

    def reset(self, initial_state: NavigationState) -> None:
        self.state = initial_state
        self.reset_calls += 1

    def step(
        self,
        imu_sample: ImuSample,
        dt_s: Scalar,
        gnss_sample: object | None = None,
    ) -> NavigationState:
        del gnss_sample
        self.received_sample = imu_sample
        self.received_dt_s = dt_s
        return self.state


@dataclass
class ResettableErrorModel:
    reset_calls: int = 0

    def apply(self, sample: ImuSample, dt_s: Scalar) -> ImuSample:
        del dt_s
        return sample

    def reset(self) -> None:
        self.reset_calls += 1


def test_irs_applies_sensor_errors_before_calling_navigation_algorithm() -> None:
    algorithm = RecordingAlgorithm(_initial_state())
    irs = InertialReferenceSystem(
        algorithm,
        imu_error_model=BiasImuErrorModel(
            specific_force_bias_m_s2=Vector3(1.0, 0.0, 0.0)
        ),
    )

    result = irs.step(ImuSample.zero(), 0.25)

    assert result is algorithm.state
    assert algorithm.received_sample is not None
    assert algorithm.received_sample.a_3d == pytest.approx((1.0, 0.0, 0.0))
    assert algorithm.received_dt_s == pytest.approx(0.25)


def test_irs_reset_resets_the_algorithm_and_error_model() -> None:
    algorithm = RecordingAlgorithm(_initial_state())
    error_model = ResettableErrorModel()
    irs = InertialReferenceSystem(algorithm, imu_error_model=error_model)
    reset_state = _initial_state()

    irs.reset(reset_state)

    assert algorithm.state is reset_state
    assert algorithm.reset_calls == 1
    assert error_model.reset_calls == 1


def test_strapdown_ins_can_be_used_as_a_navigation_algorithm() -> None:
    initial_state = _initial_state()
    irs = InertialReferenceSystem(StrapdownINS(initial_state))

    updated_state = irs.step(ImuSample.zero(), 0.01)

    assert updated_state is irs.state
    assert updated_state.velocity.up_m_s < 0.0


def test_dcm_strapdown_fork_keeps_trial_state_independent() -> None:
    algorithm = DcmStrapdownINS(_initial_state())
    trial = algorithm.fork()

    trial.step(ImuSample.zero(), 0.01)

    assert algorithm.state == _initial_state()
    assert trial.state != algorithm.state


@pytest.mark.parametrize("dt_s", [0.0, -1.0, nan, inf, -inf])
def test_irs_rejects_invalid_step_duration(dt_s: float) -> None:
    irs = InertialReferenceSystem(RecordingAlgorithm(_initial_state()))

    with pytest.raises(ValueError):
        irs.step(ImuSample.zero(), dt_s)
