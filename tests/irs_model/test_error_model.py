from math import inf, nan

import pytest

from irs_generator.irs_model.error_model import (
    BiasImuErrorModel,
    CompositeImuErrorModel,
    IdealImuErrorModel,
)
from irs_generator.irs_model.imu import ImuSample
from irs_generator.utils.math import Vector3


def test_ideal_error_model_returns_the_original_sample() -> None:
    sample = ImuSample.from_components(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)

    assert IdealImuErrorModel().apply(sample, 0.01) is sample


def test_bias_error_model_offsets_both_sensor_channels() -> None:
    sample = ImuSample.from_components(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
    error_model = BiasImuErrorModel(
        specific_force_bias_m_s2=Vector3(0.5, -0.5, 1.0),
        angular_rate_bias_rad_s=Vector3(0.01, -0.02, 0.03),
    )

    observed = error_model.apply(sample, 0.01)

    assert observed.a_3d == pytest.approx((1.5, 1.5, 4.0))
    assert observed.w_3d == pytest.approx((0.11, 0.18, 0.33))


def test_composite_error_model_applies_models_in_sequence() -> None:
    error_model = CompositeImuErrorModel(
        (
            BiasImuErrorModel(specific_force_bias_m_s2=Vector3(1.0, 0.0, 0.0)),
            BiasImuErrorModel(specific_force_bias_m_s2=Vector3(0.0, 2.0, 0.0)),
        )
    )

    observed = error_model.apply(ImuSample.zero(), 0.1)

    assert observed.a_3d == pytest.approx((1.0, 2.0, 0.0))


@pytest.mark.parametrize("dt_s", [0.0, -1.0, nan, inf, -inf])
def test_error_models_reject_invalid_step_duration(dt_s: float) -> None:
    with pytest.raises(ValueError):
        IdealImuErrorModel().apply(ImuSample.zero(), dt_s)
