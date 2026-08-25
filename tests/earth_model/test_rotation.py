from math import inf, nan

import pytest

from irs_generator.earth_model.rotation import RotationParameters


@pytest.mark.parametrize("angular_velocity_rad_s", [0.0, 7.292_115e-5])
def test_rotation_accepts_non_negative_finite_velocity(
    angular_velocity_rad_s: float,
) -> None:
    rotation = RotationParameters(angular_velocity_rad_s)
    assert rotation.angular_velocity_rad_s == pytest.approx(angular_velocity_rad_s)


@pytest.mark.parametrize("angular_velocity_rad_s", [-1.0, nan, inf, -inf])
def test_rotation_rejects_invalid_velocity(angular_velocity_rad_s: float) -> None:
    with pytest.raises(ValueError):
        RotationParameters(angular_velocity_rad_s)
