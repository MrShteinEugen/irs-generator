from math import inf, nan, pi

import numpy as np
import pytest

from irs_generator.utils._validation import (
    _finite_scalar,
    _positive_scalar,
    _validated_latitude,
)


@pytest.mark.parametrize("value", [0, 1, -1.5, "2.5"])
def test_finite_scalar_accepts_and_converts_finite_values(value: float | str) -> None:
    assert _finite_scalar(value, name="value") == pytest.approx(float(value))


def test_finite_scalar_uses_longdouble_as_the_default_numeric_scalar() -> None:
    assert isinstance(_finite_scalar("1.25", name="value"), np.longdouble)


@pytest.mark.parametrize("value", [nan, inf, -inf])
def test_finite_scalar_rejects_non_finite_values(value: float) -> None:
    with pytest.raises(ValueError, match="value must be finite"):
        _finite_scalar(value, name="value")


@pytest.mark.parametrize("value", [0.1, 1, "3.5"])
def test_positive_scalar_accepts_positive_values(value: float | str) -> None:
    assert _positive_scalar(value, name="value") == pytest.approx(float(value))


@pytest.mark.parametrize("value", [0, -1, nan, inf, -inf])
def test_positive_scalar_rejects_non_positive_or_non_finite_values(
    value: float,
) -> None:
    with pytest.raises(ValueError):
        _positive_scalar(value, name="value")


@pytest.mark.parametrize("latitude_rad", [-pi / 2, 0.0, pi / 2])
def test_validated_latitude_accepts_closed_interval(latitude_rad: float) -> None:
    assert _validated_latitude(latitude_rad) == pytest.approx(latitude_rad)


@pytest.mark.parametrize("latitude_rad", [-pi, pi, nan, inf, -inf])
def test_validated_latitude_rejects_invalid_values(latitude_rad: float) -> None:
    with pytest.raises(ValueError):
        _validated_latitude(latitude_rad)
