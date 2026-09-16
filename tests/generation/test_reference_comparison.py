"""Ensure portable regression budgets still reject corrupted output."""

import numpy as np
import pytest
from examples.full_flight.generate import _assert_reference_values


@pytest.mark.parametrize(
    ("filename", "column", "error"),
    [
        ("imu.dat", 0, 1e-6),
        ("imu.dat", 1, 1e-8),
        ("imu.dat", 4, 1e-8),
        ("gps.dat", 1, 1e-8),
        ("gps.dat", 3, 1e-6),
        ("gps.dat", 4, 1e-8),
        ("gps.dat", 6, 1.0),
        ("gps.dat", 7, 1.0),
    ],
)
def test_reference_comparison_rejects_changed_values(
    filename: str,
    column: int,
    error: float,
) -> None:
    expected = np.ones((2, 7 if filename == "imu.dat" else 8))
    actual = expected.copy()
    actual[1, column] += error
    with pytest.raises(AssertionError):
        _assert_reference_values(actual, expected, filename, 0.05)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_reference_comparison_rejects_non_finite_values(value: float) -> None:
    values = np.ones((2, 7))
    values[1, 4] = value
    with pytest.raises(AssertionError, match="non-finite"):
        _assert_reference_values(values, values, "imu.dat", 0.05)


def test_reference_comparison_accepts_near_zero_roundoff() -> None:
    expected = np.zeros((2, 7))
    actual = expected.copy()
    actual[1, 4] = 1e-11
    _assert_reference_values(actual, expected, "imu.dat", 0.05)


def test_reference_comparison_rejects_truncated_output() -> None:
    with pytest.raises(AssertionError, match="shapes"):
        _assert_reference_values(np.zeros((1, 7)), np.zeros((2, 7)), "imu.dat", 0.05)
