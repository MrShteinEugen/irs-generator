"""Tests for lazy trajectory validation and unit contracts."""

from collections.abc import Iterator

import pytest

from irs_generator.earth_model import GeodeticPosition
from irs_generator.generation import (
    InvalidTrajectoryError,
    TrajectoryPoint,
    TrajectoryUnits,
    TrajectoryValidationConfig,
    TrajectoryValidator,
)
from irs_generator.navigation_model import EulerAngles, NavigationVelocity


def _point(time_s: float, *, position: bool = False) -> TrajectoryPoint:
    return TrajectoryPoint(
        time_s=time_s,
        position=GeodeticPosition(0.5, 0.25, 100.0) if position else None,
        velocity=NavigationVelocity(1.0, 2.0, 3.0),
        attitude=EulerAngles(0.1, 0.2, 0.3),
    )


def _stream() -> Iterator[TrajectoryPoint]:
    yield _point(0.0, position=True)
    yield _point(1.0)


def test_validator_preserves_streamed_points() -> None:
    points = list(TrajectoryValidator().validate(_stream()))

    assert [point.time_s for point in points] == [0.0, 1.0]


def test_validator_requires_the_first_point_position() -> None:
    with pytest.raises(InvalidTrajectoryError, match="first trajectory point"):
        list(TrajectoryValidator().validate((_point(0.0), _point(1.0))))


def test_validator_rejects_non_increasing_or_non_uniform_time() -> None:
    validator = TrajectoryValidator()

    with pytest.raises(InvalidTrajectoryError, match="strictly increasing"):
        list(validator.validate((_point(0.0, position=True), _point(0.0))))
    with pytest.raises(InvalidTrajectoryError, match="must be uniform"):
        list(
            validator.validate(
                (_point(0.0, position=True), _point(1.0), _point(2.1))
            )
        )


def test_validator_can_accept_non_uniform_time() -> None:
    validator = TrajectoryValidator(
        TrajectoryValidationConfig(require_uniform_time_step=False)
    )

    points = list(
        validator.validate((_point(0.0, position=True), _point(1.0), _point(2.1)))
    )

    assert [point.time_s for point in points] == [0.0, 1.0, 2.1]


def test_trajectory_units_define_the_canonical_contract() -> None:
    assert TrajectoryUnits() == TrajectoryUnits(
        time="s",
        latitude="rad",
        longitude="rad",
        height="m",
        velocity="m/s",
        attitude="rad",
    )
