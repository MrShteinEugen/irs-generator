"""Tests for the canonical streaming trajectory contract."""

from collections.abc import Iterator

from irs_generator.earth_model import GeodeticPosition
from irs_generator.generation import TargetTrajectoryPoint, Trajectory, TrajectoryPoint
from irs_generator.navigation_model import EulerAngles, NavigationVelocity


def _points() -> Iterator[TrajectoryPoint]:
    yield TrajectoryPoint(
        time_s=0.0,
        position=GeodeticPosition(0.5, 0.25, 100.0),
        velocity=NavigationVelocity(1.0, 2.0, 3.0),
        attitude=EulerAngles(0.1, 0.2, 0.3),
    )
    yield TrajectoryPoint(
        time_s=1.0,
        velocity=NavigationVelocity(4.0, 5.0, 6.0),
        attitude=EulerAngles(0.4, 0.5, 0.6),
    )


def test_trajectory_is_a_stream_of_canonical_points() -> None:
    trajectory: Trajectory = _points()

    points = list(trajectory)

    assert [point.time_s for point in points] == [0.0, 1.0]
    assert points[0].position is not None
    assert points[1].position is None


def test_target_trajectory_point_is_the_compatible_name() -> None:
    assert TargetTrajectoryPoint is TrajectoryPoint
