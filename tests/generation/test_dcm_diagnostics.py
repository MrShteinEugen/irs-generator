"""Check returned DCM samples against their reported solver diagnostics."""

import numpy as np
import pytest

from irs_generator.earth_model import GeodeticPosition
from irs_generator.generation import DcmTrajectoryGenerator, DcmTrajectoryPoint
from irs_generator.generation.dcm import (
    _poisson_inversion,
    _project_to_so3,
    _skew,
)
from irs_generator.navigation_model import EulerAngles, NavigationVelocity


@pytest.mark.parametrize("angle", [0.0, 0.1, np.pi / 2])
def test_residual_describes_the_returned_angular_rate(angle: float) -> None:
    previous = np.eye(3, dtype=np.longdouble)
    target = np.asarray(
        [
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1],
        ],
        dtype=np.longdouble,
    )
    rate = np.zeros(3, dtype=np.longdouble)
    dt = np.longdouble("0.05")
    omega, diagnostics = _poisson_inversion(previous, target, rate, dt)
    predicted = _project_to_so3(previous + dt * _skew(omega))
    residual = float(np.max(np.abs((target - predicted).astype(np.float64))))

    assert diagnostics.residual_norm == residual
    assert diagnostics.converged == (residual < 1e-15)
    assert 1 <= diagnostics.iteration_count <= 12
    if angle == 0:
        assert diagnostics.iteration_count == 1
        assert diagnostics.converged
    elif angle == np.pi / 2:
        assert diagnostics.iteration_count == 12
        assert not diagnostics.converged
    np.testing.assert_allclose(
        predicted.T @ predicted,
        np.eye(3),
        rtol=0,
        atol=32 * np.finfo(np.float64).eps,
    )
    assert float(np.linalg.det(predicted.astype(np.float64))) == pytest.approx(1.0)


def test_generate_exposes_actual_iteration_count() -> None:
    points = [
        DcmTrajectoryPoint(
            time_s=time,
            position=GeodeticPosition(0.0, 0.5, 100.0),
            velocity=NavigationVelocity(0.0, 0.0, 0.0),
            attitude=EulerAngles(0.0, 0.0, 0.0),
        )
        for time in (0.0, 0.05)
    ]
    steps = list(DcmTrajectoryGenerator(time_step_s=0.05).generate(points))
    assert len(steps) == 2
    for step in steps:
        assert step.diagnostics.iteration_count == 1
        assert step.diagnostics.converged
