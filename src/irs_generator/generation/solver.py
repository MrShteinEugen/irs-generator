"""Numerical inverse solver for one inertial-navigation propagation step."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from irs_generator.irs_model import ImuSample, InertialNavigationAlgorithm
from irs_generator.navigation_model import NavigationState
from irs_generator.utils.math import Scalar, angle_difference_rad

from .models import GenerationDiagnostics, TrajectoryPoint

__all__ = ["InverseStepSolution", "StepSolverConfig", "solve_imu_step"]


@dataclass(frozen=True, slots=True)
class StepSolverConfig:
    """Numerical settings for one-step inverse IMU solving.

    Parameters
    ----------
    max_iterations
        Maximum Gauss-Newton iterations.
    residual_tolerance
        Convergence threshold for the normalized infinity-norm residual.
    acceleration_perturbation_m_s2
        Finite-difference perturbation for acceleration components.
    angular_rate_perturbation_rad_s
        Finite-difference perturbation for angular-rate components.
    velocity_scale_m_s
        Velocity residual scale.
    attitude_scale_rad
        Attitude residual scale.
    regularization
        Tikhonov regularization added to the normal matrix.
    max_acceleration_update_m_s2
        Per-iteration acceleration update limit.
    max_angular_rate_update_rad_s
        Per-iteration angular-rate update limit.
    minimum_line_search_scale
        Smallest accepted line-search scale.
    """

    max_iterations: int = 12
    residual_tolerance: Scalar = 1e-8
    acceleration_perturbation_m_s2: Scalar = 1e-4
    angular_rate_perturbation_rad_s: Scalar = 1e-6
    velocity_scale_m_s: Scalar = 1.0
    attitude_scale_rad: Scalar = 0.1
    regularization: Scalar = 1e-8
    max_acceleration_update_m_s2: Scalar = 50.0
    max_angular_rate_update_rad_s: Scalar = 10.0
    minimum_line_search_scale: Scalar = 1 / 64

    def __post_init__(self) -> None:
        if self.max_iterations <= 0:
            raise ValueError("max_iterations must be > 0")
        for name in (
            "residual_tolerance",
            "acceleration_perturbation_m_s2",
            "angular_rate_perturbation_rad_s",
            "velocity_scale_m_s",
            "attitude_scale_rad",
            "regularization",
            "max_acceleration_update_m_s2",
            "max_angular_rate_update_rad_s",
            "minimum_line_search_scale",
        ):
            value = np.longdouble(getattr(self, name))
            if not bool(np.isfinite(value)) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
            object.__setattr__(self, name, value)


@dataclass(frozen=True, slots=True)
class InverseStepSolution:
    """Result of one inverse IMU solve.

    Parameters
    ----------
    imu_sample
        IMU sample selected for the step.
    diagnostics
        Convergence diagnostics for the solve.
    """

    imu_sample: ImuSample
    diagnostics: GenerationDiagnostics


def solve_imu_step(
    algorithm: InertialNavigationAlgorithm,
    target: TrajectoryPoint,
    dt_s: Scalar,
    *,
    initial_guess: ImuSample | None = None,
    config: StepSolverConfig | None = None,
) -> InverseStepSolution:
    """Find an IMU sample that makes ``algorithm`` reach ``target``.

    Parameters
    ----------
    algorithm
        Current navigation algorithm state. Residual evaluations use
        ``algorithm.fork()`` and do not mutate this instance.
    target
        Target state values for the end of the step.
    dt_s
        Positive step duration in seconds.
    initial_guess
        Optional IMU sample used as the initial solver vector.
    config
        Optional solver configuration. Defaults to :class:`StepSolverConfig`.

    Returns
    -------
    InverseStepSolution
        Selected IMU sample and convergence diagnostics.

    Notes
    -----
    The residual uses velocity and attitude. Position is propagated by the
    navigation algorithm from its current state.
    """

    settings = config if config is not None else StepSolverConfig()
    dt = np.longdouble(dt_s)
    if not bool(np.isfinite(dt)) or dt <= 0.0:
        raise ValueError(f"dt_s must be finite and > 0, got {dt_s!r}")

    values = _sample_to_vector(
        ImuSample.zero() if initial_guess is None else initial_guess
    )
    perturbations = np.array(
        (
            settings.acceleration_perturbation_m_s2,
            settings.acceleration_perturbation_m_s2,
            settings.acceleration_perturbation_m_s2,
            settings.angular_rate_perturbation_rad_s,
            settings.angular_rate_perturbation_rad_s,
            settings.angular_rate_perturbation_rad_s,
        ),
        dtype=np.longdouble,
    )

    for iteration in range(1, settings.max_iterations + 1):
        residual = _normalized_residual(
            _evaluate(algorithm, values, dt),
            target,
            settings,
        )
        residual_norm = _inf_norm(residual)
        if residual_norm <= settings.residual_tolerance:
            return InverseStepSolution(
                _vector_to_sample(values),
                GenerationDiagnostics(iteration - 1, residual_norm, True),
            )

        jacobian = np.empty((6, 6), dtype=np.longdouble)
        for column, perturbation in enumerate(perturbations):
            forward = values.copy()
            backward = values.copy()
            forward[column] += perturbation
            backward[column] -= perturbation
            forward_residual = _normalized_residual(
                _evaluate(algorithm, forward, dt),
                target,
                settings,
            )
            backward_residual = _normalized_residual(
                _evaluate(algorithm, backward, dt),
                target,
                settings,
            )
            jacobian[:, column] = (forward_residual - backward_residual) / (
                2.0 * perturbation
            )

        normal_matrix = jacobian.T @ jacobian + settings.regularization * np.eye(
            6,
            dtype=np.longdouble,
        )
        update = np.asarray(
            np.linalg.solve(
                np.asarray(normal_matrix, dtype=np.float64),
                np.asarray(-jacobian.T @ residual, dtype=np.float64),
            ),
            dtype=np.longdouble,
        )
        if not np.all(np.isfinite(update)):
            raise RuntimeError("inverse IMU solver produced a non-finite update")
        update[:3] = np.clip(
            update[:3],
            -settings.max_acceleration_update_m_s2,
            settings.max_acceleration_update_m_s2,
        )
        update[3:] = np.clip(
            update[3:],
            -settings.max_angular_rate_update_rad_s,
            settings.max_angular_rate_update_rad_s,
        )
        values, accepted = _line_search(
            algorithm,
            values,
            update,
            residual_norm,
            target,
            dt,
            settings,
        )
        if not accepted:
            break

    final_residual = _normalized_residual(
        _evaluate(algorithm, values, dt),
        target,
        settings,
    )
    return InverseStepSolution(
        _vector_to_sample(values),
        GenerationDiagnostics(
            settings.max_iterations,
            _inf_norm(final_residual),
            False,
        ),
    )


def _evaluate(
    algorithm: InertialNavigationAlgorithm,
    values: np.ndarray,
    dt_s: Scalar,
) -> NavigationState:
    trial = algorithm.fork()
    return trial.step(_vector_to_sample(values), dt_s, gnss_sample=None)


def _line_search(
    algorithm: InertialNavigationAlgorithm,
    values: np.ndarray,
    update: np.ndarray,
    residual_norm: Scalar,
    target: TrajectoryPoint,
    dt_s: Scalar,
    config: StepSolverConfig,
) -> tuple[np.ndarray, bool]:
    scale = 1.0
    while scale >= config.minimum_line_search_scale:
        candidate = values + scale * update
        candidate_residual = _normalized_residual(
            _evaluate(algorithm, candidate, dt_s),
            target,
            config,
        )
        if _inf_norm(candidate_residual) < residual_norm:
            return candidate, True
        scale *= 0.5
    return values, False


def _normalized_residual(
    state: NavigationState,
    target: TrajectoryPoint,
    config: StepSolverConfig,
) -> np.ndarray:
    return np.array(
        (
            (state.velocity.east_m_s - target.velocity.east_m_s)
            / config.velocity_scale_m_s,
            (state.velocity.north_m_s - target.velocity.north_m_s)
            / config.velocity_scale_m_s,
            (state.velocity.up_m_s - target.velocity.up_m_s)
            / config.velocity_scale_m_s,
            angle_difference_rad(state.attitude.pitch_rad, target.attitude.pitch_rad)
            / config.attitude_scale_rad,
            angle_difference_rad(state.attitude.roll_rad, target.attitude.roll_rad)
            / config.attitude_scale_rad,
            angle_difference_rad(
                state.attitude.heading_rad, target.attitude.heading_rad
            )
            / config.attitude_scale_rad,
        ),
        dtype=np.longdouble,
    )


def _sample_to_vector(sample: ImuSample) -> np.ndarray:
    return np.array((*sample.a_3d, *sample.w_3d), dtype=np.longdouble)


def _vector_to_sample(values: np.ndarray) -> ImuSample:
    if values.shape != (6,):
        raise ValueError(f"IMU candidate must have shape (6,), got {values.shape}")
    return ImuSample.from_components(*values)


def _inf_norm(values: np.ndarray) -> np.longdouble:
    return np.longdouble(np.max(np.abs(values)))
