"""Numerical inverse solver for one inertial-navigation propagation step."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

import numpy as np

from irs_generator.irs_model import ImuSample, InertialNavigationAlgorithm
from irs_generator.navigation_model import NavigationState
from irs_generator.utils.math import angle_difference_rad

from .models import GenerationDiagnostics, TargetTrajectoryPoint

__all__ = ["InverseStepSolution", "StepSolverConfig", "solve_imu_step"]


@dataclass(frozen=True, slots=True)
class StepSolverConfig:
    """Numerical settings and residual scales for the inverse step solver."""

    max_iterations: int = 12
    residual_tolerance: float = 1e-8
    acceleration_perturbation_m_s2: float = 1e-4
    angular_rate_perturbation_rad_s: float = 1e-6
    velocity_scale_m_s: float = 1.0
    attitude_scale_rad: float = 0.1
    regularization: float = 1e-8
    max_acceleration_update_m_s2: float = 50.0
    max_angular_rate_update_rad_s: float = 10.0
    minimum_line_search_scale: float = 1 / 64

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
            value = float(getattr(self, name))
            if not isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
            object.__setattr__(self, name, value)


@dataclass(frozen=True, slots=True)
class InverseStepSolution:
    imu_sample: ImuSample
    diagnostics: GenerationDiagnostics


def solve_imu_step(
    algorithm: InertialNavigationAlgorithm,
    target: TargetTrajectoryPoint,
    dt_s: float,
    *,
    initial_guess: ImuSample | None = None,
    config: StepSolverConfig | None = None,
) -> InverseStepSolution:
    """Find the IMU sample that makes ``algorithm`` reach ``target``.

    Every residual evaluation advances an independent algorithm fork, leaving
    the caller's runtime untouched until it accepts the resulting sample.
    Position is intentionally excluded from the residual: it is propagated by
    the navigation algorithm from the previous self-consistent state.
    """

    settings = config if config is not None else StepSolverConfig()
    dt = float(dt_s)
    if not isfinite(dt) or dt <= 0.0:
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
        dtype=np.float64,
    )

    for iteration in range(1, settings.max_iterations + 1):
        residual = _normalized_residual(
            _evaluate(algorithm, values, dt),
            target,
            settings,
        )
        residual_norm = float(np.linalg.norm(residual, ord=np.inf))
        if residual_norm <= settings.residual_tolerance:
            return InverseStepSolution(
                _vector_to_sample(values),
                GenerationDiagnostics(iteration - 1, residual_norm, True),
            )

        jacobian = np.empty((6, 6), dtype=np.float64)
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

        update = np.linalg.solve(
            jacobian.T @ jacobian + settings.regularization * np.eye(6),
            -jacobian.T @ residual,
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
            float(np.linalg.norm(final_residual, ord=np.inf)),
            False,
        ),
    )


def _evaluate(
    algorithm: InertialNavigationAlgorithm,
    values: np.ndarray,
    dt_s: float,
) -> NavigationState:
    trial = algorithm.fork()
    return trial.step(_vector_to_sample(values), dt_s, gnss_sample=None)


def _line_search(
    algorithm: InertialNavigationAlgorithm,
    values: np.ndarray,
    update: np.ndarray,
    residual_norm: float,
    target: TargetTrajectoryPoint,
    dt_s: float,
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
        if float(np.linalg.norm(candidate_residual, ord=np.inf)) < residual_norm:
            return candidate, True
        scale *= 0.5
    return values, False


def _normalized_residual(
    state: NavigationState,
    target: TargetTrajectoryPoint,
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
        dtype=np.float64,
    )


def _sample_to_vector(sample: ImuSample) -> np.ndarray:
    return np.array((*sample.a_3d, *sample.w_3d), dtype=np.float64)


def _vector_to_sample(values: np.ndarray) -> ImuSample:
    if values.shape != (6,):
        raise ValueError(f"IMU candidate must have shape (6,), got {values.shape}")
    return ImuSample.from_components(*map(float, values))
