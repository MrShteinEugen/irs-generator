"""Public exceptions raised by the generation layer."""

from __future__ import annotations

from irs_generator.utils.math import Scalar

from .models import GenerationDiagnostics

__all__ = [
    "GenerationConvergenceError",
    "GenerationError",
    "GenerationSolverError",
    "InvalidTrajectoryError",
]


class GenerationError(RuntimeError):
    """Base class for runtime errors raised during synthetic-data generation."""


class InvalidTrajectoryError(GenerationError, ValueError):
    """Raised when a streamed trajectory cannot initialize or advance generation."""


class GenerationSolverError(GenerationError):
    """Raised when the inverse solver produces an invalid numerical result."""


class GenerationConvergenceError(GenerationError):
    """Raised when the inverse solver does not reach the configured tolerance."""

    def __init__(self, time_s: Scalar, diagnostics: GenerationDiagnostics) -> None:
        self.time_s = time_s
        self.diagnostics = diagnostics
        super().__init__(
            "inverse IMU solver did not converge at "
            f"t={time_s:.12g}s; residual={diagnostics.residual_norm:.3e}"
        )
