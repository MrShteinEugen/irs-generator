"""Streaming synthesis of ideal IMU and GNSS data from target motion."""

from .conventions import (
    AngleUnit,
    Axis,
    Handedness,
    InputConvention,
    SignedAxis,
    SignedAxisMapping,
)
from .dcm import (
    DcmTrajectoryGenerator,
    DcmTrajectoryPoint,
    DcmTrajectoryReader,
)
from .exceptions import (
    GenerationConvergenceError,
    GenerationError,
    GenerationSolverError,
    InvalidTrajectoryError,
)
from .formats import CsvOutputFormat, DatOutputFormat
from .generator import GenerationConfig, GenerationMetadata, SyntheticDataGenerator
from .io import CsvOutputWriter, CsvTrajectoryReader, CsvTrajectorySchema
from .models import (
    GeneratedStep,
    GenerationDiagnostics,
    TargetTrajectoryPoint,
    Trajectory,
    TrajectoryPoint,
)
from .providers import TrajectoryProviderAdapter
from .solver import StepSolverConfig

__all__ = [
    "AngleUnit",
    "Axis",
    "CsvOutputWriter",
    "CsvOutputFormat",
    "CsvTrajectoryReader",
    "CsvTrajectorySchema",
    "DatOutputFormat",
    "DcmTrajectoryGenerator",
    "DcmTrajectoryPoint",
    "DcmTrajectoryReader",
    "GeneratedStep",
    "GenerationConfig",
    "GenerationConvergenceError",
    "GenerationDiagnostics",
    "GenerationError",
    "GenerationMetadata",
    "GenerationSolverError",
    "Handedness",
    "InputConvention",
    "InvalidTrajectoryError",
    "SignedAxis",
    "SignedAxisMapping",
    "TargetTrajectoryPoint",
    "Trajectory",
    "TrajectoryProviderAdapter",
    "TrajectoryPoint",
    "SyntheticDataGenerator",
    "StepSolverConfig",
]
