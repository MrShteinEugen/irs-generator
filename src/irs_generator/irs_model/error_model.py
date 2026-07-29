"""Composable models of deterministic IMU measurement errors."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, Sequence, runtime_checkable

from irs_generator.utils.math import Vector3
from irs_generator.utils._validation import _validate_dt

from .imu import ImuSample

__all__ = [
    "BiasImuErrorModel",
    "CompositeImuErrorModel",
    "IdealImuErrorModel",
    "ImuErrorModel",
]


@runtime_checkable
class ImuErrorModel(Protocol):
    """Transforms an ideal IMU sample into a modeled sensor observation."""

    def apply(self, sample: ImuSample, dt_s: float) -> ImuSample:
        """Return the sample observed after applying sensor errors."""

    def reset(self) -> None:
        """Reset any error state retained between samples."""


class IdealImuErrorModel:
    """Error-free IMU model used as the default IRS sensor model."""

    __slots__ = ()

    @staticmethod
    def apply(sample: ImuSample, dt_s: float) -> ImuSample:
        _validate_dt(dt_s)
        return sample

    @staticmethod
    def reset() -> None:
        return None


@dataclass(frozen=True, slots=True)
class BiasImuErrorModel:
    """Constant accelerometer and gyroscope bias in body axes."""

    specific_force_bias_m_s2: Vector3 = field(default_factory=Vector3.zero)
    angular_rate_bias_rad_s: Vector3 = field(default_factory=Vector3.zero)

    def apply(self, sample: ImuSample, dt_s: float) -> ImuSample:
        _validate_dt(dt_s)
        return ImuSample(
            specific_force_body_m_s2=(
                sample.specific_force_body_m_s2 + self.specific_force_bias_m_s2
            ),
            angular_rate_body_rad_s=(
                sample.angular_rate_body_rad_s + self.angular_rate_bias_rad_s
            ),
        )

    @staticmethod
    def reset() -> None:
        return None


@dataclass(slots=True)
class CompositeImuErrorModel:
    """Applies independent IMU error models in a deterministic order."""

    models: Sequence[ImuErrorModel]

    def __post_init__(self) -> None:
        self.models = tuple(self.models)
        for model in self.models:
            if not isinstance(model, ImuErrorModel):
                raise TypeError("every model must implement ImuErrorModel")

    def apply(self, sample: ImuSample, dt_s: float) -> ImuSample:
        _validate_dt(dt_s)
        result = sample
        for model in self.models:
            result = model.apply(result, dt_s)
        return result

    def reset(self) -> None:
        for model in self.models:
            model.reset()
