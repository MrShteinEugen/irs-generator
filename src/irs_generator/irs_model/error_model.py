"""Composable models of deterministic IMU measurement errors."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

from irs_generator.utils._validation import _validate_dt
from irs_generator.utils.math import Scalar, Vector3

from .imu import ImuSample

__all__ = [
    "BiasImuErrorModel",
    "CompositeImuErrorModel",
    "IdealImuErrorModel",
    "ImuErrorModel",
]


@runtime_checkable
class ImuErrorModel(Protocol):
    """Transforms an ideal IMU sample into a modeled observation."""

    def apply(self, sample: ImuSample, dt_s: Scalar) -> ImuSample:
        """Apply sensor errors to one IMU sample.

        Parameters
        ----------
        sample
            Ideal IMU sample.
        dt_s
            Positive sample period in seconds.

        Returns
        -------
        ImuSample
            Modeled observed sample.
        """

    def reset(self) -> None:
        """Reset any error state retained between samples."""


class IdealImuErrorModel:
    """Error-free IMU model."""

    __slots__ = ()

    @staticmethod
    def apply(sample: ImuSample, dt_s: Scalar) -> ImuSample:
        _validate_dt(dt_s)
        return sample

    @staticmethod
    def reset() -> None:
        return None


@dataclass(frozen=True, slots=True)
class BiasImuErrorModel:
    """Constant accelerometer and gyroscope bias in body axes.

    Parameters
    ----------
    specific_force_bias_m_s2
        Additive accelerometer bias in m/s².
    angular_rate_bias_rad_s
        Additive gyroscope bias in rad/s.
    """

    specific_force_bias_m_s2: Vector3 = field(default_factory=Vector3.zero)
    angular_rate_bias_rad_s: Vector3 = field(default_factory=Vector3.zero)

    def apply(self, sample: ImuSample, dt_s: Scalar) -> ImuSample:
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
    """Apply several IMU error models in order.

    Parameters
    ----------
    models
        Sequence of error models. The output of one model is passed to the
        next model.
    """

    models: Sequence[ImuErrorModel]

    def __post_init__(self) -> None:
        self.models = tuple(self.models)
        for model in self.models:
            if not isinstance(model, ImuErrorModel):
                raise TypeError("every model must implement ImuErrorModel")

    def apply(self, sample: ImuSample, dt_s: Scalar) -> ImuSample:
        _validate_dt(dt_s)
        result = sample
        for model in self.models:
            result = model.apply(result, dt_s)
        return result

    def reset(self) -> None:
        for model in self.models:
            model.reset()
