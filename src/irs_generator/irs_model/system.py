"""High-level composition of IMU errors and a navigation algorithm."""

from __future__ import annotations

from math import isfinite

from irs_generator.gps_model import GnssSample
from irs_generator.navigation_model import NavigationState

from .algorithm import NavigationAlgorithm
from .error_model import IdealImuErrorModel, ImuErrorModel
from .imu import ImuSample

__all__ = ["InertialReferenceSystem"]


class InertialReferenceSystem:
    """IRS composed of a navigation algorithm and an IMU error model.

    The system accepts ideal samples from a scenario generator, applies the
    configured sensor-error model, and passes the resulting observation to the
    selected navigation algorithm. Alternative algorithms therefore reuse the
    same IMU and error-model infrastructure.
    """

    __slots__ = ("_algorithm", "_imu_error_model")

    def __init__(
        self,
        algorithm: NavigationAlgorithm,
        *,
        imu_error_model: ImuErrorModel | None = None,
    ) -> None:
        if not isinstance(algorithm, NavigationAlgorithm):
            raise TypeError("algorithm must implement NavigationAlgorithm")
        error_model = (
            IdealImuErrorModel() if imu_error_model is None else imu_error_model
        )
        if not isinstance(error_model, ImuErrorModel):
            raise TypeError("imu_error_model must implement ImuErrorModel")
        self._algorithm = algorithm
        self._imu_error_model = error_model

    @property
    def state(self) -> NavigationState:
        return self._algorithm.state

    @property
    def algorithm(self) -> NavigationAlgorithm:
        return self._algorithm

    @property
    def imu_error_model(self) -> ImuErrorModel:
        return self._imu_error_model

    def reset(self, initial_state: NavigationState) -> None:
        self._algorithm.reset(initial_state)
        self._imu_error_model.reset()

    def step(
        self,
        ideal_imu_sample: ImuSample,
        dt_s: float,
        gnss_sample: GnssSample | None = None,
    ) -> NavigationState:
        dt = float(dt_s)
        if not isfinite(dt) or dt <= 0.0:
            raise ValueError(f"dt_s must be finite and > 0, got {dt_s!r}")
        observed_sample = self._imu_error_model.apply(ideal_imu_sample, dt)
        return self._algorithm.step(observed_sample, dt, gnss_sample)
