from typing import SupportsFloat

import numpy as np

_HALF_PI = np.longdouble(np.pi / 2.0)

type FloatConvertible = str | SupportsFloat


def _finite_scalar(value: FloatConvertible, name: str) -> np.longdouble:
    result = np.longdouble(value)
    if not bool(np.isfinite(result)):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return result


def _positive_scalar(value: FloatConvertible, *, name: str) -> np.longdouble:
    result = _finite_scalar(value, name=name)
    if result <= 0.0:
        raise ValueError(f"{name} must be > 0, got {result!r}")
    return result


def _non_negative_scalar(value: FloatConvertible, name: str) -> np.longdouble:
    result = np.longdouble(value)
    if not bool(np.isfinite(result)) or result < 0.0:
        raise ValueError(f"{name} must be finite and >= 0, got {value!r}")
    return result


def _validated_latitude(latitude_rad: FloatConvertible) -> np.longdouble:
    latitude = _finite_scalar(latitude_rad, name="latitude_rad")
    if not -_HALF_PI <= latitude <= _HALF_PI:
        raise ValueError(f"latitude_rad must be in [-pi/2, pi/2], got {latitude!r}")
    return latitude


def _validate_dt(dt_s: FloatConvertible) -> np.longdouble:
    dt = np.longdouble(dt_s)
    if not bool(np.isfinite(dt)) or dt <= 0.0:
        raise ValueError(f"dt_s must be finite and > 0, got {dt_s!r}")
    return dt


_finite_float = _finite_scalar
_positive_float = _positive_scalar
_non_negative = _non_negative_scalar
