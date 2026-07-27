from math import isfinite, pi

_HALF_PI = pi / 2.0

def _finite_float(value: float, *, name: str) -> float:
    result = float(value)
    if not isfinite(result):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return result


def _positive_float(value: float, *, name: str) -> float:
    result = _finite_float(value, name=name)
    if result <= 0.0:
        raise ValueError(f"{name} must be > 0, got {result!r}")
    return result


def _validated_latitude(latitude_rad: float) -> float:
    latitude = _finite_float(latitude_rad, name="latitude_rad")
    if not -_HALF_PI <= latitude <= _HALF_PI:
        raise ValueError(
            "latitude_rad must be in [-pi/2, pi/2], "
            f"got {latitude!r}"
        )
    return latitude
