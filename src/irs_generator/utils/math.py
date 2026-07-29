from dataclasses import dataclass
from math import pi
from numbers import Real
from typing import TypeAlias, Iterable, Iterator

import numpy as np
from numpy._typing import NDArray

from irs_generator.utils._validation import _finite_float


VectorArray: TypeAlias = NDArray[np.float64]
VectorLike: TypeAlias = Iterable[float] | NDArray[np.floating]


def _normalize_longitude(longitude_rad: float) -> float:
    return float((longitude_rad + pi) % (2.0 * pi) - pi)


def vector3(value: VectorLike, *, name: str = "vector") -> VectorArray:
    """Return a validated, independent float64 vector with shape ``(3,)``."""

    result = np.asarray(tuple(value), dtype=np.float64)
    if result.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def angle_difference_rad(first: float, second: float) -> float:
    """Minimal signed ``first - second`` angle in ``[-pi, pi)``."""
    return float((first - second + np.pi) % (2.0 * np.pi) - np.pi)


def skew(vector: VectorLike) -> Matrix3:
    x, y, z = vector3(vector)
    return np.array(
        ((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)),
        dtype=np.float64,
    )


@dataclass(frozen=True, slots=True)
class Vector3:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", _finite_float(self.x, "x"))
        object.__setattr__(self, "y", _finite_float(self.y, "y"))
        object.__setattr__(self, "z", _finite_float(self.z, "z"))

    @classmethod
    def from_iterable(cls, values: VectorLike) -> "Vector3":
        x, y, z = vector3(values)
        return cls(float(x), float(y), float(z))

    @classmethod
    def zero(cls) -> "Vector3":
        return cls(0.0, 0.0, 0.0)

    def as_array(self) -> VectorArray:
        return np.array((self.x, self.y, self.z), dtype=np.float64)

    def as_tuple(self) -> tuple[float, float, float]:
        return self.x, self.y, self.z

    def __iter__(self) -> Iterator[float]:
        yield self.x
        yield self.y
        yield self.z

    def __add__(self, other: "Vector3") -> "Vector3":
        if not isinstance(other, Vector3):
            return NotImplemented
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __truediv__(self, scalar: Real) -> "Vector3":
        divisor = _finite_float(scalar, "scalar")
        if divisor == 0.0:
            raise ZeroDivisionError("cannot divide Vector3 by zero")
        return Vector3(self.x / divisor, self.y / divisor, self.z / divisor)
