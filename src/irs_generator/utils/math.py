from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from numbers import Real
from typing import Any

import numpy as np
from numpy.typing import NDArray

from irs_generator.utils._validation import _finite_scalar

type Scalar = float | np.longdouble
type VectorArray = NDArray[np.longdouble]
type VectorLike = Iterable[Scalar] | NDArray[np.floating[Any]]


def _normalize_longitude(longitude_rad: Scalar) -> np.longdouble:
    """Normalize longitude to the half-open interval ``[-pi, pi)``."""

    longitude = np.longdouble(longitude_rad)
    pi = np.longdouble(np.pi)
    return np.longdouble((longitude + pi) % (2.0 * pi) - pi)


def vector3(value: VectorLike, *, name: str = "vector") -> VectorArray:
    """Return a validated 3-vector.

    Parameters
    ----------
    value
        Iterable or NumPy array containing exactly three finite numeric values.
    name
        Name used in validation error messages.

    Returns
    -------
    numpy.ndarray
        Independent array with shape ``(3,)`` and dtype ``np.longdouble``.

    Raises
    ------
    ValueError
        If ``value`` does not contain exactly three finite numbers.
    """

    result = np.asarray(tuple(value), dtype=np.longdouble)
    if result.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def angle_difference_rad(first: Scalar, second: Scalar) -> np.longdouble:
    """Return the wrapped signed angle difference ``first - second``.

    Parameters
    ----------
    first, second
        Angles in radians.

    Returns
    -------
    numpy.longdouble
        Difference in the half-open interval ``[-pi, pi)``.
    """
    pi = np.longdouble(np.pi)
    return np.longdouble(
        (np.longdouble(first) - np.longdouble(second) + pi) % (2.0 * pi) - pi
    )


def skew(vector: VectorLike) -> VectorArray:
    """Return the skew-symmetric matrix for a 3-vector.

    Parameters
    ----------
    vector
        Vector ``(x, y, z)``.

    Returns
    -------
    numpy.ndarray
        Matrix ``S`` such that ``S @ a == cross(vector, a)``.
    """

    x, y, z = vector3(vector)
    return np.array(
        ((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)),
        dtype=np.longdouble,
    )


@dataclass(frozen=True, slots=True)
class Vector3:
    """Immutable 3D vector stored with the project scalar precision.

    Parameters
    ----------
    x, y, z
        Finite vector components.
    """

    x: Scalar
    y: Scalar
    z: Scalar

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", _finite_scalar(self.x, "x"))
        object.__setattr__(self, "y", _finite_scalar(self.y, "y"))
        object.__setattr__(self, "z", _finite_scalar(self.z, "z"))

    @classmethod
    def from_iterable(cls, values: VectorLike) -> "Vector3":
        """Create a vector from an iterable with three components.

        Parameters
        ----------
        values
            Iterable or NumPy array containing ``x``, ``y`` and ``z``.

        Returns
        -------
        Vector3
            Validated vector instance.
        """

        x, y, z = vector3(values)
        return cls(x, y, z)

    @classmethod
    def zero(cls) -> "Vector3":
        """Return the zero vector."""

        return cls(0.0, 0.0, 0.0)

    def as_array(self) -> VectorArray:
        """Return the vector as a new ``np.longdouble`` array."""

        return np.array((self.x, self.y, self.z), dtype=np.longdouble)

    def as_tuple(self) -> tuple[Scalar, Scalar, Scalar]:
        """Return components as ``(x, y, z)``."""

        return self.x, self.y, self.z

    def __iter__(self) -> Iterator[Scalar]:
        yield self.x
        yield self.y
        yield self.z

    def __add__(self, other: "Vector3") -> "Vector3":
        if not isinstance(other, Vector3):
            return NotImplemented
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __truediv__(self, scalar: Real) -> "Vector3":
        divisor = _finite_scalar(scalar, "scalar")
        if divisor == 0.0:
            raise ZeroDivisionError("cannot divide Vector3 by zero")
        return Vector3(self.x / divisor, self.y / divisor, self.z / divisor)
