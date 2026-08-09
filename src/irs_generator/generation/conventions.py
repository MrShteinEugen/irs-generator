"""Signed axis mappings for external trajectory conventions."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
from math import prod

import numpy as np
from numpy.typing import NDArray

from irs_generator.utils.math import VectorArray, VectorLike, vector3

__all__ = ["Axis", "Handedness", "SignedAxis", "SignedAxisMapping"]

type Matrix3 = NDArray[np.longdouble]


class Axis(IntEnum):
    """Index of a Cartesian vector component."""

    X = 0
    Y = 1
    Z = 2


class Handedness(str, Enum):
    """Orientation preserved or reversed by a signed axis mapping."""

    RIGHT_HANDED = "right_handed"
    LEFT_HANDED = "left_handed"


@dataclass(frozen=True, slots=True)
class SignedAxis:
    """Source component and sign used to construct one output component.

    Parameters
    ----------
    axis
        Source vector component.
    sign
        Multiplier applied to the source component. Must be ``-1`` or ``1``.
    """

    axis: Axis
    sign: int = 1

    def __post_init__(self) -> None:
        if not isinstance(self.axis, Axis):
            raise TypeError("axis must be an Axis")
        if isinstance(self.sign, bool) or self.sign not in (-1, 1):
            raise ValueError("sign must be -1 or 1")


@dataclass(frozen=True, slots=True)
class SignedAxisMapping:
    """Permutation and sign changes applied to a three-component vector.

    The fields define the source component for each output component. For example,
    ``SignedAxisMapping(SignedAxis(Axis.Y), SignedAxis(Axis.X),
    SignedAxis(Axis.Z, -1))`` maps ``(x, y, z)`` to ``(y, x, -z)``.

    A mapping must use every source axis exactly once. Its handedness is determined
    by the determinant of its signed permutation matrix.
    """

    x: SignedAxis
    y: SignedAxis
    z: SignedAxis

    def __post_init__(self) -> None:
        if not all(isinstance(component, SignedAxis) for component in self.axes):
            raise TypeError("x, y, and z must be SignedAxis instances")
        if len({component.axis for component in self.axes}) != 3:
            raise ValueError("each source axis must be used exactly once")

    @classmethod
    def identity(cls) -> "SignedAxisMapping":
        """Return the identity mapping ``(x, y, z) -> (x, y, z)``."""

        return cls(SignedAxis(Axis.X), SignedAxis(Axis.Y), SignedAxis(Axis.Z))

    @property
    def axes(self) -> tuple[SignedAxis, SignedAxis, SignedAxis]:
        """Signed source axes in output component order ``(x, y, z)``."""

        return self.x, self.y, self.z

    @property
    def handedness(self) -> Handedness:
        """Return whether the mapping preserves or reverses orientation."""

        source_indices = tuple(component.axis.value for component in self.axes)
        inversions = sum(
            left > right
            for index, left in enumerate(source_indices)
            for right in source_indices[index + 1 :]
        )
        permutation_sign = -1 if inversions % 2 else 1
        sign_product = prod(component.sign for component in self.axes)
        if permutation_sign * sign_product == 1:
            return Handedness.RIGHT_HANDED
        return Handedness.LEFT_HANDED

    def as_matrix(self) -> Matrix3:
        """Return the signed permutation matrix for this mapping."""

        matrix = np.zeros((3, 3), dtype=np.longdouble)
        for output_axis, component in enumerate(self.axes):
            matrix[output_axis, component.axis.value] = component.sign
        return matrix

    def transform_vector(self, vector: VectorLike) -> VectorArray:
        """Apply the mapping to a three-component vector."""

        source = vector3(vector)
        return self.as_matrix() @ source

    def inverse(self) -> "SignedAxisMapping":
        """Return the inverse signed axis mapping."""

        inverse_components: list[SignedAxis | None] = [None, None, None]
        for output_axis, component in enumerate(self.axes):
            inverse_components[component.axis.value] = SignedAxis(
                Axis(output_axis), component.sign
            )
        inverse_x, inverse_y, inverse_z = inverse_components
        assert (
            inverse_x is not None
            and inverse_y is not None
            and inverse_z is not None
        )
        return SignedAxisMapping(inverse_x, inverse_y, inverse_z)

    def validate_handedness(self, expected: Handedness) -> None:
        """Raise ``ValueError`` when the mapping does not have ``expected`` handedness."""

        if not isinstance(expected, Handedness):
            raise TypeError("expected must be a Handedness")
        if self.handedness is not expected:
            raise ValueError(
                f"expected {expected.value} mapping, got {self.handedness.value}"
            )
