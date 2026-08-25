from dataclasses import dataclass

import numpy as np

from irs_generator.utils._validation import _finite_scalar
from irs_generator.utils.math import Scalar, VectorArray

__all__ = ["EulerAngles"]


@dataclass(frozen=True, slots=True)
class EulerAngles:
    """Euler attitude angles for the project's DCM convention.

    Parameters
    ----------
    pitch_rad
        Pitch angle in radians.
    roll_rad
        Roll angle in radians.
    heading_rad
        Heading angle in radians. Stored modulo ``2*pi``.
    """

    pitch_rad: Scalar
    roll_rad: Scalar
    heading_rad: Scalar

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "pitch_rad", _finite_scalar(self.pitch_rad, "pitch_rad")
        )
        object.__setattr__(self, "roll_rad", _finite_scalar(self.roll_rad, "roll_rad"))
        heading = _finite_scalar(self.heading_rad, "heading_rad") % (
            2.0 * np.longdouble(np.pi)
        )
        object.__setattr__(self, "heading_rad", heading)

    def as_array(
        self,
        dtype: np.dtype[np.longdouble] | type[np.longdouble] = np.longdouble,
    ) -> VectorArray:
        """Return angles as ``(pitch_rad, roll_rad, heading_rad)``.

        Parameters
        ----------
        dtype
            Numeric dtype for the returned array. Defaults to ``np.longdouble``.

        Returns
        -------
        numpy.ndarray
            New array containing pitch, roll and heading.
        """

        if not np.issubdtype(np.dtype(dtype), np.number):
            raise TypeError(f"dtype must be a numeric type, got {dtype!r}")

        return np.array(
            (self.pitch_rad, self.roll_rad, self.heading_rad),
            dtype=dtype,
        )
