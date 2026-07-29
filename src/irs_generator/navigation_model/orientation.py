from dataclasses import dataclass
from math import pi

import numpy as np

from irs_generator.utils._validation import _finite_float
from irs_generator.utils.math import VectorArray

__all__ = ["EulerAngles"]


@dataclass(frozen=True, slots=True)
class EulerAngles:
    """Pitch, roll and heading using the project's existing DCM convention."""

    pitch_rad: float
    roll_rad: float
    heading_rad: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "pitch_rad", _finite_float(self.pitch_rad, "pitch_rad"))
        object.__setattr__(self, "roll_rad", _finite_float(self.roll_rad, "roll_rad"))
        heading = _finite_float(self.heading_rad, "heading_rad") % (2.0 * pi)
        object.__setattr__(self, "heading_rad", heading)

    def as_array(self, dtype=np.float64) -> VectorArray:
        if not np.issubdtype(np.dtype(dtype), np.number):
            raise TypeError(
                f"dtype must be a numeric type, got {dtype!r}"
            )

        return np.array(
            (self.pitch_rad, self.roll_rad, self.heading_rad),
            dtype=np.float64,
        )
