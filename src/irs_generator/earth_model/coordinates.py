from dataclasses import dataclass
from math import pi

import numpy as np

from irs_generator.utils._validation import _finite_float
from irs_generator.utils.math import VectorArray

__all__ = ["GeodeticPosition"]


@dataclass(frozen=True, slots=True)
class GeodeticPosition:
    """Longitude, geodetic latitude and ellipsoidal height."""

    longitude_rad: float
    latitude_rad: float
    height_m: float

    def __post_init__(self) -> None:
        longitude = _finite_float(self.longitude_rad, "longitude_rad")
        latitude = _finite_float(self.latitude_rad, "latitude_rad")
        height = _finite_float(self.height_m, "height_m")
        if not -pi / 2.0 <= latitude <= pi / 2.0:
            raise ValueError("latitude_rad must be in [-pi/2, pi/2]")
        object.__setattr__(self, "longitude_rad", longitude)
        object.__setattr__(self, "latitude_rad", latitude)
        object.__setattr__(self, "height_m", height)

    def as_array(self, dtype=np.float64) -> VectorArray:
        if not np.issubdtype(np.dtype(dtype), np.number):
            raise TypeError(
                f"dtype must be a numeric type, got {dtype!r}"
            )

        return np.array(
            (self.longitude_rad, self.latitude_rad, self.height_m),
            dtype=dtype,
        )
