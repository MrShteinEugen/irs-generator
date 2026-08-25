from dataclasses import dataclass

import numpy as np

from irs_generator.utils._validation import _finite_scalar
from irs_generator.utils.math import Scalar, VectorArray

__all__ = ["GeodeticPosition"]


@dataclass(frozen=True, slots=True)
class GeodeticPosition:
    """Geodetic position on an Earth reference surface.

    Parameters
    ----------
    longitude_rad
        Longitude in radians.
    latitude_rad
        Geodetic latitude in radians. Must be in ``[-pi/2, pi/2]``.
    height_m
        Ellipsoidal height in metres.
    """

    longitude_rad: Scalar
    latitude_rad: Scalar
    height_m: Scalar

    def __post_init__(self) -> None:
        longitude = _finite_scalar(self.longitude_rad, "longitude_rad")
        latitude = _finite_scalar(self.latitude_rad, "latitude_rad")
        height = _finite_scalar(self.height_m, "height_m")
        if not -np.longdouble(np.pi / 2.0) <= latitude <= np.longdouble(np.pi / 2.0):
            raise ValueError("latitude_rad must be in [-pi/2, pi/2]")
        object.__setattr__(self, "longitude_rad", longitude)
        object.__setattr__(self, "latitude_rad", latitude)
        object.__setattr__(self, "height_m", height)

    def as_array(
        self,
        dtype: np.dtype[np.longdouble] | type[np.longdouble] = np.longdouble,
    ) -> VectorArray:
        """Return position as ``(longitude_rad, latitude_rad, height_m)``.

        Parameters
        ----------
        dtype
            Numeric dtype for the returned array. Defaults to ``np.longdouble``.

        Returns
        -------
        numpy.ndarray
            New array containing longitude, latitude and height.
        """

        if not np.issubdtype(np.dtype(dtype), np.number):
            raise TypeError(f"dtype must be a numeric type, got {dtype!r}")

        return np.array(
            (self.longitude_rad, self.latitude_rad, self.height_m),
            dtype=dtype,
        )
