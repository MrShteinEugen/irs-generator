from dataclasses import dataclass

import numpy as np

from irs_generator.earth_model.coordinates import GeodeticPosition
from irs_generator.navigation_model.orientation import EulerAngles
from irs_generator.utils._validation import _finite_scalar
from irs_generator.utils.math import Scalar, VectorArray

__all__ = ["NavigationState", "NavigationVelocity"]


@dataclass(frozen=True, slots=True)
class NavigationVelocity:
    """Velocity in the local ENU navigation frame.

    Parameters
    ----------
    east_m_s
        East velocity component in metres per second.
    north_m_s
        North velocity component in metres per second.
    up_m_s
        Up velocity component in metres per second.
    """

    east_m_s: Scalar
    north_m_s: Scalar
    up_m_s: Scalar

    def __post_init__(self) -> None:
        object.__setattr__(self, "east_m_s", _finite_scalar(self.east_m_s, "east_m_s"))
        object.__setattr__(
            self,
            "north_m_s",
            _finite_scalar(self.north_m_s, "north_m_s"),
        )
        object.__setattr__(self, "up_m_s", _finite_scalar(self.up_m_s, "up_m_s"))

    def as_array(
        self,
        dtype: np.dtype[np.longdouble] | type[np.longdouble] = np.longdouble,
    ) -> VectorArray:
        """Return velocity as ``(east_m_s, north_m_s, up_m_s)``.

        Parameters
        ----------
        dtype
            Numeric dtype for the returned array. Defaults to ``np.longdouble``.

        Returns
        -------
        numpy.ndarray
            New array containing ENU velocity components.
        """

        if not np.issubdtype(np.dtype(dtype), np.number):
            raise TypeError(f"dtype must be a numeric type, got {dtype!r}")

        return np.array(
            (self.east_m_s, self.north_m_s, self.up_m_s),
            dtype=dtype,
        )


@dataclass(frozen=True, slots=True)
class NavigationState:
    """INS navigation state.

    Parameters
    ----------
    velocity
        Velocity in the local ENU frame.
    position
        Geodetic position.
    attitude
        Body attitude using the project Euler/DCM convention.
    correction_applied
        ``True`` when an aiding correction was used to produce this state.
    """

    velocity: NavigationVelocity
    position: GeodeticPosition
    attitude: EulerAngles
    correction_applied: bool = False

    @classmethod
    def from_components(
        cls,
        velocity_east_m_s: Scalar,
        velocity_north_m_s: Scalar,
        velocity_up_m_s: Scalar,
        longitude_rad: Scalar,
        latitude_rad: Scalar,
        height_m: Scalar,
        pitch_rad: Scalar,
        roll_rad: Scalar,
        heading_rad: Scalar,
        correction_applied: bool = False,
    ) -> "NavigationState":
        """Create a state from scalar components.

        Parameters
        ----------
        velocity_east_m_s, velocity_north_m_s, velocity_up_m_s
            ENU velocity components in metres per second.
        longitude_rad, latitude_rad, height_m
            Geodetic coordinates in radians and metres.
        pitch_rad, roll_rad, heading_rad
            Euler attitude angles in radians.
        correction_applied
            Whether aiding correction has been applied.

        Returns
        -------
        NavigationState
            State assembled from the provided components.
        """

        return cls(
            velocity=NavigationVelocity(
                velocity_east_m_s, velocity_north_m_s, velocity_up_m_s
            ),
            position=GeodeticPosition(longitude_rad, latitude_rad, height_m),
            attitude=EulerAngles(pitch_rad, roll_rad, heading_rad),
            correction_applied=bool(correction_applied),
        )

    @property
    def velocities_3d_nav(self) -> tuple[Scalar, Scalar, Scalar]:
        return (
            self.velocity.east_m_s,
            self.velocity.north_m_s,
            self.velocity.up_m_s,
        )

    @property
    def coordinates_3d_nav(self) -> tuple[Scalar, Scalar, Scalar]:
        return (
            self.position.longitude_rad,
            self.position.latitude_rad,
            self.position.height_m,
        )

    @property
    def orientation_angles_3d(self) -> tuple[Scalar, Scalar, Scalar]:
        return (
            self.attitude.pitch_rad,
            self.attitude.roll_rad,
            self.attitude.heading_rad,
        )

    def to_csv_line(
        self,
        time_s: Scalar = 0.0,
        separator: str = ",",
        end_line: bool = True,
    ) -> str:
        """Serialize the state as one delimited text row.

        Parameters
        ----------
        time_s
            Timestamp to place at the beginning of the row.
        separator
            Column separator.
        end_line
            Append a trailing newline when ``True``.

        Returns
        -------
        str
            Serialized row in the order ``time, velocity, position, attitude,
            correction_applied``.
        """

        if not separator:
            raise ValueError("separator must not be empty")
        values = (
            _finite_scalar(time_s, "time_s"),
            *self.velocities_3d_nav,
            *self.coordinates_3d_nav,
            *self.orientation_angles_3d,
            int(self.correction_applied),
        )
        line = separator.join(str(value) for value in values)
        return line + ("\n" if end_line else "")
