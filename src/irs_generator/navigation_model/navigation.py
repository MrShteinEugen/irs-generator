from dataclasses import dataclass

import numpy as np

from irs_generator.earth_model.coordinates import GeodeticPosition
from irs_generator.navigation_model.orientation import EulerAngles
from irs_generator.utils._validation import _finite_float
from irs_generator.utils.math import VectorArray

__all__ = ["NavigationState", "NavigationVelocity"]


@dataclass(frozen=True, slots=True)
class NavigationVelocity:
    """Velocity in the local ENU navigation frame."""

    east_m_s: float
    north_m_s: float
    up_m_s: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "east_m_s", _finite_float(self.east_m_s, "east_m_s"))
        object.__setattr__(
            self,
            "north_m_s",
            _finite_float(self.north_m_s, "north_m_s"),
        )
        object.__setattr__(self, "up_m_s", _finite_float(self.up_m_s, "up_m_s"))

    def as_array(
        self,
        dtype: np.dtype[np.float64] | type[np.float64] = np.float64,
    ) -> VectorArray:
        if not np.issubdtype(np.dtype(dtype), np.number):
            raise TypeError(f"dtype must be a numeric type, got {dtype!r}")

        return np.array(
            (self.east_m_s, self.north_m_s, self.up_m_s),
            dtype=dtype,
        )


@dataclass(frozen=True, slots=True)
class NavigationState:
    velocity: NavigationVelocity
    position: GeodeticPosition
    attitude: EulerAngles
    correction_applied: bool = False

    @classmethod
    def from_components(
        cls,
        velocity_east_m_s: float,
        velocity_north_m_s: float,
        velocity_up_m_s: float,
        longitude_rad: float,
        latitude_rad: float,
        height_m: float,
        pitch_rad: float,
        roll_rad: float,
        heading_rad: float,
        correction_applied: bool = False,
    ) -> "NavigationState":
        return cls(
            velocity=NavigationVelocity(
                velocity_east_m_s, velocity_north_m_s, velocity_up_m_s
            ),
            position=GeodeticPosition(longitude_rad, latitude_rad, height_m),
            attitude=EulerAngles(pitch_rad, roll_rad, heading_rad),
            correction_applied=bool(correction_applied),
        )

    @property
    def velocities_3d_nav(self) -> tuple[float, float, float]:
        return (
            self.velocity.east_m_s,
            self.velocity.north_m_s,
            self.velocity.up_m_s,
        )

    @property
    def coordinates_3d_nav(self) -> tuple[float, float, float]:
        return (
            self.position.longitude_rad,
            self.position.latitude_rad,
            self.position.height_m,
        )

    @property
    def orientation_angles_3d(self) -> tuple[float, float, float]:
        return (
            self.attitude.pitch_rad,
            self.attitude.roll_rad,
            self.attitude.heading_rad,
        )

    def to_csv_line(
        self,
        time_s: float = 0.0,
        separator: str = ",",
        end_line: bool = True,
    ) -> str:
        if not separator:
            raise ValueError("separator must not be empty")
        values = (
            _finite_float(time_s, "time_s"),
            *self.velocities_3d_nav,
            *self.coordinates_3d_nav,
            *self.orientation_angles_3d,
            int(self.correction_applied),
        )
        line = separator.join(str(value) for value in values)
        return line + ("\n" if end_line else "")
