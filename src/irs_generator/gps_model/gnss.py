from dataclasses import dataclass

from irs_generator.earth_model.coordinates import GeodeticPosition
from irs_generator.navigation_model.navigation import NavigationVelocity

__all__ = ["GnssSample"]


@dataclass(frozen=True, slots=True)
class GnssSample:
    """GNSS position/velocity solution and its validity flag."""

    velocity: NavigationVelocity
    position: GeodeticPosition
    valid: bool = True

    @classmethod
    def from_components(
        cls,
        velocity_east_m_s: float,
        velocity_north_m_s: float,
        velocity_up_m_s: float,
        longitude_rad: float,
        latitude_rad: float,
        height_m: float,
        valid: bool = True,
    ) -> "GnssSample":
        return cls(
            velocity=NavigationVelocity(velocity_east_m_s, velocity_north_m_s, velocity_up_m_s),
            position=GeodeticPosition(longitude_rad, latitude_rad, height_m),
            valid=bool(valid),
        )

