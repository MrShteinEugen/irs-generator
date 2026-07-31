from dataclasses import dataclass

from irs_generator.earth_model.coordinates import GeodeticPosition
from irs_generator.navigation_model.navigation import NavigationVelocity
from irs_generator.utils.math import Scalar

__all__ = ["GnssSample"]


@dataclass(frozen=True, slots=True)
class GnssSample:
    """GNSS position/velocity solution.

    Parameters
    ----------
    velocity
        GNSS velocity in the local ENU frame.
    position
        GNSS geodetic position.
    valid
        Whether the sample is available for aiding.
    """

    velocity: NavigationVelocity
    position: GeodeticPosition
    valid: bool = True

    @classmethod
    def from_components(
        cls,
        velocity_east_m_s: Scalar,
        velocity_north_m_s: Scalar,
        velocity_up_m_s: Scalar,
        longitude_rad: Scalar,
        latitude_rad: Scalar,
        height_m: Scalar,
        valid: bool = True,
    ) -> "GnssSample":
        """Create a GNSS sample from scalar components.

        Parameters
        ----------
        velocity_east_m_s, velocity_north_m_s, velocity_up_m_s
            ENU velocity components in metres per second.
        longitude_rad, latitude_rad, height_m
            Geodetic coordinates in radians and metres.
        valid
            Whether the sample is available for aiding.

        Returns
        -------
        GnssSample
            Sample assembled from the provided components.
        """

        return cls(
            velocity=NavigationVelocity(
                velocity_east_m_s, velocity_north_m_s, velocity_up_m_s
            ),
            position=GeodeticPosition(longitude_rad, latitude_rad, height_m),
            valid=bool(valid),
        )
