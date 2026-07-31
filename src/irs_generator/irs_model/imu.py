from dataclasses import dataclass
from numbers import Real

from irs_generator.utils.math import Scalar, Vector3

__all__ = ["ImuSample"]


@dataclass(frozen=True, slots=True)
class ImuSample:
    """IMU sample in the body frame.

    Parameters
    ----------
    specific_force_body_m_s2
        Specific force vector in m/s².
    angular_rate_body_rad_s
        Body angular-rate vector in rad/s.
    """

    specific_force_body_m_s2: Vector3
    angular_rate_body_rad_s: Vector3

    @classmethod
    def from_components(
        cls,
        acceleration_x_m_s2: Scalar,
        acceleration_y_m_s2: Scalar,
        acceleration_z_m_s2: Scalar,
        angular_rate_x_rad_s: Scalar,
        angular_rate_y_rad_s: Scalar,
        angular_rate_z_rad_s: Scalar,
    ) -> "ImuSample":
        """Create a sample from scalar components.

        Parameters
        ----------
        acceleration_x_m_s2, acceleration_y_m_s2, acceleration_z_m_s2
            Specific-force components in the body frame, m/s².
        angular_rate_x_rad_s, angular_rate_y_rad_s, angular_rate_z_rad_s
            Angular-rate components in the body frame, rad/s.

        Returns
        -------
        ImuSample
            Sample assembled from body-frame vectors.
        """

        return cls(
            specific_force_body_m_s2=Vector3(
                acceleration_x_m_s2,
                acceleration_y_m_s2,
                acceleration_z_m_s2,
            ),
            angular_rate_body_rad_s=Vector3(
                angular_rate_x_rad_s,
                angular_rate_y_rad_s,
                angular_rate_z_rad_s,
            ),
        )

    @classmethod
    def zero(cls) -> "ImuSample":
        """Return zero specific force and zero angular rate."""

        return cls(Vector3.zero(), Vector3.zero())

    @property
    def a_3d(self) -> tuple[Scalar, Scalar, Scalar]:
        """Specific-force components as ``(ax, ay, az)``."""

        return self.specific_force_body_m_s2.as_tuple()

    @property
    def w_3d(self) -> tuple[Scalar, Scalar, Scalar]:
        """Angular-rate components as ``(wx, wy, wz)``."""

        return self.angular_rate_body_rad_s.as_tuple()

    def __add__(self, other: "ImuSample") -> "ImuSample":
        if not isinstance(other, ImuSample):
            return NotImplemented
        return ImuSample(
            self.specific_force_body_m_s2 + other.specific_force_body_m_s2,
            self.angular_rate_body_rad_s + other.angular_rate_body_rad_s,
        )

    def __truediv__(self, scalar: Real) -> "ImuSample":
        return ImuSample(
            self.specific_force_body_m_s2 / scalar,
            self.angular_rate_body_rad_s / scalar,
        )
