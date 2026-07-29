from dataclasses import dataclass
from numbers import Real

from irs_generator.utils.math import Vector3

__all__ = ["ImuSample"]


@dataclass(frozen=True, slots=True)
class ImuSample:
    """Specific force and angular rate measured in the body frame."""

    specific_force_body_m_s2: Vector3
    angular_rate_body_rad_s: Vector3

    @classmethod
    def from_components(
        cls,
        acceleration_x_m_s2: float,
        acceleration_y_m_s2: float,
        acceleration_z_m_s2: float,
        angular_rate_x_rad_s: float,
        angular_rate_y_rad_s: float,
        angular_rate_z_rad_s: float,
    ) -> "ImuSample":
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
        return cls(Vector3.zero(), Vector3.zero())

    @property
    def a_3d(self) -> tuple[float, float, float]:
        return self.specific_force_body_m_s2.as_tuple()

    @property
    def w_3d(self) -> tuple[float, float, float]:
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
