from dataclasses import dataclass

from irs_generator.utils._validation import _finite_float

__all__ = ["RotationParameters"]


@dataclass(frozen=True, slots=True)
class RotationParameters:
    """Earth rotation parameters."""

    angular_velocity_rad_s: float

    def __post_init__(self) -> None:
        angular_velocity = _finite_float(
            self.angular_velocity_rad_s,
            name="angular_velocity_rad_s",
        )
        if angular_velocity < 0.0:
            raise ValueError(
                f"angular_velocity_rad_s must be >= 0, got {angular_velocity!r}"
            )
        object.__setattr__(self, "angular_velocity_rad_s", angular_velocity)
