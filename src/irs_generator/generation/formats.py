"""Output format profiles for generated navigation data."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

__all__ = ["CsvOutputFormat", "DatOutputFormat"]


@dataclass(frozen=True, slots=True)
class CsvOutputFormat:
    """Output contract for generated IMU and GNSS streams.

    Parameters
    ----------
    delimiter
        Single-character delimiter used by both files.
    include_header
        Write header rows when ``True``.
    imu_filename, gps_filename
        Output file names. Directory components are not allowed.
    imu_columns, gps_columns
        Column names for IMU and GNSS files.
    angular_rate_unit
        Unit used when writing angular rates: ``"rad/s"`` or ``"deg/s"``.
    geographic_angle_unit
        Unit used when writing latitude and longitude: ``"rad"`` or ``"deg"``.
    gnss_good_value, chassis_value
        Constant status fields written to the GNSS file.
    """

    delimiter: str = ","
    include_header: bool = True
    imu_filename: str = "imu.csv"
    gps_filename: str = "gps.csv"
    imu_columns: tuple[str, ...] = (
        "time_s",
        "specific_force_x_m_s2",
        "specific_force_y_m_s2",
        "specific_force_z_m_s2",
        "angular_rate_x_rad_s",
        "angular_rate_y_rad_s",
        "angular_rate_z_rad_s",
    )
    gps_columns: tuple[str, ...] = (
        "time_s",
        "latitude_rad",
        "longitude_rad",
        "height_m",
        "velocity_east_m_s",
        "velocity_north_m_s",
        "gnss_good",
        "chassis",
    )
    angular_rate_unit: Literal["rad/s", "deg/s"] = "rad/s"
    geographic_angle_unit: Literal["rad", "deg"] = "rad"
    gnss_good_value: int = 1
    chassis_value: int = 1

    def __post_init__(self) -> None:
        if len(self.delimiter) != 1:
            raise ValueError("delimiter must contain exactly one character")
        if not self.imu_filename:
            raise ValueError("imu_filename must not be empty")
        if not self.gps_filename:
            raise ValueError("gps_filename must not be empty")
        for filename in (self.imu_filename, self.gps_filename):
            if Path(filename).name != filename:
                raise ValueError(
                    "output filenames must not contain directory components"
                )
        if self.imu_filename == self.gps_filename:
            raise ValueError("imu_filename and gps_filename must be different")
        if len(self.imu_columns) != 7:
            raise ValueError("imu_columns must contain exactly 7 column names")
        if len(self.gps_columns) != 8:
            raise ValueError("gps_columns must contain exactly 8 column names")
        if any(not column.strip() for column in (*self.imu_columns, *self.gps_columns)):
            raise ValueError("output column names must not be empty")
        if self.angular_rate_unit not in {"rad/s", "deg/s"}:
            raise ValueError("angular_rate_unit must be 'rad/s' or 'deg/s'")
        if self.geographic_angle_unit not in {"rad", "deg"}:
            raise ValueError("geographic_angle_unit must be 'rad' or 'deg'")


@dataclass(frozen=True, slots=True)
class DatOutputFormat(CsvOutputFormat):
    """Output profile for canonical ``imu.dat`` and ``gps.dat`` files."""

    delimiter: str = " "
    imu_filename: str = "imu.dat"
    gps_filename: str = "gps.dat"
    imu_columns: tuple[str, ...] = ("Time", "Ax", "Ay", "Az", "Wx", "Wy", "Wz")
    gps_columns: tuple[str, ...] = (
        "T",
        "Lat",
        "Lon",
        "Hsns",
        "Ve",
        "Vn",
        "SNS_GOOD",
        "Shassi",
    )
    angular_rate_unit: Literal["rad/s", "deg/s"] = "deg/s"
    geographic_angle_unit: Literal["rad", "deg"] = "deg"


# Compatibility alias for code that adopted the initial package preview.
LegacyDatFormat = DatOutputFormat
