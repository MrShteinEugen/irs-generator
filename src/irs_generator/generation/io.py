"""Streaming CSV adapters for prepared trajectories and legacy outputs."""

from __future__ import annotations

import csv
from collections.abc import Iterator
from dataclasses import dataclass
from math import degrees, isclose, isfinite
from pathlib import Path
from typing import TextIO

from irs_generator.earth_model import GeodeticPosition
from irs_generator.navigation_model import EulerAngles, NavigationVelocity

from .formats import CsvOutputFormat, LegacyDatFormat
from .models import GeneratedStep, TargetTrajectoryPoint

__all__ = ["CsvOutputWriter", "CsvTrajectoryReader", "CsvTrajectorySchema"]


@dataclass(frozen=True, slots=True)
class CsvTrajectorySchema:
    """Column mapping for the prepared generic trajectory CSV format."""

    time_s: str = "t_meas_s"
    latitude_deg: str = "lat_deg"
    longitude_deg: str = "lon_deg"
    height_m: str = "alt_m"
    pitch_rad: str = "pitch_rad"
    roll_rad: str = "roll_rad"
    heading_rad: str = "heading_rad"
    velocity_east_m_s: str = "v_e_mps"
    velocity_north_m_s: str = "v_n_mps"
    velocity_up_m_s: str = "v_u_mps"
    require_uniform_time_step: bool = True
    time_step_absolute_tolerance_s: float = 1e-9
    time_step_relative_tolerance: float = 1e-6

    def __post_init__(self) -> None:
        for column in self.required_columns:
            if not isinstance(column, str) or not column.strip():
                raise ValueError("trajectory column names must be non-empty strings")
        for name in ("time_step_absolute_tolerance_s", "time_step_relative_tolerance"):
            value = float(getattr(self, name))
            if not isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and >= 0")
            object.__setattr__(self, name, value)
        if len(set(self.required_columns)) != len(self.required_columns):
            raise ValueError("trajectory column names must be unique")

    @property
    def required_columns(self) -> tuple[str, ...]:
        return (
            self.time_s,
            self.latitude_deg,
            self.longitude_deg,
            self.height_m,
            self.pitch_rad,
            self.roll_rad,
            self.heading_rad,
            self.velocity_east_m_s,
            self.velocity_north_m_s,
            self.velocity_up_m_s,
        )


class CsvTrajectoryReader:
    """Read prepared target points one at a time without buffering a CSV."""

    def __init__(
        self,
        path: str | Path,
        *,
        schema: CsvTrajectorySchema | None = None,
    ) -> None:
        self._path = Path(path)
        self._schema = schema if schema is not None else CsvTrajectorySchema()

    def __iter__(self) -> Iterator[TargetTrajectoryPoint]:
        with self._path.open("r", encoding="utf-8", newline="") as file:
            reader = csv.DictReader(file)
            if reader.fieldnames is None:
                raise ValueError("trajectory CSV must have a header row")
            missing = set(self._schema.required_columns) - set(reader.fieldnames)
            if missing:
                names = ", ".join(sorted(missing))
                raise ValueError(f"trajectory CSV is missing columns: {names}")

            previous_time_s: float | None = None
            reference_time_step_s: float | None = None
            for row_number, row in enumerate(reader, start=2):
                point = self._parse_row(
                    row,
                    row_number,
                    require_position=previous_time_s is None,
                )
                if previous_time_s is not None and point.time_s <= previous_time_s:
                    raise ValueError(
                        "time_s must be strictly increasing; "
                        f"row {row_number} is invalid"
                    )
                if previous_time_s is not None:
                    time_step_s = point.time_s - previous_time_s
                    if reference_time_step_s is None:
                        reference_time_step_s = time_step_s
                    elif self._schema.require_uniform_time_step and not isclose(
                        time_step_s,
                        reference_time_step_s,
                        rel_tol=self._schema.time_step_relative_tolerance,
                        abs_tol=self._schema.time_step_absolute_tolerance_s,
                    ):
                        raise ValueError(
                            "time grid must be uniform; "
                            f"row {row_number} has dt={time_step_s:.12g}s, "
                            f"expected {reference_time_step_s:.12g}s"
                        )
                previous_time_s = point.time_s
                yield point

    def _parse_row(
        self,
        row: dict[str, str | None],
        row_number: int,
        *,
        require_position: bool,
    ) -> TargetTrajectoryPoint:
        schema = self._schema
        try:
            position = (
                GeodeticPosition(
                    longitude_rad=_degrees_to_radians(
                        _required(row, schema.longitude_deg)
                    ),
                    latitude_rad=_degrees_to_radians(
                        _required(row, schema.latitude_deg)
                    ),
                    height_m=float(_required(row, schema.height_m)),
                )
                if require_position
                else None
            )
            return TargetTrajectoryPoint(
                time_s=float(_required(row, schema.time_s)),
                position=position,
                velocity=NavigationVelocity(
                    east_m_s=float(_required(row, schema.velocity_east_m_s)),
                    north_m_s=float(_required(row, schema.velocity_north_m_s)),
                    up_m_s=float(_required(row, schema.velocity_up_m_s)),
                ),
                attitude=EulerAngles(
                    pitch_rad=float(_required(row, schema.pitch_rad)),
                    roll_rad=float(_required(row, schema.roll_rad)),
                    heading_rad=float(_required(row, schema.heading_rad)),
                ),
            )
        except (TypeError, ValueError) as error:
            raise ValueError(f"invalid trajectory value at row {row_number}") from error


class CsvOutputWriter:
    """Write IMU, GPS, and optional diagnostics rows as they are generated."""

    def __init__(
        self,
        output_dir: str | Path,
        *,
        format: CsvOutputFormat | None = None,
        debug_path: str | Path | None = None,
    ) -> None:
        self._format = format if format is not None else LegacyDatFormat()
        self._output_dir = Path(output_dir)
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._imu_file = (self._output_dir / self._format.imu_filename).open(
            "w", encoding="utf-8", newline=""
        )
        self._gps_file = (self._output_dir / self._format.gps_filename).open(
            "w", encoding="utf-8", newline=""
        )
        self._debug_file = None if debug_path is None else _open_debug_file(debug_path)
        self._imu_writer = csv.writer(self._imu_file, delimiter=self._format.delimiter)
        self._gps_writer = csv.writer(self._gps_file, delimiter=self._format.delimiter)
        self._debug_writer = (
            None if self._debug_file is None else csv.writer(self._debug_file)
        )
        if self._format.include_header:
            self._imu_writer.writerow(self._format.imu_columns)
            self._gps_writer.writerow(self._format.gps_columns)
            if self._debug_writer is not None:
                self._debug_writer.writerow(
                    ("time_s", "iteration_count", "residual_norm", "converged")
                )

    def write(self, step: GeneratedStep) -> None:
        acceleration = step.imu_sample.specific_force_body_m_s2
        angular_rate = step.imu_sample.angular_rate_body_rad_s
        state = step.navigation_state
        self._imu_writer.writerow(
            (
                step.time_s,
                acceleration.x,
                acceleration.y,
                acceleration.z,
                _angular_rate_for_output(angular_rate.x, self._format),
                _angular_rate_for_output(angular_rate.y, self._format),
                _angular_rate_for_output(angular_rate.z, self._format),
            )
        )
        self._gps_writer.writerow(
            (
                step.time_s,
                _geographic_angle_for_output(
                    state.position.latitude_rad,
                    self._format,
                ),
                _geographic_angle_for_output(
                    state.position.longitude_rad,
                    self._format,
                ),
                state.position.height_m,
                state.velocity.east_m_s,
                state.velocity.north_m_s,
                self._format.gnss_good_value,
                self._format.chassis_value,
            )
        )
        if self._debug_writer is not None:
            diagnostics = step.diagnostics
            self._debug_writer.writerow(
                (
                    step.time_s,
                    diagnostics.iteration_count,
                    diagnostics.residual_norm,
                    diagnostics.converged,
                )
            )

    def close(self) -> None:
        self._imu_file.close()
        self._gps_file.close()
        if self._debug_file is not None:
            self._debug_file.close()

    def __enter__(self) -> CsvOutputWriter:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def _required(row: dict[str, str | None], column: str) -> str:
    value = row[column]
    if value is None or not value.strip():
        raise ValueError(f"{column} must not be empty")
    return value


def _degrees_to_radians(value: str) -> float:
    from math import radians

    return radians(float(value))


def _angular_rate_for_output(value_rad_s: float, format: CsvOutputFormat) -> float:
    return degrees(value_rad_s) if format.angular_rate_unit == "deg/s" else value_rad_s


def _geographic_angle_for_output(value_rad: float, format: CsvOutputFormat) -> float:
    return degrees(value_rad) if format.geographic_angle_unit == "deg" else value_rad


def _open_debug_file(path: str | Path) -> TextIO:
    debug_path = Path(path)
    debug_path.parent.mkdir(parents=True, exist_ok=True)
    return debug_path.open("w", encoding="utf-8", newline="")
