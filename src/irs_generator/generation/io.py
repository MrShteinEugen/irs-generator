"""Streaming CSV adapters for prepared trajectories and generated outputs."""

from __future__ import annotations

import csv
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import TextIO

import numpy as np

from irs_generator.earth_model import GeodeticPosition
from irs_generator.utils.math import Scalar

from .conventions import InputConvention
from .formats import CsvOutputFormat, DatOutputFormat
from .models import GeneratedStep, TrajectoryPoint

__all__ = ["CsvOutputWriter", "CsvTrajectoryReader", "CsvTrajectorySchema"]


@dataclass(frozen=True, slots=True)
class CsvTrajectorySchema:
    """Column mapping and validation policy for prepared trajectory CSV.

    Parameters
    ----------
    time_s
        Timestamp column name.
    latitude_deg, longitude_deg, height_m
        Position column names. Angles are read in degrees.
    pitch_rad, roll_rad, heading_rad
        Attitude column names. Units and frame conversion are configured through
        :class:`InputConvention` on ``CsvTrajectoryReader``.
    velocity_east_m_s, velocity_north_m_s, velocity_up_m_s
        Velocity column names in metres per second. Their order and signs are
        converted by ``input_convention``.
    retain_position_after_initial
        Keep position in every yielded point when ``True``. When ``False``,
        only the first point carries position.
    require_uniform_time_step
        Validate that all time steps match the first step when ``True``.
    time_step_absolute_tolerance_s, time_step_relative_tolerance
        Absolute and relative tolerances used for uniform time-step validation.
    """

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
    retain_position_after_initial: bool = False
    require_uniform_time_step: bool = True
    time_step_absolute_tolerance_s: Scalar = 1e-9
    time_step_relative_tolerance: Scalar = 1e-6

    def __post_init__(self) -> None:
        for column in self.required_columns:
            if not isinstance(column, str) or not column.strip():
                raise ValueError("trajectory column names must be non-empty strings")
        for name in ("time_step_absolute_tolerance_s", "time_step_relative_tolerance"):
            value = np.longdouble(getattr(self, name))
            if not bool(np.isfinite(value)) or value < 0.0:
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
    """Stream prepared target points from a CSV file.

    Parameters
    ----------
    path
        Path to the input CSV file.
    schema
        Optional column mapping and validation policy.
    input_convention
        Convention used to convert velocity and attitude values to the canonical
        project format. Defaults to the canonical ENU/body convention.
    """

    def __init__(
        self,
        path: str | Path,
        *,
        schema: CsvTrajectorySchema | None = None,
        input_convention: InputConvention | None = None,
    ) -> None:
        self._path = Path(path)
        self._schema = schema if schema is not None else CsvTrajectorySchema()
        self._input_convention = (
            input_convention
            if input_convention is not None
            else InputConvention.canonical()
        )
        if not isinstance(self._input_convention, InputConvention):
            raise TypeError("input_convention must be an InputConvention")

    def __iter__(self) -> Iterator[TrajectoryPoint]:
        return self.iter_points()

    def iter_points(self) -> Iterator[TrajectoryPoint]:
        """Yield CSV rows as canonical target trajectory points."""

        with self._path.open("r", encoding="utf-8", newline="") as file:
            reader = csv.DictReader(file)
            if reader.fieldnames is None:
                raise ValueError("trajectory CSV must have a header row")
            missing = set(self._schema.required_columns) - set(reader.fieldnames)
            if missing:
                names = ", ".join(sorted(missing))
                raise ValueError(f"trajectory CSV is missing columns: {names}")

            previous_time_s: np.longdouble | None = None
            reference_time_step_s: np.longdouble | None = None
            for row_number, row in enumerate(reader, start=2):
                point = self._parse_row(
                    row,
                    row_number,
                    require_position=(
                        previous_time_s is None
                        or self._schema.retain_position_after_initial
                    ),
                )
                if previous_time_s is not None and point.time_s <= previous_time_s:
                    raise ValueError(
                        "time_s must be strictly increasing; "
                        f"row {row_number} is invalid"
                    )
                if previous_time_s is not None:
                    time_step_s = np.longdouble(point.time_s - previous_time_s)
                    if reference_time_step_s is None:
                        reference_time_step_s = time_step_s
                    elif self._schema.require_uniform_time_step and not bool(
                        np.isclose(
                            time_step_s,
                            reference_time_step_s,
                            rtol=self._schema.time_step_relative_tolerance,
                            atol=self._schema.time_step_absolute_tolerance_s,
                        )
                    ):
                        raise ValueError(
                            "time grid must be uniform; "
                            f"row {row_number} has dt={time_step_s:.12g}s, "
                            f"expected {reference_time_step_s:.12g}s"
                        )
                previous_time_s = np.longdouble(point.time_s)
                yield point

    def _parse_row(
        self,
        row: dict[str, str | None],
        row_number: int,
        *,
        require_position: bool,
    ) -> TrajectoryPoint:
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
                    height_m=np.longdouble(_required(row, schema.height_m)),
                )
                if require_position
                else None
            )
            return TrajectoryPoint(
                time_s=np.longdouble(_required(row, schema.time_s)),
                position=position,
                velocity=self._input_convention.convert_navigation_velocity(
                    (
                        np.longdouble(_required(row, schema.velocity_east_m_s)),
                        np.longdouble(_required(row, schema.velocity_north_m_s)),
                        np.longdouble(_required(row, schema.velocity_up_m_s)),
                    )
                ),
                attitude=self._input_convention.convert_attitude(
                    np.longdouble(_required(row, schema.pitch_rad)),
                    np.longdouble(_required(row, schema.roll_rad)),
                    np.longdouble(_required(row, schema.heading_rad)),
                ),
            )
        except (TypeError, ValueError) as error:
            raise ValueError(f"invalid trajectory value at row {row_number}") from error


class CsvOutputWriter:
    """Write generated IMU, GNSS and optional diagnostic rows.

    Parameters
    ----------
    output_dir
        Directory where output files are created.
    format
        Output file names, units, headers and delimiter. Defaults to
        :class:`DatOutputFormat`.
    debug_path
        Optional CSV path for solver diagnostics.
    """

    def __init__(
        self,
        output_dir: str | Path,
        *,
        format: CsvOutputFormat | None = None,
        debug_path: str | Path | None = None,
    ) -> None:
        self._format = format if format is not None else DatOutputFormat()
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
        """Write one generated row to all configured streams.

        Parameters
        ----------
        step
            Generated sample, navigation state and diagnostics.
        """

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
        """Close all open output files."""

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


def _degrees_to_radians(value: str) -> np.longdouble:
    return np.longdouble(np.deg2rad(np.longdouble(value)))


def _angular_rate_for_output(
    value_rad_s: Scalar,
    format: CsvOutputFormat,
) -> Scalar:
    return (
        np.rad2deg(np.longdouble(value_rad_s))
        if format.angular_rate_unit == "deg/s"
        else value_rad_s
    )


def _geographic_angle_for_output(value_rad: Scalar, format: CsvOutputFormat) -> Scalar:
    return (
        np.rad2deg(np.longdouble(value_rad))
        if format.geographic_angle_unit == "deg"
        else value_rad
    )


def _open_debug_file(path: str | Path) -> TextIO:
    debug_path = Path(path)
    debug_path.parent.mkdir(parents=True, exist_ok=True)
    return debug_path.open("w", encoding="utf-8", newline="")
