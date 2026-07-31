"""Reference-grade DCM trajectory reader and synthetic-data generator."""

from __future__ import annotations

import csv
from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol, cast

import numpy as np
import pandas as pd  # type: ignore[import-untyped]

from irs_generator.earth_model import GeodeticPosition
from irs_generator.irs_model import ImuSample
from irs_generator.navigation_model import (
    EulerAngles,
    NavigationState,
    NavigationVelocity,
)
from irs_generator.utils._validation import _finite_scalar
from irs_generator.utils.math import Scalar, Vector3

from .models import GeneratedStep, GenerationDiagnostics

__all__ = [
    "DcmTrajectoryGenerator",
    "DcmTrajectoryPoint",
    "DcmTrajectoryReader",
]


@dataclass(frozen=True, slots=True)
class DcmTrajectoryPoint:
    """One prepared trajectory row for DCM synthesis.

    Parameters
    ----------
    time_s
        Timestamp in seconds.
    position
        Geodetic position at ``time_s``.
    velocity
        ENU velocity at ``time_s``.
    attitude
        Body attitude at ``time_s``.
    """

    time_s: Scalar
    position: GeodeticPosition
    velocity: NavigationVelocity
    attitude: EulerAngles

    def __post_init__(self) -> None:
        object.__setattr__(self, "time_s", _finite_scalar(self.time_s, "time_s"))

    def to_navigation_state(self) -> NavigationState:
        """Return the point as a navigation state."""

        return NavigationState(
            velocity=self.velocity,
            position=self.position,
            attitude=self.attitude,
        )


class DcmTrajectoryReader:
    """Stream DCM trajectory points from the canonical prepared CSV.

    Parameters
    ----------
    path
        Path to a CSV file with columns ``t_meas_s``, ``lat_deg``, ``lon_deg``,
        ``alt_m``, ``pitch_rad``, ``roll_rad``, ``heading_rad``, ``v_e_mps``,
        ``v_n_mps`` and ``v_u_mps``.
    """

    _required_columns = (
        "t_meas_s",
        "lat_deg",
        "lon_deg",
        "alt_m",
        "pitch_rad",
        "roll_rad",
        "heading_rad",
        "v_e_mps",
        "v_n_mps",
        "v_u_mps",
    )

    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)

    def __iter__(self) -> Iterator[DcmTrajectoryPoint]:
        chunks = pd.read_csv(self._path, chunksize=10_000)
        previous_time: np.longdouble | None = None
        row_number = 2
        for chunk in chunks:
            missing = set(self._required_columns) - set(chunk.columns)
            if missing:
                raise ValueError(
                    "trajectory CSV is missing columns: " + ", ".join(sorted(missing))
                )
            for row in chunk.loc[:, self._required_columns].itertuples(
                index=False,
                name=None,
            ):
                point = _dcm_point_from_values(row, row_number)
                if previous_time is not None and point.time_s <= previous_time:
                    raise ValueError(
                        "time_s must be strictly increasing; "
                        f"row {row_number} is invalid"
                    )
                previous_time = np.longdouble(point.time_s)
                yield point
                row_number += 1

    def time_step_s(self) -> np.longdouble:
        """Return the global median time step.

        Returns
        -------
        numpy.longdouble
            Median difference between adjacent timestamps, in seconds.

        Raises
        ------
        ValueError
            If the trajectory contains fewer than two points.
        """

        timestamps = np.fromiter((point.time_s for point in self), dtype=np.longdouble)
        if timestamps.size < 2:
            raise ValueError("target trajectory must contain at least two points")
        return np.longdouble(
            float(np.median(np.diff(timestamps).astype(np.float64)))
        )


class _RowWriter(Protocol):
    def writerow(self, row: Iterable[Any]) -> Any:
        """Write one row."""


@dataclass(frozen=True, slots=True)
class _DcmEarth:
    """Earth equations used by the DCM reference synthesis profile."""

    semi_major_axis_m: np.longdouble = np.longdouble(6_378_137.0)
    semi_minor_axis_m: np.longdouble = np.longdouble(6_356_752.314)
    rotation_rad_s: np.longdouble = np.longdouble(15.0 * np.pi / 648_000.0)

    @property
    def eccentricity_squared(self) -> np.longdouble:
        return cast(
            np.longdouble,
            np.divide(
                self.semi_major_axis_m**2 - self.semi_minor_axis_m**2,
                self.semi_major_axis_m**2,
                dtype=np.longdouble,
            ),
        )

    def meridional_radius_m(self, latitude_rad: np.longdouble) -> np.longdouble:
        sin_latitude_squared = np.longdouble(np.sin(latitude_rad)) ** 2
        return (
            self.semi_major_axis_m
            * (1.0 - self.eccentricity_squared)
            / (1.0 - self.eccentricity_squared * sin_latitude_squared) ** 1.5
        )

    def prime_vertical_radius_m(self, latitude_rad: np.longdouble) -> np.longdouble:
        sin_latitude_squared = np.longdouble(np.sin(latitude_rad)) ** 2
        return np.longdouble(
            self.semi_major_axis_m
            / np.sqrt(1.0 - self.eccentricity_squared * sin_latitude_squared)
        )

    @staticmethod
    def gravity_m_s2(
        height_m: np.longdouble,
        latitude_rad: np.longdouble,
    ) -> np.longdouble:
        sin_latitude_squared = np.longdouble(np.sin(latitude_rad)) ** 2
        sin_double_latitude_squared = np.longdouble(np.sin(2.0 * latitude_rad)) ** 2
        return np.longdouble(
            9.780318
            * (
                1.0
                + 0.005302 * sin_latitude_squared
                - 0.000006 * sin_double_latitude_squared
            )
            - 0.000003086 * height_m
        )


class DcmTrajectoryGenerator:
    """Generate ideal IMU samples from adjacent DCM trajectory points.

    Parameters
    ----------
    time_step_s
        Fixed synthesis time step in seconds.
    """

    def __init__(self, *, time_step_s: Scalar) -> None:
        self._earth = _DcmEarth()
        self._time_step_s = np.longdouble(time_step_s)
        if self._time_step_s <= 0.0:
            raise ValueError("time_step_s must be positive")

    def generate(
        self,
        points: Iterable[DcmTrajectoryPoint],
    ) -> Iterator[GeneratedStep]:
        """Generate IMU samples and aligned navigation states.

        Parameters
        ----------
        points
            Iterable of DCM trajectory points. At least two points are required.

        Yields
        ------
        GeneratedStep
            Generated IMU sample and the corresponding truth state.
        """

        iterator = iter(points)
        previous = next(iterator, None)
        current = next(iterator, None)
        if previous is None or current is None:
            raise ValueError("target trajectory must contain at least two points")
        first_imu, first_diagnostics = self._derive_sample(previous, current)
        yield _generated_step(previous, first_imu, first_diagnostics)

        while True:
            imu_sample, diagnostics = self._derive_sample(previous, current)
            yield _generated_step(current, imu_sample, diagnostics)
            previous = current
            current = next(iterator, None)
            if current is None:
                return

    def write(
        self,
        points: Iterable[DcmTrajectoryPoint],
        output_dir: str | Path,
    ) -> None:
        """Write canonical DAT output files.

        Parameters
        ----------
        points
            Iterable of DCM trajectory points.
        output_dir
            Directory where ``imu.dat`` and ``gps.dat`` are written.

        Raises
        ------
        ValueError
            If fewer than two points are provided.
        """

        directory = Path(output_dir)
        directory.mkdir(parents=True, exist_ok=True)
        iterator = iter(points)
        previous = next(iterator, None)
        current = next(iterator, None)
        if previous is None or current is None:
            raise ValueError("target trajectory must contain at least two points")
        with (
            (directory / "imu.dat").open("w", encoding="utf-8", newline="") as imu_file,
            (directory / "gps.dat").open(
                "w", encoding="utf-8", newline=""
            ) as gnss_file,
        ):
            imu_writer = csv.writer(imu_file, delimiter=" ")
            gnss_writer = csv.writer(gnss_file, delimiter=" ")
            imu_writer.writerow(("Time", "Ax", "Ay", "Az", "Wx", "Wy", "Wz"))
            gnss_writer.writerow(
                ("T", "Lat", "Lon", "Hsns", "Ve", "Vn", "SNS_GOOD", "Shassi")
            )
            raw_sample = self._derive_raw_sample(previous, current)
            _write_dat_row(imu_writer, gnss_writer, previous, raw_sample)
            while True:
                raw_sample = self._derive_raw_sample(previous, current)
                _write_dat_row(imu_writer, gnss_writer, current, raw_sample)
                previous = current
                current = next(iterator, None)
                if current is None:
                    return

    def _derive_sample(
        self,
        previous: DcmTrajectoryPoint,
        current: DcmTrajectoryPoint,
    ) -> tuple[ImuSample, GenerationDiagnostics]:
        acceleration_body, angular_rate_body, residual = self._derive_raw_sample(
            previous,
            current,
        )
        return (
            ImuSample(
                specific_force_body_m_s2=Vector3.from_iterable(acceleration_body),
                angular_rate_body_rad_s=Vector3.from_iterable(angular_rate_body),
            ),
            GenerationDiagnostics(12, residual, residual <= 1e-15),
        )

    def _derive_raw_sample(
        self,
        previous: DcmTrajectoryPoint,
        current: DcmTrajectoryPoint,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        if current.time_s <= previous.time_s:
            raise ValueError("target trajectory time must be strictly increasing")
        dt_s = self._time_step_s

        previous_velocity = _long_vector(previous)
        current_velocity = _long_vector(current)
        previous_attitude = _long_attitude(previous)
        current_attitude = _long_attitude(current)
        latitude = np.longdouble(previous.position.latitude_rad)
        height = np.longdouble(previous.position.height_m)

        previous_dcm = _dcm_from_euler(previous_attitude)
        current_dcm = _dcm_from_euler(current_attitude)
        navigation_rate = self._navigation_rate(previous_velocity, latitude, height)
        acceleration_nav = self._specific_force(
            previous_velocity,
            current_velocity,
            navigation_rate,
            latitude,
            height,
            dt_s,
        )
        acceleration_body = previous_dcm.T @ acceleration_nav
        angular_rate_body, residual = _poisson_inversion(
            previous_dcm,
            current_dcm,
            navigation_rate,
            dt_s,
        )
        return acceleration_body, angular_rate_body, residual

    def _navigation_rate(
        self,
        velocity: np.ndarray,
        latitude_rad: np.longdouble,
        height_m: np.longdouble,
    ) -> np.ndarray:
        meridional_radius = self._earth.meridional_radius_m(latitude_rad) + height_m
        prime_vertical_radius = (
            self._earth.prime_vertical_radius_m(latitude_rad) + height_m
        )
        east, north, _ = velocity
        return np.array(
            (
                -north / meridional_radius,
                east / prime_vertical_radius
                + self._earth.rotation_rad_s * np.cos(latitude_rad),
                east / prime_vertical_radius * np.tan(latitude_rad)
                + self._earth.rotation_rad_s * np.sin(latitude_rad),
            ),
            dtype=np.longdouble,
        )

    def _specific_force(
        self,
        previous_velocity: np.ndarray,
        current_velocity: np.ndarray,
        navigation_rate: np.ndarray,
        latitude_rad: np.longdouble,
        height_m: np.longdouble,
        dt_s: np.longdouble,
    ) -> np.ndarray:
        east, north, up = previous_velocity
        derivative = (current_velocity - previous_velocity) / dt_s
        rate_x, rate_y, rate_z = navigation_rate
        earth_rate = self._earth.rotation_rad_s
        sin_latitude = np.sin(latitude_rad)
        cos_latitude = np.cos(latitude_rad)
        return np.array(
            (
                derivative[0]
                - north * (rate_z + earth_rate * sin_latitude)
                + up * (earth_rate * cos_latitude + rate_y),
                derivative[1]
                + east * (rate_z + earth_rate * sin_latitude)
                + up * rate_x,
                derivative[2]
                - east * (rate_y + earth_rate * cos_latitude)
                + north * rate_x
                + self._earth.gravity_m_s2(height_m, latitude_rad),
            ),
            dtype=np.longdouble,
        )


def _dcm_from_euler(attitude: np.ndarray) -> np.ndarray:
    pitch, roll, heading = attitude
    c_pitch, s_pitch = np.cos(pitch), np.sin(pitch)
    c_roll, s_roll = np.cos(roll), np.sin(roll)
    c_heading, s_heading = np.cos(heading), np.sin(heading)
    dcm = np.array(
        (
            (
                c_roll * c_heading + s_roll * s_pitch * s_heading,
                c_pitch * s_heading,
                s_roll * c_heading - s_pitch * c_roll * s_heading,
            ),
            (
                -c_roll * s_heading + s_roll * s_pitch * c_heading,
                c_pitch * c_heading,
                -s_roll * s_heading - s_pitch * c_roll * c_heading,
            ),
            (-c_pitch * s_roll, s_pitch, c_pitch * c_roll),
        ),
        dtype=np.longdouble,
    )
    return _project_to_so3(dcm)


def _poisson_inversion(
    previous_dcm: np.ndarray,
    target_dcm: np.ndarray,
    navigation_rate: np.ndarray,
    dt_s: np.longdouble,
) -> tuple[np.ndarray, float]:
    dcm_dot = (target_dcm - previous_dcm) / dt_s
    initial_matrix = previous_dcm.T @ (dcm_dot + _skew(navigation_rate) @ previous_dcm)
    angular_rate = _vee((initial_matrix - initial_matrix.T) * np.longdouble(0.5))
    residual = float("inf")
    for _ in range(12):
        predicted_dot = (
            previous_dcm @ _skew(angular_rate) - _skew(navigation_rate) @ previous_dcm
        )
        predicted_dcm = _orthonormalize(previous_dcm + dt_s * predicted_dot)
        delta = target_dcm - predicted_dcm
        residual = float(np.max(np.abs(delta.astype(np.float64))))
        if residual < 1e-15:
            break
        update_matrix = previous_dcm.T @ (delta / dt_s)
        angular_rate = angular_rate + _vee(
            (update_matrix - update_matrix.T) * np.longdouble(0.5)
        )
    return angular_rate, residual


def _orthonormalize(dcm: np.ndarray) -> np.ndarray:
    return _project_to_so3(dcm)


def _project_to_so3(matrix: np.ndarray) -> np.ndarray:
    """Project a matrix onto the closest proper rotation matrix."""

    u, _, vt = np.linalg.svd(
        np.asarray(matrix, dtype=np.float64)
    )
    result = u @ vt
    if np.linalg.det(result) < 0.0:
        u[:, -1] *= -1.0
        result = u @ vt
    return np.asarray(result, dtype=np.longdouble)


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array(
        ((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)),
        dtype=np.longdouble,
    )


def _vee(skew_matrix: np.ndarray) -> np.ndarray:
    return np.array(
        (skew_matrix[2, 1], skew_matrix[0, 2], skew_matrix[1, 0]),
        dtype=np.longdouble,
    )


def _long_vector(point: DcmTrajectoryPoint) -> np.ndarray:
    return point.velocity.as_array()


def _long_attitude(point: DcmTrajectoryPoint) -> np.ndarray:
    return point.attitude.as_array()


def _generated_step(
    point: DcmTrajectoryPoint,
    imu_sample: ImuSample,
    diagnostics: GenerationDiagnostics,
) -> GeneratedStep:
    return GeneratedStep(
        time_s=point.time_s,
        imu_sample=imu_sample,
        navigation_state=point.to_navigation_state(),
        diagnostics=diagnostics,
    )


def _dcm_point_from_values(
    row: tuple[Any, ...],
    row_number: int,
) -> DcmTrajectoryPoint:
    def value(index: int) -> np.longdouble:
        raw_value = row[index]
        if pd.isna(raw_value):
            raise ValueError(
                f"{DcmTrajectoryReader._required_columns[index]} "
                f"must not be empty at row {row_number}"
            )
        return np.longdouble(raw_value)

    return DcmTrajectoryPoint(
        time_s=value(0),
        position=GeodeticPosition(
            latitude_rad=np.longdouble(np.deg2rad(value(1))),
            longitude_rad=np.longdouble(np.deg2rad(value(2))),
            height_m=value(3),
        ),
        attitude=EulerAngles(
            pitch_rad=value(4),
            roll_rad=value(5),
            heading_rad=value(6),
        ),
        velocity=NavigationVelocity(
            east_m_s=value(7),
            north_m_s=value(8),
            up_m_s=value(9),
        ),
    )


def _write_dat_row(
    imu_writer: _RowWriter,
    gnss_writer: _RowWriter,
    point: DcmTrajectoryPoint,
    raw_sample: tuple[np.ndarray, np.ndarray, float],
) -> None:
    acceleration, angular_rate, _ = raw_sample
    imu_writer.writerow(
        (
            point.time_s,
            acceleration[0],
            acceleration[1],
            acceleration[2],
            np.rad2deg(angular_rate[0]),
            np.rad2deg(angular_rate[1]),
            np.rad2deg(angular_rate[2]),
        )
    )
    gnss_writer.writerow(
        (
            point.time_s,
            np.rad2deg(point.position.latitude_rad),
            np.rad2deg(point.position.longitude_rad),
            point.position.height_m,
            point.velocity.east_m_s,
            point.velocity.north_m_s,
            1,
            1,
        )
    )
