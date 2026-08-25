"""Integration checks for the bundled full-flight example data."""

from __future__ import annotations

import csv
from itertools import islice
from pathlib import Path

import numpy as np
import pytest

from irs_generator.earth_model import GeodeticPosition
from irs_generator.generation import (
    DcmTrajectoryGenerator,
    DcmTrajectoryPoint,
    DcmTrajectoryReader,
)
from irs_generator.navigation_model import EulerAngles, NavigationVelocity

EXAMPLE_DIRECTORY = Path(__file__).parents[2] / "examples" / "full_flight"
INPUT_PATH = EXAMPLE_DIRECTORY / "input" / "prepared_trajectory.csv"
EXPECTED_IMU_PATH = EXAMPLE_DIRECTORY / "reference" / "expected_imu.dat"
EXPECTED_GNSS_PATH = EXAMPLE_DIRECTORY / "reference" / "expected_gnss.dat"


@pytest.mark.parametrize("time_step_s", (float("nan"), float("inf"), 0.0, -1.0))
def test_dcm_generator_rejects_non_finite_or_non_positive_time_step(
    time_step_s: float,
) -> None:
    with pytest.raises(ValueError, match="finite and positive"):
        DcmTrajectoryGenerator(time_step_s=time_step_s)


def test_dcm_generator_rejects_a_timestamp_interval_that_differs_from_its_step(
) -> None:
    points = (
        DcmTrajectoryPoint(
            time_s=0.0,
            position=GeodeticPosition(0.5, 0.25, 100.0),
            velocity=NavigationVelocity(0.0, 0.0, 0.0),
            attitude=EulerAngles(0.0, 0.0, 0.0),
        ),
        DcmTrajectoryPoint(
            time_s=2.0,
            position=GeodeticPosition(0.5, 0.25, 100.0),
            velocity=NavigationVelocity(0.0, 0.0, 0.0),
            attitude=EulerAngles(0.0, 0.0, 0.0),
        ),
    )

    with pytest.raises(ValueError, match="must match time_step_s"):
        list(DcmTrajectoryGenerator(time_step_s=1.0).generate(points))


def test_full_flight_assets_share_the_expected_dat_shape() -> None:
    with INPUT_PATH.open(encoding="utf-8", newline="") as input_file:
        input_reader = csv.reader(input_file)
        input_header = next(input_reader)
        input_count = sum(1 for _ in input_reader)
    with EXPECTED_IMU_PATH.open(encoding="utf-8", newline="") as imu_file:
        imu_reader = csv.reader(imu_file, delimiter=" ")
        imu_header = next(imu_reader)
        imu_count = sum(1 for _ in imu_reader)
    with EXPECTED_GNSS_PATH.open(encoding="utf-8", newline="") as gnss_file:
        gnss_reader = csv.reader(gnss_file, delimiter=" ")
        gnss_header = next(gnss_reader)
        gnss_count = sum(1 for _ in gnss_reader)

    assert input_header == [
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
    ]
    assert imu_header == ["Time", "Ax", "Ay", "Az", "Wx", "Wy", "Wz"]
    assert gnss_header == [
        "T",
        "Lat",
        "Lon",
        "Hsns",
        "Ve",
        "Vn",
        "SNS_GOOD",
        "Shassi",
    ]
    assert input_count == imu_count == gnss_count == 45_997


def test_generator_reads_and_writes_a_full_flight_prefix(tmp_path: Path) -> None:
    reader = DcmTrajectoryReader(INPUT_PATH)
    points = list(islice(reader, 8))
    generator = DcmTrajectoryGenerator(time_step_s=reader.time_step_s())
    generator.write(points, tmp_path)

    with (tmp_path / "imu.dat").open(encoding="utf-8", newline="") as file:
        imu_rows = list(csv.reader(file, delimiter=" "))
    with (tmp_path / "gps.dat").open(encoding="utf-8", newline="") as file:
        gnss_rows = list(csv.reader(file, delimiter=" "))

    assert len(imu_rows) == len(points) + 1
    assert len(gnss_rows) == len(points) + 1
    assert imu_rows[0] == ["Time", "Ax", "Ay", "Az", "Wx", "Wy", "Wz"]
    assert gnss_rows[0] == [
        "T",
        "Lat",
        "Lon",
        "Hsns",
        "Ve",
        "Vn",
        "SNS_GOOD",
        "Shassi",
    ]

    with EXPECTED_IMU_PATH.open(encoding="utf-8", newline="") as file:
        expected_imu_rows = list(
            islice(csv.reader(file, delimiter=" "), 1, len(points) + 1)
        )
    with EXPECTED_GNSS_PATH.open(encoding="utf-8", newline="") as file:
        expected_gnss_rows = list(
            islice(csv.reader(file, delimiter=" "), 1, len(points) + 1)
        )

    for generated_row, expected_row in zip(
        imu_rows[1:],
        expected_imu_rows,
        strict=True,
    ):
        assert float(generated_row[0]) == pytest.approx(float(expected_row[0]))
        assert [float(value) for value in generated_row[1:]] == pytest.approx(
            [float(value) for value in expected_row[1:]],
            abs=2e-14,
        )
    for generated_row, expected_row in zip(
        gnss_rows[1:],
        expected_gnss_rows,
        strict=True,
    ):
        assert [float(value) for value in generated_row[:6]] == pytest.approx(
            [float(value) for value in expected_row[:6]],
            abs=2e-14,
        )
        assert generated_row[6:] == expected_row[6:]


def test_dcm_generator_matches_the_full_reference_to_float64_ulp(
    tmp_path: Path,
) -> None:
    reader = DcmTrajectoryReader(INPUT_PATH)
    DcmTrajectoryGenerator(time_step_s=reader.time_step_s()).write(reader, tmp_path)

    for output_name, expected_path in (
        ("imu.dat", EXPECTED_IMU_PATH),
        ("gps.dat", EXPECTED_GNSS_PATH),
    ):
        with (tmp_path / output_name).open(encoding="utf-8", newline="") as file:
            generated_rows = list(csv.reader(file, delimiter=" "))
        with expected_path.open(encoding="utf-8", newline="") as file:
            expected_rows = list(csv.reader(file, delimiter=" "))

        assert len(generated_rows) == len(expected_rows)
        assert generated_rows[0] == expected_rows[0]
        generated_values = np.asarray(generated_rows[1:], dtype=np.float64)
        expected_values = np.asarray(expected_rows[1:], dtype=np.float64)
        np.testing.assert_array_max_ulp(generated_values, expected_values, maxulp=4)
