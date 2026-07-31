import csv
from pathlib import Path

import pytest

from irs_generator.earth_model import GeodeticPosition
from irs_generator.generation import (
    CsvOutputFormat,
    CsvOutputWriter,
    CsvTrajectoryReader,
    CsvTrajectorySchema,
    GeneratedStep,
    GenerationDiagnostics,
)
from irs_generator.irs_model import ImuSample
from irs_generator.navigation_model import (
    EulerAngles,
    NavigationState,
    NavigationVelocity,
)


def test_reader_requires_position_only_in_the_first_data_row(tmp_path: Path) -> None:
    path = tmp_path / "trajectory.csv"
    path.write_text(
        "t_meas_s,lat_deg,lon_deg,alt_m,pitch_rad,roll_rad,heading_rad,v_e_mps,v_n_mps,v_u_mps\n"
        "0,55,37,100,0,0,0,1,2,3\n"
        "1,,,,0.1,0.2,0.3,4,5,6\n",
        encoding="utf-8",
    )

    points = list(CsvTrajectoryReader(path))

    assert len(points) == 2
    assert points[0].position is not None
    assert points[1].position is None
    assert points[1].velocity.east_m_s == 4.0


def test_dat_writer_streams_imu_gps_and_debug_rows(tmp_path: Path) -> None:
    state = NavigationState(
        velocity=NavigationVelocity(1.0, 2.0, 3.0),
        position=GeodeticPosition(0.5, 0.25, 100.0),
        attitude=EulerAngles(0.0, 0.0, 0.0),
    )
    step = GeneratedStep(
        time_s=2.0,
        imu_sample=ImuSample.from_components(1.0, 2.0, 3.0, 1.0, 0.0, 0.0),
        navigation_state=state,
        diagnostics=GenerationDiagnostics(4, 1e-9, True),
    )

    with CsvOutputWriter(tmp_path, debug_path=tmp_path / "debug.csv") as writer:
        writer.write(step)

    with (tmp_path / "imu.dat").open(newline="", encoding="utf-8") as file:
        imu_rows = list(csv.reader(file, delimiter=" "))
    with (tmp_path / "gps.dat").open(newline="", encoding="utf-8") as file:
        gps_rows = list(csv.reader(file, delimiter=" "))

    assert imu_rows[0] == ["Time", "Ax", "Ay", "Az", "Wx", "Wy", "Wz"]
    assert imu_rows[1][0:4] == ["2.0", "1.0", "2.0", "3.0"]
    assert gps_rows[0] == ["T", "Lat", "Lon", "Hsns", "Ve", "Vn", "SNS_GOOD", "Shassi"]
    assert gps_rows[1][0] == "2.0"
    assert (tmp_path / "debug.csv").exists()


def test_writer_applies_custom_csv_profile_without_changing_internal_units(
    tmp_path: Path,
) -> None:
    state = NavigationState(
        velocity=NavigationVelocity(1.0, 2.0, 3.0),
        position=GeodeticPosition(0.5, 0.25, 100.0),
        attitude=EulerAngles(0.0, 0.0, 0.0),
    )
    step = GeneratedStep(
        time_s=2.0,
        imu_sample=ImuSample.from_components(1.0, 2.0, 3.0, 1.0, 0.0, 0.0),
        navigation_state=state,
        diagnostics=GenerationDiagnostics(1, 0.0, True),
    )
    profile = CsvOutputFormat(
        delimiter=";",
        imu_columns=("t", "ax", "ay", "az", "wx", "wy", "wz"),
        gps_columns=("t", "lat", "lon", "h", "ve", "vn", "ok", "gear"),
    )

    with CsvOutputWriter(tmp_path, format=profile) as writer:
        writer.write(step)

    with (tmp_path / "imu.csv").open(newline="", encoding="utf-8") as file:
        imu_rows = list(csv.reader(file, delimiter=";"))
    with (tmp_path / "gps.csv").open(newline="", encoding="utf-8") as file:
        gps_rows = list(csv.reader(file, delimiter=";"))

    assert imu_rows[0] == ["t", "ax", "ay", "az", "wx", "wy", "wz"]
    assert imu_rows[1][4] == "1.0"
    assert gps_rows[0] == ["t", "lat", "lon", "h", "ve", "vn", "ok", "gear"]
    assert gps_rows[1][1:3] == ["0.25", "0.5"]


def test_reader_rejects_non_uniform_time_grid_by_default(tmp_path: Path) -> None:
    path = tmp_path / "trajectory.csv"
    path.write_text(
        "t_meas_s,lat_deg,lon_deg,alt_m,pitch_rad,roll_rad,heading_rad,v_e_mps,v_n_mps,v_u_mps\n"
        "0,55,37,100,0,0,0,1,2,3\n"
        "1,,,,0,0,0,1,2,3\n"
        "2.1,,,,0,0,0,1,2,3\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="time grid must be uniform"):
        list(CsvTrajectoryReader(path))


def test_reader_can_explicitly_accept_a_non_uniform_time_grid(tmp_path: Path) -> None:
    path = tmp_path / "trajectory.csv"
    path.write_text(
        "t_meas_s,lat_deg,lon_deg,alt_m,pitch_rad,roll_rad,heading_rad,v_e_mps,v_n_mps,v_u_mps\n"
        "0,55,37,100,0,0,0,1,2,3\n"
        "1,,,,0,0,0,1,2,3\n"
        "2.1,,,,0,0,0,1,2,3\n",
        encoding="utf-8",
    )

    points = list(
        CsvTrajectoryReader(
            path,
            schema=CsvTrajectorySchema(require_uniform_time_step=False),
        )
    )

    assert [point.time_s for point in points] == [0.0, 1.0, 2.1]
