"""Integration checks for the ``irs-generator`` distribution installed from PyPI.

Run this file with the interpreter from ``pypi_validation/.venv``.  The tests
verify the installed wheel rather than the copy of the source repository.
"""

from __future__ import annotations

import csv
import importlib.metadata
import sys
import tempfile
import unittest
from pathlib import Path

from irs_generator.earth_model import GeodeticPosition
from irs_generator.generation import (
    DcmTrajectoryGenerator,
    DcmTrajectoryReader,
    SyntheticDataGenerator,
    TrajectoryPoint,
)
from irs_generator.irs_model import DcmStrapdownINS
from irs_generator.navigation_model import (
    EulerAngles,
    NavigationState,
    NavigationVelocity,
)

VALIDATION_DIRECTORY = Path(__file__).resolve().parent
WORKSPACE_DIRECTORY = VALIDATION_DIRECTORY.parent
SOURCE_PACKAGE_DIRECTORY = WORKSPACE_DIRECTORY / "src" / "irs_generator"
VENV_DIRECTORY = Path(sys.executable).resolve().parents[1]


def _is_inside(path: Path, directory: Path) -> bool:
    """Return whether ``path`` is located under ``directory``."""

    try:
        path.resolve().relative_to(directory.resolve())
    except ValueError:
        return False
    return True


def _initial_state() -> NavigationState:
    """Build a stationary initial state in the canonical ENU convention."""

    return NavigationState(
        velocity=NavigationVelocity(0.0, 0.0, 0.0),
        position=GeodeticPosition(longitude_rad=0.0, latitude_rad=0.5, height_m=100.0),
        attitude=EulerAngles(pitch_rad=0.0, roll_rad=0.0, heading_rad=0.0),
    )


class PyPiInstallationTests(unittest.TestCase):
    """End-to-end checks covering the public package interfaces."""

    def test_import_uses_the_validation_environment(self) -> None:
        """Ensure the test did not import the project's checkout by accident."""

        import irs_generator

        module_path = Path(irs_generator.__file__).resolve()
        self.assertTrue(
            _is_inside(module_path, VENV_DIRECTORY),
            f"irs_generator was imported from {module_path}, not {VENV_DIRECTORY}",
        )
        self.assertFalse(
            _is_inside(module_path, SOURCE_PACKAGE_DIRECTORY),
            f"irs_generator was imported from the source checkout: {module_path}",
        )
        self.assertTrue(importlib.metadata.version("irs-generator"))

    def test_generic_generator_produces_two_aligned_steps(self) -> None:
        """Generate a minimal stationary trajectory through the public API."""

        initial_state = _initial_state()
        points = (
            TrajectoryPoint(
                time_s=0.0,
                position=initial_state.position,
                velocity=initial_state.velocity,
                attitude=initial_state.attitude,
            ),
            TrajectoryPoint(
                time_s=0.1,
                velocity=initial_state.velocity,
                attitude=initial_state.attitude,
            ),
        )

        generated = list(
            SyntheticDataGenerator(DcmStrapdownINS(initial_state)).generate(points)
        )

        self.assertEqual(len(generated), 2)
        self.assertEqual([float(step.time_s) for step in generated], [0.0, 0.1])
        self.assertTrue(all(step.diagnostics.converged for step in generated))

    def test_dcm_generator_writes_canonical_dat_files(self) -> None:
        """Read a prepared CSV and write the IMU and GNSS DAT outputs."""

        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            trajectory_path = temporary_path / "trajectory.csv"
            output_directory = temporary_path / "output"
            trajectory_path.write_text(
                "t_meas_s,lat_deg,lon_deg,alt_m,pitch_rad,roll_rad,heading_rad,"
                "v_e_mps,v_n_mps,v_u_mps\n"
                "0.0,45.0,37.0,100.0,0.0,0.0,0.0,0.0,0.0,0.0\n"
                "0.1,45.0,37.0,100.0,0.0,0.0,0.0,0.0,0.0,0.0\n",
                encoding="utf-8",
            )

            generator = DcmTrajectoryGenerator(time_step_s=0.1)
            generator.write(DcmTrajectoryReader(trajectory_path), output_directory)

            imu_path = output_directory / "imu.dat"
            gnss_path = output_directory / "gps.dat"
            self.assertTrue(imu_path.is_file())
            self.assertTrue(gnss_path.is_file())
            with imu_path.open(encoding="utf-8", newline="") as imu_file:
                imu_rows = list(csv.reader(imu_file, delimiter=" "))
            with gnss_path.open(encoding="utf-8", newline="") as gnss_file:
                gnss_rows = list(csv.reader(gnss_file, delimiter=" "))
            self.assertEqual(
                imu_rows[0],
                ["Time", "Ax", "Ay", "Az", "Wx", "Wy", "Wz"],
            )
            self.assertEqual(
                gnss_rows[0],
                ["T", "Lat", "Lon", "Hsns", "Ve", "Vn", "SNS_GOOD", "Shassi"],
            )
            self.assertEqual(len(imu_rows), 3)
            self.assertEqual(len(gnss_rows), 3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
