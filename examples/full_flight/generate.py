"""Generate ideal DAT data for the bundled full-flight trajectory."""

from __future__ import annotations

import argparse
from collections.abc import Iterable
from itertools import islice
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from irs_generator.generation import (
    DcmTrajectoryGenerator,
    DcmTrajectoryPoint,
    DcmTrajectoryReader,
)

EXAMPLE_DIRECTORY = Path(__file__).parent
DEFAULT_INPUT_PATH = EXAMPLE_DIRECTORY / "input" / "prepared_trajectory.csv"
DEFAULT_OUTPUT_DIRECTORY = EXAMPLE_DIRECTORY / "output"
REFERENCE_DIRECTORY = EXAMPLE_DIRECTORY / "reference"


def main() -> None:
    """Synthesize IMU and GNSS rows without loading the full trajectory."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT_PATH)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIRECTORY)
    parser.add_argument(
        "--check",
        action="store_true",
        help="Verify the generated full flight against the committed reference.",
    )
    parser.add_argument(
        "--points",
        type=int,
        default=None,
        help="Generate at most this many target points; omit for the full flight.",
    )
    arguments = parser.parse_args()
    if arguments.points is not None and arguments.points < 2:
        parser.error("--points must be at least 2")

    reader = DcmTrajectoryReader(arguments.input)
    points: Iterable[DcmTrajectoryPoint] = reader
    if arguments.points is not None:
        points = islice(points, arguments.points)

    time_step_s = float(reader.time_step_s())
    DcmTrajectoryGenerator(time_step_s=time_step_s).write(
        points,
        arguments.output_dir,
    )
    if arguments.check:
        _verify_against_reference(arguments.output_dir, time_step_s)


def _assert_reference_values(
    generated: NDArray[np.float64],
    reference: NDArray[np.float64],
    filename: str,
    time_step_s: float,
) -> None:
    """Check DAT values using per-quantity float64 regression budgets.

    The gyro budget allows 32 epsilon of matrix round-off per iteration
    for each of two runs, accumulated over 12 iterations and divided by dt.
    This is a conservative regression allowance, not a physical error bound.
    NaNs, infinities, shape changes and GNSS flag changes always fail.
    """

    columns = {"imu.dat": 7, "gps.dat": 8}[filename]
    if generated.shape != reference.shape or generated.ndim != 2:
        raise AssertionError(f"{filename}: incompatible shapes")
    if generated.shape[1] != columns or generated.shape[0] == 0:
        raise AssertionError(f"{filename}: invalid DAT shape")
    if not np.isfinite(generated).all() or not np.isfinite(reference).all():
        raise AssertionError(f"{filename}: non-finite values")
    if not np.isfinite(time_step_s) or time_step_s <= 0:
        raise ValueError("time_step_s must be finite and positive")
    eps = float(np.finfo(np.float64).eps)
    np.testing.assert_allclose(
        generated[:, 0],
        reference[:, 0],
        rtol=4 * eps,
        atol=4 * eps,
        err_msg=f"{filename}: time (s)",
    )
    if filename == "imu.dat":
        np.testing.assert_allclose(
            generated[:, 1:4],
            reference[:, 1:4],
            rtol=64 * eps,
            atol=64 * eps * 9.81,
            err_msg="specific force (m/s^2)",
        )
        gyro_atol = float(np.rad2deg(2 * 12 * 32 * eps / time_step_s))
        np.testing.assert_allclose(
            generated[:, 4:7],
            reference[:, 4:7],
            rtol=64 * eps,
            atol=gyro_atol,
            err_msg="angular rate (deg/s)",
        )
    else:
        np.testing.assert_allclose(
            generated[:, 1:6],
            reference[:, 1:6],
            rtol=8 * eps,
            atol=8 * eps,
            err_msg="GNSS position (deg, m) and velocity (m/s)",
        )
        np.testing.assert_array_equal(generated[:, 6:], reference[:, 6:])


def _verify_against_reference(output_directory: Path, time_step_s: float) -> None:
    """Assert full-flight agreement using quantity-specific error budgets."""

    maximum_errors: list[float] = []
    for filename, reference_filename in (
        ("imu.dat", "expected_imu.dat"),
        ("gps.dat", "expected_gnss.dat"),
    ):
        generated = np.loadtxt(output_directory / filename, skiprows=1)
        reference = np.loadtxt(REFERENCE_DIRECTORY / reference_filename, skiprows=1)
        if generated.shape != reference.shape:
            raise AssertionError(
                f"{filename}: shape differs: {generated.shape} != {reference.shape}"
            )
        _assert_reference_values(generated, reference, filename, time_step_s)
        maximum_errors.append(float(np.max(np.abs(generated - reference))))

    print(
        "Reference check passed: "
        f"maximum absolute error {max(maximum_errors):.3e}; "
        "all quantity-specific tolerances satisfied."
    )


if __name__ == "__main__":
    main()
