"""Generate ideal DAT data for the bundled full-flight trajectory."""

from __future__ import annotations

import argparse
from itertools import islice
from pathlib import Path

import numpy as np

from irs_generator.generation import (
    DcmTrajectoryGenerator,
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
    points = reader
    if arguments.points is not None:
        points = islice(points, arguments.points)

    DcmTrajectoryGenerator(time_step_s=reader.time_step_s()).write(
        points,
        arguments.output_dir,
    )
    if arguments.check:
        _verify_against_reference(arguments.output_dir)


def _verify_against_reference(output_directory: Path) -> None:
    """Assert full-flight agreement within cross-platform float64 round-off."""

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
        np.testing.assert_array_max_ulp(generated, reference, maxulp=4)
        maximum_errors.append(float(np.max(np.abs(generated - reference))))

    print(
        "Reference check passed: "
        f"maximum absolute error {max(maximum_errors):.3e}; "
        "maximum error no more than 4 float64 ULP."
    )


if __name__ == "__main__":
    main()
