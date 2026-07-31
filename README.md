# IRS Generator

`irs-generator` is a Python library for generating synthetic IMU and
GNSS measurements from reference trajectories.

The library provides several Earth models, implementations of strapdown
inertial navigation systems (SINS), and a trajectory generator designed
for the development, testing, and debugging of inertial navigation
algorithms.

## Features

- Earth models (WGS 84, GRS 80, configurable spherical model)
- Multiple gravity models: constant gravity, inverse-square gravity, and Somigliana normal gravity
- Strapdown INS implementation based on DCM
- Streaming IMU and GNSS data generation
- Extensible navigation algorithm interface
- Configurable CSV output

## Installation

### Install from PyPI

Install the latest stable release.

**pip**

```bash
pip install irs-generator
```

**uv**

```bash
uv add irs-generator
```

### Install for development

Clone the repository and install all development dependencies:

```bash
uv sync --extra test --extra lint
```

The `test` extra installs the packages required to run the test suite,
while the `lint` extra installs the tools used for formatting and static
analysis.

## Quick Start

```python
generator = SyntheticDataGenerator(
    DcmStrapdownINS(initial_state)
)

with CsvOutputWriter("output") as writer:
    for sample in generator.generate(points):
        writer.write(sample)
```

See the API documentation for complete examples.

## Full Flight Example

`examples/full_flight` contains a prepared trajectory, reference IMU and
GNSS data files, and an executable example.

```bash
python examples/full_flight/generate.py
```

## Architecture

```text
irs_generator/
├── earth_model/
│   ├── coordinates.py
│   ├── geometry.py
│   ├── gravity.py
│   ├── models.py
│   └── rotation.py
├── navigation_model/
│   ├── navigation.py
│   └── orientation.py
├── gps_model/
│   └── gnss.py
├── irs_model/
│   ├── algorithm.py
│   ├── imu.py
│   ├── error_model.py
│   ├── mechanization.py
│   ├── system.py
│   └── rotation.py
├── generation/
│   ├── models.py
│   ├── generator.py
│   ├── solver.py
│   ├── dcm.py
│   ├── io.py
│   └── formats.py
└── utils/
    ├── math.py
    └── _validation.py
```

- `earth_model` — Earth geometry and gravity models.
- `navigation_model` — Navigation state representation.
- `gps_model` — GNSS data model.
- `irs_model` — INS implementations and IMU models.
- `generation` — Synthetic data generation.
- `utils` — Shared utilities.

## Coordinate System

The navigation frame is ENU (East, North, Up).

Body-frame axes:

- x — right
- y — forward
- z — up

## Units

All internal calculations use SI units.

| Quantity | Unit |
|----------|------|
| Time | s |
| Position | rad, m |
| Velocity | m/s |
| Specific force | m/s² |
| Angular rate | rad/s |

## Data Formats

The library supports configurable CSV formats and the legacy
`imu.dat` / `gps.dat` format used by MFS24.

## Extending

New INS implementations can be added by implementing the
`InertialNavigationAlgorithm` interface.

## Limitations

- Only the DCM implementation is currently available.
- No GNSS correction during generation.
- No command-line interface.

## Development

```bash
ruff check .
mypy src tests
pytest
```

## Distribution Audit

Before publishing a release, verify that the generated distribution
artifacts do not contain unintended repository files.

Build the distributions:

```bash
python -m build
```

Run the audit:

```bash
python tools/audit_distribution.py
```

The audit checks both the wheel (`.whl`) and source distribution
(`.tar.gz`) and fails if unexpected files are included. This helps
prevent accidental publication of development artifacts.

## License

See `LICENSE`.
