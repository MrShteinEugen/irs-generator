# IRS Generator Documentation

`irs-generator` is a Python library for synthesizing ideal IMU and GNSS measurements from a trajectory.
The library includes mathematical Earth and gravity models, as well as data generators that use
a navigation algorithm.

Mathematically, the generator solves the inverse problem of the navigation algorithm. Its input
consists of navigation parameters used to determine gyroscope and accelerometer readings.
Accelerometer data synthesis is straightforward. Gyroscope data synthesis uses a novel iterative
approach to generate ideal synthetic readings.

The main use case is preparing input data for the development, testing, and debugging of
inertial navigation algorithms.

---
## Getting Started

### Installation

To add the dependency with `pip`:

```bash
pip install irs-generator
```

To add the dependency with `uv`:

```bash
uv add irs-generator
```

To work with the source code:

```bash
uv sync --extra test --extra lint
```

### Full-Flight Example

The repository includes a prepared file containing an aircraft flight trajectory.
Running the full data-generation example creates `imu.dat` and `gps.dat`:

```bash
python examples/full_flight/generate.py
```

To compare the result with the reference files, run:

```bash
python examples/full_flight/generate.py --check
```

For details, see the [full-flight example](../examples/full_flight/README.md).

---
## User Guide

- [Synthetic Data Generation](generation.md) describes input trajectory preparation, generator selection,
  and writing results to CSV or DAT files. Use this section to generate IMU and GNSS data for a trajectory.
- [Coordinate Systems, Attitude and Navigation Parameters](conventions.md) defines the ENU coordinate system,
  the body frame, attitude parameters, DCM, and units of measurement.

---
## Main Features

- WGS 84 and GRS 80 Earth models, plus a configurable spherical model.
- Constant, inverse-square, and Somigliana normal gravity models.
- DCM-based strapdown INS implementation.
- Streaming trajectory input and IMU/GNSS data output.
- General-purpose generator based on the INS algorithm interface.
- Specialized DCM generator for prepared canonical trajectories.
- GNSS-aiding corrections for direct `DcmStrapdownINS` propagation.
- CSV and DAT output profiles (`imu.dat` / `gps.dat`).

---
## API Reference

The [public API reference](api-reference.md) describes the available modules, classes,
functions, and protocols. Public names are exported by the packages of their respective layers.
It also documents GNSS-aiding corrections available to `DcmStrapdownINS`.

Main namespaces:

```python
from irs_generator.earth_model import GeodeticPosition, WGS84EarthModel
from irs_generator.generation import DcmTrajectoryGenerator, DcmTrajectoryReader
from irs_generator.irs_model import DcmStrapdownINS, ImuSample
from irs_generator.navigation_model import NavigationState
```

---
## Conventions

Internal calculations use SI units:

| Quantity          | Unit       |
|-------------------|------------|
| Time              | `s`        |
| Geodetic position | `rad`, `m` |
| Velocity          | `m/s`      |
| Specific force    | `m/s²`     |
| Angular rate      | `rad/s`    |

The navigation coordinate system is ENU (`East`, `North`, `Up`).
The body-frame axes are defined as follows: `x` points right, `y` points forward,
and `z` points up.

---
## Current Limitations

- Only a DCM-based INS algorithm is implemented.

---
## Development

Architectural decisions are documented in [ADR](adr/README.md).
The file and dependency placement policy is documented in the
[repository policy](repository-policy.md).
---
