```md
# Public API Reference

This document describes the main public entities in the package. It does not replace
the docstrings in the source code; instead, it provides an overview of what to import,
what each entity is for, and which units are used.

The public status of a name is defined by ADR 0002: the name must be exported through
`__all__` and intended for user imports.

---
## `irs_generator`

| Name          | Purpose                        |
|---------------|--------------------------------|
| `__version__` | Version of the installed package. |

The root package is intentionally minimal. Main classes are imported from the
layer packages.

---
## `irs_generator.earth_model`

Earth models, geodetic coordinates, rotation, and gravity.

| Name                      | Purpose |
|---------------------------|---------|
| `GeodeticPosition`        | Geodetic position: longitude, latitude, and altitude. |
| `ReferenceEllipsoid`      | Reference ellipsoid parameters and radii of curvature. |
| `RotationParameters`      | Earth rotation angular rate. |
| `GravityModel`            | Gravity-model protocol. |
| `SomiglianaNormalGravity` | Normal gravity on an ellipsoid. |
| `InverseSquareGravity`    | Central inverse-square gravity model. |
| `ConstantGravity`         | Constant gravity. |
| `EarthModel`              | Earth-model protocol. |
| `EllipsoidalEarthModel`   | Earth model based on an ellipsoid, rotation, and gravity. |
| `WGS84EarthModel`         | Ready-to-use WGS 84 Earth model. |
| `GRS80EarthModel`         | Ready-to-use GRS 80 Earth model. |
| `SphericalEarthModel`     | Configurable spherical Earth model. |

### `GeodeticPosition`

Fields:

| Field           | Unit  | Description |
|-----------------|-------|-------------|
| `longitude_rad` | `rad` | Geodetic longitude. |
| `latitude_rad`  | `rad` | Geodetic latitude. |
| `height_m`      | `m`   | Altitude above the selected Earth model. |

Latitude is validated in the range `[-pi / 2, pi / 2]`.

### `ReferenceEllipsoid`

Defined by the semi-major axis and inverse flattening. Provides calculations for
radii of curvature and surface radii.

Main methods:

- `mean_radius_m`;
- `volumetric_radius_m`;
- `meridional_radius_m(latitude_rad)`;
- `prime_vertical_radius_m(latitude_rad)`;
- `geocentric_surface_radius_m(latitude_rad)`.

---
## `irs_generator.navigation_model`

Navigation state and attitude.

| Name                 | Purpose |
|----------------------|---------|
| `NavigationVelocity` | Velocity in the local ENU frame. |
| `EulerAngles`        | Project aviation attitude angles. |
| `NavigationState`    | Complete state: velocity, position, and attitude. |

### `NavigationVelocity`

Fields:

| Field       | Unit  | Axis |
|-------------|-------|------|
| `east_m_s`  | `m/s` | East |
| `north_m_s` | `m/s` | North |
| `up_m_s`    | `m/s` | Up |

### `EulerAngles`

Fields:

| Field         | Unit  | Positive direction |
|---------------|-------|--------------------|
| `pitch_rad`   | `rad` | Nose up. |
| `roll_rad`    | `rad` | Right wing down. |
| `heading_rad` | `rad` | Clockwise from north. |

### `NavigationState`

Combines:

- `velocity: NavigationVelocity`;
- `position: GeodeticPosition`;
- `attitude: EulerAngles`.

Used as the INS algorithm state and as part of the generation result.

---
## `irs_generator.gps_model`

| Name         | Purpose |
|--------------|---------|
| `GnssSample` | GNSS measurement: position and ENU velocity. |

`GnssSample` is used as an external navigation sample. In ideal data-generation
scenarios, it represents the ground-truth trajectory rather than a correction.

---
## `irs_generator.irs_model`

Inertial system: IMU, INS algorithms, errors, and IRS.

| Name                          | Purpose |
|-------------------------------|---------|
| `ImuSample`                   | IMU measurement: specific force and angular rate in the body frame. |
| `NavigationAlgorithm`         | Minimal navigation-algorithm protocol. |
| `InertialNavigationAlgorithm` | INS-algorithm protocol with `fork()`. |
| `DcmStrapdownINS`             | DCM strapdown INS mechanization. |
| `MechanizationConfig`         | DCM mechanization configuration. |
| `InertialReferenceSystem`     | IRS composed of an INS algorithm and an IMU error model. |
| `ImuErrorModel`               | IMU error-model protocol. |
| `IdealImuErrorModel`          | Error-free model. |
| `BiasImuErrorModel`           | Constant accelerometer and gyroscope bias. |
| `CompositeImuErrorModel`      | Sequential composition of multiple error models. |
| `AnalyticAlignment`           | Analytical initial alignment from an IMU measurement. |

### `ImuSample`

Fields:

| Field                      | Unit    | Frame |
|----------------------------|---------|-------|
| `specific_force_body_m_s2` | `m/s²`  | Body frame |
| `angular_rate_body_rad_s`  | `rad/s` | Body frame |

Body-frame component order:

```text
(x_body, y_body, z_body) = (right, forward, up)
```

### `NavigationAlgorithm`

Minimal protocol:

- `state`;
- `reset(initial_state)`;
- `step(imu_sample, dt_s, gnss_sample=None)`.

Used in a standard IRS scenario.

### `InertialNavigationAlgorithm`

Extends `NavigationAlgorithm` with:

- `fork()`.

The generator uses this method for trial numerical steps.

---
## `irs_generator.generation`

Streaming IMU/GNSS data synthesis and file input/output.

| Name                     | Purpose |
|--------------------------|---------|
| `TargetTrajectoryPoint`  | A single target trajectory point for the generator. |
| `Axis`                   | Cartesian component identifier: `X`, `Y`, or `Z`. |
| `AngleUnit`              | Unit of external attitude angles: radians or degrees. |
| `SignedAxis`             | Source axis and sign for one output component. |
| `SignedAxisMapping`      | Signed axis permutation with handedness validation. |
| `Handedness`             | Whether a mapping preserves or reverses orientation. |
| `InputConvention`        | Conversion from external vectors and angles to project conventions. |
| `GeneratedStep`          | A single generation output step. |
| `GenerationDiagnostics`  | Generation-step diagnostics. |
| `GenerationConfig`       | General-purpose generator configuration. |
| `StepSolverConfig`       | Inverse IMU solver settings. |
| `SyntheticDataGenerator` | General-purpose generator based on the INS interface. |
| `DcmTrajectoryPoint`     | Point of a canonical DCM trajectory. |
| `DcmTrajectoryReader`    | Reader for a canonical prepared DCM trajectory. |
| `DcmTrajectoryGenerator` | Exact DCM generator. |
| `CsvTrajectorySchema`    | Definition of input CSV trajectory column names. |
| `CsvTrajectoryReader`    | Streaming reader for an input CSV trajectory. |
| `CsvOutputWriter`        | Writer for IMU/GNSS results. |
| `CsvOutputFormat`        | Configurable CSV output profile. |
| `DatOutputFormat`        | Output profile for `imu.dat` and `gps.dat`. |

### `TargetTrajectoryPoint`

Describes the target state at a time instant:

- `time_s`;
- `velocity`;
- `attitude`;
- `position`.

The first point must contain a position. For subsequent points, the INS algorithm
may reconstruct the position.

### `GeneratedStep`

Describes the result of one generation step:

- time;
- synthetic IMU measurement;
- GNSS sample;
- navigation state;
- diagnostics.

### `SyntheticDataGenerator`

Works with any algorithm that implements `InertialNavigationAlgorithm`.

Uses a numerical inverse algorithm to select an IMU sample that brings the
algorithm to the target state.

### `DcmTrajectoryGenerator`

Specialized exact generator for canonical DCM trajectories. Uses adjacent trajectory
points and DCM kinematics to calculate the ideal IMU measurement directly.

---
## Internal Modules

Modules and names with the `_` prefix are not part of the public API. They may
change without compatibility guarantees, provided that public contracts remain intact.
```
