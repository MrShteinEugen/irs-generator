# ADR 0001: Package Layers and Dependency Direction

## Status

Accepted.

## Context

`irs-generator` is developed as a Python library. The following parts of the project must be independently modifiable:

* Earth models;
* GNSS and IMU models;
* inertial navigation algorithms;
* the synthetic data generator;
* input/output adapters.

## Decision

The package is divided into the following architectural layers:

```text
irs_generator/
├── utils/
├── earth_model/
├── navigation_model/
├── gps_model/
├── irs_model/
└── generation/
```

Layer responsibilities:

* `utils` contains small numerical and validation utilities. It is a low-level technical layer and must not contain high-level domain logic.
* `earth_model` defines Earth geometry, ellipsoid parameters, Earth rotation, and gravity models.
* `navigation_model` defines position, velocity, orientation, and the complete navigation state.
* `gps_model` defines GNSS measurements as a domain data model.
* `irs_model` defines the inertial system: the data produced by the IMU, the way the INS computes position, velocity, and orientation, the handling of sensor errors, and how these components are combined into an IRS.
* `generation` contains the streaming IMU/GNSS data generator, the iterative algorithm used to improve generation accuracy, input/output functionality, and input data formats.

Dependency rules:

* `utils` must not depend on domain-specific layers.
* `earth_model` must not depend on `navigation_model`, `gps_model`, `irs_model`, or `generation`.
* `navigation_model` may depend on `earth_model` and `utils`, but must not depend on INS algorithms or the generator.
* `gps_model` may use basic navigation classes, but must not depend on IRS, IMU, or the generator.
* `irs_model` may use Earth models, navigation state models, and GNSS models, but must not depend on `generation`.
* `generation` is responsible for the data generation process itself. It uses only the common inertial algorithm interface defined by `irs_model` and must not depend on the internal implementation of a particular algorithm.

## Consequences

The generator must not depend on whether the navigation algorithm is based on a direction cosine matrix, quaternions, or another mathematical representation. It depends only on the common algorithm interface: access to the current state, an integration step, and the ability to create an independent copy for trial computations.

This separation makes the project extensible. New gravity models, INS algorithms, input/output formats, and generation scenarios can be added without extensive changes to lower-level layers.

## Rejected Alternatives

### Separate Generator Subclasses for Each INS Algorithm

Rejected as the primary strategy. Different INS algorithms must be integrated through a common navigation algorithm interface. The generator architecture must not grow through a separate class hierarchy for every mathematical model.

### Direct Access to Internal INS State

Rejected because it would couple the generator to a specific implementation and make it harder to add alternative INS algorithms.
