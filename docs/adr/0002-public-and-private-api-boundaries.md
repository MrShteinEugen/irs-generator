# ADR 0002: Public and Internal API Boundaries

## Status

Accepted.

## Context

The library should provide a stable and user-friendly interface without restricting 
the project's internal evolution. Exposing every class and function as part of the 
public API encourages consumers to depend on internal implementation details, making 
even routine refactoring a potentially breaking change.

To address this, the project clearly distinguishes the public API from internal 
implementation details and defines explicit conventions for re-export modules.

## Decision

The following are considered part of the public API:

* names exported from package-level `__init__.py` files via `__all__`;
* names exported from public modules via `__all__`;
* documented classes, functions, and protocols intended for users to import.

The following are considered internal API:

* modules, functions, classes, and attributes prefixed with `_`;
* helper utilities used exclusively within a layer;
* implementation details of numerical methods unless they are explicitly exported
through `__all__`;
* temporary decorator functions unless they are explicitly declared as part of the
stable API.

Whenever possible, users should import symbols through the layer's top-level 
re-export modules.

```python
from irs_generator.earth_model import GeodeticPosition, WGS84EarthModel
from irs_generator.irs_model import DcmStrapdownINS, ImuSample
from irs_generator.generation import SyntheticDataGenerator, CsvOutputWriter
```

Direct imports from a specific module are acceptable for advanced use cases. However, 
the stability of such imports depends on whether the imported name is included in 
that module's `__all__`.

### Rules for `__all__`

Every public module intended for direct user imports should define `__all__`.

Only names that the project is prepared to support as part of the library's long-term 
public API should be included in `__all__`, such as:

* domain value classes;
* protocols and configuration objects;
* Earth, gravity, IMU, and GNSS models;
* navigation algorithm implementations;
* generators, readers, writers, and output formats;
* exceptions, if they are part of the library's intended user-facing behavior.

The following should **not** be included in `__all__`:

* private helper functions;
* implementation details that can be replaced without affecting the semantics of the
public API;
* temporary functions unless they are explicitly documented as part of the supported API.


### Rules for Re-export Modules

Package-level `__init__.py` files should provide users with a concise and predictable 
import path.

Each layer may re-export its primary user-facing abstractions. For example, 
`irs_generator.irs_model` may re-export `ImuSample`, `DcmStrapdownINS`, 
`InertialReferenceSystem`, and the algorithm protocol interfaces.


### Consequences

Developers are free to refactor internal helper functions without affecting users.

Users benefit from a clear and predictable import model.

Any new public symbol must:

* be included in the appropriate `__all__`;
* be documented or covered by public API tests;
* belong to the architectural layer in which it is exposed.


### Rejected Alternatives

### Treat Everything Under `src/irs_generator` as Public API

Rejected because it would make routine internal refactoring impractical.

### Re-export Everything from the Top-Level `irs_generator` Package

Rejected at this early stage of the project.

### Avoid Using `__all__`

Rejected because the project requires an explicit boundary between the user-facing API and internal implementation details.

