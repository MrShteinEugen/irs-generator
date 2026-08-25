# ADR 0003: API Versioning and Compatibility Policy

## Status

Accepted.

## Context

Although the project is still in its early stages, it should be developed 
with the expectation that it will eventually become a public library. 
Users should have a clear understanding of which changes are considered 
backward compatible, which require migration, and which—once the API has 
been stabilized—will be treated as breaking changes.


## Decision

The project follows **Semantic Versioning**:

```text
MAJOR.MINOR.PATCH
```

The version components have the following meaning:

* `PATCH` — bug fixes, documentation improvements, and internal changes 
that do not affect the library's public behavior;
* `MINOR` — new functionality, new public classes, new data formats, 
new algorithms, and other backward-compatible API extensions;
* `MAJOR` — changes that break the public API.

Before the release of version `1.0.0`, the public API is considered 
unstable. During this period, changes to the public API are permitted 
when necessary to improve the architecture, increase numerical accuracy, 
or support future extensibility. Such changes must be clearly documented 
and, when they affect user code, accompanied by migration guidance.

Starting with version `1.0.0`, the library guarantees backward compatibility 
of its public API throughout the `1.x` major release series.


## Compatibility Policy for the `1.x` Release Series

Within the `1.x` release series, the following changes are considered backward compatible:

* adding new public classes, functions, configuration objects, and algorithms;
* adding optional parameters with default values;
* extending the set of supported input and output formats;
* improving numerical accuracy or performance without changing the documented semantics 
of the results.

The following changes are **not** permitted within the `1.x` release series and 
require a major version increment:

* removing a public name from `__all__`;
* renaming public classes or functions;
* changing the units of measurement used by public class attributes or function 
parameters;
* changing the orientation of coordinate axes or the adopted coordinate system;
* changing the required parameters of a public constructor or function;
* making incompatible changes to the `NavigationAlgorithm` or 
`InertialNavigationAlgorithm` protocols.

## Deprecation Policy

When a public API needs to be replaced, the existing name should first be 
marked as deprecated. It may only be removed after an appropriate migration period.

For the stable `1.x` release series, the minimum policy is:

* a deprecated API must remain functional until at least the next minor release;
* the deprecation warning should clearly indicate the recommended replacement;
* the documentation should present the new, preferred API.

For the `0.x` release series, the deprecation period may be shorter. However, 
the change must still be explicitly documented in the changelog, release notes, 
or the corresponding commit or issue description.


## Numerical Changes

Changes to numerical results are not necessarily considered breaking changes. 
Such changes are acceptable if they:

* correct an implementation error;
* improve numerical accuracy;
* preserve the documented units, coordinate systems, and semantics of the results;
* are accompanied by updated tests and a documented explanation of the change.

If a change in numerical results is caused by adopting a different mathematical 
model or changing coordinate system conventions, it is considered potentially 
incompatible and requires a separate architectural decision.


## Consequences

Before version `1.0.0`, the project is free to evolve with relatively 
few compatibility constraints.

Starting with version `1.0.0`, public re-exported names, algorithm 
interfaces, units of measurement, and coordinate system conventions 
become stable and are guaranteed to remain backward compatible throughout 
the `1.x` release series.


## Rejected Alternatives

### Do Not Define a Compatibility Policy Until `1.0.0`

Rejected because the project is already being designed as a library, and 
architectural decisions should account for future API stability from the outset.

### Guarantee Full Backward Compatibility Throughout the `0.x` Series

Rejected as an unnecessary early constraint. Before `1.0.0`, the project 
should retain the flexibility to refine its architecture and improve the 
numerical accuracy of its synthesized outputs.


