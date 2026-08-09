# Architecture Decision Records (ADR)

This directory contains the project's architectural solutions.

An ADR is a document that records significant architectural decisions 
and their rationale. The document must answer the questions: "what was 
decided," "why was it decided this way," "what alternatives were 
considered," and "what consequences will this decision lead to."

## ADR Naming Convention

Each ADR file must follow this naming pattern:

```text
NNNN-short-descriptive-title.md
```

Where:

* `NNNN` is a four-digit sequential number, starting from `0001`;
* the title briefly describes the decision;
* words are written in lowercase and separated with hyphens;
* the filename must not contain spaces, underscores, dates, or implementation-specific details.

Examples:

```text
0001-package-layers-and-dependency-direction.md
0002-public-and-private-api-boundaries.md
0003-versioning-and-api-compatibility-policy.md
```

The document title must use the following format:

```text
# ADR NNNN: Descriptive Decision Title
```

Example:

```text
# ADR 0003: Versioning and API Compatibility Policy
```

ADR numbers are assigned sequentially and must never be reused, including after an ADR has been deprecated or superseded.

The name should describe the architectural decision or policy. Avoid vague or procedural names such as:

```text
0004-refactoring.md
0005-api-changes.md
0006-fix-dependencies.md
```

Prefer names that identify the actual decision:

```text
0004-separation-of-domain-and-infrastructure-layers.md
0005-error-handling-in-the-public-api.md
0006-third-party-dependency-policy.md
```


## Decision Log

1. [Package Layers and Dependency Direction](0001-package-layers-and-dependency-direction.md)
2. [Public and Internal API Boundaries](0002-public-and-private-api-boundaries.md)
3. [API Versioning and Compatibility Policy](0003-versioning-and-api-compatibility-policy.md)

## Criteria for Creating a New ADR

A new ADR should be created if the change:

- modifies the package architecture or layer structure;
- changes the public API or its re-export rules;
- changes the project's dependency policy;
- changes numerical conventions, coordinate systems, or units of measurement;
- affects compatibility with existing user code;
- defines a long-term approach to extending the project.

---
