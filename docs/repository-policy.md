# Repository hygiene and distribution boundaries

This document defines which files may enter Git history and which files may be
published in Python package distributions.

## Files intended for Git history

The repository may track:

- Python package sources under `src/irs_generator/`;
- tests under `tests/`;
- public documentation under `README.md`, `docs/` and example README files;
- packaging and workflow configuration such as `pyproject.toml`,
  `MANIFEST.in`, `uv.lock` and `.github/workflows/`;
- small curated examples and regression fixtures under `examples/`.

The current `examples/full_flight` dataset is treated as a public,
curated example and regression fixture. It is not private research data.

## Files excluded from Git history

The repository must not track:

- virtual environments and local tool installations;
- Python caches, type-checker caches and test caches;
- build outputs such as `dist/`, `build/` and package metadata directories;
- generated example outputs;
- local databases, logs, debug exports and temporary files;
- notebooks, plots, reports and research workspaces unless they are explicitly
  promoted to curated public documentation;
- simulator binaries and provider-specific binaries, including `.exe`, `.dll`,
  `.so`, `.dylib` and `.bin` files;
- secrets, credentials, private notes and machine-local configuration.

Cleanup must not silently delete user data. Local research assets should stay
in ignored directories such as `data/`, `research/`, `notebooks/`, `plots/` or
`reports/` until a separate migration decision defines their public location.

## Package distribution boundary

The wheel is intended to contain only the importable `irs_generator` package and
standard wheel metadata.

The source distribution is intended to contain:

- `src/irs_generator/`;
- setuptools-generated source metadata under `src/irs_generator.egg-info/`;
- `README.md`;
- `LICENSE`;
- `pyproject.toml`;
- `MANIFEST.in`;
- public policy documents under `docs/`.

The source distribution intentionally excludes tests, examples, generated data,
research assets, local environments, IDE metadata, caches, external binaries and
development outputs.

## Reproducible package-content audit

Build artifacts from a clean checkout:

```bash
python -m pip install --upgrade build
python -m build
```

Inspect and validate the produced wheel and source distribution:

```bash
python scripts/audit_distribution.py
```

The audit fails if package artifacts contain ignored development files,
generated outputs, simulator/provider binaries, examples, tests or research
assets.
