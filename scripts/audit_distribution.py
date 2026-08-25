"""Audit built package artifacts for repository-boundary violations."""

from __future__ import annotations

import argparse
import sys
import tarfile
import zipfile
from collections.abc import Iterable
from pathlib import Path

FORBIDDEN_PREFIXES = (
    ".github/",
    ".idea/",
    ".vscode/",
    ".venv/",
    ".tools/",
    ".draw/",
    ".documentation/",
    ".package-check/",
    "build/",
    "data/",
    "debug/",
    "dist/",
    "env/",
    "examples/",
    "exports/",
    "htmlcov/",
    "notebooks/",
    "plots/",
    "reports/",
    "research/",
    "tests/",
    "venv/",
)

FORBIDDEN_PARTS = {
    "__pycache__",
    ".ipynb_checkpoints",
    ".mypy_cache",
    ".pytest_cache",
    ".pytest-tmp",
    ".ruff_cache",
}

FORBIDDEN_SUFFIXES = (
    ".bak",
    ".bin",
    ".coverage",
    ".dll",
    ".dylib",
    ".exe",
    ".log",
    ".pyc",
    ".pyd",
    ".pyo",
    ".so",
    ".sqlite3",
    ".tmp",
)

WHEEL_ALLOWED_PREFIXES = ("irs_generator/",)
WHEEL_ALLOWED_SUFFIXES = (".dist-info/", ".data/")

SDIST_ALLOWED_ROOT_FILES = {
    "LICENSE",
    "MANIFEST.in",
    "PKG-INFO",
    "README.md",
    "pyproject.toml",
    "setup.cfg",
}
SDIST_ALLOWED_PREFIXES = ("docs/", "src/irs_generator/", "src/irs_generator.egg-info/")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check wheel and sdist contents for unintended files."
    )
    parser.add_argument(
        "dist",
        nargs="?",
        type=Path,
        default=Path("dist"),
        help="Directory containing built .whl and .tar.gz artifacts.",
    )
    args = parser.parse_args()

    artifacts = sorted(args.dist.glob("*.whl")) + sorted(args.dist.glob("*.tar.gz"))
    if not artifacts:
        print(f"No wheel or sdist artifacts found in {args.dist}.", file=sys.stderr)
        return 1

    failed = False
    for artifact in artifacts:
        names = _artifact_names(artifact)
        violations = _violations(artifact, names)
        print(f"{artifact}: {len(names)} entries")
        if violations:
            failed = True
            for violation in violations:
                print(f"  forbidden: {violation}")

    if failed:
        return 1

    print("Distribution audit passed.")
    return 0


def _artifact_names(artifact: Path) -> list[str]:
    if artifact.suffix == ".whl":
        with zipfile.ZipFile(artifact) as wheel:
            return sorted(name for name in wheel.namelist() if not name.endswith("/"))

    if artifact.name.endswith(".tar.gz"):
        with tarfile.open(artifact, "r:gz") as sdist:
            return sorted(
                member.name for member in sdist.getmembers() if member.isfile()
            )

    raise ValueError(f"Unsupported artifact type: {artifact}")


def _violations(artifact: Path, names: Iterable[str]) -> list[str]:
    violations: list[str] = []
    for name in names:
        normalized = (
            _strip_sdist_root(name) if artifact.name.endswith(".tar.gz") else name
        )
        if _has_forbidden_component(normalized):
            violations.append(name)
            continue
        if artifact.suffix == ".whl" and not _is_allowed_wheel_entry(normalized):
            violations.append(name)
            continue
        if artifact.name.endswith(".tar.gz") and not _is_allowed_sdist_entry(
            normalized
        ):
            violations.append(name)
    return violations


def _strip_sdist_root(name: str) -> str:
    parts = name.split("/", maxsplit=1)
    if len(parts) == 1:
        return name
    return parts[1]


def _has_forbidden_component(name: str) -> bool:
    parts = set(name.split("/"))
    if parts & FORBIDDEN_PARTS:
        return True
    if name.endswith(FORBIDDEN_SUFFIXES):
        return True
    return name.startswith(FORBIDDEN_PREFIXES)


def _is_allowed_wheel_entry(name: str) -> bool:
    if name.startswith(WHEEL_ALLOWED_PREFIXES):
        return True
    return any(part in name for part in WHEEL_ALLOWED_SUFFIXES)


def _is_allowed_sdist_entry(name: str) -> bool:
    if name in SDIST_ALLOWED_ROOT_FILES:
        return True
    return name.startswith(SDIST_ALLOWED_PREFIXES)


if __name__ == "__main__":
    raise SystemExit(main())
