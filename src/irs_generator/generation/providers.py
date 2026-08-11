"""Contracts for source-specific target trajectory adapters."""

from __future__ import annotations

from collections.abc import Iterator
from typing import Protocol, runtime_checkable

from .models import TrajectoryPoint

__all__ = ["TrajectoryProviderAdapter"]


@runtime_checkable
class TrajectoryProviderAdapter(Protocol):
    """Adapter that yields provider data as canonical target trajectory points.

    Implementations own provider-specific details such as file access, column names,
    and record parsing. They apply the configured input convention before yielding
    points, so consumers receive only canonical project values.
    """

    def __iter__(self) -> Iterator[TrajectoryPoint]:
        """Yield canonical target trajectory points."""

        ...

    def iter_points(self) -> Iterator[TrajectoryPoint]:
        """Yield canonical target trajectory points through the adapter API."""

        ...
