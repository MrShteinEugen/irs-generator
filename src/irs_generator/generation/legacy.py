"""Compatibility aliases for the initial generation API preview."""

from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path

import numpy as np

from .dcm import DcmTrajectoryGenerator, DcmTrajectoryPoint, DcmTrajectoryReader

__all__ = [
    "LegacyDcmReferenceGenerator",
    "LegacyTrajectoryPoint",
    "LegacyTrajectoryReader",
]


LegacyTrajectoryPoint = DcmTrajectoryPoint


class LegacyTrajectoryReader(DcmTrajectoryReader):
    """Backward-compatible name for ``DcmTrajectoryReader``."""

    def legacy_time_step_s(self) -> np.longdouble:
        return self.time_step_s()


class LegacyDcmReferenceGenerator(DcmTrajectoryGenerator):
    """Backward-compatible name for ``DcmTrajectoryGenerator``."""

    def write_legacy(
        self,
        points: Iterable[DcmTrajectoryPoint],
        output_dir: str | Path,
    ) -> None:
        self.write(points, output_dir)
