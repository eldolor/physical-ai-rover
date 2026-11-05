# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""Frozen copy of `robot/safety.py`'s `SafetyController.footprint_clearance()`
as it was before 3.141.E (dev at 5bc43563), verbatim, on top of
`tests/pivot_reference.py`'s frozen turn guard: together, dev's vet before
3.141.E's numpy. `tests/test_vet_cost_3141.py` checks the new code against
it. Do not edit or "simplify" it."""

from typing import Optional, Tuple

from robot.safety import _corridor_room, scan_points_cm
from tests.pivot_reference import ReferenceSafety


class VetReference(ReferenceSafety):
    """dev's controller before 3.141.E: the pre-3.141 turn guard and the
    corridor's per-point loop. `forward_clearance()`, `reverse_clearance()`
    and `vet_wheel_velocity()` reach these through `self`."""

    def footprint_clearance(self, direction: int = 1, scan: Optional[dict] = None
                            ) -> Tuple[Optional[float], str]:
        """Room in the corridor the chassis sweeps -- 3.18.

        `direction` +1 is forward, -1 is astern. Every scan return is put in
        the body frame (`scan_points_cm()`: the lidar sits `LIDAR_X_M`
        ahead of `base_link`, 3.27); a return counts if it lies within
        half the chassis width plus `FOOTPRINT_SIDE_MARGIN_CM` of the
        centre-line AND beyond the leading edge in the direction of travel.
        The clearance is its distance past that edge -- how far the chassis
        can move before the leading edge reaches it, the same meaning the
        cone's number has. A return INSIDE the outline's width and within
        its length (the body is already on it) reads 0.0.

        `(None, "no_scan")` for a backend with no usable scan: the cone and
        the scalar still answer, exactly as before 3.18. `(None,
        "scan_footprint_no_target")` when the corridor is empty.
        """
        scan = self._scan() if scan is None else scan
        if not scan or not scan.get("usable"):
            return None, "no_scan"
        best = None
        for x, y in scan_points_cm(scan):
            room = _corridor_room(direction * x, abs(y))
            if room is None:
                continue
            best = room if best is None else min(best, room)
        if best is None:
            return None, "scan_footprint_no_target"
        return round(best, 1), "scan_footprint"
