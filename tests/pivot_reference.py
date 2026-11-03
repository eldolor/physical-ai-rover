# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""Frozen copy of `robot/safety.py`'s `SafetyController.pivot_scale()` and
`pivot_blocked()` as they were before 3.141 (dev at 2a5ff609), verbatim.
3.141 made `pivot_scale()`'s search do its per-scan work once;
`tests/test_pivot_3141.py` checks it against this one. Do not edit or
"simplify" it: it is the definition of the right answer, the way
`tests/walk_reference.py` is for 3.120.
"""

import math
from typing import Optional, Tuple

from robot.safety import (FOOTPRINT_LENGTH_M, FOOTPRINT_WIDTH_M, PIVOT_AGE_SAMPLES,
                          PIVOT_LOOKAHEAD_S, PIVOT_MARGIN_CM, PIVOT_MIN_LOOKAHEAD_DEG,
                          SafetyController, scan_points_cm)


class ReferenceSafety(SafetyController):
    """dev's controller with dev's turn guard: `vet_wheel_velocity()` and
    `run_verb()` reach these through `self`."""

    def pivot_scale(self, omega_rad_s: float, scan: Optional[dict] = None
                    ) -> Tuple[float, Optional[str]]:
        """How much of a turn at `omega_rad_s` may go ahead -- 3.19.

        `(1.0, None)` when all of it may. Otherwise the largest fraction
        (to 1/64) whose look-ahead keeps every closing return outside
        `PIVOT_MARGIN_CM`, and the reason; `0.0` stops the turn. Scaling
        rather than zeroing lets a turn slow into the margin: zeroing it
        stopped robots a whole control period (~3 degrees at 1 rad/s)
        short of where they could safely have turned.
        """
        scan = self._scan() if scan is None else scan
        if self.pivot_blocked(omega_rad_s, scan) is None:
            return 1.0, None
        lo, hi = 0.0, 1.0
        for _ in range(6):
            mid = (lo + hi) / 2
            if self.pivot_blocked(omega_rad_s * mid, scan, exact=True) is None:
                lo = mid
            else:
                hi = mid
        return lo, self.pivot_blocked(omega_rad_s, scan)

    def pivot_blocked(self, omega_rad_s: float, scan: Optional[dict] = None,
                      exact: bool = False) -> Optional[str]:
        """Why a turn at `omega_rad_s` (CCW positive, REP-103) must be
        refused, or None -- 3.19. `exact` drops the minimum look-ahead,
        for `pivot_scale()`'s search over slower turns.

        Every scan return is put in the body frame (x ahead, y left) and its
        distance to the chassis rectangle compared now and after the turn
        the next vet could not see. Refused only if that distance ends under
        `PIVOT_MARGIN_CM` AND shrinks. No usable scan: never refused -- the
        pre-3.19 behaviour, and a rotation on a scan-less backend is still
        a rotation within the circle the forward checks already cleared.
        """
        if omega_rad_s == 0:
            return None
        scan = self._scan() if scan is None else scan
        if not scan or not scan.get("usable"):
            return None
        hl, hw = FOOTPRINT_LENGTH_M * 50.0, FOOTPRINT_WIDTH_M * 50.0
        d = omega_rad_s * PIVOT_LOOKAHEAD_S
        if not exact and abs(d) < math.radians(PIVOT_MIN_LOOKAHEAD_DEG):
            d = math.copysign(math.radians(PIVOT_MIN_LOOKAHEAD_DEG), d)
        c, s = math.cos(-d), math.sin(-d)     # the world turns the other way
        # 3.42: the turn the scan has missed. A real lidar's points are up
        # to ~150 ms old and the body may have been turning since, so each
        # point's bearing NOW lies on an arc between where the scan put it
        # and that turn later. Check the look-ahead from across the arc.
        missed = self._turn_since_scan()
        arc = [missed * k / PIVOT_AGE_SAMPLES for k in range(PIVOT_AGE_SAMPLES + 1)] \
            if missed else [0.0]
        rots = [(math.cos(-a), math.sin(-a)) for a in arc]

        def dist(x, y):
            return math.hypot(max(abs(x) - hl, 0.0), max(abs(y) - hw, 0.0))

        for x0, y0 in scan_points_cm(scan):
            for ca, sa in rots:
                x, y = ca * x0 - sa * y0, sa * x0 + ca * y0
                now = dist(x, y)
                if now > PIVOT_MARGIN_CM + 20.0:
                    continue
                after = dist(c * x - s * y, s * x + c * y)
                if after < PIVOT_MARGIN_CM and after < now - 1e-6:
                    side = "left" if omega_rad_s > 0 else "right"
                    return (f"turn {side} clamped: a corner would come within "
                            f"{after:.1f}cm < {PIVOT_MARGIN_CM}cm (scan_footprint"
                            + (f", aged {math.degrees(missed):.1f} deg)" if missed else ")"))
        return None
