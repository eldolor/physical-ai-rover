# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.141 criterion 2: `vet_bench.py`'s cases plus three corner-turn boxes,
dev's turn guard (`tests/pivot_reference.py`) against 3.141's, side by
side, p50 / p90 of one `vet_wheel_velocity()` in ms, and whether the
answers are the same. Run from a directory holding `robot/`, `tests/` and
this file's `vet_bench.py` (on the board, a copy in /tmp).

    python vet_compare.py [repeats]
"""
import sys
import time

sys.path.insert(0, ".")

import vet_bench as vb  # noqa: E402
from robot.safety import SafetyController  # noqa: E402
from tests.pivot_reference import ReferenceSafety  # noqa: E402

CASES = dict(vb.CASES, **{
    "corner box 0.171x0.140": (vb.box(0.171, 0.140), 0.05, (-8.0, 8.0)),
    "corner box 0.167x0.152": (vb.box(0.167, 0.152), 0.05, (-8.0, 8.0)),
    "corner box 0.159x0.140": (vb.box(0.159, 0.140), 0.05, (-8.0, 8.0)),
})


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 200
    for name, (ranges, turned, lr) in CASES.items():
        row = []
        for cls in (ReferenceSafety, SafetyController):
            body = vb.Body(ranges, turned)
            body.lr = lr
            s = cls(body)
            ts = []
            for _ in range(n):
                t0 = time.perf_counter()
                out = s.vet_wheel_velocity(*lr)
                ts.append((time.perf_counter() - t0) * 1000)
            ts.sort()
            row.append((ts[n // 2], ts[int(0.9 * n)], out))
        print(f"{name:24s} dev p50 {row[0][0]:6.2f} p90 {row[0][1]:6.2f} | 3.141 p50 {row[1][0]:6.2f} "
              f"p90 {row[1][1]:6.2f} | same {row[0][2] == row[1][2]}", flush=True)


if __name__ == "__main__":
    main()
