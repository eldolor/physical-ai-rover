# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.141 criterion 1 at full size: `tests/test_pivot_3141.py`'s random
cases (scans around the chassis, aged arcs, turns and translations), the
new turn guard against the frozen `tests/pivot_reference.py`, 200,000 cases
over worker processes. Writes one JSON summary.

    python evaluations/wheel-loop-3141/equivalence.py <out.json> [cases] [workers]
"""

import json
import random
import sys
import time
from multiprocessing import Pool
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from robot.safety import SafetyController  # noqa: E402
from tests.pivot_reference import ReferenceSafety  # noqa: E402
from tests.test_pivot_3141 import answers, case  # noqa: E402

BATCH = 2000


def batch(seed: int) -> dict:
    rng = random.Random(seed)
    out = {"cases": 0, "differences": 0, "free": 0, "scaled": 0, "stopped": 0, "first": None}
    for _ in range(BATCH):
        c = case(rng)
        ref, new = answers(ReferenceSafety, *c), answers(SafetyController, *c)
        out["cases"] += 1
        scale = new[0][0]
        out["free" if scale == 1.0 else "stopped" if scale == 0.0 else "scaled"] += 1
        if ref != new:
            out["differences"] += 1
            if out["first"] is None:
                out["first"] = {"seed": seed, "ref": repr(ref), "new": repr(new)}
    return out


def main():
    out = Path(sys.argv[1])
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 200_000
    workers = int(sys.argv[3]) if len(sys.argv) > 3 else 6
    import desktop_lock
    desktop_lock.hold_for_life("shared", "3.141 criterion 1 equivalence")
    t0 = time.time()
    seeds = [3141_000 + i for i in range(n // BATCH)]
    with Pool(workers) as pool:
        parts = pool.map(batch, seeds)
    total = {k: sum(p[k] for p in parts) for k in ("cases", "differences", "free", "scaled", "stopped")}
    total["first_difference"] = next((p["first"] for p in parts if p["first"]), None)
    total["seconds"] = round(time.time() - t0, 1)
    out.write_text(json.dumps(total, indent=1) + "\n")
    print(json.dumps(total))


if __name__ == "__main__":
    main()
