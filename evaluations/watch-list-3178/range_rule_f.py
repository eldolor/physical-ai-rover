# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.178.F, diagnosis only, TUNING seeds only: E's placed sweep with the
inventory's range rule swapped, to see whether ranging a sighting differently
recovers the passes lost to ranging (its section gives the numbers).

    python evaluations/watch-list-3178/range_rule_f.py closest --placed placements.json \
        --steps 150 --seed 3178 --workers 4 --out /tmp/x.json

`orig` is `inventory.range_at` (the nearest return within RANGE_GATE_DEG);
`closest` takes the return nearest in BEARING within the same gate; `median`
the median range in the gate. The swap is a monkeypatch in this process (and
its forked workers): nothing in `brain/` changes.
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))

from brain import inventory as inv  # noqa: E402
from brain.goal_pose import _wrap  # noqa: E402

FRESH = ("5178", "5179", "6178", "6179")     # E's and F's fresh seeds: never here
orig = inv.range_at


def closest(returns, bearing_deg, gate_deg=inv.RANGE_GATE_DEG):
    near = [(abs(_wrap(b - bearing_deg)), r) for b, r in returns
            if abs(_wrap(b - bearing_deg)) <= gate_deg]
    return min(near)[1] if near else None


def median(returns, bearing_deg, gate_deg=inv.RANGE_GATE_DEG):
    near = sorted(r for b, r in returns if abs(_wrap(b - bearing_deg)) <= gate_deg)
    return near[len(near) // 2] if near else None


if __name__ == "__main__":
    mode = sys.argv[1]
    if any(a in FRESH for a in sys.argv):
        raise SystemExit("tuning seeds only")
    inv.range_at = {"closest": closest, "median": median, "orig": orig}[mode]
    import sweep  # noqa: E402
    sweep.main(sys.argv[2:])
