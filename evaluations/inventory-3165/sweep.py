# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.165 criterion 4: 3.60's sweep (`tests.inventory_sweep`, seed 3463, 20
frontier missions in complex_house, detector errors on), with every
mission's landmark report kept so two runs compare field by field.

    .venv/bin/python evaluations/inventory-3165/sweep.py run <out.json>
    .venv/bin/python evaluations/inventory-3165/sweep.py compare <dev.json> <after.json>
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

SEED = 3463
FIELDS = ("label", "belief", "hits", "misses", "reported", "disputed")


def run(out):
    from brain import inventory as inv
    from tests import inventory_sweep as sw
    most = [0]
    real = inv.Inventory.observe

    def observe(self, *a, **kw):
        real(self, *a, **kw)
        most[0] = max([most[0]] + [len(lm.points) for lm in self.landmarks])

    inv.Inventory.observe = observe
    try:
        runs = [sw.run_one(s, seed=SEED * 100 + k, max_steps=sw.MAX_STEPS)
                for k, s in enumerate(sw.starts(seed=SEED))]
    finally:
        inv.Inventory.observe = real
    res = {"score": sw.score(runs), "most_members": most[0],
           "reports": [r["report"]["reported"] + r["report"]["candidates"] for r in runs]}
    Path(out).write_text(json.dumps(res, indent=1) + "\n")
    print(json.dumps({k: v for k, v in res.items() if k != "reports"}, indent=1))


def _key(d):
    return (d["label"], round(d["x_m"], 3), round(d["y_m"], 3))


def compare(a, b):
    A, B = (json.loads(Path(p).read_text()) for p in (a, b))
    differ = total = 0
    for ra, rb in zip(A["reports"], B["reports"]):
        da = {_key(d): d for d in ra}
        db = {_key(d): d for d in rb}
        total += len(set(da) | set(db))
        for k in set(da) | set(db):
            if k not in da or k not in db or any(da[k][f] != db[k][f] for f in FIELDS):
                differ += 1
    out = {"landmarks": total, "differ": differ,
           "score": {k: [A["score"][k], B["score"][k]] for k in A["score"]}}
    print(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    if sys.argv[1] == "run":
        run(sys.argv[2])
    else:
        compare(sys.argv[2], sys.argv[3])
