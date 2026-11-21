# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.205.A N4 (d) and N5: 3.192.B's sweeps (`nogo-3192/fix/sweep_c1.py find`,
`sweep2.py survey`) on this branch against 3.192.B's pinned rows in
`evaluations/nogo-3192/fix/`, row for row, and the zones arms' goals in a zone.

    python evaluations/carried-no-memory-3205/same3192.py PINNED.json BRANCH.json
"""
import json
import sys

KEYS = ("outcome", "sim_s", "goals", "goals_in_zone", "goals_in_grown", "entries",
        "ends_near_zone", "reseen", "reseen_places", "coverage_outside", "mapped_free_outside_m2")


def same(a, b):
    def load(p):
        d = json.load(open(p))
        return {(r["house"], r["seed"], r["arm"], r.get("chooser")): r for r in d["rows"]}
    ra, rb = load(a), load(b)
    differ = {}
    for k in sorted(set(ra) | set(rb), key=str):
        x, y = ra.get(k), rb.get(k)
        if x is None or y is None:
            differ[str(k)] = "missing"
            continue
        d = [f for f in KEYS if x.get(f) != y.get(f)]
        if d:
            differ[str(k)] = d
    zones = [r for r in rb.values() if r["arm"] == "zones"]
    out = {"rows": len(rb), "identical": not differ and len(ra) == len(rb), "differ": differ,
           "N5": {"goals_in_zone": sum(len(r.get("goals_in_zone") or []) for r in zones),
                  "goals_in_grown": sum(len(r.get("goals_in_grown") or []) for r in zones),
                  "entries": sum(len(r.get("entries") or []) for r in zones)}}
    print(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    same(sys.argv[1], sys.argv[2])
