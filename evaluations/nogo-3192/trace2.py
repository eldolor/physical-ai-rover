# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.192.B: trace F1's remaining losses (chooser `rule`) as 3.161.B part 1
traced Z5's: the first goal where the arms part, the no-zone goal there and
its distance to the nearest zone (inside a zone grown 0.12 m: D3's own drop),
and whether the no-zone goal is a frontier the zones arm also had.

    python evaluations/nogo-3192/trace2.py OUT.json HOUSE:SEED ...
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, HERE)
args = sys.argv[1:]
sys.argv = [sys.argv[0]]
import sweep2                                            # noqa: E402
from world import nogo                                   # noqa: E402

out = []
for spec in args[1:]:
    h, s = spec.rsplit(":", 1)
    s = int(s)
    zones = sweep2.sw.place_zones(h, s)
    z = sweep2.sw.run(h, s, "zones", zones=zones)
    n = sweep2.sw.run(h, s, "none")
    gz, gn = z["goal_seq"], n["goal_seq"]
    k = next((i for i in range(min(len(gz), len(gn))) if gz[i] != gn[i]), None)
    zs = [nogo.clean_zone(q) for q in zones]
    nz = gn[k] if k is not None and k < len(gn) else None
    row = {"mission": f"{h} s{s}", "zones_outcome": z["outcome"], "none_outcome": n["outcome"],
           "goals_zones": len(gz), "goals_none": len(gn), "part_at_goal": k,
           "goal_none": nz, "goal_zones": gz[k] if k is not None and k < len(gz) else None,
           "none_goal_in_grown_zone": bool(nz and nogo.zone_at(zs, nz[0], nz[1], nogo.HALF_WIDTH_M)),
           "none_goal_dist_to_zone_m": round(nogo.nearest_distance(zs, *nz), 2) if nz else None,
           "zones": [[q["x0"], q["y0"], q["x1"], q["y1"]] for q in zones]}
    print(json.dumps(row), flush=True)
    out.append(row)
with open(args[0], "w") as fh:
    json.dump(out, fh, indent=1)
