# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.161.B part 1: trace Z5's 7 lost finds -- the first goal where the zones arm and the
no-zone arm part, and whether the no-zone goal there lies in a zone (grown 0.12 m).
Run from the repo root: python evaluations/nogo-3161/rerun-2026-11-20/trace.py"""
import json, sys, os
sys.path.insert(0, os.getcwd())
sys.argv = [sys.argv[0]]
import importlib.util
spec = importlib.util.spec_from_file_location("sweep", "evaluations/nogo-3161/sweep.py")
sw = importlib.util.module_from_spec(spec); spec.loader.exec_module(sw)
from world import nogo
from sim.maps import build_world
LOST = [('procthor:val:3', 0), ('procthor:val:13', 1), ('procthor:val:26', 0), ('procthor:val:39', 0),
        ('procthor:val:39', 1), ('procthor:val:44', 0), ('procthor:val:44', 1)]
out = []
for h, s in LOST:
    zones = sw.place_zones(h, s)
    z = sw.run(h, s, "zones", zones=zones)
    n = sw.run(h, s, "none")
    gz, gn = z["goal_seq"], n["goal_seq"]
    k = next((i for i in range(min(len(gz), len(gn))) if gz[i] != gn[i]), None)
    g = build_world(h)
    tgt = None
    for o in getattr(g, "objects", []) or []:
        if "backpack" in str(getattr(o, "name", o)):
            tgt = o
    nz = gn[k] if k is not None and k < len(gn) else None
    row = {"house": h, "seed": s, "z": z["outcome"], "n": n["outcome"], "first_diverge": k,
           "goal_none": nz, "goal_zones": gz[k] if k is not None else None,
           "none_goal_in_grown_zone": bool(nz and nogo.zone_at(
               [nogo.clean_zone(q) for q in zones], nz[0], nz[1], nogo.HALF_WIDTH_M)),
           "none_goal_dist_to_zone": round(nogo.nearest_distance(
               [nogo.clean_zone(q) for q in zones], *nz), 2) if nz else None,
           "zones": [[q["x0"], q["y0"], q["x1"], q["y1"]] for q in zones]}
    print(json.dumps(row), flush=True)
    out.append(row)
json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "trace.json"), "w"))
