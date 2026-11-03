# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.143: A2 and A3n on procthor:val:0 / plunger in process, every log line kept.

    python evaluations/soak-find-3143/plunger_diff.py <out.json>
"""
import sys, json, logging
sys.path.insert(0, "evaluations/soak-find-3143")
import ladder
from control import mission_runner as mr
out = {}
for arm in ("A2", "A3n"):
    lines = []
    h = logging.Handler(); h.emit = lambda r, L=lines: L.append(r.getMessage())
    root = logging.getLogger(); root.addHandler(h); root.setLevel(logging.INFO)
    logging.disable(logging.NOTSET)
    row = ladder.run(arm, "procthor:val:0", "plunger")
    root.removeHandler(h)
    out[arm] = {"row": {k: row.get(k) for k in ("outcome","steps","goals","closest_m","arrivals")}, "log": lines}
json.dump(out, open(sys.argv[1], "w"), indent=0)
for a in out: print(a, out[a]["row"], len(out[a]["log"]))
