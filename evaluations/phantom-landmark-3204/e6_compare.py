# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.204.B E6: dev's rows against key off's, identical except wall-clock fields (keys containing 'wall').

    python evaluations/phantom-landmark-3204/e6_compare.py DEV KEYOFF
"""
import json, sys
def strip(x):
    if isinstance(x, dict): return {k: strip(v) for k, v in x.items() if "wall" not in k}
    if isinstance(x, list): return [strip(v) for v in x]
    return x
def rows(p):
    t = open(p).read()
    if p.endswith(".json"): return json.loads(t)["rows"]
    return [json.loads(l) for l in t.splitlines() if l.strip()]
a, b = rows(sys.argv[1]), rows(sys.argv[2])
key = lambda r: json.dumps(r, sort_keys=True)
sa, sb = sorted(map(key, strip(a))), sorted(map(key, strip(b)))
diff = sum(x != y for x, y in zip(sa, sb)) + abs(len(sa) - len(sb))
print(sys.argv[1].split("/")[-1], len(a), len(b), "IDENTICAL" if diff == 0 else f"DIFFER {diff}")
