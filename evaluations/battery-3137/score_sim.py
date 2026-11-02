# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""Score results-sim.json against 3.137 criterion 1 (bars in the plan)."""
import json
import sys
from pathlib import Path

d = json.loads(Path(sys.argv[1] if len(sys.argv) > 1 else
                    Path(__file__).with_name("results-sim.json")).read_text())
rows = d["rows"]
drain = d["drain_v_per_s"]
N = {tuple(r["start"]): r for r in rows if r["arm"] == "N"}
L = {tuple(r["start"]): r for r in rows if r["arm"] == "L"}
R = [r for r in rows if r["arm"] == "R"]
t_low = d["crossings_sim_s"]["low"]

# 1d: a mission that finds before Low (in N) ends the same in L.
before = [s for s, r in N.items() if r["outcome"] == "found" and r["end_sim_s"] < t_low]
differ = [s for s in before if (L[s]["outcome"], L[s]["end_sim_s"]) != ("found", N[s]["end_sim_s"])]
print(f"1d: {len(before)} of {len(N)} N runs found before Low; {len(differ)} differ in L "
      f"(bar 0) -> {'met' if not differ else 'FAILED'}")
late = [s for s, r in N.items() if not (r["outcome"] == "found" and r["end_sim_s"] < t_low)]
print(f"    the other {len(late)}: N {[N[s]['outcome'] for s in late]}, "
      f"L {[L[s]['outcome'] + ' at ' + str(L[s]['end_sim_s']) for s in late]}")

# 1a: each level fires within 2 board samples of its crossing. Here a board
# sample is a poll (0.5 s of sim time, the pack measured every read).
lat = []
for r in R:
    for lvl, bar in (("reserve", 10.2), ("poweroff", 10.0)):
        f = r["fired"].get(lvl)
        if not f:
            continue
        cross = (r["v0"] - bar) / drain
        if f["how"].startswith("reserve held"):
            continue
        lat.append((lvl, round(f["sim_s"] - cross, 2)))
        # In polls (board samples here: the pack is measured every 0.5 s poll).
        r.setdefault("_samples", {})[lvl] = int((f["sim_s"] - cross) // 0.5) + 1
worst = {lvl: max([x for l, x in lat if l == lvl], default=None) for lvl in ("reserve", "poweroff")}
print(f"1a: fire after crossing, worst: {worst} s (bar 2 samples = 1.0 s of polls, plus one "
      f"poll's phase) -> {'met' if all(v is not None and v <= 1.5 for v in worst.values()) else 'CHECK'}")
from collections import Counter
print("    fired on board sample n after the crossing:",
      {lvl: dict(Counter(r.get("_samples", {}).get(lvl) for r in R if lvl in r.get("_samples", {})))
       for lvl in ("reserve", "poweroff")})
print(f"    sag tests: {[(s['refresh_s'], s['drain_v_per_s'], s['fired'], s['checks'], s['cleared']) for s in d['sag']]}")

# 1b: 0 autonomous motion after Reserve; a person's FORWARD at reduced speed.
mot = [r["autonomous_motion_after_reserve_m"] for r in R]
print(f"1b: autonomous motion after Reserve, max {max(m for m in mot if m is not None) * 100:.2f} cm "
      f"over {sum(m is not None for m in mot)} runs (bar 2 cm)")
print(f"    brain at reserve: {sorted(set(str(r.get('brain_at_reserve')) for r in R))}; person: "
      f"{sorted(set(str((r.get('person_at_reserve') or {}).get('speed')) for r in R))} speed, "
      f"executed {sorted(set(str((r.get('person_at_reserve') or {}).get('executed')) for r in R))}")

# 1c: the stub exactly once, after brain, ROS, VLM, with the wheels at 0.
orders = {tuple(s["step"] for s in r["shutdown"]) for r in R}
once = [r["poweroff_calls"] for r in R]
wheels = {r["shutdown"][0]["rc"] for r in R if r["shutdown"]}
print(f"1c: orders {orders}; power-off calls {sorted(set(once))}; wheels-at-rest rc {wheels}; "
      f"person at power-off {sorted(set(r['person_at_poweroff'] for r in R))}")
print(f"    R outcomes: {sorted(set(r['outcome'] for r in R))}")
