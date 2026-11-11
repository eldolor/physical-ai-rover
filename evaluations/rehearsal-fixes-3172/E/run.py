#!/usr/bin/env python3
"""3.172.E: 3.158.B's rehearsal again, unchanged but for a fresh map volume.

    eval "$(python tools/plan_section.py env)"; PICAR_ROS_NETWORK=host \\
    ROS_IMAGE=vision-picar-ros:plan-3.156 python evaluations/rehearsal-fixes-3172/E/run.py OUT.jsonl

Exit 0: the schedule is finished; 3: run another batch (`run.sh` loops).

Amendment 1 (2026-11-11, before mission 27): a start the robot server refuses for
`battery_low` ends the batch as the batch timer does (exit 3), so the next batch is
a fresh stack: a fresh battery, the same map volume. The harness itself still exits
4 on any other refused start.

`--extra N OUT2.jsonl` runs the sheet's mission N once more into OUT2 (amendment 2:
mission 9 ran at load 5.95, over 3.79's ~4; the re-run is a kept extra row).
"""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

B = Path(__file__).resolve().parents[2] / "autonomy-3158" / "B"
sys.path.insert(0, str(B))

import rehearsal as R          # noqa: E402

R.VOLUME = "picar-maps-3172e"

if "--extra" in sys.argv:
    k = sys.argv.index("--extra")
    n, out2 = int(sys.argv[k + 1]), sys.argv[k + 2]
    sheet = json.loads((B / "sheet.json").read_text())
    sheet["schedule"] = [dict(sheet["schedule"][n], extra_of=n)]
    R.json = SimpleNamespace(loads=lambda s: sheet if isinstance(s, str) and '"schedule"' in s
                             else json.loads(s), dumps=json.dumps)
    sys.argv = [sys.argv[0], out2] + sys.argv[k + 3:]     # OUT.jsonl (argv[1]) is not this run's

battery_low = []
_mission = R.mission


def mission(*a, **kw):
    rec = _mission(*a, **kw)
    if "start" in rec and "battery_low" in str(rec.get("detail")):
        battery_low.append(rec)
    return rec


R.mission = mission
rc = R.main()
if rc == 4 and battery_low:
    print("amendment 1: battery_low at the start; a fresh stack next", flush=True)
    rc = 3
sys.exit(rc)
