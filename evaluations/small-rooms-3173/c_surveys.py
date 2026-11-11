#!/usr/bin/env python3
"""3.173.C: five live surveys on the saved map, through 3.158.B's harness.

    eval "$(python tools/plan_section.py env)"; PICAR_ROS_NETWORK=host \\
    ROS_IMAGE=vision-picar-ros:plan-3.156 python evaluations/small-rooms-3173/c_surveys.py OUT.jsonl --batch-s 1

`evaluations/autonomy-3158/B/rehearsal.py` unchanged: the house, its configuration,
stand-ins and truth recorder (rooms entered, contacts, falls, the return to the
dock). Only the schedule is five counted surveys, and the map volume is
`picar-maps-3173c`, a copy of 3.172.E's 45-mission saved map (`picar-maps-3172e`).
With `--batch-s 1` every survey is its own batch: a fresh stack and a full battery,
so no survey ends on a battery a find before it spent (3.172.E's mission 34).

Exit 0: all five done; 3: run the next batch (`c_run.sh` loops); 4: a refused start.
"""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

B = Path(__file__).resolve().parents[1] / "autonomy-3158" / "B"
sys.path.insert(0, str(B))

import rehearsal as R          # noqa: E402

R.VOLUME = "picar-maps-3173c"
SURVEYS = 5
sheet = json.loads((B / "sheet.json").read_text())
sheet["schedule"] = [{"job": "survey", "phase": "counted"} for _ in range(SURVEYS)]
R.json = SimpleNamespace(loads=lambda s: sheet if isinstance(s, str) and '"schedule"' in s
                         else json.loads(s), dumps=json.dumps)
sys.exit(R.main())
