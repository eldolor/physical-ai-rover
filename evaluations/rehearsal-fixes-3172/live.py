#!/usr/bin/env python3
"""3.172's live check: a few missions off 3.158.B's harness (its stand-ins, its truth,
its stack), a schedule of the caller's, the map in this section's own volume.

    python evaluations/rehearsal-fixes-3172/live.py OUT.jsonl find:toilet [survey] ...
"""
import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace

B = Path(__file__).resolve().parents[1] / "autonomy-3158" / "B"
sys.path.insert(0, str(B))
os.environ.setdefault("PICAR_ROS_NETWORK", "host")

import rehearsal as R          # noqa: E402

R.VOLUME = "picar-maps-3172"
sheet = json.loads((B / "sheet.json").read_text())
out, jobs = sys.argv[1], sys.argv[2:]
sheet["schedule"] = [{"job": "survey", "phase": "counted"} if j == "survey"
                     else {"job": "find", "target": j.split(":", 1)[1], "present": True, "phase": "counted"}
                     for j in jobs]
R.json = SimpleNamespace(loads=lambda s: sheet if isinstance(s, str) and '"schedule"' in s else json.loads(s), dumps=json.dumps)
sys.argv = [sys.argv[0], out, "--batch-s", "100000"]
sys.exit(R.main())
