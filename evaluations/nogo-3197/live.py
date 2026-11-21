# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3197/live.py -- 3.197's live runs: 3.195's
`evaluations/nogo-3195/live.py` as it is (a fresh stack, `scaled_house`,
the three `sweep.place_zones("scaled_house", 0)` zones, Z6's taps, one find
with the SLAM pose beside every truth sample, the mission-end clear, then
3.193 D2's Z3), with ONE change: the bridge's log lines read for the
mission window also include 3.197's vent lines (`zone_vent`,
`zone_vent_moved`, `zone_vent_closed`), and the bridge's `/health` `nogo`
block is kept as it is (with `vents` / `vent_cells` on 3.197's image).

The rotated runs set SIM_START_TURN_DEG (sim/maps/__init__.py) in the
environment: the robot starts turned, so SLAM's map frame -- and every zone
the bridge draws -- lies at a slant to the costmap's lattice.

    eval "$(python tools/plan_section.py env)"
    PICAR_ROS_NETWORK=host ROS_IMAGE=vision-picar-ros:plan-3.197 \\
        PICAR_MAPS_VOLUME=<fresh> NOGO_DIR=<fresh> [SIM_START_TURN_DEG=30] \\
        python evaluations/nogo-3197/live.py OUT.json
"""

import importlib.util
import json
import os
import subprocess
import sys

ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))

_spec = importlib.util.spec_from_file_location(
    "live3195", os.path.join(ROOT, "evaluations", "nogo-3195", "live.py"))
L5 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(L5)

WORDS = ("zone_suspended", "zone_restored", "zone_margin_off", "zone_margin_on",
         "zone_vent")                     # zone_vent, zone_vent_moved, zone_vent_closed


def _zone_log(skip=0):
    """3.195's _zone_log with the vent's lines too (raises if docker cannot
    read the container's log: never an empty list for a missing one)."""
    out = subprocess.run(["docker", "logs", L5.de.CONTAINER], capture_output=True, text=True)
    if out.returncode != 0:
        raise RuntimeError(f"docker logs {L5.de.CONTAINER}: {out.stderr[-200:]}")
    lines = [ln.strip()[-160:] for ln in (out.stdout + out.stderr).splitlines()
             if any(w in ln for w in WORDS)]
    return lines[skip:]


L5._zone_log = _zone_log


if __name__ == "__main__":
    try:
        L5.main(sys.argv[1])
    finally:
        with open(sys.argv[1]) as fh:
            res = json.load(fh)
        res["start_turn_deg"] = os.environ.get("SIM_START_TURN_DEG")
        with open(sys.argv[1], "w") as fh:
            json.dump(res, fh, indent=1)
        print({"start_turn_deg": res["start_turn_deg"]})
