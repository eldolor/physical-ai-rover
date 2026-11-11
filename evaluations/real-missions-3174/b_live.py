#!/usr/bin/env python3
"""3.174.B's live measurement: the recorder beside 3.158.B's harness, and a control without it.

    eval "$(python tools/plan_section.py env)"; PICAR_ROS_NETWORK=host \\
    ROS_IMAGE=vision-picar-ros:plan-3.156 python evaluations/real-missions-3174/b_live.py \\
        OUT.jsonl --arm rec|control --batch-s 1

`evaluations/autonomy-3158/B/rehearsal.py` unchanged (the house, stand-ins and truth
recorder), on volume `picar-maps-3174b`, a copy of 3.173.C's saved map. The schedule:
`rec` 4 finds and 2 surveys with the recorder running in this process, pointed at the
stack's ports as it would be at the car's; `control` the same 4 finds without it.
Added to each harness row, read by the harness (never by the recorder): the mission's
id, the robot server's wheel-loop deltas (B6) and the truth track's length (B3). The
truth track is kept locally for the scorer (`~/picar-kept/3174b/truth/`), never committed.

Exit 0: all done; 3: run the next batch (`b_run.sh` loops); 4: a refused start.
"""
import argparse
import importlib.util
import json
import math
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import httpx

HERE = Path(__file__).resolve().parent
B = HERE.parents[0] / "autonomy-3158" / "B"
sys.path.insert(0, str(B))
import rehearsal as R          # noqa: E402

_spec = importlib.util.spec_from_file_location("recorder_3174", HERE / "recorder.py")
rec = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rec)

KEPT = Path("~/picar-kept/3174b").expanduser()
FINDS = ["breakfast table", "red backpack", "toilet", "vanity"]
ARMS = {
    "rec": [{"job": "find", "target": FINDS[0], "present": True},
            {"job": "find", "target": FINDS[1], "present": True},
            {"job": "survey"},
            {"job": "find", "target": FINDS[2], "present": True},
            {"job": "find", "target": FINDS[3], "present": True},
            {"job": "survey"}],
    "control": [{"job": "find", "target": t, "present": True} for t in FINDS],
}

ap = argparse.ArgumentParser()
ap.add_argument("out")
ap.add_argument("--arm", choices=sorted(ARMS), required=True)
a, rest = ap.parse_known_args()
sys.argv = [sys.argv[0], a.out, *rest]

R.VOLUME = "picar-maps-3174b"
sheet = json.loads((B / "sheet.json").read_text())
sheet["schedule"] = [{**m, "phase": "counted"} for m in ARMS[a.arm]]
R.json = SimpleNamespace(loads=lambda s: sheet if isinstance(s, str) and '"schedule"' in s
                         else json.loads(s), dumps=json.dumps)

_init, _sample, _mission = R.Truth.__init__, R.Truth.sample, R.mission
_last = {}


def init(self, house, robot):
    _init(self, house, robot)
    self.track = []
    _last["truth"] = self


def sample(self):
    t = _sample(self)
    self.track.append([round(time.time(), 2), t["x_m"], t["y_m"], t["heading_deg"]])
    return t


def mission(robot, brain, house, m):
    th = None
    if a.arm == "rec":
        headers = dict(brain.headers)
        rob = httpx.Client(base_url=str(robot.base_url), timeout=5, headers=headers)
        brn = httpx.Client(base_url=str(brain.base_url), timeout=5, headers=headers)
        r = rec.Recorder(rob, brn, KEPT / "batch-rec",
                         # the brain runs in the work tree (`demo_explore.stack`), at
                         # brain_config's default `intervention_dir`
                         interventions=R.ROOT / "recordings" / "interventions" / "interventions.jsonl")
        th = threading.Thread(target=r.run, kwargs={"missions": 1}, daemon=True)
        th.start()
    h0 = robot.get("/health").json()
    out = _mission(robot, brain, house, m)
    if "start" in out:
        return out
    h1 = robot.get("/health").json()
    if th is not None:
        th.join(timeout=rec.END_GRACE_S + 60)
        out["recorder_joined"] = not th.is_alive()
    s = brain.get("/mission/status").json()
    out["mission_id"] = s.get("mission_id")
    out["camera_failures"] = s.get("camera_failures")
    w0, w1 = h0.get("wheel_loop") or {}, h1.get("wheel_loop") or {}
    out["late_ticks"] = w1.get("late_ticks", 0) - w0.get("late_ticks", 0)
    out["moving_ticks"] = w1.get("moving_ticks", 0) - w0.get("moving_ticks", 0)
    tr = _last.get("truth")
    if tr is not None:
        out["truth_dist_m"] = round(sum(math.hypot(q[1] - p[1], q[2] - p[2])
                                        for p, q in zip(tr.track, tr.track[1:])), 2)
        (KEPT / "truth").mkdir(parents=True, exist_ok=True)
        (KEPT / "truth" / f"{out['mission_id']}.json").write_text(json.dumps(tr.track))
    out["arm"] = a.arm
    return out


R.Truth.__init__, R.Truth.sample, R.mission = init, sample, mission
sys.exit(R.main())
