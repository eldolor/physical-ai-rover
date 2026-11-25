# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/kill-stop-3216/progress_p2.py

3.216 criterion 3 (P2, amendment 2) and criterion 4's mission half: frontier
missions on `scaled_house` through the brain's HTTP API, on 3.135's stack
(the body `kill_body.py`, the robot server as `sweep.py` starts it, the
shipped brain), each stopped at `--window` seconds. Per mission: outcome,
the TRUE path travelled (summed from the truth) and path per minute; the
run's `sends.jsonl` is criterion 4's cadence (`analyse.py`).

    python evaluations/kill-stop-3216/progress_p2.py <out> [--shipped | --heartbeat-ms N --t-react S]
        [--missions 3] [--window 600]
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import sweep as sw  # noqa: E402  (puts the FMEA harness on the path)
from fmea import start_mission, status  # noqa: E402
from stack import Stack  # noqa: E402


class MissionStack(sw.KillStack):
    def start_brain(self) -> None:
        Stack.start_brain(self)               # the shipped brain this time


def path_m(rows: list) -> float:
    return sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(rows, rows[1:]))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--shipped", action="store_true")
    ap.add_argument("--heartbeat-ms", type=int, default=0)
    ap.add_argument("--t-react", type=float, default=0.0)
    ap.add_argument("--missions", type=int, default=3)
    ap.add_argument("--window", type=float, default=600.0)
    a = ap.parse_args(argv)
    out = Path(a.out)
    variant = {}
    if a.heartbeat_ms:
        variant["K3216_HEARTBEAT_MS"] = str(a.heartbeat_ms)
    if a.t_react:
        variant["K3216_T_REACT_S"] = str(a.t_react)
    st = MissionStack(out, a.shipped, variant)
    st.start()
    truth = sw.Truth(out / "truth.jsonl")
    rows_out = []
    try:
        for i in range(a.missions):
            sw.wait_ready(st)
            start_mission(st, "frontier")
            t0 = time.monotonic()
            s = {}
            while time.monotonic() - t0 < a.window:
                s = status(st)
                if s.get("running") is False and s.get("outcome") not in (None, "idle", "running"):
                    break
                time.sleep(2.0)
            t1 = time.monotonic()
            if s.get("running") is not False:
                st.brain("POST", "/mission/stop", {})
                time.sleep(2.0)
                s = status(st)
            rows = truth.window(t0, t1)
            p = path_m(rows)
            rec = {"i": i, "outcome": s.get("outcome"), "seconds": round(t1 - t0, 1),
                   "path_m": round(p, 2), "path_m_per_min": round(p / ((t1 - t0) / 60.0), 3),
                   "robot_alive": st.alive("robot")}
            rows_out.append(rec)
            print(json.dumps(rec), flush=True)
            with open(out / "missions.jsonl", "a") as f:
                f.write(json.dumps(rec) + "\n")
    finally:
        st.stop()
    print(json.dumps({"median_path_m_per_min": statistics.median(r["path_m_per_min"] for r in rows_out),
                      "found": sum(r["outcome"] == "found" for r in rows_out)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
