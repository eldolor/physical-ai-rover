# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-perf-3136/plant_b.py

3.136 criterion 3 for the measurers 3.136.B builds (amendment 1), on the
desktop, each as it is and with 100 ms planted where it measures, read back
through the scorer the board run uses (`scorers.py`):

| Row | Measured by | Planted where |
|---|---|---|
| P12 `segment()` per call, p90 | the brain's timing shim (`shim/sitecustomize.py`) | inside `segment()` (PERF_SHIM_PLANT) |
| P12 walk per call, p90 | the same shim | inside `_walk_dist()` |
| P11 controller rate (mean period) | `ros_pace.py` in a ROS container, `scorers.controller_rate` | each 50 ms controller cycle (`plant_pace_talker.py`) |
| P11 scan lag p90 | `ros_pace.py`, `scorers.scan_lag` | between a scan's capture stamp and its publication |

The room work runs the brain's own code: find missions with
`find_room_labels` and `explore_goal_policy: rooms` in process
(`evaluations/goal-learn-396/train.py`'s `mission(..., "wired-ask")`, the
shipped MissionRunner and RoomLabeller) on ProcTHOR `val` houses, in a
child process that loads the shim as the board's brain does (PYTHONPATH).

    python evaluations/jetson-perf-3136/plant_b.py [--out DIR]

Run it under the desktop lock, exclusive (timing).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

import scorers  # noqa: E402
from plant import DELAY_MS, TOL, verdict  # noqa: E402

PY = sys.executable
IMAGE = os.environ.get("ROS_IMAGE", "vision-picar-ros:latest")
HOUSES = (12, 2, 9, 0, 4, 10, 15, 18, 3, 5, 11, 13)


def run_room(work: Path, plant: bool) -> list:
    log = work / f"shim-{'planted' if plant else 'base'}.jsonl"
    log.unlink(missing_ok=True)
    env = {**os.environ, "PERF_SHIM_LOG": str(log),
           "PYTHONPATH": os.pathsep.join([str(HERE / "shim"), str(ROOT / "evaluations/goal-learn-396"),
                                          str(ROOT)])}
    env.pop("PERF_SHIM_PLANT", None)
    if plant:
        env["PERF_SHIM_PLANT"] = "segment,walk"
    code = ("import json, train\n"
            f"for i in {HOUSES!r}:\n"
            "    r = train.mission('val', i, 'R', 'wired-ask')\n"
            "    print(json.dumps({k: r.get(k) for k in ('index', 'target', 'outcome', 'asks')}), flush=True)\n")
    r = subprocess.run([PY, "-c", code], cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=3600)
    (work / f"room-{'planted' if plant else 'base'}.log").write_text(r.stdout + r.stderr)
    if r.returncode != 0:
        raise RuntimeError(f"room missions failed: {r.stderr[-800:]}")
    return scorers.jsonl(log)


def check_room(work: Path) -> dict:
    base, planted = run_room(work, False), run_room(work, True)
    out = {}
    for kind in ("segment", "walk"):
        b = scorers.durations_ms(scorers.shim_spans(base, kind))
        p = scorers.durations_ms(scorers.shim_spans(planted, kind))
        v = verdict(scorers.percentile(b, 90), scorers.percentile(p, 90))
        out[kind] = {"statistic": "p90 ms per call", "n_base": len(b), "n_planted": len(p),
                     **v, "met": v["met"] and len(b) >= 200 and len(p) >= 200}
    # The pause scorer over each run's own span of calls (in-process sim time
    # runs faster than the wall, so only the per-call numbers are checked).
    for name, rows in (("base", base), ("planted", planted)):
        t0 = min(r["t0"] for r in rows)
        t1 = max(r["t1"] for r in rows)
        out[f"room_work_{name}"] = scorers.room_work(rows, t0, t1)
    return out


def run_pace(work: Path, plant_s: float, seconds: float) -> list:
    name = f"pace-{'planted' if plant_s else 'base'}.jsonl"
    (work / name).unlink(missing_ok=True)
    domain = os.environ.get("ROS_DOMAIN_ID", "136")
    cmd = (f"python3 /p/ros_pace.py /w/{name} {seconds + 6} & P=$!; sleep 1; "
           f"python3 /p/plant_pace_talker.py {seconds} {plant_s}; sleep 2; kill $P; wait $P")
    r = subprocess.run(["docker", "run", "--rm", "-e", f"ROS_DOMAIN_ID={domain}",
                        "-e", "ROS_LOCALHOST_ONLY=1", "-u", f"{os.getuid()}:{os.getgid()}",
                        "-e", "HOME=/tmp", "-v", f"{HERE}:/p:ro", "-v", f"{work}:/w", IMAGE,
                        "bash", "-c", cmd], capture_output=True, text=True, timeout=seconds + 120)
    (work / f"{name}.log").write_text(r.stdout + r.stderr)
    return scorers.jsonl(work / name)


def check_pace(work: Path, seconds: float) -> dict:
    base, planted = run_pace(work, 0.0, seconds), run_pace(work, DELAY_MS / 1000, seconds)
    cb, cp = scorers.controller_rate(base), scorers.controller_rate(planted)
    sb, sp = scorers.scan_lag(base), scorers.scan_lag(planted)
    out = {"controller": {"statistic": "mean period ms (1000 / pooled rate)", "base_rate": cb,
                          "planted_rate": cp}, "scan": {"statistic": "lag p90 ms", "base_lag": sb,
                                                        "planted_lag": sp}}
    if cb["mean_period_ms"] and cp["mean_period_ms"]:
        v = verdict(cb["mean_period_ms"], cp["mean_period_ms"])
        out["controller"].update(v, met=v["met"] and cb["intervals"] >= 200 and cp["intervals"] >= 200)
    else:
        out["controller"]["met"] = False
    if sb["p90_ms"] is not None and sp["p90_ms"] is not None:
        v = verdict(sb["p90_ms"], sp["p90_ms"])
        out["scan"].update(v, met=v["met"] and sb["n"] >= 200 and sp["n"] >= 200)
    else:
        out["scan"]["met"] = False
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(HERE / "desktop"))
    ap.add_argument("--pace-seconds", type=float, default=40.0)
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix="plant-3136b-"))
    os.chmod(work, 0o777)
    res = {"started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
           "rev": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True,
                                 text=True).stdout.strip(),
           "image": IMAGE, "delay_ms": DELAY_MS, "tolerance": TOL,
           "desktop_load_start": os.getloadavg(), "rows": {}}
    try:
        room = check_room(work)
        res["rows"]["P12 segment() per call"] = room["segment"]
        res["rows"]["P12 walk per call"] = room["walk"]
        res["room_work_scorer"] = {k: v for k, v in room.items() if k.startswith("room_work")}
        print("room", json.dumps(room), flush=True)
        pace = check_pace(work, a.pace_seconds)
        res["rows"]["P11 controller rate"] = pace["controller"]
        res["rows"]["P11 scan lag"] = pace["scan"]
        print("pace", json.dumps(pace), flush=True)
    finally:
        res["desktop_load_end"] = os.getloadavg()
        res["work"] = str(work)
        res["met"] = len(res["rows"]) == 4 and all(r.get("met") for r in res["rows"].values())
        (out / "planted-b.json").write_text(json.dumps(res, indent=1))
    print("criterion 3 (rows built in 3.136.B):", "MET" if res["met"] else "FAILED")
    return 0 if res["met"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
