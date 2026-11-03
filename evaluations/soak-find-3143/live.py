# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/soak-find-3143/live.py

3.143 step B (criterion 2, amendment 3): one house/target pair live on the
desktop, through the brain's HTTP API on `tests/demo_explore.py`'s stack
(robot server, brain, the ROS container with SLAM and nav2), with A2's
tiers: `policy: tiered`, `tier_search: explore`, `explore_goal_policy:
rooms`, the $0 cloud stand-in (`evaluations/tiered-380/E/standin_cloud.py`)
as the vision service, and the local tier installed in the brain process
(`live_brain.py`): `truth` is ladder.py's TruthLocal (the frames the robot
server sends are the simulator's, detections included), `no` its NoLocal
(A3n's, fix V on). Each run to its end (<= 1,200 s). Recorded per run in
<out>/<house>-<target>-<arm>-<n>/: the mission's summary (`run.json`, from
`demo_explore.run_mission`: outcome, contacts, travel), the brain's status
with every log line, the true pose every 0.5 s (`truth.jsonl.gz`) and the
stand-in's calls.

    PICAR_ROS_NETWORK=host python evaluations/soak-find-3143/live.py \\
        <out> <house> <target> <truth|no> [runs]

Holds the desktop lock exclusively for its life (`demo_explore.stack()`).
"""

import gzip
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import httpx

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))

import tests.demo_explore as de  # noqa: E402

LIMIT_S = 1200                    # criterion 2: each run to its end, <= 1,200 s
MARGIN_S = 300                    # the loop waits past it for the brain's own end
MAX_STEPS = 2000                  # a backstop only; the time limit ends a run
ARMS = ("truth", "no")
REAL_POPEN = subprocess.Popen     # demo_explore's `subprocess` is this module: patched globally


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def brain_popen(arm):
    """`demo_explore.stack()` starts the brain as `uvicorn control.brain_server:app`;
    the same command, through live_brain.py, which installs the arm's local
    tier first. Every other process passes through untouched."""
    def popen(args, *a, **k):
        if isinstance(args, list) and "control.brain_server:app" in args:
            i = args.index("control.brain_server:app")
            args = [args[0], str(HERE / "live_brain.py"), arm, *args[i + 1:]]
        return REAL_POPEN(args, *a, **k)
    return popen


def track_truth(robot_url, path, stop):
    """The true pose every 0.5 s until `stop` is set."""
    c = httpx.Client(base_url=robot_url, timeout=5)
    with gzip.open(path, "wt") as f:
        while not stop.is_set():
            try:
                t = c.get("/world/truth").json()
                f.write(json.dumps({"t": round(time.time(), 2), "x_m": t["x_m"], "y_m": t["y_m"],
                                    "heading_deg": t["heading_deg"]}) + "\n")
            except Exception as e:  # noqa: BLE001 -- recorded, the run goes on
                f.write(json.dumps({"t": round(time.time(), 2), "error": str(e)[:200]}) + "\n")
            stop.wait(0.5)


def closest(truth_path, target_at):
    """The closest true approach (m) of the chassis centre to the target's cells."""
    best = math.inf
    with gzip.open(truth_path, "rt") as f:
        for line in f:
            r = json.loads(line)
            if "x_m" not in r:
                continue
            for cx, cy in target_at:
                best = min(best, math.dist((r["x_m"], r["y_m"]), ((cx + 0.5) * 0.3, (cy + 0.5) * 0.3)))
    return round(best, 2) if best < math.inf else None


class _Client(httpx.Client):
    """demo_explore's client with the mission's own time limit: `run_mission`
    sends its loop bound (MISSION_LIMIT_S) as the brain's limit, and the brain
    checks its limit only between steps (15-40 s apart under explore), so the
    loop must outlast it to see the brain's own end (review)."""

    def post(self, url, *a, json=None, **k):
        if url == "/mission/start" and json is not None:
            json = {**json, "mission_time_limit_s": LIMIT_S}
        return super().post(url, *a, json=json, **k)


def wait_up(url, proc, limit=30):
    end = time.monotonic() + limit
    while time.monotonic() < end:
        if proc.poll() is not None:
            raise RuntimeError(f"the cloud stand-in exited ({proc.returncode})")
        try:
            if httpx.get(url + "/health", timeout=1).status_code == 200:
                return
        except httpx.HTTPError:
            pass
        time.sleep(0.2)
    raise RuntimeError("the cloud stand-in never answered /health")


def _quietly(fn, *a):
    try:
        fn(*a)
    except Exception as e:  # noqa: BLE001 -- the next cleanup step still runs
        print(f"cleanup {getattr(fn, '__name__', fn)}: {type(e).__name__}: {e}", file=sys.stderr)


def one_run(out, house, target, arm, n):
    run_dir = out / f"{house.replace(':', '_')}-{target.replace(' ', '_')}-{arm}-{n}"
    run_dir.mkdir(parents=True, exist_ok=False)
    row = {"house": house, "target": target, "arm": arm, "run": n}
    stub_port = free_port()
    stub = subprocess.Popen([de.PY, str(ROOT / "evaluations/tiered-380/E/standin_cloud.py"),
                             str(stub_port), str(run_dir / "cloud_calls.jsonl")],
                            cwd=ROOT, stdout=open(run_dir / "standin.log", "w"),
                            stderr=subprocess.STDOUT)
    stop = threading.Event()
    tracker = None
    try:
        wait_up(f"http://127.0.0.1:{stub_port}", stub)
        os.environ.update(VISION_URL=f"http://127.0.0.1:{stub_port}", LOCAL_VLM_URL=f"stub://{arm}",
                          TIER_SEARCH="explore", EXPLORE_GOAL_POLICY="rooms",
                          SIM_ARRIVAL_CONFIRM="geometry")
        subprocess.Popen = brain_popen(arm)
        robot = de.stack(house)
        tracker = threading.Thread(target=track_truth,
                                   args=(f"http://127.0.0.1:{de.ROBOT}", run_dir / "truth.jsonl.gz", stop),
                                   daemon=True)
        tracker.start()
        row.update(de.run_mission(robot, house, policy="tiered", target=target, max_steps=MAX_STEPS))
        # The loop outlasting the brain's limit by MARGIN_S: a `running` here
        # is the harness's cut, not the mission's end.
        row["harness_cut"] = row.get("outcome") in (None, "running", "idle")
    except Exception as e:  # noqa: BLE001 -- recorded; the batch goes on
        row["error"] = f"{type(e).__name__}: {e}"
    finally:
        stop.set()
        if tracker:
            tracker.join()             # past its last request (<= 5 s): the gzip is closed
        subprocess.Popen = REAL_POPEN
        _quietly(de._kill_ports)
        row["standin_alive_at_end"] = stub.poll() is None
        _quietly(stub.terminate)
        _quietly(stub.wait, 10)
    if de.RUN_DIR and Path(de.RUN_DIR).is_dir():
        if row.get("status_file") and (Path(de.RUN_DIR) / row["status_file"]).exists():
            shutil.move(str(Path(de.RUN_DIR) / row["status_file"]), str(run_dir / "status.json"))
        shutil.copytree(de.RUN_DIR, run_dir / "stack")       # robot, brain and nav2 logs
    if (run_dir / "truth.jsonl.gz").exists():
        row["closest_m"] = closest(run_dir / "truth.jsonl.gz", row.get("target_at") or [])
    calls = run_dir / "cloud_calls.jsonl"
    row["cloud_calls"] = sum(1 for _ in open(calls)) if calls.exists() else 0
    (run_dir / "run.json").write_text(json.dumps(row, indent=1) + "\n")
    return row


def main(argv):
    out, house, target, arm = Path(argv[0]), argv[1], argv[2], argv[3]
    runs = int(argv[4]) if len(argv) > 4 else 2
    if arm not in ARMS:
        raise SystemExit(f"arm must be one of {ARMS}")
    if os.environ.get("PICAR_ROS_NETWORK") != "host":
        raise SystemExit("PICAR_ROS_NETWORK=host is needed on this machine")
    de.MISSION_LIMIT_S = LIMIT_S + MARGIN_S    # the loop's bound; the brain's is LIMIT_S
    de.httpx = SimpleNamespace(Client=_Client)
    out.mkdir(parents=True, exist_ok=True)
    for n in range(1, runs + 1):
        row = one_run(out, house, target, arm, n)
        print(json.dumps({k: row.get(k) for k in ("house", "target", "arm", "run", "outcome",
                                                  "seconds", "closest_m", "contact_samples",
                                                  "cloud_calls", "harness_cut", "error")}),
              flush=True)


if __name__ == "__main__":
    main(sys.argv[1:])
