# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/action-budget-3208/live.py

3.208 criterion 6: missions against the live local stack -- the robot server
and the brain server as processes, the mission started and read through the
brain's HTTP API -- with reset_proxy.py between the brain and the robot.

    eval "$(python tools/plan_section.py env)"
    python tools/desktop_lock.py exclusive --why "3.208 live" -- \
        python evaluations/action-budget-3208/live.py out.json

Run A: two non-consecutive moves reset (the 5th before the robot sees it,
the 12th after the robot answered); expected `found`. Run B: every move from
the 3rd on reset; expected `failed`, "robot unreachable 3 times in a row".
Each run gets a fresh robot server (there is no reset endpoint, on purpose).
"""

import json
import os
import subprocess
import sys
import time
from pathlib import Path

import httpx

REPO = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
ROBOT = int(os.environ.get("PICAR_ROBOT_PORT", 11080))
BRAIN = int(os.environ.get("PICAR_BRAIN_PORT", 11081))
PROXY = int(os.environ.get("PICAR_BRIDGE_PORT", 11085))
LOGDIR = Path(os.environ.get("EXPLORE_LOGDIR", "/tmp/vision-picar-3.208"))
RUNS = {
    "A_two_resets": {"resets": {"5": "before", "12": "after"}},
    "B_link_gone": {"from": 3},
}


def wait(url, timeout=60):
    t = time.time() + timeout
    while time.time() < t:
        try:
            if httpx.get(url, timeout=2).status_code == 200:
                return
        except httpx.HTTPError:
            pass
        time.sleep(0.3)
    raise RuntimeError(f"{url} never answered")


def start(cmd, env, log):
    return subprocess.Popen(cmd, cwd=REPO, env=env, stdout=open(log, "w"),
                            stderr=subprocess.STDOUT)


def one(name, plan):
    env = {**os.environ, "ROBOT_MODE": "sim", "WORLD_MODE": "sim", "SIM_MAP": "starter_house"}
    py = sys.executable
    procs = [
        start([py, "-m", "uvicorn", "robot.server:app", "--port", str(ROBOT)], env,
              LOGDIR / f"{name}-robot.log"),
        start([py, str(HERE / "reset_proxy.py"), str(PROXY), f"http://127.0.0.1:{ROBOT}"], env,
              LOGDIR / f"{name}-proxy.log"),
    ]
    try:
        wait(f"http://127.0.0.1:{ROBOT}/health")
        benv = {**env, "ROBOT_URL": f"http://127.0.0.1:{PROXY}"}
        procs.append(start([py, "-m", "uvicorn", "control.brain_server:app", "--port", str(BRAIN)],
                           benv, LOGDIR / f"{name}-brain.log"))
        wait(f"http://127.0.0.1:{BRAIN}/health")
        httpx.post(f"http://127.0.0.1:{PROXY}/__plan", json=plan, timeout=5).raise_for_status()
        t0 = time.time()
        r = httpx.post(f"http://127.0.0.1:{BRAIN}/mission/start",
                       json={"target_object": "red backpack", "policy": "frontier"}, timeout=30)
        r.raise_for_status()
        peak = 0
        status = {}
        while time.time() - t0 < 600:
            status = httpx.get(f"http://127.0.0.1:{BRAIN}/mission/status", timeout=10).json()
            peak = max(peak, status.get("action_failures") or 0)
            if not status.get("running"):
                break
            time.sleep(0.25)
        wall = time.time() - t0
        wire = httpx.get(f"http://127.0.0.1:{PROXY}/__plan", timeout=5).json()["wire"]
    finally:
        for p in reversed(procs):
            p.terminate()
        for p in procs:
            try:
                p.wait(10)
            except subprocess.TimeoutExpired:
                p.kill()
    resets = [i for i, w in enumerate(wire) if w["how"]]
    # what crossed the wire right after each reset POST
    after = [wire[i + 1]["action"] if i + 1 < len(wire) else None for i in resets]
    # the same move twice in a row with no stop between would be a re-send
    resent = [i for i in resets if i + 1 < len(wire) and wire[i + 1]["action"] == wire[i]["action"]]
    log = status.get("log_tail") or []
    return {
        "run": name, "plan": plan, "outcome": status.get("outcome"),
        "end_reason": status.get("error") or status.get("end_reason"),
        "steps": status.get("step"), "wall_s": round(wall, 1),
        "action_failures_peak_seen": peak,
        "unreachable_lines": [line for line in log if "robot unreachable" in line],
        "moves_on_wire": sum(1 for w in wire if w["n"]),
        "resets": [wire[i] for i in resets],
        "next_post_after_each_reset": after,
        "resent_same_action_next": len(resent),
        "log_tail": log,
    }


def main():
    LOGDIR.mkdir(parents=True, exist_ok=True)
    out = [one(name, plan) for name, plan in RUNS.items()]
    for r in out:
        print(json.dumps({k: v for k, v in r.items() if k != "log_tail"}, indent=1))
    if len(sys.argv) > 1:
        Path(sys.argv[1]).write_text(json.dumps(out, indent=1) + "\n")


if __name__ == "__main__":
    main()
