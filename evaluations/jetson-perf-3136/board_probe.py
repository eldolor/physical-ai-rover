# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-perf-3136/board_probe.py

Two board-side loads for `steady.sh` (3.136.B), each running until its flag
file is removed:

    python board_probe.py missions <flag> <out.jsonl> <target> [<target> ...]
    python board_probe.py mappoll  <flag> <out.jsonl> [period_s]

* `missions` -- the full stack's mission load: tiered find missions, one
  after another through the brain's HTTP API, cycling through the targets
  (each ends as missions end; the next starts at once). One line per
  mission: its target, outcome, start and end, steps, room-label asks. At
  the flag's removal the running mission is stopped (`/mission/stop`).
* `mappoll` -- P12's `/world/map` reply time and bytes, as explore fetches
  it (ETag / 304), `scorers.MapPoller` (3.136.A's measurer and planted
  check), one line per request.

APP_SHARED_SECRET is read from the environment.
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import scorers  # noqa: E402

BRAIN = "http://127.0.0.1:8001/brain"
ROBOT = "http://127.0.0.1:8000"


def missions(flag: str, out: str, targets: list) -> int:
    import httpx

    h = {"x-app-secret": os.environ.get("APP_SHARED_SECRET", "")}
    b = httpx.Client(base_url=BRAIN, timeout=60, headers=h)
    i = 0
    with open(out, "a", buffering=1) as f:
        while os.path.exists(flag):
            target = targets[i % len(targets)]
            i += 1
            t0 = time.time()
            try:
                r = b.post("/mission/start", json={"target_object": target, "policy": "tiered"})
            except httpx.HTTPError as e:
                f.write(json.dumps({"target": target, "start_error": str(e), "t0": t0}) + "\n")
                time.sleep(5)
                continue
            if r.status_code != 200:
                f.write(json.dumps({"target": target, "start_status": r.status_code,
                                    "body": r.text[:300], "t0": t0}) + "\n")
                time.sleep(5)
                continue
            s, stopped = {}, False
            while True:
                if not os.path.exists(flag) and not stopped:
                    try:
                        b.post("/mission/stop")
                    except httpx.HTTPError:
                        pass
                    stopped = True
                try:
                    s = b.get("/mission/status").json()
                except (httpx.HTTPError, ValueError):
                    time.sleep(1)
                    continue
                if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
                    break
                if stopped and time.time() - t0 > 3600:
                    break
                time.sleep(1)
            rl = s.get("room_labels") or {}
            f.write(json.dumps({"target": target, "t0": round(t0, 3), "t1": round(time.time(), 3),
                                "outcome": s.get("outcome"), "step": s.get("step"),
                                "stopped_by_harness": stopped, "room_labels": rl,
                                "error": s.get("error")}) + "\n")
    return 0


def mappoll(flag: str, out: str, period_s: float) -> int:
    p = scorers.MapPoller(ROBOT, secret=os.environ.get("APP_SHARED_SECRET", ""), period_s=period_s)
    with open(out, "a", buffering=1) as f:
        while os.path.exists(flag):
            f.write(json.dumps(p.one()) + "\n")
            time.sleep(period_s)
    return 0


def main(argv) -> int:
    cmd = argv[0]
    if cmd == "missions":
        return missions(argv[1], argv[2], argv[3:])
    if cmd == "mappoll":
        return mappoll(argv[1], argv[2], float(argv[3]) if len(argv) > 3 else 1.0)
    raise SystemExit(f"unknown {cmd}")


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
