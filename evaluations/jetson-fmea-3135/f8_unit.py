# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/f8_unit.py

F8's second case (amendment 4), ON the board, the twin's units as found:
SIGKILL the production `picar-stack`'s robot server (found in the unit's own
cgroup by its command line, so nothing else is touched) and time it to its
`/health` answering 200 again on :8000 -- `run.sh` exits when any server
does (3.71) and systemd brings the unit back (`Restart=on-failure`). Bar:
<= 10 s. A run whose unit was not active, or whose robot server did not
answer, before the kill is invalid. No sudo: the unit runs as this user.

    python evaluations/jetson-fmea-3135/f8_unit.py <out> [--runs 5]
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
import urllib.request
from pathlib import Path

UNIT = "picar-stack"
HEALTH = "http://127.0.0.1:8000/health"
BAR_S = 10.0
WAIT_S = 120.0
SETTLE_S = 30.0                # between runs: the brain and ngrok back too


def show(prop: str) -> str:
    return subprocess.run(["systemctl", "show", "-p", prop, "--value", UNIT], capture_output=True,
                          text=True, timeout=10).stdout.strip()


def robot_pid() -> int | None:
    cg = show("ControlGroup")
    try:
        pids = Path(f"/sys/fs/cgroup{cg}/cgroup.procs").read_text().split()
    except OSError:
        return None
    for pid in pids:
        try:
            cmd = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode()
        except OSError:
            continue
        if "uvicorn" in cmd and "robot.server:app" in cmd:
            return int(pid)
    return None


def healthy() -> bool:
    try:
        with urllib.request.urlopen(HEALTH, timeout=1.0) as r:
            return r.status == 200
    except OSError:
        return False


def one(i: int) -> dict:
    rec = {"i": i, "active_before": show("ActiveState"), "restarts_before": show("NRestarts"),
           "healthy_before": healthy()}
    pid = robot_pid()
    rec["pid"] = pid
    if rec["active_before"] != "active" or not rec["healthy_before"] or pid is None:
        rec["valid"] = False
        return rec
    rec["valid"] = True
    t0 = time.monotonic()
    os.kill(pid, signal.SIGKILL)
    down_seen = False
    while time.monotonic() - t0 < WAIT_S:
        h = healthy()
        if not h:
            down_seen = True
        elif down_seen or robot_pid() not in (None, pid):
            rec["back_s"] = round(time.monotonic() - t0, 2)
            break
        time.sleep(0.1)
    rec["down_seen"] = down_seen
    rec["new_pid"] = robot_pid()
    rec["restarts_after"] = show("NRestarts")
    rec["active_after"] = show("ActiveState")
    b = rec.get("back_s")
    rec["pass"] = b is not None and b <= BAR_S
    return rec


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--runs", type=int, default=5)
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    end = time.monotonic() + 180
    while not healthy() and time.monotonic() < end:       # the unit as found, warmed up
        time.sleep(1.0)
    recs = []
    with open(out / "f8_unit.jsonl", "a", buffering=1) as f:
        for i in range(1, a.runs + 4):
            if sum(1 for r in recs if r.get("valid")) >= a.runs:
                break
            r = one(i)
            recs.append(r)
            f.write(json.dumps(r) + "\n")
            print(json.dumps(r), flush=True)
            time.sleep(SETTLE_S)
    s = [r for r in recs if r.get("valid")][:a.runs]
    state = "FAILED" if any(not r["pass"] for r in s) else "met" if len(s) == a.runs else "incomplete"
    v = {"row": "F8-unit", "state": state, "runs_wanted": a.runs, "runs_scored": len(s),
         "back_s": [r.get("back_s") for r in s], "bar_s": BAR_S}
    (out / "row.json").write_text(json.dumps(v, indent=1))
    print(json.dumps(v))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
