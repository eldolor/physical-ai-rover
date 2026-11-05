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


# Amendment 6: at most this many status reads (1 s apart) after a mission
# ends, for its end's own goal cancel (its thread starts after `running` ends).
END_CANCEL_READS = 30


def missions(flag: str, out: str, targets: list) -> int:
    import httpx

    h = {"x-app-secret": os.environ.get("APP_SHARED_SECRET", "")}
    # A new target's first build may take a minute or more (3.81 T1); a
    # client timeout would leave the mission running unrecorded.
    b = httpx.Client(base_url=BRAIN, timeout=600, headers=h)
    i = 0
    # Amendment 5 (3.136.G): each send's goal id and kind, from status.
    sends_path = os.path.join(os.path.dirname(out) or ".", "explore_sends.jsonl")
    # Amendment 6 (3.136.B): the cancels the brain sent, by goal id.
    cancels_path = os.path.join(os.path.dirname(out) or ".", "explore_cancels.jsonl")
    with open(out, "a", buffering=1) as f, open(sends_path, "a", buffering=1) as sends, \
            open(cancels_path, "a", buffering=1) as cancels:
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
            s, stopped, seen, seen_c = {}, False, 0, 0

            def take_cancels(ex):
                # [n, goal_id, wall time, bridge answered cancelled], numbered as the sends are.
                nonlocal seen_c
                new_c = [e for e in ex.get("cancels") or [] if e[0] > seen_c]
                if new_c:
                    missing_c = new_c[0][0] - seen_c - 1
                    seen_c = new_c[-1][0]
                    cancels.write(json.dumps({"t": round(time.time(), 3), "mission": i,
                                              "cancels": new_c, "missing": missing_c}) + "\n")
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
                ex = s.get("explore") or {}
                # Each entry is [n, goal_id, kind]: take those numbered past
                # the last one written; a jump past the next number is sends
                # the 64-entry window lost before a poll saw them.
                new = [e for e in ex.get("sent") or [] if e[0] > seen]
                if new:
                    missing = new[0][0] - seen - 1
                    seen = new[-1][0]
                    sends.write(json.dumps({"t": round(time.time(), 3), "mission": i,
                                            "sent": [e[1:] for e in new],
                                            "missing": missing}) + "\n")
                take_cancels(ex)
                if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
                    break
                if stopped and time.time() - t0 > 3600:
                    break
                time.sleep(1)
            # The mission's end cancels its own goal on a thread of its own,
            # after `running` turns false: read its cancels until the gate says
            # it closed (every cancel recorded), before the next start replaces
            # the runner (review); bounded, and the row says if it was seen.
            end_seen, no_explore, explored = False, 0, False
            for _ in range(END_CANCEL_READS):
                time.sleep(1)
                try:
                    st = b.get("/mission/status", timeout=5).json()
                except (httpx.HTTPError, ValueError):
                    continue
                ex = st.get("explore") if isinstance(st, dict) else None
                if not ex:
                    no_explore += 1
                    if no_explore >= 2:     # no explore search: no goals, no cancels
                        if not explored:
                            end_seen = None
                        break
                    continue
                no_explore, explored = 0, True
                take_cancels(ex)
                if ex.get("goal_end_done") is True:
                    end_seen = True
                    break
            rl = s.get("room_labels") or {}
            f.write(json.dumps({"target": target, "t0": round(t0, 3), "t1": round(time.time(), 3),
                                "outcome": s.get("outcome"), "step": s.get("step"),
                                "stopped_by_harness": stopped, "room_labels": rl,
                                # Amendment 6: True, the gate's close seen (every cancel
                                # taken); None, no explore block in any read; False otherwise
                                # (the reads ran out or failed, or the block went away).
                                "cancels_end_seen": end_seen,
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
