# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/local-vlm-3152/live.py

3.152 criterion 9: one tiered mission through the brain's HTTP API on the
live local stack -- the robot server (sim, `scaled_house`), the brain, the
$0 stand-in cloud (`evaluations/tiered-380/E/standin_cloud.py`) and a
stand-in VE (`standin_ve.py`: VE's own service app over its fake driver) --
with VE stopped mid-mission and started again. Read: `local_vlm` goes
`down` then `up`, VE answers again after `up` (the tier's local answers
grow), the mission ends with an outcome, 0 contacts by ground truth
(`tests/footprint_sweep.py`'s geometry, as `tests/demo_explore.py`).

Ports are 3.152's (`plan_section.py env`). Take the desktop lock first:

    python tools/desktop_lock.py exclusive --why "3.152 live" -- \
        python evaluations/local-vlm-3152/live.py OUT_DIR
"""

import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import httpx

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from sim.grid_world import CELL_WALL  # noqa: E402
from sim.maps import build_world  # noqa: E402
from tests.footprint_sweep import CELL_CM, chassis, gap, penetration, square  # noqa: E402

ROBOT, BRAIN, CLOUD, VE = 10520, 10521, 10527, 10526
HOUSE, TARGET = "scaled_house", "red backpack"
LIMIT_S = 600
KILL_AT_STEP = 4          # VE is stopped once the mission has run this many steps
DOWN_FOR_S = 15           # and started again this long after the brain reads it down


def _start(cmd, log, env=None):
    return subprocess.Popen(cmd, cwd=ROOT, stdout=open(log, "w"), stderr=subprocess.STDOUT,
                            env={**os.environ, **(env or {})}, start_new_session=True)


def _stop(p):
    if p is not None and p.poll() is None:
        os.killpg(p.pid, signal.SIGTERM)
        try:
            p.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)


def _wait_http(url, timeout_s=60, ok=lambda r: r.status_code == 200):
    end = time.time() + timeout_s
    while time.time() < end:
        try:
            if ok(httpx.get(url, timeout=2)):
                return True
        except httpx.HTTPError:
            pass
        time.sleep(0.5)
    return False


def main(out_dir):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    py = sys.executable
    procs = {}
    events = []

    def note(what, **kw):
        events.append({"t": round(time.time(), 2), "what": what, **kw})
        print(what, kw, flush=True)

    def start_ve(n):
        procs["ve"] = _start([py, "evaluations/local-vlm-3152/standin_ve.py", str(VE),
                              str(out / "ve-work")], out / f"ve-{n}.log")
        assert _wait_http(f"http://127.0.0.1:{VE}/health",
                          ok=lambda r: r.json().get("ready") is True), "stand-in VE not ready"
        note("ve_ready", n=n)

    try:
        procs["robot"] = _start([py, "-m", "uvicorn", "robot.server:app", "--port", str(ROBOT)],
                                out / "robot.log", {"SIM_MAP": HOUSE})
        procs["cloud"] = _start([py, "evaluations/tiered-380/E/standin_cloud.py", str(CLOUD),
                                 str(out / "cloud-calls.jsonl")], out / "cloud.log")
        start_ve(1)
        procs["brain"] = _start(
            [py, "-m", "uvicorn", "control.brain_server:app", "--port", str(BRAIN)],
            out / "brain.log",
            {"ROBOT_URL": f"http://127.0.0.1:{ROBOT}", "VISION_URL": f"http://127.0.0.1:{CLOUD}",
             "LOCAL_VLM_URL": f"http://127.0.0.1:{VE}", "PICAR_LOG_LEVEL": "INFO"})
        assert _wait_http(f"http://127.0.0.1:{ROBOT}/health"), "robot server not up"
        assert _wait_http(f"http://127.0.0.1:{BRAIN}/health"), "brain not up"

        robot = httpx.Client(base_url=f"http://127.0.0.1:{ROBOT}", timeout=30)
        brain = httpx.Client(base_url=f"http://127.0.0.1:{BRAIN}", timeout=60)
        world = build_world(HOUSE)
        walls = {(x, y) for y, row in enumerate(world.layout) for x, c in enumerate(row)
                 if c == CELL_WALL}
        r = brain.post("/mission/start", json={"target_object": TARGET, "policy": "tiered",
                                               "mission_time_limit_s": LIMIT_S})
        assert r.status_code == 200, r.text[:300]
        note("started")
        t0, contacts, inside, samples, min_gap = time.time(), 0, 0, 0, math.inf
        states, killed, restarted, local_at_up = [], None, None, None
        status = {}
        while time.time() - t0 < LIMIT_S + 60:
            truth = robot.get("/world/truth").json()
            objs = {(o["x"], o["y"]) for o in robot.get("/sim/objects").json()["objects"]}
            x, y = truth["x_m"] / 0.3, truth["y_m"] / 0.3
            A = chassis(x, y, math.radians(truth["heading_deg"]) - math.pi / 2)
            for c in walls | objs:
                if abs(c[0] + 0.5 - x) > 1.6 or abs(c[1] + 0.5 - y) > 1.6:
                    continue
                g = gap(A, square(*c)) * CELL_CM
                min_gap = min(min_gap, g)
                contacts += g < 1.0
                inside += penetration(A, square(*c)) > 0
            samples += 1
            status = brain.get("/mission/status").json()
            st = (status.get("tier") or {}).get("stats") or {}
            lv = (st.get("local_vlm") or {}).get("state")
            answered = sum(v for k, v in (st.get("local_frame") or {}).items() if k != "absent")
            if not states or states[-1][1] != lv:
                states.append((round(time.time() - t0, 1), lv))
                note("local_vlm", state=lv, step=status.get("step"), local_answers=answered)
            if killed is None and (status.get("step") or 0) >= KILL_AT_STEP:
                _stop(procs.pop("ve"))
                killed = time.time()
                note("ve_killed", step=status.get("step"))
            if killed and restarted is None and lv == "down" and time.time() - killed >= DOWN_FOR_S:
                start_ve(2)
                restarted = time.time()
            if restarted and lv == "up" and local_at_up is None:
                local_at_up = answered
            if not status.get("running") and status.get("outcome") not in (None, "idle", "running"):
                break
            time.sleep(0.5)
        else:
            brain.post("/mission/stop")
            status = brain.get("/mission/status").json()
        st = (status.get("tier") or {}).get("stats") or {}
        answered_end = sum(v for k, v in (st.get("local_frame") or {}).items() if k != "absent")
        cloud_calls = sum(1 for _ in open(out / "cloud-calls.jsonl"))
        seq = [s for _, s in states]
        rec = {
            "house": HOUSE, "target": TARGET, "outcome": status.get("outcome"),
            "steps": status.get("step"), "seconds": round(time.time() - t0),
            "local_vlm_states": states, "local_vlm_final": st.get("local_vlm"),
            "local_frame": st.get("local_frame"), "local_answers_at_up": local_at_up,
            "local_answers_end": answered_end, "stub_cloud_navigate_calls": cloud_calls,
            "tier_cloud_calls": st.get("cloud_calls"),
            "min_gap_cm": round(min_gap, 1), "contact_samples": contacts,
            "inside_samples": inside, "samples": samples, "events": events,
            "met": {
                "down_then_up": "down" in seq and "up" in seq[seq.index("down"):],
                "ve_answers_after_up": (local_at_up is not None
                                        and answered_end > local_at_up),
                "outcome": status.get("outcome") not in (None, "idle", "running"),
                "zero_contacts": contacts == 0 and inside == 0,
            },
        }
        (out / "live.json").write_text(json.dumps(rec, indent=1) + "\n")
        print(json.dumps({k: rec[k] for k in ("outcome", "steps", "local_vlm_states", "met")}))
        return 0 if all(rec["met"].values()) else 1
    finally:
        for p in procs.values():
            _stop(p)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1]))
