# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.137 criterion 1, once through the live local stack.

A real robot server, brain and ROS container (tests/demo_explore.py's
`stack()`, which takes the desktop's exclusive lock), `scaled_house`, the sim
body's pack at 11.0 V draining 0.02 V/s. An `explore` mission through the
brain's HTTP API, then the robot waits while the pack drains on:

* Low (25 s): the mission ends `out_of_budget` on the brain's floor, and a
  second start is refused `battery_low`;
* Reserve (40 s): the brain's verb is refused `battery_reserve`, a person's
  tap goal too; a person's FORWARD runs at speed 30;
* Power-off (50 s): the shutdown steps really stop the brain (its port) and
  the ROS container, then the power-off stub writes one line. The order and
  times are read from `/battery` and the step log.

Run: PICAR_ROS_NETWORK=host python evaluations/battery-3137/ladder_live.py
(after `eval "$(python tools/plan_section.py env)"`).
"""

import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
OUT = Path(__file__).with_name("results-live.json")


def main():
    import tests.demo_explore as de
    log = Path(de.LOGDIR) / f"battery-steps-{int(time.time())}.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    os.environ.update(
        SIM_BATTERY_V="11.0", SIM_BATTERY_DRAIN_V_PER_S="0.02",
        BATTERY_SHUTDOWN_CMDS=json.dumps([
            ["brain", f"echo brain >> {log}; fuser -k {de.BRAIN}/tcp; sleep 1"],
            ["ros", f"echo ros >> {log}; docker stop -t 5 {de.CONTAINER} >/dev/null"],
            ["vlm", f"echo vlm >> {log}"],
        ]),
        BATTERY_POWEROFF_CMD=f"echo poweroff >> {log}")
    robot = de.stack("scaled_house")
    import httpx
    brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=30)
    rec = {"events": []}
    t0 = time.time()

    def note(what, **kw):
        b = robot.get("/battery").json()
        rec["events"].append({"t_s": round(time.time() - t0, 1), "what": what,
                              "v": b.get("v"), "level": b.get("level"), **kw})
        print(rec["events"][-1], flush=True)

    note("stack up")
    r = brain.post("/mission/start", json={"target_object": de.TARGET, "policy": "explore"})
    note("mission start", status=r.status_code)
    while True:
        s = brain.get("/mission/status").json()
        if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
            break
        time.sleep(0.5)
    note("mission ended", outcome=s.get("outcome"), end_reason=s.get("end_reason"))
    rec["mission"] = {"outcome": s.get("outcome"), "end_reason": s.get("end_reason")}

    def wait_level(level, limit=120):
        end = time.time() + limit
        while time.time() < end:
            b = robot.get("/battery").json()
            if b.get("level") == level:
                return b
            time.sleep(0.25)
        return None

    if wait_level("low", 30):
        r = brain.post("/mission/start", json={"target_object": de.TARGET, "policy": "explore"})
        note("start at low", status=r.status_code, detail=r.json().get("detail"))
    if wait_level("reserve"):
        truth0 = robot.get("/world/truth").json()
        r = robot.post("/action", json={"action": "FORWARD"}, headers={"x-driver": "brain"}).json()
        note("brain verb at reserve", reason=r.get("reason"))
        r = robot.post("/world/goal", json={"x_m": truth0["x_m"] + 0.6, "y_m": truth0["y_m"]},
                       headers={"x-driver": "twin-dpad"}).json()
        note("person goal at reserve", reason=r.get("reason"))
        truth1 = robot.get("/world/truth").json()
        r = robot.post("/action", json={"action": "FORWARD", "speed": 100, "duration": 0.5},
                       headers={"x-driver": "twin-dpad"}).json()
        truth2 = robot.get("/world/truth").json()
        note("person forward at reserve", executed=r.get("executed"),
             speed=(r.get("result") or {}).get("speed"),
             moved_cm=round(100 * ((truth2["x_m"] - truth1["x_m"]) ** 2
                                   + (truth2["y_m"] - truth1["y_m"]) ** 2) ** 0.5, 1),
             autonomous_motion_cm=round(100 * ((truth1["x_m"] - truth0["x_m"]) ** 2
                                                + (truth1["y_m"] - truth0["y_m"]) ** 2) ** 0.5, 1))
    b = wait_level("poweroff")
    end = time.time() + 120
    while time.time() < end:
        b = robot.get("/battery").json()
        if any(s["step"] == "poweroff" for s in b["ladder"]["shutdown"]):
            break
        time.sleep(0.5)
    note("power-off done", shutdown=b["ladder"]["shutdown"], poweroff_why=b["ladder"]["poweroff_why"])
    r = robot.post("/action", json={"action": "FORWARD"}, headers={"x-driver": "twin-dpad"}).json()
    note("person at power-off", reason=r.get("reason"))
    try:
        brain.get("/health", timeout=3)
        rec["brain_down"] = False
    except Exception:  # noqa: BLE001
        rec["brain_down"] = True
    import subprocess
    rec["ros_running"] = bool(subprocess.run(
        ["docker", "ps", "-q", "-f", f"name={de.CONTAINER}"],
        capture_output=True, text=True).stdout.strip())
    rec["steps_log"] = log.read_text().split()
    OUT.write_text(json.dumps(rec, indent=1))
    print("brain down", rec["brain_down"], "ros running", rec["ros_running"],
          "steps", rec["steps_log"])
    print("wrote", OUT)
    de._kill_ports()
    subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)


if __name__ == "__main__":
    main()
