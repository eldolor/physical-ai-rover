# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.225 criterion 4: one survey through the brain's HTTP API on the full
ROS stack (`tests/demo_explore.stack()`), the return to the dock on and a
simulated pack that sags under the motors. Prints one JSON line: the
outcome, the end reason and the battery readings the brain saw.

    eval "$(python tools/plan_section.py env)"
    PICAR_ROS_NETWORK=host python evaluations/battery-sag-3225/live_survey.py
"""
import httpx
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

# A Rover-like pack, before the stack starts (the robot server reads these):
# 11.8 V, the real drain (~2.1 V an hour), a 0.5 V sag for 5 s after the
# wheels start, a reading refreshed every 1 s.
os.environ.update(SIM_BATTERY_V="11.8", SIM_BATTERY_DRAIN_V_PER_S="0.0006",
                  SIM_BATTERY_SAG_V="0.5", SIM_BATTERY_SAG_S="5.0",
                  SIM_BATTERY_REFRESH_S="1.0")

from tests import demo_explore as de   # noqa: E402

LIMIT_S = 300.0


def main():
    de.stack("scaled_house")
    brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
    try:
        r = brain.post("/mission/start", json={"job": "survey", "policy": "explore",
                                               "mission_time_limit_s": LIMIT_S})
        r.raise_for_status()
        seen, t0, s = [], time.time(), {}
        while time.time() - t0 < LIMIT_S + 240:
            s = brain.get("/mission/status").json()
            v = ((s.get("budget") or {}).get("battery_v"))
            if v is not None and (not seen or seen[-1][1] != v):
                seen.append((round(time.time() - t0, 1), v))
            if not s.get("running"):
                break
            time.sleep(2.0)
        out = {"outcome": s.get("outcome"), "end_reason": s.get("end_reason"),
               "elapsed_s": (s.get("budget") or {}).get("elapsed_s"),
               "home": (s.get("budget") or {}).get("home"),
               "return": (s.get("return") or {}).get("state"),
               "battery_min_v": min((v for _, v in seen), default=None),
               "battery_max_v": max((v for _, v in seen), default=None),
               "battery_readings": len(seen), "run_dir": de.RUN_DIR}
        print(json.dumps(out))
    finally:
        de._kill_ports()


if __name__ == "__main__":
    main()
