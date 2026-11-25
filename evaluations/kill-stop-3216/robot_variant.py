# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/kill-stop-3216/robot_variant.py

3.216's candidates, run without editing the shipped code: the robot server
with module constants set from the environment before its app is built.
Unset means the shipped value. Used only to compare candidates; the chosen
fix is then measured as shipped (`sweep.py --shipped`).

  K3216_HEARTBEAT_MS   robot.hardware_robot.HEARTBEAT_MS (candidate a)
  K3216_T_REACT_S      robot.safety.T_REACT_S (candidate b: the stop
                       distance's reaction time covers the board's own stop)

The two are independent on purpose: since 3.216 the shipped `T_REACT_S` is
derived from the shipped heartbeat once, at import, so a heartbeat
candidate keeps the shipped stop distance unless `K3216_T_REACT_S` is given
too (candidate (a) alone is exactly that).

    python evaluations/kill-stop-3216/robot_variant.py <port>
"""

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)

from robot import hardware_robot, safety  # noqa: E402

set_to = {}
if os.environ.get("K3216_HEARTBEAT_MS"):
    hardware_robot.HEARTBEAT_MS = set_to["HEARTBEAT_MS"] = int(os.environ["K3216_HEARTBEAT_MS"])
if os.environ.get("K3216_T_REACT_S"):
    safety.T_REACT_S = set_to["T_REACT_S"] = float(os.environ["K3216_T_REACT_S"])
print("robot variant: " + json.dumps({"set": set_to, "heartbeat_ms": hardware_robot.HEARTBEAT_MS,
                                      "t_react_s": safety.T_REACT_S}), file=sys.stderr, flush=True)

import robot.server as server  # noqa: E402  (builds the app with the values above)
import uvicorn  # noqa: E402

uvicorn.run(server.app, host="127.0.0.1", port=int(sys.argv[1]), log_level="warning")
