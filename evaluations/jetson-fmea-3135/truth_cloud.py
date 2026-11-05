# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/truth_cloud.py

3.135.B amendment 17: F3a2's $0 stand-in cloud, answering as a correct cloud
would, from the simulator's ground truth. The arrival's cloud call is an
ordinary `/navigate` with nothing to tell it from a frame question, so this
stand-in answers every question by what is truly in front of the robot:

* the red backpack visible (`target_visible: true`, centred, FORWARD) when the
  body's true pose (`truth_body.py` `GET /fmea/now`) is within `NEAR_M` of
  the target's centre and the target lies within `HALF_FOV_DEG` of the
  camera's true heading (`cam_th`: the body's plus the pan);
* otherwise the 3.80.E stand-in's answer ("not visible", RIGHT).

Each `/navigate` is one JSON line in the log, with the answer and the true
distance and bearing it was given on (`pose: null` when the body did not
answer: the harness reads those as the instrument's failure). Known limits:
no line-of-sight test, and the pose is the one at the request, not at the
frame's capture (an arrival stands still). `scaled_house` only (amendment 17).

    python evaluations/jetson-fmea-3135/truth_cloud.py <port> <calls.jsonl> <body_url>
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from sim.grid_world import FOOTPRINT_CELL_M as CELL_M  # noqa: E402
from sim.maps.scaled_house import OBJECTS  # noqa: E402

TARGET = "red backpack"
TARGET_CELL = next(c for c, name in OBJECTS.items() if name == TARGET)
TARGET_XY = ((TARGET_CELL[0] + 0.5) * CELL_M, (TARGET_CELL[1] + 0.5) * CELL_M)
NEAR_M = 1.0
HALF_FOV_DEG = 20.0

NOT_VISIBLE = {"target_visible": False, "target_direction": "not_visible", "target_reached": False,
               "obstacle_ahead": False, "room_guess": "unclear", "action": "RIGHT",
               "reasoning": "stand-in (truth): not in view", "distance_estimate": "unknown"}
VISIBLE = {"target_visible": True, "target_direction": "center", "target_reached": False,
           "obstacle_ahead": False, "room_guess": "kitchen", "action": "FORWARD",
           "reasoning": "stand-in (truth): the target is ahead", "distance_estimate": "near"}


def answer(pose: dict | None) -> tuple[dict, dict]:
    """The reply for one question, and what it was decided on (pure)."""
    try:
        x, y, heading = float(pose["x"]), float(pose["y"]), float(pose["cam_th"])
    except (TypeError, KeyError, ValueError):
        return NOT_VISIBLE, {"pose": None}
    dx, dy = TARGET_XY[0] - x, TARGET_XY[1] - y
    dist = math.hypot(dx, dy)
    off = math.degrees(math.atan2(dy, dx) - heading)
    off = (off + 180.0) % 360.0 - 180.0
    seen = dist <= NEAR_M and abs(off) <= HALF_FOV_DEG
    return (VISIBLE if seen else NOT_VISIBLE), {"dist_m": round(dist, 3), "off_deg": round(off, 1)}


def main() -> None:
    port, log, body = int(sys.argv[1]), sys.argv[2], sys.argv[3].rstrip("/")
    out = open(log, "a", buffering=1)
    lock = threading.Lock()

    def pose() -> dict | None:
        try:
            with urllib.request.urlopen(body + "/fmea/now", timeout=1.0) as r:
                return json.loads(r.read())
        except (OSError, ValueError):
            return None

    class Handler(BaseHTTPRequestHandler):
        def _send(self, obj: dict) -> None:
            data = json.dumps(obj).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path.startswith("/navigate/models"):
                return self._send({"default": "standin", "models": ["standin"], "prompt_variants": ["default"]})
            return self._send({"status": "ok"})

        def do_POST(self):
            self.rfile.read(int(self.headers.get("Content-Length") or 0))
            reply, why = answer(pose()) if self.path == "/navigate" else (NOT_VISIBLE, {})
            if self.path == "/navigate":
                line = json.dumps({"t": round(time.time(), 3), "path": self.path,
                                   "visible": reply["target_visible"], **why}) + "\n"
                with lock:
                    out.write(line)
            self._send(reply)

        def log_message(self, *a):
            pass

    ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()


if __name__ == "__main__":
    main()
