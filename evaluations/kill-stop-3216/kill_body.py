# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/kill-stop-3216/kill_body.py

3.216's simulator body: 3.135's `truth_body.py` (the shipped body server, the
fake board copied from the firmware, the ground-truth recorder) plus two
things from outside, for the harness only. Nothing in sim/ is edited.

* `POST /k3216/place?x=&y=&th=`: the chassis put at a pose (metres, radians)
  under the board's lock -- the start of a run. Refused while the board
  drives.
* `$FMEA_DIR/sends.jsonl`: one line per `T:1` the board received
  (`t` monotonic, `L`, `R`) -- the robot server's real send cadence, which
  the heartbeat must sit above.

    FMEA_DIR=/tmp/x python evaluations/kill-stop-3216/kill_body.py <port>
"""

import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "evaluations" / "jetson-fmea-3135"))
sys.path.insert(0, str(ROOT))

import truth_body as tb  # noqa: E402
from fastapi import HTTPException  # noqa: E402
from truth_recorder import Recorder  # noqa: E402

_sends = None
_handle = tb.fake_esp32.FakeEsp32._handle


def _logged_handle(self, line: bytes):
    if _sends is not None and b'"T": 1,' in line:
        try:
            cmd = json.loads(line)
            if isinstance(cmd, dict) and cmd.get("T") == 1:
                _sends.write(json.dumps({"t": round(time.monotonic(), 4),
                                         "L": cmd.get("L"), "R": cmd.get("R")}) + "\n")
        except (ValueError, OSError):
            pass
    return _handle(self, line)


tb.fake_esp32.FakeEsp32._handle = _logged_handle


@tb.app.post("/k3216/place")
def place(x: float, y: float, th: float):
    with tb.board.lock:
        if any(tb.board.setpoint):
            raise HTTPException(status_code=409, detail=f"moving: {tb.board.setpoint}")
        g = tb.grid
        g.x, g.y, g.theta = x / tb.CELL_M, y / tb.CELL_M, th
    return tb.sample()


if __name__ == "__main__":
    import uvicorn

    out = Path(os.environ["FMEA_DIR"])
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "truth.jsonl", "w", buffering=1) as log, \
            open(out / "sends.jsonl", "w", buffering=1) as sends:
        _sends = sends
        tb.recorder = Recorder(tb.sample, log, tb.PERIOD_S).start()
        tb.app.router.on_shutdown.append(tb.recorder.stop)
        try:
            uvicorn.run(tb.app, host="127.0.0.1", port=int(sys.argv[1]), log_level="warning")
        finally:
            tb.recorder.stop()
