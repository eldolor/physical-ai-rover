"""
evaluations/jetson-fmea-3135/truth_body.py

3.135's simulator body: the shipped `sim/body_server.py` app (physics, the
fake motor board, the truth) with a ground-truth recorder and two simulator
faults added from outside. Nothing in sim/ is edited; the board is the
simulator's, never the code under test.

* Recorder: every 20 ms, under the board's lock (one instant), one JSON line
  in `$FMEA_DIR/truth.jsonl`: `t` (CLOCK_MONOTONIC, the same clock in every
  process on the machine), the true pose in metres and radians, the board's
  wheel setpoints in m/s (`sp`, what the firmware drives) and `gap_cm`, the
  chassis' true distance to the nearest wall or solid object
  (`tests/footprint_sweep.py`'s ground truth, which shares no code with what
  it judges; < 1 cm is a contact, its G bar).
* `GET /fmea/now`: the newest sample (the injectors wait on it for motion).
* `POST /fmea/silence?s=5`: the board stops sending its `T:1001` feedback
  frames for `s` seconds (the firmware copy's own `silent_until`): F10.
* Planted bugs (criterion 4), from `FMEA_PLANT` (comma-separated):
  `heartbeat` -- the fake board ignores its heartbeat (F8's guard off).

    FMEA_DIR=/tmp/x python evaluations/jetson-fmea-3135/truth_body.py <port>
"""

import json
import math
import os
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)

PERIOD_S = 0.02
PLANTS = {p for p in os.environ.get("FMEA_PLANT", "").split(",") if p}
CELL_M = 0.30

from sim import fake_esp32  # noqa: E402

if "heartbeat" in PLANTS:
    fake_esp32.FakeEsp32._heartbeat = lambda self: None       # planted: the board never times out

from fastapi import HTTPException  # noqa: E402

from sim.body_server import create_app  # noqa: E402
from tests import footprint_sweep as fs  # noqa: E402

app = create_app()
board, grid = app.state.board, app.state.body.world
latest: dict = {}


def recorder(path: Path) -> None:
    with open(path, "a", buffering=1) as f:
        nxt = time.monotonic()
        while True:
            with board.lock:
                t = time.monotonic()
                x, y, th = grid.x, grid.y, grid.theta
                sp = [round(v, 4) for v in board.setpoint]
                try:
                    gap = fs.truth(grid)[1]
                except Exception:                               # noqa: BLE001
                    gap = None
            row = {"t": round(t, 4), "x": round(x * CELL_M, 5), "y": round(y * CELL_M, 5),
                   "th": round(th, 5), "sp": sp,
                   "gap_cm": None if gap is None or math.isinf(gap) else round(gap, 2)}
            latest.clear()
            latest.update(row)
            f.write(json.dumps(row) + "\n")
            nxt += PERIOD_S
            time.sleep(max(0.0, nxt - time.monotonic()))


@app.get("/fmea/now")
def fmea_now():
    return dict(latest)


@app.post("/fmea/silence")
def fmea_silence(s: float = 5.0):
    if not 0 < s <= 60:
        raise HTTPException(status_code=422, detail="0 < s <= 60")
    t0 = time.monotonic()
    board.silent_until = t0 + s
    return {"silent_from": t0, "silent_until": board.silent_until}


if __name__ == "__main__":
    import uvicorn

    out = Path(os.environ["FMEA_DIR"])
    out.mkdir(parents=True, exist_ok=True)
    threading.Thread(target=recorder, args=(out / "truth.jsonl",), daemon=True,
                     name="fmea-truth").start()
    uvicorn.run(app, host="127.0.0.1", port=int(sys.argv[1]), log_level="warning")
