# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
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
  it judges; < 1 cm is a contact, its G bar; 999 when nothing is within its
  2.5-cell look, null only if the geometry raised).
  Since 3.151 the sampling and the file writes are two threads
  (`truth_recorder.Recorder`): a write stalled in reclaim no longer delays a
  sample.
* `POST /fmea/mlock`: lock the process's touched pages
  (`truth_recorder.lock_memory`, 3.151), answering what happened; the stack
  calls it once the body is healthy, after raising the body's memlock limit.
* `GET /fmea/now`: the newest sample (the injectors wait on it for motion).
* `POST /fmea/battery?v=10.35&drain=0.005`: the fake board's pack set now (F24).
* `POST /fmea/silence?s=5`: the board stops sending its `T:1001` feedback
  frames for `s` seconds (the firmware copy's own `silent_until`): F10.
* Planted bugs (criterion 4), from `FMEA_PLANT` (comma-separated):
  `heartbeat` -- the fake board ignores its heartbeat (F8's guard off).

    FMEA_DIR=/tmp/x python evaluations/jetson-fmea-3135/truth_body.py <port>
"""

import math
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)

PERIOD_S = 0.02
FAR_CM = 999.0
PLANTS = {p for p in os.environ.get("FMEA_PLANT", "").split(",") if p}
CELL_M = 0.30

from sim import fake_esp32  # noqa: E402

if "heartbeat" in PLANTS:
    fake_esp32.FakeEsp32._heartbeat = lambda self: None       # planted: the board never times out

from fastapi import HTTPException  # noqa: E402

from sim.body_server import create_app  # noqa: E402
from tests import footprint_sweep as fs  # noqa: E402
from truth_recorder import Recorder, lock_memory  # noqa: E402

app = create_app()
board, grid = app.state.board, app.state.body.world
recorder: "Recorder | None" = None


def sample() -> dict:
    """One truth row, of one instant."""
    with board.lock:                            # one instant; held only to copy it
        t = time.monotonic()
        x, y, th = grid.x, grid.y, grid.theta
        cam = grid.view_angle()                 # body heading plus the pan (amendment 17)
        sp = [round(v, 4) for v in board.setpoint]
        solid = grid.solid_cells                # the objects of the same instant
    try:                                        # the geometry outside the board's lock
        gap = fs.truth(SimpleNamespace(x=x, y=y, theta=th, layout=grid.layout,
                                       solid_cells=solid))[1]
    except Exception:                           # noqa: BLE001 -- null: contact unjudged
        gap = None
    return {"t": round(t, 4), "x": round(x * CELL_M, 5), "y": round(y * CELL_M, 5),
            "th": round(th, 5), "cam_th": round(cam, 5), "sp": sp,
            # footprint_sweep looks 2.5 cells out: inf is "nothing that
            # near", written as FAR_CM; null only when it raised.
            "gap_cm": None if gap is None else FAR_CM if math.isinf(gap) else round(gap, 2)}


@app.get("/fmea/now")
def fmea_now():
    return dict(recorder.latest) if recorder else {}


@app.post("/fmea/mlock")
def fmea_mlock():
    """3.151: lock the touched pages; what happened (a refusal included)."""
    return lock_memory()


@app.post("/fmea/silence")
def fmea_silence(s: float = 5.0):
    if not 0 < s <= 60:
        raise HTTPException(status_code=422, detail="0 < s <= 60")
    t0 = time.monotonic()
    board.silent_until = t0 + s
    return {"silent_from": t0, "silent_until": board.silent_until}


@app.post("/fmea/battery")
def fmea_battery(v: float, drain: float = 0.0):
    """F24 (amendment 18): the fake board's pack set to `v` volts now, draining
    `drain` V/s from now (the same line `SIM_BATTERY_V` / `_DRAIN_V_PER_S` draw
    from the board's start)."""
    if not (0 < v <= 13.0 and 0 <= drain <= 0.1):
        raise HTTPException(status_code=422, detail="0 < v <= 13, 0 <= drain <= 0.1")
    with board.lock:
        b = board.battery
        b.v0, b.drain_v_per_s, b.since = float(v), float(drain), b.clock()
        b._held = b._held_at = None
    return {"v": v, "drain": drain, "since": b.since}


if __name__ == "__main__":
    import uvicorn

    out = Path(os.environ["FMEA_DIR"])
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "truth.jsonl", "w", buffering=1) as log:     # one stack, one log
        recorder = Recorder(sample, log, PERIOD_S).start()
        # The drain runs in the app's shutdown: uvicorn re-raises SIGTERM (the
        # stack's teardown) with its default action once it has shut down, so
        # nothing after `uvicorn.run` runs then (review). Within the stack's
        # 8 s kill.
        app.router.on_shutdown.append(recorder.stop)
        try:
            uvicorn.run(app, host="127.0.0.1", port=int(sys.argv[1]), log_level="warning")
        finally:
            recorder.stop()                     # other ends (an exception, Ctrl-C's return)
