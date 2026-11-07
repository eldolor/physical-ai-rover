# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-perf-3136/lat_body.py

3.136.C's simulator body: 3.135's `truth_body.py` (the shipped
`sim/body_server.py` app, the fake board, the 20 ms truth recorder: pose,
the board's wheel setpoints `sp`, the true gap) with P5's object drop added
from outside. Nothing in sim/ or in 3.135's files is edited.

* `GET /lat/ahead`: the true travel to contact straight ahead (`T_cm`) and
  the true gap (`G_cm`) now (`tests/footprint_sweep.py`'s ground truth).
* `POST /lat/drop?lo=8&hi=16&avoid=<name>&clear=74`: with the board's lock held (one
  instant, the pose frozen), move one object the lidar can see (not a
  mover, not low, not named `avoid`; the farthest from the robot) onto the
  floor cell that puts the true travel to contact ahead in [lo, hi] cm and
  nearer than anything before it, through the sim's own `move_object`.
  Answers its monotonic stamp (the truth log's clock), the board's
  setpoints then, the cells and the true T and gap; 409 when no cell fits this instant, or when the true
  travel to contact ahead is under `clear` cm before it.
* `POST /lat/restore` {"src", "dst"}: the object put back.
* Criterion 3's plant (`LAT_PLANT_FILE` names a file; the word `zero` in it
  when the board takes a command): a command that takes the forward speed
  to zero (a stop, or the veto's pivot under `drive: ros`) is applied 100 ms
  late, the commands after it held with it (the end P5 measures moved by
  the plant's size).

    FMEA_DIR=<out> python evaluations/jetson-perf-3136/lat_body.py <port>
"""

import math
import os
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "jetson-fmea-3135"))

import truth_body as tb  # noqa: E402  (chdir to the repo, the app, board and grid)
from fastapi import HTTPException  # noqa: E402
from pydantic import BaseModel  # noqa: E402

from sim import fake_esp32  # noqa: E402
from tests import footprint_sweep as fs  # noqa: E402

PLANT_FILE = os.environ.get("LAT_PLANT_FILE", "")
PLANT_S = 0.1
app, board, grid = tb.app, tb.board, tb.grid


def planted(word: str) -> bool:
    if not PLANT_FILE:
        return False
    try:
        return word in Path(PLANT_FILE).read_text().split()
    except OSError:
        return False


_set = fake_esp32.FakeEsp32._set_goal_speed


FWD_ZERO_M_S = 0.005        # lat_score's forward-zero line
_pending: dict = {}


def _set_goal_speed(self, left, right):
    """The plant: a command that takes the forward speed (the mean of the
    two setpoints) from moving to zero -- an exact stop, or under `drive:
    ros` the veto's pivot in place -- and every command after it are held
    PLANT_S, then the newest is applied (on a timer, under the board's lock
    as the firmware's own call is), so the wheels go on forward 100 ms
    longer. Called with the board's lock held."""
    if planted("zero"):
        if _pending:                                  # a stop is being held: the newest wins
            _pending["cmd"] = (left, right)
            return
        if sum(self.setpoint) / 2 > FWD_ZERO_M_S and (left + right) / 2 <= FWD_ZERO_M_S:
            _pending["cmd"] = (left, right)

            def late():
                with self.lock:
                    cmd = _pending.pop("cmd")
                    _set(self, *cmd)
            threading.Timer(PLANT_S, late).start()
            return
    _set(self, left, right)


fake_esp32.FakeEsp32._set_goal_speed = _set_goal_speed


def _truth(solid) -> tuple:
    return fs.truth(SimpleNamespace(x=grid.x, y=grid.y, theta=grid.theta, layout=grid.layout,
                                    solid_cells=solid))


@app.get("/lat/ahead")
def lat_ahead():
    with board.lock:
        t = time.monotonic()
        T, G, _ = _truth(grid.solid_cells)
    return {"t": round(t, 4), "T_cm": None if math.isinf(T) else round(T, 2),
            "G_cm": None if math.isinf(G) else round(G, 2)}


@app.post("/lat/drop")
def lat_drop(lo: float = 8.0, hi: float = 16.0, avoid: str = "", clear: float = 0.0):
    with board.lock:
        solid = grid.solid_cells
        T0, G0, _ = _truth(solid)
        if T0 < clear:
            raise HTTPException(status_code=409, detail=f"not clear ahead (T {T0:.1f} cm)")
        x, y = grid.x, grid.y
        movers = {m.cell for m in grid.movers}
        lidar = grid.lidar_solid_cells
        srcs = [c for c, name in grid.objects.items()
                if c in lidar and c not in movers and name != avoid]
        if not srcs:
            raise HTTPException(status_code=409, detail="no object to move")
        src = max(srcs, key=lambda c: math.hypot(c[0] + 0.5 - x, c[1] + 0.5 - y))
        best = None
        for cy in range(int(y) - 3, int(y) + 4):
            for cx in range(int(x) - 3, int(x) + 4):
                c = (cx, cy)
                if c in solid or not grid._is_passable(cx, cy):
                    continue
                T, G, P = _truth(solid | {c})
                if P > 0 or not (lo <= T <= hi) or T >= T0 - 1.0:
                    continue
                mid = abs(T - (lo + hi) / 2)
                if best is None or mid < best[0]:
                    best = (mid, c, T, G)
        if best is None:
            raise HTTPException(status_code=409, detail=f"no cell fits now (T before {T0:.1f} cm)")
        _, dst, T, G = best
        try:
            grid.move_object(src, dst)
        except ValueError as e:
            raise HTTPException(status_code=409, detail=str(e))
        t = time.monotonic()
        sp = [round(v, 4) for v in board.setpoint]      # what the wheels did in the drop's instant
    return {"t": round(t, 4), "sp": sp, "src": list(src), "dst": list(dst), "name": grid.objects[dst],
            "T_cm": round(T, 2), "G_cm": round(G, 2),
            "T_before_cm": None if math.isinf(T0) else round(T0, 2)}


class Restore(BaseModel):
    src: list
    dst: list


@app.post("/lat/restore")
def lat_restore(req: Restore):
    try:
        grid.move_object(tuple(req.dst), tuple(req.src))
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e))
    return {"restored": True}


if __name__ == "__main__":
    import uvicorn

    out = Path(os.environ["FMEA_DIR"])
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "truth.jsonl", "w", buffering=1) as log:     # as truth_body's own main
        recorder = tb.recorder = tb.Recorder(tb.sample, log, tb.PERIOD_S).start()
        app.router.on_shutdown.append(recorder.stop)
        try:
            uvicorn.run(app, host="127.0.0.1", port=int(sys.argv[1]), log_level="warning")
        finally:
            recorder.stop()
