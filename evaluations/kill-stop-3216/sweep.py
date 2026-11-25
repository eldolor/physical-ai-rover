# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/kill-stop-3216/sweep.py

3.216 criterion 1's instrument: the robot server SIGKILLed mid-move, as
3.202's K3 kills it, across speeds, directions, start gaps and kill moments,
judged on GROUND TRUTH only (the body's `gap_cm`, `tests/footprint_sweep.py`'s
geometry, which shares no code with what it judges).

The stack is 3.135's (`stack.py`: the split body, two sensor servers, the
fake D500 on a pty), with the body `kill_body.py` (a placement route and a
log of every `T:1`), the robot server `robot_variant.py` (a candidate's
constants) or, with `--shipped`, the shipped launcher, and no brain: the
verbs are posted to `/action` straight, the path a brain's verb takes.

One run: the chassis placed in scaled_house's west room facing the west
wall (FORWARD) or with its back to it (REVERSE), its true gap `need(v) +
extra`, where need is the shipped vet's bar at that speed; a verb posted at
`speed` for 3 s; once the board drives, the robot server is SIGKILLed after
`frac` of the time the robot needs to cover `extra` (so kills land from far
out to at the line); then 2.5 s watched. Measured from the truth: the least
gap after the kill, the gap and setpoint at the kill, and when the board's
setpoint fell to zero. A run whose board was not driving at the kill is
`void` (counted, never judged).

    python evaluations/kill-stop-3216/sweep.py <out> [--shipped] [--quick]
        [--heartbeat-ms N] [--t-react S]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FMEA = ROOT / "evaluations" / "jetson-fmea-3135"
sys.path.insert(0, str(FMEA))
sys.path.insert(0, str(ROOT))

from stack import PORTS, PY, Stack, StackError, http  # noqa: E402

from tests import footprint_sweep as fs  # noqa: E402

CELL_M = 0.30
WALL_X_M = 1 * CELL_M                 # scaled_house: the west room's wall face, x = 1 cell
# The row: 63 cm of true gap to the solid object at cell (2, 2) and to the
# south wall alike, so through every run the least gap is the west wall's
# (amendment 1: at row 4.5 the object's side gap, ~33 cm, was the least).
ROW_Y_M = 5.5 * CELL_M
HALF_LEN_M = fs.HALF_LENGTH * fs.CELL_CM / 100.0
SPEEDS = (25, 50, 75, 100)            # 0.15, 0.30, 0.45, 0.60 m/s: every verb speed band
# `/wheels` (a person's D-pad, nav2's twists) has no linear cap of its own
# below the board's 2.0 m/s window: one standing command past the verbs' top.
WHEELS_M_S = 1.0
WHEEL_RADIUS_M = 0.040
DIRS = ("FORWARD", "REVERSE")
EXTRAS_CM = (5.0, 20.0, 45.0)
FRACS = (0.1, 0.5, 0.8, 0.95, 1.1)
WATCH_S = 2.5
# Amendment 1 (before the baseline): the smoke run's vet stopped a 0.3 m/s
# FORWARD at 26 cm of TRUE gap against its 20 cm bar (the lidar's age taken
# off the clearance). Starts sit this much further out, so the extra is room
# the robot really drives at speed and the late kills land near the line.
STOP_OFFSET_CM = 6.0
VERB_S = 3.0


def need_cm(v: float) -> float:
    """The SHIPPED vet's bar at v (min_distance 20 cm; 3.44's stop distance
    with T_REACT 0.15 s, braking 0.5 m/s^2): where a run starts from, so a
    candidate's change of bar moves the same start nearer or further."""
    return max(20.0, 3.0 + 100.0 * (v * 0.15 + v * v / 1.0))


class KillStack(Stack):
    def __init__(self, out: Path, shipped: bool, variant_env: dict):
        super().__init__(out, house="scaled_house",
                         extra_env={"FMEA_TRUTH_LOCK": "0", "FMEA_ROBOT_LOCK": "0"})
        self.shipped = shipped
        self.variant_env = variant_env

    def _spawn(self, role, argv, env=None):
        if role == "body":
            argv = [PY, str(HERE / "kill_body.py"), str(PORTS["body"])]
        elif role == "robot" and not self.shipped:
            argv = [PY, str(HERE / "robot_variant.py"), str(PORTS["robot"])]
            env = dict(env or self.env, **self.variant_env)
        return super()._spawn(role, argv, env)

    def start_brain(self) -> None:
        pass                                # no brain: the verbs are posted straight


class Truth:
    """truth.jsonl, read incrementally."""

    def __init__(self, path: Path):
        self.path, self.pos, self.rows = path, 0, []

    def poll(self) -> list:
        with open(self.path, "rb") as f:
            f.seek(self.pos)
            data = f.read()
        cut = data.rfind(b"\n") + 1
        self.pos += cut
        for line in data[:cut].splitlines():
            try:
                self.rows.append(json.loads(line))
            except ValueError:
                pass
        return self.rows

    def window(self, t0: float, t1: float) -> list:
        self.poll()
        return [r for r in self.rows if t0 <= r["t"] <= t1]


def moving(sp) -> bool:
    return bool(sp) and any(abs(v) > 1e-6 for v in sp)


def wait_ready(st: KillStack, timeout_s: float = 20.0) -> None:
    """The robot server up and hearing the board (its wheels measured)."""
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        try:
            code, body = st.robot("GET", "/wheels", timeout=1.0)
            if code == 200 and isinstance(body, dict) and body.get("usable"):
                return
        except OSError:
            pass
        time.sleep(0.2)
    raise StackError("the robot server never measured the wheels")


def one_run(st: KillStack, truth: Truth, action: str, speed, extra: float, frac: float) -> dict:
    """`speed` 0-100 is a verb; "wheels" is a standing `/wheels` command at
    `WHEELS_M_S`."""
    wheels = speed == "wheels"
    v = WHEELS_M_S if wheels else speed / 100.0 * 0.6
    start_gap = need_cm(v) + STOP_OFFSET_CM + extra
    th = math.pi if action == "FORWARD" else 0.0
    x = WALL_X_M + HALF_LEN_M + start_gap / 100.0
    rec = {"action": action, "speed": speed, "v": v, "extra_cm": extra, "frac": frac,
           "start_gap_target_cm": round(start_gap, 2)}
    if not st.alive("robot"):
        st.start_robot()
    wait_ready(st)
    for _ in range(50):                      # a stopped board may still coast for a sample
        code, placed = http("POST", f"{st.urls['body']}/k3216/place?x={x}&y={ROW_Y_M}&th={th}",
                            secret=st.secret, timeout=2.0)
        if code == 200:
            break
        time.sleep(0.1)
    else:
        rec["void"] = f"place refused: {placed}"
        return rec
    time.sleep(0.6)                          # the sensors and the lidar see the new pose
    rec["start_gap_cm"] = st.now().get("gap_cm")
    answer = {}

    def post():
        try:
            if wheels:
                w = (1.0 if action == "FORWARD" else -1.0) * v / WHEEL_RADIUS_M
                answer["r"] = st.robot("POST", "/wheels", {"left_rad_s": w, "right_rad_s": w},
                                       timeout=10.0)
            else:
                answer["r"] = st.robot("POST", "/action", {"action": action, "speed": speed,
                                                          "duration": VERB_S}, timeout=10.0)
        except Exception as e:  # noqa: BLE001 -- the kill ends the request
            answer["r"] = (None, f"{type(e).__name__}: {e}")

    t_post = time.monotonic()
    th_post = threading.Thread(target=post, daemon=True)
    th_post.start()
    t_drive = None
    while time.monotonic() - t_post < 2.0:
        n = st.now()
        if moving(n.get("sp")):
            t_drive = time.monotonic()
            break
        time.sleep(0.005)
    if t_drive is None:
        th_post.join(timeout=5)
        rec["void"] = f"never drove: {answer.get('r')}"
        return rec
    delay = frac * (extra / 100.0) / v
    time.sleep(max(0.0, t_drive + delay - time.monotonic()))
    t_kill = st.signal("robot", signal.SIGKILL)
    st.procs["robot"].wait(timeout=5)
    time.sleep(WATCH_S)
    th_post.join(timeout=2)
    rows = truth.window(t_kill - 0.2, t_kill + WATCH_S)
    before = [r for r in rows if r["t"] <= t_kill]
    after = [r for r in rows if r["t"] > t_kill]
    at = before[-1] if before else None
    rec.update({"kill_delay_s": round(delay, 3),
                "gap_at_kill_cm": at and at["gap_cm"], "sp_at_kill": at and at["sp"],
                "samples_after": len(after)})
    if not at or not moving(at["sp"]):
        rec["void"] = "not driving at the kill"
        return rec
    zero = next((r for r in after if not moving(r["sp"])), None)
    gaps = [r["gap_cm"] for r in after if r["gap_cm"] is not None]
    rec.update({"least_gap_cm": min(gaps) if gaps else None,
                "zeroed_after_s": round(zero["t"] - t_kill, 3) if zero else None,
                "travel_cm": round(100.0 * math.hypot(after[-1]["x"] - at["x"], after[-1]["y"] - at["y"]), 2)
                if after else None})
    return rec


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--shipped", action="store_true", help="the shipped robot server, no variant")
    ap.add_argument("--heartbeat-ms", type=int, default=0)
    ap.add_argument("--t-react", type=float, default=0.0)
    ap.add_argument("--quick", action="store_true", help="one speed and direction, for a smoke run")
    a = ap.parse_args(argv)
    out = Path(a.out)
    variant = {}
    if a.heartbeat_ms:
        variant["K3216_HEARTBEAT_MS"] = str(a.heartbeat_ms)
    if a.t_react:
        variant["K3216_T_REACT_S"] = str(a.t_react)
    if a.shipped and variant:
        raise SystemExit("--shipped runs the shipped constants: no variant beside it")
    grid = [(d, s, e, f) for d in DIRS for s in (*SPEEDS, "wheels") for e in EXTRAS_CM for f in FRACS]
    if a.quick:
        grid = [(d, s, e, f) for d, s, e, f in grid if d == "FORWARD" and s == 50 and e == 20.0]
    st = KillStack(out, a.shipped, variant)
    st.start()
    (out / "run.json").write_text(json.dumps({"shipped": a.shipped, "variant": variant, "runs": len(grid),
                                              "ports": PORTS, "started": time.time()}, indent=1))
    truth = Truth(out / "truth.jsonl")
    try:
        with open(out / "runs.jsonl", "a", buffering=1) as f:
            for i, (d, s, e, fr) in enumerate(grid):
                try:
                    rec = one_run(st, truth, d, s, e, fr)
                except StackError as ex:
                    rec = {"action": d, "speed": s, "extra_cm": e, "frac": fr, "void": f"stack: {ex}"}
                rec["i"] = i
                f.write(json.dumps(rec) + "\n")
                print(json.dumps(rec), flush=True)
    finally:
        st.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
