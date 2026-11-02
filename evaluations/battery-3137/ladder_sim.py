"""3.137 criterion 1, in process: the battery ladder through the real path.

`MissionRunner` -> `RemoteRobot` (over ASGI) -> one in-process
`robot/server.py` -> `robot/safety.py` -> a `MockRobot` whose simulated pack
drains 0.02 V/s from 11.0 V on the simulator's clock. Tiered missions in the
scaled house, 3.32's twenty starts (tests/test_bearing_turns.py), each run in
three arms:

* N -- no battery: the baseline outcome and when it ended (sim seconds);
* L -- the battery, the brain's floor at 10.5 V (the shipped config): a
  mission that finds before Low ends as N did (criterion 1d);
* R -- the battery, the brain's floor off, so the mission drives into Reserve:
  0 autonomous motion after it (1b, ground truth), a person's FORWARD at
  reduced speed, then Power-off: the steps in order and the power-off stub
  called once (1c).

Plus the planted sag (1a): 20 motor starts, each sagging the pack 1.0 V for
1 s, read every poll, fire nothing.

Run: python evaluations/battery-3137/ladder_sim.py [--out results.json]
"""

import argparse
import json
import logging
import math
import os
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from fastapi.testclient import TestClient  # noqa: E402

import robot.server as server  # noqa: E402
from brain.perceive import FrameReportedPipeline  # noqa: E402
from brain.tiered import TieredVision  # noqa: E402
from control.mission_runner import MissionRunner  # noqa: E402
from control.remote_robot import RemoteRobot  # noqa: E402
from control.remote_world import RemoteWorld  # noqa: E402
from robot.battery_ladder import LOW, POWEROFF, RESERVE  # noqa: E402
from sim.battery import SimBattery  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.mock_robot import DEFAULT_CELL_M, MockRobot  # noqa: E402
from tests.test_bearing_turns import (CLEAR_STARTS, GOAL, HOUSE, JAMB_STARTS,  # noqa: E402
                                      SEARCH_OFFSETS, SEARCH_START, TARGET, _quiet_cloud)

BASE = "http://asgi"
V0, DRAIN = 11.0, 0.02
STARTS = (CLEAR_STARTS + JAMB_STARTS + [(*SEARCH_START, o) for o in SEARCH_OFFSETS])[:20]
MAX_STEPS = 400


def served(battery, log: Path):
    """`battery`: None, or the pack's starting volts."""
    grid = build_world(HOUSE)
    body = MockRobot(grid, render=False)
    if battery:
        body.battery = SimBattery(v0=battery, drain_v_per_s=DRAIN, clock=lambda: grid.sim_time)
    os.environ["BATTERY_SHUTDOWN_CMDS"] = json.dumps(
        [[n, f"echo {n} >> {log}"] for n in ("brain", "ros", "vlm")])
    os.environ["BATTERY_POWEROFF_CMD"] = f"echo poweroff >> {log}"
    server.get_robot = lambda *a, **kw: body
    app = server.create_app()
    return app, TestClient(app, base_url=BASE), body, grid


def pose_m(grid):
    return grid.x * DEFAULT_CELL_M, grid.y * DEFAULT_CELL_M


def place(grid, start):
    x, y, off = start
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)


def mission(start, arm, tmp, v0=V0):
    log = Path(tmp) / f"steps-{arm}.log"
    log.unlink(missing_ok=True)
    app, client, body, grid = served(battery=None if arm == "N" else v0, log=log)
    place(grid, start)
    bat = app.state.battery
    ladder = bat["ladder"]
    fired = {}
    real_fire = ladder._fire

    def fire(level, now, how):
        # Where the robot truly stood when each level latched (ground truth).
        fired[level] = {"sim_s": round(now, 2), "pose": pose_m(grid), "how": how,
                        "v_true": round(body.battery.true_v(now), 3)}
        return real_fire(level, now, how)
    ladder._fire = fire
    low_at = {}

    def idle(dt):
        # Time passing while the mission waits (a stuck cooldown) -- and the
        # robot server's own battery loop, which in process has no lifespan
        # to run it: it polls every BATTERY_POLL_S of the robot's clock.
        body.pass_time(dt)
        bat["poll"]()

    remote = RemoteRobot(BASE, client=client)
    world = RemoteWorld(BASE, client=client)
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud,
                        steer_on_sight=True, hold_goal=True)
    runner = MissionRunner(remote, target_object=TARGET, max_steps=MAX_STEPS, policy="tiered",
                           vision_fn=tier, world=world, mission_time_limit_s=None,
                           battery_floor_v=(10.5 if arm == "L" else None),
                           clock=lambda: grid.sim_time, idle=idle)
    runner.start()
    while runner.tick():
        if ladder.level == LOW and "low" not in low_at:
            low_at["low"] = round(grid.sim_time, 2)
    s = runner.status()
    row = {"start": list(start), "arm": arm, "v0": None if arm == "N" else v0,
           "outcome": s["outcome"],
           "end_reason": s.get("end_reason"), "end_sim_s": round(grid.sim_time, 2),
           "steps": s["step"]}
    if arm == "N":
        return row
    row["low_seen_sim_s"] = low_at.get("low")
    row["fired"] = {k: {**v, "pose": [round(c, 4) for c in v["pose"]]} for k, v in fired.items()}
    if arm == "R":
        # After the mission: the pack keeps draining while the robot waits.
        end_pose = pose_m(grid)
        reserve_pose = fired.get(RESERVE, {}).get("pose")
        row["autonomous_motion_after_reserve_m"] = (
            round(math.dist(reserve_pose, end_pose), 4) if reserve_pose else None)
        # The brain once more, then a person's slow FORWARD, at Reserve.
        if ladder.level == RESERVE:
            r = client.post("/action", json={"action": "FORWARD"},
                            headers={"x-driver": "brain"}).json()
            row["brain_at_reserve"] = r.get("reason")
            p = client.post("/action", json={"action": "FORWARD", "speed": 100},
                            headers={"x-driver": "twin-dpad"}).json()
            row["person_at_reserve"] = {"executed": p.get("executed"),
                                        "speed": (p.get("result") or {}).get("speed"),
                                        "reason": p.get("reason")}
        deadline = grid.sim_time + 200
        while not bat["batt"]["shutdown_done"].is_set() and grid.sim_time < deadline:
            body.pass_time(0.5)
            bat["poll"](force=True)
        bat["batt"]["shutdown_done"].wait(10)
        time.sleep(0.05)
        row["shutdown"] = bat["batt"]["shutdown"]
        row["steps_log"] = log.read_text().split() if log.exists() else []
        row["poweroff_calls"] = row["steps_log"].count("poweroff")
        p = client.post("/action", json={"action": "FORWARD"},
                        headers={"x-driver": "twin-dpad"}).json()
        row["person_at_poweroff"] = p.get("reason")
    client.close()
    return row


def crossing(level_v):
    return (V0 - level_v) / DRAIN


def sag_test(tmp, refresh_s=0.0, n=20, drain=0.0):
    """1a's planted sag: a person's standing wheel command starts the motors
    20 times; each start sags the pack 1.0 V for 1 s (11.0 -> 10.0, under
    Reserve). Polled every 0.25 s of sim time while it lasts."""
    log = Path(tmp) / "steps-sag.log"
    app, client, body, grid = served(battery=True, log=log)
    body.battery = SimBattery(v0=V0, sag_v=1.0, sag_s=1.0, refresh_s=refresh_s,
                              drain_v_per_s=drain, clock=lambda: grid.sim_time)
    bat = app.state.battery
    polls_in_sag = 0
    for _ in range(n):
        client.post("/wheels", json={"left_rad_s": 1.0, "right_rad_s": -1.0},
                    headers={"x-driver": "twin-dpad"})
        for _ in range(8):                     # 2 s
            before = body.battery.sagged_reads
            grid.advance_time(0.25)
            bat["poll"](force=True)
            polls_in_sag += body.battery.sagged_reads > before
        client.post("/wheels", json={"left_rad_s": 0.0, "right_rad_s": 0.0},
                    headers={"x-driver": "twin-dpad"})
        for _ in range(8):
            grid.advance_time(0.25)
            bat["poll"](force=True)
    lad = bat["ladder"]
    client.close()
    return {"refresh_s": refresh_s, "drain_v_per_s": drain, "sags": n,
            "readings_inside_a_sag": polls_in_sag, "end_v_true": round(body.battery.true_v(), 3),
            "level": lad.level, "checks": lad.checks, "cleared": lad.sags,
            "fired": lad.level in (RESERVE, POWEROFF)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(Path(__file__).with_name("results-sim.json")))
    ap.add_argument("--starts", type=int, default=len(STARTS))
    args = ap.parse_args()
    logging.disable(logging.WARNING)
    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        for i, start in enumerate(STARTS[:args.starts]):
            n_end = None
            for arm in ("N", "L", "R"):
                t0 = time.monotonic()
                # Amendment 1: these missions find in ~10 s of sim time, so
                # from 11.0 V no mission reaches Reserve. Arm R starts where
                # the drain crosses Reserve halfway through N's mission.
                v0 = V0 if arm != "R" else round(10.2 + DRAIN * 0.5 * n_end, 4)
                row = mission(start, arm, tmp, v0=v0)
                if arm == "N":
                    n_end = row["end_sim_s"]
                row["wall_s"] = round(time.monotonic() - t0, 2)
                rows.append(row)
                print(f"{i:2d} {arm} {row['outcome']:<14} end {row['end_sim_s']:7.1f} s  "
                      f"{row.get('end_reason') or ''}"[:140], flush=True)
        # The board-like read (10 s refresh), every frame with a held value
        # (M2.7's distinct rule alone), and every frame draining (each read a
        # new sample: the rest re-read decides).
        sags = [sag_test(tmp, 10.0), sag_test(tmp, 0.0), sag_test(tmp, 0.0, drain=0.002)]
    for s in sags:
        print("sag", s)
    Path(args.out).write_text(json.dumps({
        "v0": V0, "drain_v_per_s": DRAIN,
        "crossings_sim_s": {"low": crossing(10.5), "reserve": crossing(10.2),
                            "poweroff": crossing(10.0)},
        "rows": rows, "sag": sags}, indent=1))
    print("wrote", args.out)


if __name__ == "__main__":
    main()
