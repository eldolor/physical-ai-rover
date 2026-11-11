"""3.173 S3: in-process surveys on A2's 23 houses, rooms entered per survey,
amendment 3's rule against `dev`'s (B as merged: a piece with no goal stop).

    python evaluations/small-rooms-3173/s3_sweep.py OUT.jsonl [house ...]

`dev`'s rule is reproduced in process: `_small_regions` returns the cut's
pieces that hold no goal stop -- B's old filter, its area the whole piece's.
One difference (review round 2): both arms judge "tried" by amendment 3's rule
(a region the sent goal reaches through floor within 0.35 m), where `dev`
marked only the piece under the goal's cell. Rooms entered: the truth room under the robot after every tick
(`FakeNav` moves it goal to goal). Contacts (S4) are not read here: `FakeNav`
moves the robot without driving it. Three starts a house (`survey_sweep`'s),
both arms. Holds the desktop lock shared and steps aside between surveys.
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
import desktop_lock  # noqa: E402
import brain.explore as ex  # noqa: E402
from control.mission_runner import MissionRunner  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from sim.mock_world import MockWorld  # noqa: E402
from tests import survey_sweep as sw  # noqa: E402
from tests.test_explore import FakeNav  # noqa: E402

HOUSES = (["home_first_floor_stairs", "scaled_house", "complex_house"]
          + [f"procthor:val:{i}" for i in range(20)])
A3 = ex.ExploreAgent._small_regions


def dev_regions(self, m, seg, rch):
    """B as merged on `dev`: a piece holding any goal stop is no candidate."""
    reg = seg.copy()
    if len(rch):
        for r in set(seg.ravel()[rch.order].tolist()):
            reg[seg == r] = -1
    return reg


def one(house, start, arm):
    ex.ExploreAgent._small_regions = A3 if arm == "a3" else dev_regions
    grid = build_world(house)
    (cx, cy), theta, _ = start
    grid.x, grid.y, grid.theta = cx + 0.5, cy + 0.5, theta
    truth = build_world(house)
    room = {c: n for n, cs in sorted(truth.rooms.items(), key=lambda kv: -len(kv[1])) for c in cs}
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, job="survey", policy="explore", navigator=FakeNav(robot),
                           world=MockWorld(grid), mission_time_limit_s=sw.TIME_LIMIT_S,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    entered = set()
    runner.start()
    t = 0
    while True:
        entered.add(room.get((int(np.floor(grid.x)), int(np.floor(grid.y)))))
        if not runner.tick() or t >= sw.TICK_CAP:
            break
        t += 1
    entered.discard(None)
    ev = runner.agent.events
    small = [e for e in ev if e.get("event") == "goal_sent" and e.get("kind") == "small_room"]
    s = runner.status()
    return {"house": house, "start": start[2], "arm": arm, "outcome": s["outcome"],
            "rooms": len(truth.rooms), "entered": sorted(entered), "n_entered": len(entered),
            "small": [[room.get((int(e["goal"][0] / 0.3), int(e["goal"][1] / 0.3))), e.get("area_m2")]
                      for e in small],
            "sim_s": round(grid.sim_time, 1), "capped": t >= sw.TICK_CAP}


def main(out, houses):
    desktop_lock.hold_for_life("shared", f"3.173 S3 sweep ({__file__})")
    with open(out, "a") as f:
        for h in houses or HOUSES:
            for st in sw.starts(h):
                for arm in ("dev", "a3"):
                    desktop_lock.step_aside()
                    try:
                        row = one(h, st, arm)
                    except Exception as e:  # noqa: BLE001 -- the sweep counts these
                        row = {"house": h, "start": st[2], "arm": arm, "outcome": "exception",
                               "error": f"{type(e).__name__}: {e}"}
                    f.write(json.dumps(row) + "\n")
                    f.flush()
                    print(h, st[2], arm, row.get("outcome"), row.get("n_entered"), row.get("small"), flush=True)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
