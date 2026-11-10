"""3.173.B: in-process surveys (tests/survey_sweep.py's houses and starts): how
many `small_room` goals fire, where (truth room), and how they end; goals and
label calls beside the arm with the goal off.

    python evaluations/small-rooms-3173/b_sweep_small.py [house ...]
"""
import collections
import sys
from pathlib import Path

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


def one(house, start, on):
    ex.SMALL_ROOM_CLEARANCE_M = 0.25 if on else 0.0
    grid = build_world(house)
    (cx, cy), theta, _ = start
    grid.x, grid.y, grid.theta = cx + 0.5, cy + 0.5, theta
    truth = build_world(house)
    room = {c: n for n, cs in sorted(truth.rooms.items(), key=lambda kv: -len(kv[1])) for c in cs}
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, job="survey", policy="explore", navigator=FakeNav(robot),
                           world=MockWorld(grid), mission_time_limit_s=sw.TIME_LIMIT_S,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    runner.start()
    t = 0
    while runner.tick() and t < sw.TICK_CAP:
        t += 1
    ev = runner.agent.events
    small = [e for e in ev if e.get("event") == "goal_sent" and e.get("kind") == "small_room"]
    where = collections.Counter((room.get((int(e["goal"][0] / 0.3), int(e["goal"][1] / 0.3))), e.get("area_m2"), e.get("open_m"))
                                for e in small)
    s = runner.status()
    return {"goals": (s.get("explore") or {}).get("goals_sent"), "labels": s["survey"]["label_calls"],
            "small": len(small), "where": dict(where), "outcome": s["outcome"]}


def main(houses):
    desktop_lock.hold_for_life("shared", f"3.173.B small-room sweep ({__file__})")
    for h in houses or sw.HOUSES:
        for st in sw.starts(h):
            desktop_lock.step_aside()
            off, on = one(h, st, False), one(h, st, True)
            print(h, st[2], "off", off, "| on", on, flush=True)


if __name__ == "__main__":
    main(sys.argv[1:])
