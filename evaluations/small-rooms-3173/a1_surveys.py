"""3.173.A1: for each room a survey never entered, what the robot knew of it and
which goals went toward it, from each survey's brain log.

    python evaluations/small-rooms-3173/a1_surveys.py ROWS.jsonl [ROWS.jsonl ...]

Rooms are the simulator's truth rooms of `home_first_floor_stairs`; "entered" is
the row's truth track. A goal is "toward" a room when its point lies in the room
or within NEAR_M of it. Room labels the robot held are read from `goal_sent`
events of kind `room` and from any `room` field the events carry.
"""
import collections
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from sim.maps import build_world  # noqa: E402

CELL = 0.30
NEAR_M = 0.6


def main(paths):
    w = build_world("home_first_floor_stairs")
    cells = {n: set(cs) for n, cs in w.rooms.items()}

    def dist(room, x, y):
        return min(math.hypot((cx + .5) * CELL - x, (cy + .5) * CELL - y) for cx, cy in cells[room])

    rows = [json.loads(l) for p in paths for l in open(p)]
    by = collections.OrderedDict()
    for r in rows:
        by.setdefault(r["run_dir"], []).append(r)
    total = collections.Counter()
    for d, rs in by.items():
        lines = open(Path(d) / "explore_brain.log", errors="replace").read().splitlines()
        starts = [i for i, l in enumerate(lines) if "mission started:" in l]
        for r, i0, i1 in zip(rs, starts, starts[1:] + [len(lines)]):
            if r["job"] != "survey":
                continue
            ev = []
            for l in lines[i0:i1]:
                if '"event"' in l and " explore " in l:
                    try:
                        ev.append(json.loads(l[l.index("{"):]))
                    except ValueError:
                        pass
            missed = sorted(set(w.rooms) - set(r["rooms_truth"] or []))
            labels = sorted({e.get("room") for e in ev if e.get("room")})
            room_goals = collections.Counter((e.get("room"), e.get("event")) for e in ev
                                             if e.get("kind") == "room" or e.get("room"))
            print(f"\n{Path(d).name} mission {r['i']}: {r['outcome']} {r['seconds']} s; "
                  f"missed {missed}; robot room names seen {labels}")
            print("  room-goal events:", dict(room_goals) or "none")
            for room in missed:
                near = [e for e in ev if e.get("event") in ("goal_sent", "goal_ended") and e.get("goal")
                        and dist(room, *e["goal"]) <= NEAR_M]
                ends = collections.Counter((e.get("kind"), e.get("state")) for e in near
                                           if e["event"] == "goal_ended")
                sent = collections.Counter(e.get("kind") for e in near if e["event"] == "goal_sent")
                closest = min((dist(room, *e["pose"][:2]) for e in ev if e.get("pose")), default=None)
                total[(room, "surveys")] += 1
                total[(room, "with a goal near")] += bool(sent)
                print(f"  {room}: goals sent near {dict(sent) or 0}, ended {dict(ends) or '-'}, "
                      f"closest pose {closest and round(closest, 2)} m")
    print("\ntotals:", dict(total))


if __name__ == "__main__":
    main(sys.argv[1:])
