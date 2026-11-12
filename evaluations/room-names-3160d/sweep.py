# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/room-names-3160d/sweep.py -- 3.160.D's bars D1-D5: a find with no
labeller reads the person's room names. On 3.116's 100 selection houses, in
both target modes, through MissionRunner (3.96.C's "wired" spec:
`explore_goal_policy: rooms`, `find_room_labels` off), one arm at a time:

  dev    nothing patched (run on `dev` before the change: D4's reference)
  noid   the pose has no frame_id (D4: must equal dev)
  empty  a frame_id, and a store with no rooms for it (D4: must equal dev)
  names  a frame_id, and the house's true rooms stored as the person's names

The person's names stand in as the grid's own rooms (each 0.30 m grid cell's
store cells), under the fixed frame_id FRAME. Nothing is written to disk but
OUT.

    python evaluations/room-names-3160d/sweep.py ARM OUT.json
"""
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "evaluations", "goal-learn-396"))
sys.path.insert(0, REPO)
import train as T                                             # noqa: E402

FRAME = "map-3160d000"
ARMS = ("dev", "noid", "empty", "names")
_grid = []          # the house the current mission built (one mission a process at a time)
_runner = []


def _patch(arm: str) -> None:
    if arm == "dev":
        return
    import brain.room_names as rn
    import control.mission_runner as mr
    from sim.mock_world import MockWorld

    real_build = T.build_world

    def build(name):
        g = real_build(name)
        _grid[:] = [g]
        return g

    T.build_world = build

    class Runner(mr.MissionRunner):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            _runner[:] = [self]

    T.MissionRunner = Runner
    if arm == "noid":
        return
    real_pose = MockWorld.get_pose

    def pose(self):
        p = real_pose(self)
        return {**p, "frame_id": FRAME} if p.get("usable") else p

    MockWorld.get_pose = pose

    class Store:
        def __init__(self):
            self.memo = None

        def load(self, frame_id):
            assert frame_id == FRAME, frame_id
            rooms = []
            if arm == "names":
                g = _grid[0]
                if self.memo is None or self.memo[0] is not g:
                    k = round(T.CELL / rn.STORE_RES_M)
                    self.memo = (g, [{"name": n, "_keys": {(i * k + a, j * k + b)
                                                           for i, j in cells
                                                           for a in range(k) for b in range(k)}}
                                     for n, cells in sorted(g.rooms.items())])
                rooms = self.memo[1]
            return {"version": 1, "frame_id": frame_id, "rooms": rooms, "history": [], "rev": 1}

        def rooms(self, frame_id):
            return self.load(frame_id)["rooms"]

    store = Store()
    rn.default_store = lambda: store


def _one(args):
    arm, s, h, mode = args
    _patch(arm)
    r = T.mission(s, h, mode, "wired", 0)
    row = {"split": s, "index": h, "mode": mode, "target": r.get("target"),
           "outcome": r["outcome"], "goals": r.get("goals"), "metres": r.get("metres"),
           "reward": r.get("reward")}
    if r.get("error"):
        row["error"] = r["error"]
    if _runner:
        run = _runner[0]
        st = (run.status().get("room_labels") or {})
        row["asks"] = (run.room_labeller.calls if run.room_labeller else 0)
        row["person_rooms"] = st.get("person_rooms", 0)
    return row


def main(arm: str, path: str) -> None:
    assert arm in ARMS, arm
    jobs = [(arm, s, h, m) for s, h in T.select_houses() for m in ("R", "U")]
    # One process per mission: an arm's patches stay in its own process.
    with ProcessPoolExecutor(int(os.environ.get("WORKERS", "4")), max_tasks_per_child=1) as ex:
        rows = list(ex.map(_one, jobs))
    report = {"arm": arm, "pairs": len(rows), "rows": rows}
    for m in ("R", "U"):
        rs = [r for r in rows if r["mode"] == m and r["outcome"] != "no_target"]
        report[m] = {"missions": len(rs), "found": sum(r["outcome"] == "found" for r in rs),
                     "exceptions": sum(r["outcome"] == "exception" for r in rs)}
    T.save_atomic(path, lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps({k: v for k, v in report.items() if k != "rows"}))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
