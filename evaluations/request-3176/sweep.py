# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/request-3176/sweep.py -- 3.176.B's bars S1-S7: a find's room hint
in the simulator. On 3.116's 100 selection houses, in both target modes,
through MissionRunner (3.96.C's "wired" spec: `explore_goal_policy: rooms`,
`find_room_labels` off), on 3.160.D's harness (`evaluations/room-names-3160d/
sweep.py`): the house's true rooms stored as the person's names under a fixed
`frame_id`. One arm at a time:

  dev       no frame_id, no hint (run on `dev` before the change: S6's reference)
  none      names, no hint
  right     names; the hint is the type of the room holding the target
            instance nearest the start by path (mode U: the backpack's room)
  wrong     names; a type present in the house holding no instance of the
            target, drawn by a seed of the house index (none: left out, counted)
  absent    names; the hint "garage" (no ProcTHOR house has one)
  nolabels  no frame_id; the right hint (S6: must equal dev)
  parsed    names; "find the {target}" parsed, no hint (S6: must equal none)

Every hint arm builds its request as "find the {target}, probably in the
{type}" and parses it with brain/request.py; a parse that does not give back
that target and type stops the mission before it runs (`parse_mismatch`,
counted). Each row carries the simulator's seconds at the end (`sim_s`).

    python evaluations/request-3176/sweep.py ARM OUT.json

`REPO` (environment) names the checkout whose brain runs (a `dev` worktree
for the `dev` arm); nothing is written but OUT.
"""
import json
import os
import random
import sys
from collections import deque
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.environ.get("REPO") or os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "evaluations", "goal-learn-396"))
sys.path.insert(0, REPO)
import train as T                                             # noqa: E402

FRAME = "map-3176b000"
ARMS = ("dev", "none", "right", "wrong", "absent", "nolabels", "parsed")
NAMES = {"none", "right", "wrong", "absent", "parsed"}        # arms with a frame_id and names
HINTED = {"right", "wrong", "absent", "nolabels"}
WRONG_SEED = 31760
_grid = []          # the house the current mission built (one mission a process at a time)
_runner = []
_info = {}


def room_of(grid, cell):
    for name, cells in grid.rooms.items():
        if cell in cells:
            return name
    return None


def nearest_instance(grid, target):
    """The target's cell nearest the start by path (4-connected, the
    harness's own passability), or None."""
    from sim.grid_world import CELL_WALL
    start = (int(grid.x), int(grid.y))
    seen, q = {start}, deque([start])
    while q:
        c = q.popleft()
        for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
            if n in seen:
                continue
            seen.add(n)
            if grid.objects.get(n) == target:
                return n
            if grid._cell(*n) != CELL_WALL and n not in grid.objects:
                q.append(n)
    return None


def hint_type(arm, grid, target, index):
    """The arm's room type for this house, or None (the arm has no hint here)."""
    from brain.goal_policy import room_type
    if arm == "absent":
        return "garage"
    have = {c for c, n in grid.objects.items() if n == target}
    holding = {room_type(room_of(grid, c)) for c in have if room_of(grid, c)}
    if arm in ("right", "nolabels"):
        c = nearest_instance(grid, target)
        r = room_of(grid, c) if c else None
        return room_type(r) if r else None
    if arm == "wrong":
        types = sorted({room_type(n) for n in grid.rooms} - holding)
        return random.Random(WRONG_SEED + index).choice(types) if types else None
    return None


def _patch(arm: str) -> None:
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
            target = k.get("target_object")
            if arm in HINTED or arm == "parsed":
                from brain.request import parse
                g, index = _grid[0], _info["index"]
                t = hint_type(arm, g, target, index) if arm in HINTED else None
                text = (f"find the {target}, probably in the {t}" if t
                        else f"find the {target}")
                p = parse(text)
                _info.update(text=text, hint=t)
                if arm in HINTED and t is None:
                    _info["skip"] = "no_hint"
                    raise _Skip()
                if p.target != target or (p.room_hint or {}).get("type") != t:
                    _info["skip"] = "parse_mismatch"
                    raise _Skip()
                k["target_object"] = p.target
                if p.room_hint:
                    k["room_hint"] = p.room_hint
                k["request"] = text
            super().__init__(*a, **k)
            _runner[:] = [self]

    T.MissionRunner = Runner
    if arm not in NAMES:
        return
    import brain.room_names as rn
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
            g = _grid[0]
            if self.memo is None or self.memo[0] is not g:
                k = round(T.CELL / rn.STORE_RES_M)
                self.memo = (g, [{"name": n, "_keys": {(i * k + a, j * k + b)
                                                       for i, j in cells
                                                       for a in range(k) for b in range(k)}}
                                 for n, cells in sorted(g.rooms.items())])
            return {"version": 1, "frame_id": frame_id, "rooms": self.memo[1],
                    "history": [], "rev": 1}

        def rooms(self, frame_id):
            return self.load(frame_id)["rooms"]

    store = Store()
    rn.default_store = lambda: store


class _Skip(Exception):
    pass


def _one(args):
    arm, s, h, mode = args
    _info.clear()
    _info["index"] = h
    _patch(arm)
    try:
        r = T.mission(s, h, mode, "wired", 0)
    except _Skip:
        r = {"outcome": "skipped", "target": None}
    row = {"split": s, "index": h, "mode": mode, "target": r.get("target"),
           "outcome": r["outcome"], "goals": r.get("goals"), "metres": r.get("metres"),
           "reward": r.get("reward"),
           "sim_s": round(_grid[0].sim_time, 2) if _grid else None}
    if _info.get("skip"):
        row.update(outcome=_info["skip"], goals=None, metres=None, reward=None)
    elif r.get("error"):
        row["error"] = r["error"]
    for k in ("text", "hint"):
        if k in _info:
            row[k] = _info[k]
    if _runner and not _info.get("skip"):
        st = _runner[0].status()
        row["person_rooms"] = (st.get("room_labels") or {}).get("person_rooms", 0)
        rh = st.get("room_hint")
        if rh:
            row["hint_acts"], row["hint_cells"] = rh["acts"], rh["cells"]
    return row


def main(arm: str, path: str) -> None:
    assert arm in ARMS, arm
    jobs = [(arm, s, h, m) for s, h in T.select_houses() for m in ("R", "U")]
    # One process per mission: an arm's patches stay in its own process.
    with ProcessPoolExecutor(int(os.environ.get("WORKERS", "4")), max_tasks_per_child=1) as ex:
        rows = list(ex.map(_one, jobs))
    report = {"arm": arm, "repo": REPO, "pairs": len(rows), "rows": rows}
    for m in ("R", "U"):
        rs = [r for r in rows if r["mode"] == m]
        report[m] = {"missions": len(rs), "found": sum(r["outcome"] == "found" for r in rs),
                     "exceptions": sum(r["outcome"] == "exception" for r in rs),
                     "skipped": sum(r["outcome"] in ("no_hint", "parse_mismatch") for r in rs)}
    T.save_atomic(path, lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps({k: v for k, v in report.items() if k != "rows"}))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
