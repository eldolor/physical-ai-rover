# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/find-last-seen-370/stale.py -- 3.70.M3: a memory that is out of
date costs a detour, never a wrong Found. M2's sweep (`sweep.py`, `FakeNav`,
amendment 1) with every target moved to another room after the survey. How the
bars are read is in `docs/plans/ros-alignment/3.70-survey-and-mission-ends.md`,
"M3: how it is measured", written before any run.

    python tools/desktop_lock.py shared --why "3.70.M3 sweep" -- \\
        python evaluations/find-last-seen-370/stale.py run ROWS.jsonl WORKDIR [WORKERS [trust]]
    (trust: the red run -- the surveyed arm only, under a policy that trusts the memory)
    python evaluations/find-last-seen-370/stale.py score ROWS.jsonl

The live find ("M3's live find: how it is measured"):

    eval "$(python tools/plan_section.py env)"
    PICAR_ROS_NETWORK=host python tools/desktop_lock.py exclusive --why "3.70.M3 live" -- \\
        python evaluations/find-last-seen-370/stale.py live
    python evaluations/find-last-seen-370/stale.py live-score
"""

import importlib.util
import json
import math
import random
import shutil
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

_spec = importlib.util.spec_from_file_location("find_last_seen_370", Path(__file__).parent / "sweep.py")
sweep = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sweep)

NAV = "fake"               # amendment 1's navigator
MOVE_SEED = 37030
STALE_M = 0.9              # the stale landmark: a reported X within this of X's old place
REPORT_BELIEF = 0.8        # brain/inventory.py's; beside only


def _moved_3175():
    spec = importlib.util.spec_from_file_location(
        "memory_moved_3175", ROOT / "evaluations" / "memory-moved-3175" / "sweep.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def move_for(m, at=None):
    """X's new cell for plan entry `m`: another room, then a cell in it, both
    drawn from Random(MOVE_SEED + k); the draws `move_object` would refuse
    (the robot's turning circle at its start, or at `at`, (x, y) in cells,
    where the live robot stands) are skipped."""
    from sim.grid_world import movers_point_to_cell
    from sim.maps import build_world
    moved = _moved_3175()
    grid = build_world(sweep.HOUSE)
    grid.x, grid.y = at if at is not None else (m["start"][0] + 0.5, m["start"][1] + 0.5)
    p0 = next(c for c, n in grid.objects.items() if n == m["target"])
    room = grid.room_at(*p0)
    by_room = {}
    for c in moved.open_cells(grid, moved.floor_from(grid, (grid.robot_x, grid.robot_y))):
        by_room.setdefault(grid.room_at(*c), []).append(c)
    rng = random.Random(MOVE_SEED + m["k"])
    rooms = sorted(r for r in by_room if r not in (room, "unknown"))
    while rooms:
        r = rooms.pop(rng.randrange(len(rooms)))
        cells = list(by_room[r])
        while cells:
            c = cells.pop(rng.randrange(len(cells)))
            if movers_point_to_cell(grid.x, grid.y, c) >= grid._turning_circle_cells():
                return {"p0": list(p0), "p1": list(c), "from_room": room, "to_room": r}
    raise ValueError(f"no cell to move {m['target']} to")


class _Mover:
    """`prepare`: picklable, moves X in the find's own world."""

    def __init__(self, mv):
        self.mv = mv

    def __call__(self, grid):
        grid.move_object(tuple(self.mv["p0"]), tuple(self.mv["p1"]))


def _trust(runner, name, fields):
    """The red run's policy: memory taken for sight. A recall reached (its goal
    succeeded, or faced where the robot stands) marks the mission found."""
    if (name == "goal_ended" and fields.get("kind") == "recall"
            and fields.get("state") == "succeeded") or (name == "recall" and fields.get("here")):
        runner.memory.found = True


def _job(args):
    m, arm, work, cells, reported, trust = args
    d = Path(work) / f"{arm}-{m['k']}"
    if d.exists():
        shutil.rmtree(d)
    if arm == "surveyed":
        shutil.copytree(Path(work) / "survey", d)
    else:
        d.mkdir(parents=True)
    r = sweep.find(m, arm, d, cells if arm == "surveyed" else None, reported, NAV,
                   prepare=_Mover(m["move"]), watch=_trust if trust else None)
    # Measured to the NEW place (sweep.find reads the target where it stands).
    return {**r, "move": m["move"], "trust": trust}


def stale_landmark(reported, target, p0):
    """The survey's reported landmark of X nearest X's old cell, within STALE_M."""
    old = ((p0[0] + 0.5) * sweep.CELL, (p0[1] + 0.5) * sweep.CELL)
    near = [(math.dist(old, (r["x_m"], r["y_m"])), r) for r in reported
            if r["label"] == target and math.dist(old, (r["x_m"], r["y_m"])) <= STALE_M]
    return min(near, key=lambda t: t[0])[1] if near else None


def end_belief(row, lm):
    """The stale landmark's belief in the find's end report: the X landmark
    nearest its survey place within inventory's MERGE_M (it is not seen
    again, so it does not move), or None if the report no longer holds it."""
    near = [(math.dist((lm["x_m"], lm["y_m"]), (e["x_m"], e["y_m"])), e) for e in row["end_landmarks"]]
    near = [t for t in near if t[0] <= 0.5]
    return min(near, key=lambda t: t[0])[1]["belief"] if near else None


def run(out, work, workers, trust=False):
    import multiprocessing as mp
    work = Path(work)
    sdir = work / "survey"
    if Path(out).exists():
        sys.exit(f"{out} exists: rows are never overwritten (pinned data); name a new file")
    if sdir.exists():
        shutil.rmtree(sdir)
    sdir.mkdir(parents=True)
    row, cells = sweep.survey(sdir, nav=NAV)
    print(json.dumps({k: v for k, v in row.items() if k != "reported"}), flush=True)
    plans = [{**m, "move": move_for(m)} for m in sweep.plan(row["reported"])]
    arms = ("surveyed",) if trust else sweep.ARMS
    with open(out, "w") as f:
        f.write(json.dumps({**row, "plan": plans, "nav": NAV, "trust": trust}) + "\n")
        jobs = [(m, arm, str(work), cells, row["reported"], trust) for m in plans for arm in arms]
        with mp.Pool(workers) as pool:
            for r in pool.imap_unordered(_job, jobs):
                f.write(json.dumps(r) + "\n")
                f.flush()
                print(r["arm"], r["k"], r["target"], r["move"]["to_room"], r["outcome"],
                      r["target_m"], r["first_act"], flush=True)


def score(path):
    with open(path) as f:
        rows = [json.loads(line) for line in f if line.strip()]
    sv = rows[0]
    arms = {a: sorted((r for r in rows[1:] if r.get("arm") == a), key=lambda r: r["k"])
            for a in sweep.ARMS}
    s, u = arms["surveyed"], arms["unsurveyed"]
    n = sweep.MISSIONS
    found = {a: sum(r["outcome"] == "found" and not r["wrong_found"] for r in arms[a]) for a in sweep.ARMS}
    out = {"trust": sv.get("trust", False), "nav": sv.get("nav"),
           "survey": {k: sv[k] for k in ("outcome", "search_s", "contacts", "capped")},
           "survey_reported": len(sv["reported"]),
           "targets": sorted({m["target"] for m in sv["plan"]}),
           "moved": all(m["move"]["from_room"] != m["move"]["to_room"] for m in sv["plan"]),
           "missions": {a: len(arms[a]) for a in sweep.ARMS}}
    out["K1_found"] = {**found, "bar": 19,
                       "met": (len(s) == n and len(u) == n and found["surveyed"] >= 19
                               and found["surveyed"] >= found["unsurveyed"])}
    fell, under, none, ends = 0, 0, [], []
    for r in s:
        lm = stale_landmark(sv["reported"], r["target"], r["move"]["p0"])
        if lm is None:
            none.append(r["k"])
            continue
        b = end_belief(r, lm)
        ends.append([r["k"], lm["belief"], b])
        fell += b is None or b < lm["belief"]
        under += b is None or b < REPORT_BELIEF
    out["K2_belief"] = {"fell": fell, "of": len(s) - len(none), "no_stale_landmark": none,
                        "under_report": under, "ends": ends,
                        "met": len(s) == n and fell == len(s) - len(none)}
    every = rows[1:]
    out["K3_memory_alone"] = {"wrong_found": {a: sum(r["wrong_found"] for r in arms[a]) for a in sweep.ARMS},
                              "wrong": [[r["arm"], r["k"], r["target"], r["target_m"]]
                                        for r in every if r["wrong_found"]],
                              "met": len(s) == n and len(u) == n and not any(r["wrong_found"] for r in every)}
    out["K4_contacts"] = {"contacts": sum(r["contacts"] for r in every) + sv["contacts"],
                          "met": len(s) == n and len(u) == n
                          and not any(r["contacts"] for r in every) and not sv["contacts"]}
    out["beside"] = {a: {"median_goals": statistics.median(r["goals"] for r in arms[a]),
                         "median_steps": statistics.median(r["steps"] for r in arms[a]),
                         "median_s": statistics.median(r["search_s"] for r in arms[a]),
                         "outcomes": sorted({r["outcome"] for r in arms[a]}),
                         "misses": [[r["k"], r["target"], r["outcome"]] for r in arms[a]
                                    if not (r["outcome"] == "found" and not r["wrong_found"])]}
                     for a in sweep.ARMS if arms[a]}
    out["first_recall"] = sum(r["first_recall_ok"] for r in s)
    print(json.dumps(out, indent=1))
    return out


# ---------------------------------------------------------------- the live find

LIVE_ROWS = Path(__file__).parent / "live-m3.jsonl"
LIVE_FINDS = 1
LIVE_LIMIT_S = 1200.0     # the sim's find limit (3.175's)


def _live_move(m, truth):
    at = None
    if truth.get("x_m") is not None and truth.get("y_m") is not None:
        at = (truth["x_m"] / sweep.CELL, truth["y_m"] / sweep.CELL)
    return move_for(m, at)


def live_run(rows=LIVE_ROWS):
    """M3's live find: M2's live harness (`live.py`), one find (plan entry
    k 0), its target moved by `move_for` before the find starts."""
    spec = importlib.util.spec_from_file_location("live_370", Path(__file__).parent / "live.py")
    live = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(live)
    live.run(rows=rows, finds=LIVE_FINDS, find_limit_s=LIVE_LIMIT_S, move=_live_move)


def live_score(path=LIVE_ROWS):
    """The bars of "M3's live find: how it is measured"."""
    rows = [json.loads(x) for x in Path(path).read_text().splitlines() if x.strip()]
    s1 = next((r for r in rows if r.get("row") == "survey"), None)
    s2 = next((r for r in rows if r.get("row") == "reload"), None)
    finds = [r for r in rows if r.get("row") == "find"]
    map_id = (s1 or {}).get("map_id")
    f = finds[0] if len(finds) == 1 else None
    pre = {"survey_saved_one_map": bool(s1 and map_id and s1.get("report_frame_id") == map_id
                                        and s1.get("targets")),
           "reload_matched": bool(s2 and (s2.get("reload") or {}).get("result") == "matched"
                                  and s2.get("map_id") == map_id),
           "one_find_moved": bool(f and (f.get("moved") or {}).get("status") == 200
                                  and f["moved"]["from_room"] != f["moved"]["to_room"]),
           "find_on_the_reload": bool(f and (f.get("saved_map") or {}).get("frame_id") == map_id
                                      and (f.get("saved_map") or {}).get("reloaded") is True)}
    valid = all(pre.values())
    verdict = (lambda ok: ("MET" if ok else "FAILED") if valid else "INVALID")
    f = f or {}
    found_ok = f.get("outcome") == "found" and f.get("found_m") is not None and f["found_m"] <= sweep.FOUND_M
    wrong = f.get("outcome") == "found" and f.get("found_m") is not None and f["found_m"] > sweep.FOUND_M
    stale_lm = (stale_landmark(s1["reported_rows"], f["target"], f["moved"]["p0"])
                if valid else None)
    end_b = end_belief(f, stale_lm) if (stale_lm and "end_landmarks" in f) else None
    return {"preconditions": pre, "valid": valid,
            "LM1_found": {"found_m": f.get("found_m"), "outcome": f.get("outcome"), "verdict": verdict(found_ok)},
            "LM2_stability": {"contacts": f.get("contacts"), "wrong_found": wrong,
                              "failed": f.get("outcome") in ("failed", "start_refused"),
                              "verdict": verdict(f.get("contacts") == 0 and not wrong
                                                 and f.get("outcome") not in ("failed", "start_refused"))},
            "beside": {"target": f.get("target"), "moved": f.get("moved"), "first_act": f.get("first_act"),
                       "first_recall_ok": f.get("first_recall_ok"), "resight": f.get("resight"),
                       "stale_belief": [stale_lm and stale_lm["belief"], end_b],
                       "seconds": f.get("seconds"), "mission_s": f.get("mission_s"),
                       "goals": f.get("goals"), "distance_m": f.get("distance_m"),
                       "min_gap_cm": f.get("min_gap_cm"), "ret": f.get("ret"),
                       "end_reason": f.get("end_reason"), "wall_capped": f.get("wall_capped", False)}}


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "run":
        run(a[1], a[2], int(a[3]) if len(a) > 3 else 4, len(a) > 4 and a[4] == "trust")
    elif a and a[0] == "score":
        score(a[1])
    elif a and a[0] == "live":
        live_run()
    elif a and a[0] == "live-score":
        print(json.dumps(live_score(Path(a[1]) if len(a) > 1 else LIVE_ROWS), indent=1))
    else:
        sys.exit(__doc__)
