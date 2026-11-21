# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/recall-other-side-3200/wide.py -- 3.200.C: R1 (a recall whose turns
to face it were all refused keeps its place for one goal from another side),
judged on an instrument rich in face refusals. How the bars are read is in
docs/plans/ros-alignment/3.200-recall-other-side.md, "3.200.C", written before
the R1 run.

3.70.M2's find (`evaluations/find-last-seen-370/sweep.py`'s `survey()` and
`find()`, unchanged: `MissionRunner` -> explore -> `robot/safety.py` ->
`MockRobot` on the simulator's clock, `FakeNav`, a real `InventoryStore`, the
survey's end map), widened: SURVEYS surveys of `complex_house` from its dock,
each with its own detector seed, and STARTS surveyed finds after each, from
spread starts of its own seed. The surveyed arm only: an unsurveyed find
carries no recall place, so the rule cannot reach it (3.200.B: 40 of 40
identical but for wall-clock `search_s`).

    python tools/desktop_lock.py shared --why "3.200.C sweep" -- \\
        python evaluations/recall-other-side-3200/wide.py run ROWS.jsonl WORKDIR WORKERS RULE [SURVEYS STARTS]
    (RULE: off -- the R1 code with the rule off, which counts refused faces;
     on -- R1; dev -- run from a checkout of dev, where the rule's attribute is never read)
    python evaluations/recall-other-side-3200/wide.py score ROWS.jsonl
    python evaluations/recall-other-side-3200/wide.py compare DEV.jsonl OFF.jsonl R1.jsonl
"""

import importlib.util
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
_spec = importlib.util.spec_from_file_location(
    "find_last_seen_370", ROOT / "evaluations" / "find-last-seen-370" / "sweep.py")
sweep = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sweep)

NAV = "fake"
SURVEYS = 6
STARTS = 40
SURVEY_SEED = 32000        # fresh: 3.70.M2 used 37020 + k
START_SEED = 32100
FIND_SEED = 32200


def _rule(rule):
    import brain.explore as ex
    ex.RECALL_FACE_RETRY = (rule == "on")


def plan(i, reported, starts=STARTS):
    from tests.inventory_sweep import starts as spread
    names = sweep.targets(reported)
    if not names:
        return []
    return [{"survey": i, "k": i * 1000 + k, "start": [cx, cy], "theta": theta, "room": room,
             "target": names[k % len(names)], "seed": FIND_SEED + i * 1000 + k}
            for k, ((cx, cy), theta, room) in enumerate(spread(starts, seed=START_SEED + i))]


def _survey(args):
    i, work, rule = args
    _rule(rule)
    d = Path(work) / f"survey-{i}"
    if d.exists():
        shutil.rmtree(d)
    d.mkdir(parents=True)
    row, cells = sweep.survey(d, seed=SURVEY_SEED + i, nav=NAV)
    return i, row, cells


def _find(args):
    m, work, cells, reported, rule = args
    _rule(rule)
    d = Path(work) / f"find-{m['k']}"
    if d.exists():
        shutil.rmtree(d)
    shutil.copytree(Path(work) / f"survey-{m['survey']}", d)
    r = sweep.find(m, "surveyed", d, cells, reported, nav=NAV)
    shutil.rmtree(d)
    return {**r, "survey": m["survey"], "rule": rule}


def run(out, work, workers, rule, surveys=SURVEYS, starts=STARTS):
    import multiprocessing as mp
    if Path(out).exists():
        sys.exit(f"{out} exists: rows are never overwritten (pinned data); name a new file")
    work = Path(work)
    work.mkdir(parents=True, exist_ok=True)
    with open(out, "w") as f, mp.Pool(workers) as pool:
        got = {}
        for i, row, cells in pool.imap_unordered(_survey, [(i, str(work), rule) for i in range(surveys)]):
            got[i] = (row, cells)
            plans = plan(i, row["reported"], starts)
            f.write(json.dumps({**row, "survey": i, "plan": plans, "nav": NAV, "rule": rule}) + "\n")
            f.flush()
            print(json.dumps({k: v for k, v in row.items() if k != "reported"}), flush=True)
        jobs = [(m, str(work), got[i][1], got[i][0]["reported"], rule)
                for i in range(surveys) for m in plan(i, got[i][0]["reported"], starts)]
        for r in pool.imap_unordered(_find, jobs):
            f.write(json.dumps(r) + "\n")
            f.flush()
            print(r["k"], r["target"], r["outcome"], r["search_s"], r["recall_unlooked"],
                  r["recall_retries"], flush=True)


def rows(path):
    with open(path) as f:
        rs = [json.loads(line) for line in f if line.strip()]
    surveys = {r["survey"]: r for r in rs if r.get("mission") == "survey"}
    finds = {r["k"]: r for r in rs if r.get("mission") != "survey"}
    return surveys, finds


def ok(r):
    return r["outcome"] == "found" and not r["wrong_found"]


def score(path):
    surveys, finds = rows(path)
    planned = sum(len(s["plan"]) for s in surveys.values())
    out = {"surveys": {i: {k: s[k] for k in ("outcome", "search_s", "contacts", "capped")}
                       | {"reported": len(s["reported"]), "planned": len(s["plan"])}
                       for i, s in sorted(surveys.items())},
           "finds": len(finds), "planned": planned,
           "found": sum(ok(r) for r in finds.values()),
           "face_refused": sum(r["recall_unlooked"] > 0 for r in finds.values()),
           "face_refused_events": sum(r["recall_unlooked"] for r in finds.values()),
           "face_refused_not_found": sorted(k for k, r in finds.items()
                                            if r["recall_unlooked"] and not ok(r)),
           "retries": sum(r["recall_retries"] for r in finds.values()),
           "spent": sum(r["recall_spent"] for r in finds.values()),
           "most_per_place": max((r["recall_most_per_place"] for r in finds.values()), default=0),
           "most_recall_goals": max((r["recall_goals"] for r in finds.values()), default=0),
           "wrong_found": sum(r["wrong_found"] for r in finds.values()),
           "contacts": sum(r["contacts"] for r in finds.values())
           + sum(s["contacts"] for s in surveys.values()),
           "capped": sum(r["capped"] for r in finds.values())
           + sum(s["capped"] for s in surveys.values())}
    print(json.dumps(out, indent=1))
    return out


SAME = ("outcome", "goals", "steps", "distance_m")
NOISE_S = 0.3              # the runner's wall-clock guard (M2's re-run 0.3 s, 3.200.B 0.2 s)


def same(a, b):
    """C5's reading: outcome, goals, steps and distance identical; search_s
    within NOISE_S."""
    return all(a[f] == b[f] for f in SAME) and abs(a["search_s"] - b["search_s"]) <= NOISE_S + 1e-9


def compare(dev_path, off_path, r1_path):
    """3.200.C's bars C1-C6 (C7, C8 are not in the rows)."""
    dsv, dev = rows(dev_path)
    osv, off = rows(off_path)
    rsv, r1 = rows(r1_path)
    strip = lambda s: {k: v for k, v in s.items() if k != "rule"}   # noqa: E731
    refused = {k for k, r in off.items() if r["recall_unlooked"]}
    found = {n: sum(ok(r) for r in x.values()) for n, x in (("dev", dev), ("off", off), ("R1", r1))}
    out = {"finds": {n: len(x) for n, x in (("dev", dev), ("off", off), ("R1", r1))},
           "refused": len(refused), "found": found,
           "C1": {"bar": found["dev"] + 4, "met": len(r1) == len(dev) == 240 and found["R1"] >= found["dev"] + 4},
           "won": sorted(k for k in r1 if ok(r1[k]) and not ok(dev[k])),
           "lost": sorted(k for k in r1 if ok(dev[k]) and not ok(r1[k])),
           "refused_found": {"dev": sum(ok(dev[k]) for k in refused), "R1": sum(ok(r1[k]) for k in refused)}}
    out["C2"] = {"met": not out["lost"]}
    every = list(r1.values())
    out["C3"] = {"wrong_found": sum(r["wrong_found"] for r in every),
                 "contacts": sum(r["contacts"] for r in every) + sum(s["contacts"] for s in rsv.values())}
    out["C3"]["met"] = out["C3"]["wrong_found"] == 0 and out["C3"]["contacts"] == 0
    out["C4"] = {"per_place": max(r["recall_most_per_place"] for r in every),
                 "per_mission": max(r["recall_goals"] for r in every),
                 "capped": sum(r["capped"] for r in every) + sum(s["capped"] for s in rsv.values()),
                 "retries": sum(r["recall_retries"] for r in every),
                 "spent": sum(r["recall_spent"] for r in every)}
    out["C4"]["met"] = out["C4"]["per_place"] <= 2 and out["C4"]["per_mission"] <= 6 and not out["C4"]["capped"]
    unref = sorted(set(dev) - refused)
    c5_diff = [k for k in unref if not same(dev[k], r1[k])]
    out["C5"] = {"missions": len(unref), "differ": c5_diff,
                 "max_search_s_delta": round(max(abs(dev[k]["search_s"] - r1[k]["search_s"]) for k in unref), 2),
                 "surveys_identical": {i: strip(s) for i, s in rsv.items()} == {i: strip(s) for i, s in dsv.items()}}
    out["C5"]["met"] = not c5_diff and out["C5"]["surveys_identical"]
    c6_diff = [k for k in dev if not same(dev[k], off[k])]
    out["C6"] = {"differ": c6_diff,
                 "max_search_s_delta": round(max(abs(dev[k]["search_s"] - off[k]["search_s"]) for k in dev), 2),
                 "surveys_identical": {i: strip(s) for i, s in osv.items()} == {i: strip(s) for i, s in dsv.items()}}
    out["C6"]["met"] = len(off) == len(dev) and not c6_diff and out["C6"]["surveys_identical"]
    print(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "run":
        run(a[1], a[2], int(a[3]), a[4], *(int(x) for x in a[5:7]))
    elif a and a[0] == "compare":
        compare(a[1], a[2], a[3])
    elif a and a[0] == "score":
        score(a[1])
    else:
        sys.exit(__doc__)
