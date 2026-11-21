# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/carried-no-memory-3205/sweep.py -- 3.205.A: a find on a saved map
whose inventory has no memory of the target. M2's sweep
(`evaluations/find-last-seen-370/sweep.py`, `FakeNav`, amendment 1) imported
unchanged; its surveyed arm with every saved row naming X removed before the
find (`carried`), beside its unsurveyed arm. How the bars are read is in
`docs/plans/ros-alignment/3.205-carried-map-no-memory.md`, "3.205.A:
criteria", written before any run.

    python tools/desktop_lock.py shared --why "3.205.A sweep" -- \\
        python evaluations/carried-no-memory-3205/sweep.py run ROWS.jsonl WORKDIR [WORKERS]
    python evaluations/carried-no-memory-3205/sweep.py score ROWS.jsonl
    python evaluations/carried-no-memory-3205/sweep.py same A.jsonl B.jsonl   (N4: row identity)
"""

import importlib.util
import json
import shutil
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

_spec = importlib.util.spec_from_file_location(
    "find_last_seen_370", ROOT / "evaluations" / "find-last-seen-370" / "sweep.py")
sweep = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sweep)

NAV = "fake"               # amendment 1's navigator
ARMS = ("carried", "unsurveyed")
SAME = ("outcome", "search_s", "goals", "steps", "target_m", "contacts", "end_reason")


def drop_target(store_dir, target):
    """Every saved row the found rule names as `target` removed from every
    report in `store_dir`; returns how many were removed. Review: only a
    report that changed is rewritten, through a temporary file and a rename,
    its mtime kept (the store's head breaks a `carry_seq` tie by mtime)."""
    import os
    from brain.memory import MissionMemory
    names = MissionMemory(mission="3.205", target_object=target).names_target
    gone = 0
    for p in sorted(Path(store_dir).glob("*.json")):
        rep = json.loads(p.read_text())
        if not isinstance(rep, dict):
            continue
        n = 0
        for key in ("reported", "candidates"):
            rows = rep.get(key) or []
            keep = [r for r in rows if not names(r.get("label"))]
            n += len(rows) - len(keep)
            rep[key] = keep
        if n:
            st = p.stat()
            tmp = p.with_name(f".{p.name}.tmp")
            tmp.write_text(json.dumps(rep, indent=1, sort_keys=True))
            os.replace(tmp, p)
            os.utime(p, ns=(st.st_atime_ns, st.st_mtime_ns))
            gone += n
    return gone


class _Resight:
    """`watch`: counts explore's `resight` events (picklable)."""

    def __init__(self):
        self.n = 0

    def __call__(self, runner, name, fields):
        self.n += name == "resight"


def _job(args):
    m, arm, work, cells, reported = args
    d = Path(work) / f"{arm}-{m['k']}"
    if d.exists():
        shutil.rmtree(d)
    dropped = 0
    if arm == "carried":
        shutil.copytree(Path(work) / "survey", d)
        dropped = drop_target(d, m["target"])
    else:
        d.mkdir(parents=True)
    watch = _Resight()
    r = sweep.find(m, arm, d, cells if arm == "carried" else None, reported, NAV, watch=watch)
    return {**r, "dropped": dropped, "resight": watch.n}


def run(out, work, workers):
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
    plans = sweep.plan(row["reported"])
    with open(out, "w") as f:
        f.write(json.dumps({**row, "plan": plans, "nav": NAV}) + "\n")
        jobs = [(m, arm, str(work), cells, row["reported"]) for m in plans for arm in ARMS]
        with mp.Pool(workers) as pool:
            for r in pool.imap_unordered(_job, jobs):
                f.write(json.dumps(r) + "\n")
                f.flush()
                print(r["arm"], r["k"], r["target"], r["outcome"], r["search_s"], r["goals"],
                      r["resight"], flush=True)


def _rows(path):
    with open(path) as f:
        rows = [json.loads(line) for line in f if line.strip()]
    return rows[0], {a: sorted((r for r in rows[1:] if r.get("arm") == a), key=lambda r: r["k"])
                     for a in ARMS}


def score(path):
    sv, arms = _rows(path)
    c, u = arms["carried"], arms["unsurveyed"]
    n = sweep.MISSIONS
    ok = lambda r: r["outcome"] == "found" and not r["wrong_found"]     # noqa: E731
    found = {a: sum(ok(r) for r in arms[a]) for a in ARMS}
    every = c + u
    out = {"nav": sv.get("nav"), "survey": {k: sv[k] for k in ("outcome", "search_s", "contacts", "capped")},
           "survey_reported": len(sv["reported"]), "targets": sorted({m["target"] for m in sv["plan"]}),
           "missions": {a: len(arms[a]) for a in ARMS},
           "dropped_min": min((r["dropped"] for r in c), default=None)}
    out["N1_exhausted_unfound"] = {"carried": [r["k"] for r in c if r["outcome"] == "exhausted" and not ok(r)],
                                   "unsurveyed": [r["k"] for r in u if r["outcome"] == "exhausted" and not ok(r)]}
    out["N2_found"] = {**found, "met_vs_unsurveyed": len(c) == n and len(u) == n
                       and found["carried"] >= found["unsurveyed"]}
    out["N3"] = {"wrong_found": sum(r["wrong_found"] for r in every),
                 "contacts": sum(r["contacts"] for r in every) + sv["contacts"],
                 "met": len(c) == n and len(u) == n and not any(r["wrong_found"] or r["contacts"] for r in every)
                 and not sv["contacts"]}
    out["beside"] = {a: {"median_goals": statistics.median(r["goals"] for r in arms[a]),
                         "median_steps": statistics.median(r["steps"] for r in arms[a]),
                         "median_s": statistics.median(r["search_s"] for r in arms[a]),
                         "resight": sum(r.get("resight", 0) for r in arms[a]),
                         "outcomes": {o: sum(r["outcome"] == o for r in arms[a])
                                      for o in sorted({r["outcome"] for r in arms[a]})},
                         "misses": [[r["k"], r["target"], r["outcome"]] for r in arms[a] if not ok(r)]}
                     for a in ARMS if arms[a]}
    print(json.dumps(out, indent=1))
    return out


def same(a, b, keys=SAME):
    """N4: rows of two runs of one harness, matched by (arm, k) (or by
    position for a run's first row, the survey), the fields that differ."""
    def load(p):
        rows = [json.loads(x) for x in Path(p).read_text().splitlines() if x.strip()]
        out = {("survey",): rows[0], **{(r.get("arm"), r.get("k")): r for r in rows[1:]}}
        assert len(out) == len(rows), f"{p}: rows share a key"      # review: none collapse
        return out
    ra, rb = load(a), load(b)
    diff = {}
    for key in sorted(set(ra) | set(rb), key=str):
        x, y = ra.get(key), rb.get(key)
        if x is None or y is None:
            diff[str(key)] = "missing"
            continue
        fields = keys if key != ("survey",) else ("outcome", "search_s", "contacts", "reported")
        d = [f for f in fields if x.get(f) != y.get(f)]
        if d:
            diff[str(key)] = d
    out = {"rows": len(ra), "differ": diff, "identical": not diff and len(ra) == len(rb)}
    print(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "run":
        run(a[1], a[2], int(a[3]) if len(a) > 3 else 4)
    elif a and a[0] == "score":
        score(a[1])
    elif a and a[0] == "same":
        same(a[1], a[2])
    else:
        sys.exit(__doc__)
