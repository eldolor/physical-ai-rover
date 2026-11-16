# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/harness-repo-3186/rerun.py -- 3.186.B: re-run a merged result's
`dev` arm with the brain of the commit it names, and compare.

The `dev` arms of 3.160.D's D4 and 3.116.B's criterion 7 are both
`train.mission(split, index, mode, "wired", 0)` on 3.116's 100 selection
houses in modes R and U. Their records do not say where the `dev` arm was
launched, and before 3.186 `evaluations/explore-365/harness.py` put the
launch directory first on `sys.path` (3.180's candidates). Run this from
INSIDE a worktree at the recorded commit, with REPO naming that worktree:

    cd <worktree at C> && REPO=$PWD python <this file> OUT.json RECORD.json KEY C

It refuses unless REPO's HEAD is C and its tree is clean, and unless `brain`
loads from REPO; it records the brain path and REPO's commit, and compares (outcome, goals, metres) against the record's
rows (KEY: `rows` for a sweep report, `rows_dev` for off-is-off-select).
"""
import json
import os
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor

REPO = os.path.realpath(os.environ["REPO"])
sys.path.insert(0, os.path.join(REPO, "evaluations", "goal-learn-396"))
sys.path.insert(0, REPO)
import train as T                                             # noqa: E402
import brain                                                  # noqa: E402

BRAIN = os.path.dirname(os.path.realpath(brain.__file__))
if not BRAIN.startswith(REPO + os.sep):
    raise SystemExit(f"brain is {BRAIN}, not under REPO {REPO}")


def _git(*args) -> str:
    return subprocess.run(["git", "-C", REPO, *args], capture_output=True, text=True,
                          check=True).stdout.strip()


def _one(job):
    s, h, mode = job
    r = T.mission(s, h, mode, "wired", 0)
    return [s, h, mode, r["outcome"], r.get("goals"), r.get("metres")]


def _key(row):
    if isinstance(row, dict):
        return (row["split"], row["index"], row["mode"]), [row["outcome"], row.get("goals"), row.get("metres")]
    return tuple(row[:3]), list(row[3:6])


def main(out: str, record: str, key: str, expect: str) -> None:
    commit = _git("rev-parse", "--short=8", "HEAD")
    if not _git("rev-parse", "HEAD").startswith(expect) or _git("status", "--porcelain"):
        raise SystemExit(f"REPO {REPO} is at {commit} (want {expect}) or not clean")
    jobs = [(s, h, m) for s, h in T.select_houses() for m in ("R", "U")]
    # One mission a fresh process, as 3.160.D's sweep ran its arms; each mission
    # builds its own house, robot and runner, so the pool cannot change a row.
    with ProcessPoolExecutor(int(os.environ.get("WORKERS", "4")), max_tasks_per_child=1) as ex:
        rows = list(ex.map(_one, jobs))
    rec = dict(_key(r) for r in json.load(open(record))[key])
    differ = [r for r in rows if rec.get(tuple(r[:3])) != r[3:6]]
    report = {"repo_commit": commit, "brain": BRAIN, "record": os.path.basename(record),
              "key": key, "pairs": len(rows), "matched": len(rows) - len(differ),
              "differ": differ, "rows": rows}
    json.dump(report, open(out, "w"), indent=0)
    print(json.dumps({k: v for k, v in report.items() if k not in ("rows", "differ")}),
          "differ:", len(differ))


if __name__ == "__main__":
    main(*sys.argv[1:5])
