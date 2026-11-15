# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/explore-view-3180/sweep.py -- 3.180.A's sweep: 3.176.B's harness
(`evaluations/request-3176/sweep.py`, 3.116's 100 selection houses x modes R
and U, the house's true rooms as a person's names) in four arms (`dev`,
`none`, `right`, `wrong`), each row adding the view goals sent and
`view_reuses` (3.180.A; null on a brain before it). Run once from a `dev`
worktree (the baseline) and once from this branch, paired by house and mode.

    REPO=<checkout> python evaluations/explore-view-3180/sweep.py ARM OUT.json
"""
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

REPO = os.environ.get("REPO") or os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "evaluations", "request-3176"))
import sweep as S                                             # noqa: E402
import subprocess                                             # noqa: E402
import brain                                                  # noqa: E402

# "." is on the path (the harness's imports): run from REPO's own directory,
# or the brain is another checkout's (3.180.A's first baseline was). Checked
# here too, not only in request-3176's harness: with REPO a `dev` worktree,
# `sweep` above is THAT tree's copy, which may predate its own check.
BRAIN = os.path.dirname(os.path.realpath(brain.__file__))
if not BRAIN.startswith(os.path.realpath(REPO) + os.sep):
    raise SystemExit(f"brain is {BRAIN}, not under REPO {REPO}: run from REPO's directory")

ARMS = ("dev", "none", "right", "wrong")


def _one(args):
    row = S._one(args)
    if S._runner and not S._info.get("skip") and row["outcome"] != "no_target":
        ag = S._runner[0].agent
        row["view_goals"] = len(getattr(ag, "_view_goals", ()))
        row["view_reuses"] = getattr(ag, "view_reuses", None)
    return row


def main(arm, path):
    assert arm in ARMS, arm
    jobs = [(arm, s, h, m) for s, h in S.T.select_houses() for m in ("R", "U")]
    with ProcessPoolExecutor(int(os.environ.get("WORKERS", "4")), max_tasks_per_child=1) as ex:
        rows = list(ex.map(_one, jobs))
    git = subprocess.run(["git", "-C", REPO, "rev-parse", "--short", "HEAD"],
                         capture_output=True, text=True).stdout.strip()
    report = {"arm": arm, "commit": git, "brain": BRAIN, "pairs": len(rows), "rows": rows}
    S.T.save_atomic(path, lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps({"arm": arm, "commit": git,
                      "found": sum(r["outcome"] == "found" for r in rows)}))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
