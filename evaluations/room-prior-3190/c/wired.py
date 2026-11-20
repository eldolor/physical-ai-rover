# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/room-prior-3190/c/wired.py -- 3.190.C, criteria 8 and 9, on
3.116's 100 selection houses, one mission a fresh process. Free: no call.

  off      criterion 8: `find_prior_source` off -- MissionRunner with no
           `room_prior`, 3.96.C's "wired" spec (`explore_goal_policy: rooms`,
           no labels), modes R and U. Run once on `dev` before the change and
           once on the branch (REPO names the checkout; as 3.186's rerun.py).
  harness  criterion 9's reference: 3.190.A's cloud arm (3.116.C's replies in
           T.PRIOR, F-large greedy, the stand-in labeller) on these houses,
           mode R.
  wired    criterion 9: the brain's path -- the brain's cache seeded with
           3.116.C's `llm_prior.json`, read by `brain.room_prior.prior_at_start`
           (source `cache`), handed to MissionRunner as `room_prior`; the room
           types from a person's names on the saved map (3.160.D's oracle
           names: each house's true rooms), `explore_goal_policy: rooms`.
  shipped  the decomposition: `wired` with no room_prior (the shipped prior)
           beside `harness-shipped` (the stand-in labeller, the shipped
           prior): what the label source alone changes.
  harness-ask / wired-ask   amendment 3's same-labeller pair: 3.116.A's
           `ask` arm (labels asked of the frame, uncapped) with 3.116.C's
           replies in T.PRIOR, against MissionRunner with `find_room_labels`
           (frame, uncapped: 3.116.B's "wired-ask") and the cache's prior as
           `room_prior` -- the pair 3.116.B found equal, 100 of 100.

    REPO=$PWD python evaluations/room-prior-3190/c/wired.py ARM OUT.json

`compare A.json B.json` counts the missions that differ in outcome, goals or
metres.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.realpath(os.environ.get("REPO") or os.path.dirname(os.path.dirname(os.path.dirname(HERE))))
sys.path.insert(0, os.path.join(REPO, "evaluations", "goal-learn-396"))
sys.path.insert(0, REPO)
import train as T                                             # noqa: E402

ARMS = ("off", "harness", "harness-shipped", "wired", "shipped", "harness-ask", "wired-ask")
SEED = os.path.join(REPO, "evaluations", "room-prior-3116", "llm_prior.json")
NAMES = os.path.join(REPO, "evaluations", "room-names-3160d", "sweep.py")
_runner = []


def _commit() -> str:
    return subprocess.run(["git", "-C", REPO, "rev-parse", "--short=8", "HEAD"],
                          capture_output=True, text=True, check=True).stdout.strip()


def _names() -> None:
    """3.160.D's patches (a frame_id on the pose, each house's true rooms as
    the person's names), loaded from its sweep."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("sweep3160d", NAMES)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod._patch("names")


def _wire(with_prior: bool, names: bool = True) -> None:
    from brain import room_prior as rp
    if names:
        _names()
    base = T.MissionRunner        # 3.160.D's Runner subclass, after _names()
    cache = None
    if with_prior:
        d = tempfile.mkdtemp(prefix="rp3190c-")
        cache = os.path.join(d, "room-prior.json")
        shutil.copy(SEED, cache)

    class Runner(base):
        def __init__(self, robot, **k):
            if with_prior:
                st = rp.prior_at_start(k["target_object"], None, path=cache)
                if st["source"] != "cache":
                    raise SystemExit(f"{k['target_object']!r}: {st}")
                k["room_prior"] = st
            super().__init__(robot, **k)
            _runner[:] = [self]

    T.MissionRunner = Runner


def _harness_prior(cloud: bool) -> None:
    sys.path.insert(0, os.path.join(REPO, "evaluations", "room-prior-3190"))
    import arms
    if cloud:
        arms._use("cloud")


def _one(args):
    arm, s, h, mode = args
    if arm == "off":
        r = T.mission(s, h, mode, "wired", 0)
    elif arm in ("harness", "harness-shipped", "harness-ask"):
        _harness_prior(arm != "harness-shipped")
        sys.path.insert(0, os.path.join(REPO, "evaluations", "room-prior-3116"))
        import ask as A
        spec, drop, labels = A._spec("ask" if arm == "harness-ask" else "stand-in")
        r = T.mission(s, h, mode, spec, 0, drop, labels=labels)
    elif arm == "wired-ask":
        _wire(True, names=False)
        r = T.mission(s, h, mode, "wired-ask", 0)
    else:
        _wire(arm == "wired")
        r = T.mission(s, h, mode, "wired", 0)
    row = {"split": s, "index": h, "mode": mode, "target": r.get("target"),
           "outcome": r["outcome"], "goals": r.get("goals"), "metres": r.get("metres"),
           "reward": r.get("reward")}
    if r.get("error"):
        row["error"] = r["error"]
    if _runner:
        st = _runner[0].status()
        row["room_prior"] = st.get("room_prior")
        row["person_rooms"] = (st.get("room_labels") or {}).get("person_rooms", 0)
    return row


def main(arm: str, out: str) -> None:
    assert arm in ARMS, arm
    modes = ("R", "U") if arm == "off" else ("R",)
    jobs = [(arm, s, h, m) for s, h in T.select_houses() for m in modes]
    with ProcessPoolExecutor(int(os.environ.get("WORKERS", "4")), max_tasks_per_child=1) as ex:
        rows = list(ex.map(_one, jobs))
    report = {"arm": arm, "commit": _commit(), "repo": REPO, "pairs": len(rows), "rows": rows}
    for m in modes:
        rs = [r for r in rows if r["mode"] == m and r["outcome"] != "no_target"]
        report[m] = {"missions": len(rs), "found": sum(r["outcome"] == "found" for r in rs),
                     "exceptions": sum(r["outcome"] == "exception" for r in rs)}
    T.save_atomic(out, lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps({k: v for k, v in report.items() if k != "rows"}))


def compare(a: str, b: str) -> dict:
    ka = {(r["split"], r["index"], r["mode"]): r for r in json.load(open(a))["rows"]}
    kb = {(r["split"], r["index"], r["mode"]): r for r in json.load(open(b))["rows"]}
    if set(ka) != set(kb):
        raise SystemExit("the two runs hold different missions")
    f = ("outcome", "goals", "metres")
    differ = [[*k, [ka[k][x] for x in f], [kb[k][x] for x in f]]
              for k in sorted(ka) if [ka[k][x] for x in f] != [kb[k][x] for x in f]]
    out = {"a": os.path.basename(a), "b": os.path.basename(b), "missions": len(ka),
           "differ": len(differ), "rows": differ}
    print(json.dumps({k: v for k, v in out.items() if k != "rows"}))
    return out


if __name__ == "__main__":
    if sys.argv[1] == "compare":
        compare(sys.argv[2], sys.argv[3])
    else:
        main(sys.argv[1], sys.argv[2])
