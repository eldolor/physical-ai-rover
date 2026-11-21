# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.204's sweep: does the inventory pin a landmark where no object of its
class ever was? The bars are in docs/plans/ros-alignment/3.204-phantom-landmark.md,
written before the fix was measured.

    python evaluations/phantom-landmark-3204/sweep.py run OUT.jsonl [--noisy] [--n 20]
        [--seed S] [--edge GAP] [--from K]
    python evaluations/phantom-landmark-3204/sweep.py score OUT.jsonl

Each mission is 3.46's survey (`tests.inventory_sweep.run_one(survey=True)`):
the whole path, `MissionRunner` -> explore (FakeNav) -> `robot/safety.py` ->
`MockRobot` in `complex_house`, the inventory fed by the runner's own frame
hook and ranged by the scan. Starts: `inventory_sweep.starts(seed=3462)`.
The detector is the sim's own (clean, as the live stack's) unless --noisy
(3.46's erring detector: bearing noise, drops, relabels).

3.204.B added two arguments and nothing else: `--seed` (default 3462, A's)
and `--edge GAP`, which turns the inventory's depth-edge gate on at GAP
metres (`MissionRunner(inventory_edge_gate=True)`, its `edge_gap_m` set to
GAP); absent, the key is off and the run is A's. `--from K` starts at
mission K (rows are appended, so a run can be split across processes).

A PHANTOM is a landmark a later mission would carry (belief at least
`CARRY_MIN_BELIEF`) farther than `MERGE_M` from every instance of its class
(`inventory_sweep._nearest_of_class`, distance to the nearest cell).
"""
import json
import logging
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

SEED = 3462
MISSIONS = 20


def mission(k: int, noisy: bool, seed: int = SEED, edge=None) -> dict:
    from brain import inventory as inv
    from tests import inventory_sweep as sw
    st = sw.starts(seed=seed)
    if edge is None:
        r = sw.run_one(st[k], noisy=noisy, seed=seed * 100 + k, survey=True)
    else:
        # 3.204.B: the key on, through the runner's own constructor argument,
        # then the gap under test set on the inventory it built.
        real = sw.MissionRunner

        from brain.inventory import Inventory
        gap = float(edge)
        Inventory(edge_gap_m=gap)              # refuses NaN, 0, negatives

        def gated(*a, **kw):
            kw["inventory_edge_gate"] = True
            runner = real(*a, **kw)
            if runner.inventory is None:
                raise SystemExit("--edge needs the inventory on")
            runner.inventory.edge_gap_m = gap
            return runner
        sw.MissionRunner = gated
        try:
            r = sw.run_one(st[k], noisy=noisy, seed=seed * 100 + k, survey=True)
        finally:
            sw.MissionRunner = real
    rep = r["report"]
    rows = []
    for lm in rep["reported"] + rep["candidates"]:
        if lm["belief"] < inv.CARRY_MIN_BELIEF:
            continue
        _, d = sw._nearest_of_class(lm)
        rows.append({"label": lm["label"], "x_m": lm["x_m"], "y_m": lm["y_m"],
                     "belief": lm["belief"], "reported": lm["reported"], "off_m": round(d, 3)})
    sc = sw.score([r])
    return {"k": k, "noisy": noisy, "start_room": r["start_room"], "outcome": r["outcome"],
            "recall": sc["recall"], "seen": sc["seen"], "recall_small": sc["recall_small"],
            "unplaced": rep["counts"]["unplaced"], "observations": rep["counts"]["observations"],
            "carried": rows}


def score(path) -> dict:
    from brain import inventory as inv
    rows = [json.loads(x) for x in Path(path).read_text().splitlines() if x.strip()]
    if not rows:
        return {"missions": 0}
    carried = [c for r in rows for c in r["carried"]]
    ph = [c for c in carried if c["off_m"] > inv.MERGE_M]
    true = [c for c in carried if c["off_m"] <= inv.MERGE_M]
    return {"missions": len(rows),
            "phantoms": len(ph), "phantoms_reported": sum(c["reported"] for c in ph),
            "true_carried": len(true), "true_reported": sum(c["reported"] for c in true),
            "recall_mean": round(statistics.mean(r["recall"] for r in rows), 4),
            "seen": sum(r["seen"] for r in rows),
            "found": round(sum(r["recall"] * r["seen"] for r in rows)),
            "unplaced_share": round(sum(r["unplaced"] for r in rows)
                                    / max(1, sum(r["observations"] for r in rows)), 4),
            "worst": sorted(((c["off_m"], c["label"], c["belief"]) for c in ph), reverse=True)[:5]}


def main(argv):
    logging.disable(logging.WARNING)
    if argv[:1] == ["run"]:
        out = Path(argv[1])
        k0 = int(argv[argv.index("--from") + 1]) if "--from" in argv else 0
        if out.exists():
            # Appending is only continuing: the file holds exactly missions 0..k0-1.
            ks = [json.loads(x)["k"] for x in out.read_text().splitlines() if x.strip()]
            if k0 == 0 or ks != list(range(k0)):
                raise SystemExit(f"{out} exists (missions {ks[:3]}...): never overwritten")
        noisy = "--noisy" in argv
        n = int(argv[argv.index("--n") + 1]) if "--n" in argv else MISSIONS
        seed = int(argv[argv.index("--seed") + 1]) if "--seed" in argv else SEED
        edge = float(argv[argv.index("--edge") + 1]) if "--edge" in argv else None
        for k in range(k0, n):
            row = mission(k, noisy, seed, edge)
            with open(out, "a") as f:
                f.write(json.dumps(row) + "\n")
            print(k, row["outcome"], len(row["carried"]), file=sys.stderr)
        print(json.dumps(score(out), indent=1))
    elif argv[:1] == ["score"]:
        print(json.dumps(score(argv[1]), indent=1))
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main(sys.argv[1:])
