# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/kill-stop-3216/analyse.py

3.216's scorer for one run directory of `sweep.py` (criterion 1) or of the
live missions (criterion 4 only):

* G: judged runs, void share, the least true gap after a kill (overall and
  per speed and direction), runs under the 3.0 cm bar, the slowest zeroing.
* C: from `sends.jsonl` (every `T:1` the board heard), the longest gap
  between two sends while the earlier one was non-zero. Each kill leaves
  one such gap (the board hears nothing until the next robot server), so
  the N longest are left out, N the number of kills; everything else is
  judged.

    python evaluations/kill-stop-3216/analyse.py <dir> [--heartbeat-ms N]   (default: the shipped HEARTBEAT_MS)
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from robot import hardware_robot  # noqa: E402

G_BAR_CM = 3.0
VOID_MAX = 0.25


def read(path: Path) -> list:
    if not path.exists():
        return []
    out = []
    for line in path.read_text().splitlines():
        try:
            out.append(json.loads(line))
        except ValueError:
            pass
    return out


def cadence(sends: list, kills: int) -> dict:
    """The longest gap after a non-zero send, kills and restarts apart:
    the `kills` longest gaps after a non-zero send are the kills (each kill
    leaves one: the board hears nothing until the next server)."""
    gaps = []
    for a, b in zip(sends, sends[1:]):
        if (a.get("L") or a.get("R")):
            gaps.append(b["t"] - a["t"])
    gaps.sort()
    judged = gaps[:len(gaps) - kills] if kills else gaps
    return {"sends": len(sends), "moving_gaps": len(gaps), "kills_left_out": min(kills, len(gaps)),
            "max_gap_s": round(judged[-1], 4) if judged else None,
            "p99_gap_s": round(judged[int(0.99 * (len(judged) - 1))], 4) if judged else None,
            "median_gap_s": round(statistics.median(judged), 4) if judged else None,
            "over_100ms": sum(g > 0.1 for g in judged), "over_125ms": sum(g > 0.125 for g in judged)}


def score(d: Path, heartbeat_ms: int) -> dict:
    runs = read(d / "runs.jsonl")
    judged = [r for r in runs if "void" not in r]
    void = [r for r in runs if "void" in r]
    by = defaultdict(list)
    for r in judged:
        by[f"{r['action']} {r['speed']}"].append(r)
    least = [r["least_gap_cm"] for r in judged if r.get("least_gap_cm") is not None]
    out = {"dir": str(d), "runs": len(runs), "judged": len(judged), "void": len(void),
           "void_share": round(len(void) / len(runs), 3) if runs else None,
           "void_reasons": sorted({r["void"][:60] for r in void}),
           "least_gap_cm": min(least) if least else None,
           "under_bar": sum(g < G_BAR_CM for g in least),
           "contacts": sum(g < 1.0 for g in least),
           "unjudged_gap": sum(1 for r in judged if r.get("least_gap_cm") is None),
           "max_zeroed_after_s": max((r["zeroed_after_s"] for r in judged if r.get("zeroed_after_s") is not None),
                                     default=None),
           "never_zeroed": sum(1 for r in judged if r.get("zeroed_after_s") is None),
           "max_travel_cm": max((r["travel_cm"] for r in judged if r.get("travel_cm") is not None), default=None),
           "per_kind": {k: {"n": len(v),
                            "least_gap_cm": min((r["least_gap_cm"] for r in v if r.get("least_gap_cm") is not None),
                                                default=None),
                            "max_travel_cm": max((r["travel_cm"] for r in v if r.get("travel_cm") is not None),
                                                 default=None)}
                        for k, v in sorted(by.items())}}
    kills = sum(1 for r in runs if r.get("kill_delay_s") is not None)
    out["cadence"] = cadence(read(d / "sends.jsonl"), kills)
    mg = out["cadence"]["max_gap_s"]
    out["verdict"] = {
        "G": bool(runs) and out["under_bar"] == 0 and out["unjudged_gap"] == 0
        and out["void_share"] is not None and out["void_share"] <= VOID_MAX,
        "C": mg is not None and mg <= heartbeat_ms / 2000.0,
    }
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--heartbeat-ms", type=int, default=hardware_robot.HEARTBEAT_MS,
                    help="the heartbeat the run used (default: the shipped code's)")
    a = ap.parse_args(argv)
    print(json.dumps(score(Path(a.dir), a.heartbeat_ms), indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
