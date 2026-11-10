"""3.172.C's scorer: C1-C4 from live.py's rows and each batch's brain log.

    python evaluations/rehearsal-fixes-3172/score_C.py ROWS.jsonl
"""
import collections
import json
import statistics
import sys
from pathlib import Path


def first_goals(rows):
    """Per row: the kind of the first goal sent, and the places recalled."""
    out = {}
    by_dir = collections.OrderedDict()
    for r in rows:
        by_dir.setdefault(r["run_dir"], []).append(r)
    for d, rs in by_dir.items():
        lines = open(Path(d) / "explore_brain.log", errors="replace").read().splitlines()
        starts = [i for i, l in enumerate(lines) if "mission started:" in l]
        for r, i0, i1 in zip(rs, starts, starts[1:] + [len(lines)]):
            first, recalled = None, None
            for l in lines[i0:i1]:
                if '"event"' not in l or " explore " not in l:
                    continue
                try:
                    e = json.loads(l[l.index("{"):])
                except ValueError:
                    continue
                if e.get("event") == "recall_set" and recalled is None:
                    recalled = len(e.get("places") or [])
                if e.get("event") == "goal_sent" and first is None:
                    first = e.get("kind")
            out[r["i"]] = (first, recalled)
    return out


def main(path):
    rows = [json.loads(l) for l in open(path)]
    first = first_goals(rows)
    print("i target outcome s first_goal recalled found_m docked contacts")
    for r in rows:
        print(r["i"], r.get("target"), r["outcome"], r["seconds"], *first[r["i"]],
              r["found_target_m"], r["docked"], r["contacts"])
    found = sum(r["outcome"] == "found" for r in rows)
    wrong = sum(r["outcome"] == "found" and (r["found_target_m"] or 9) > 1.0 for r in rows)
    bad = sum(r["contacts"] + bool(r["fell"]) + (r["outcome"] == "failed") for r in rows) + wrong
    print(f"C1 median mission s: {statistics.median(r['seconds'] for r in rows):.0f} (bar < 250)")
    print(f"C2 found {found} of {len(rows)} (bar >= 7)")
    print(f"C3 first goal recall {sum(f == 'recall' for f, _ in first.values())} of {len(rows)} (bar 8)")
    print(f"C4 contacts+falls+wrong+failed {bad} (wrong {wrong}); docked "
          f"{sum(r['docked'] for r in rows)} of {len(rows)}")


if __name__ == "__main__":
    main(sys.argv[1])
