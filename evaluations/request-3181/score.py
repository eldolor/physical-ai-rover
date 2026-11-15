# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/request-3181/score.py -- 3.181: 3.176.B's rule parser scored once
on the blind set (`docs/plans/ros-alignment/3.181-request-blind-set.md`).

    .venv/bin/python evaluations/request-3181/score.py [out.json]

Bars R1-R5 on the `rules` rows; the `gap` rows reported beside them, no bar.
Every miss is listed with its sentence, what was expected and what came back.
No network, no model: `brain/request.py` is pure Python (3.176, 3.49).
"""

import json
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SET = Path(__file__).with_name("blind_set.jsonl")
BARS = {"R1": ("target", 0.90), "R2": ("room", 0.95), "R4": ("attributes", 0.95)}
R3_MAX = 1


def got_of(req) -> dict:
    return {"target": req.target, "room": dict(req.room_hint) if req.room_hint else None,
            "colours": sorted(set(req.colours or ())), "size": req.size}


def expected_of(row: dict) -> dict:
    return {"target": row["target"], "room": row["room"],
            "colours": sorted(set(row["colours"])), "size": row["size"]}


def judge(row: dict, got: dict) -> dict:
    """Per-row verdicts. R3 counts a room given where none was said, or the
    wrong room (a room missed is R2's, not R3's)."""
    want = expected_of(row)
    room_ok = got["room"] == want["room"]
    return {"target": got["target"] == want["target"],
            "room": room_ok,
            "wrong_room": got["room"] is not None and not room_ok,
            "attributes": got["colours"] == want["colours"] and got["size"] == want["size"]}


def score(rows: list, parse) -> dict:
    out = {"rows": [], "misses": []}
    for row in rows:
        got = got_of(parse(row["text"], names=row["names"]))
        v = judge(row, got)
        out["rows"].append({"id": row["id"], "group": row["group"], "kind": row["kind"], **v})
        bad = [k for k in ("target", "room", "attributes") if not v[k]]
        if bad:
            out["misses"].append({"id": row["id"], "group": row["group"], "kind": row["kind"],
                                  "text": row["text"], "names": row["names"], "failed": bad,
                                  "expected": {k: row[k] for k in ("target", "room", "colours",
                                                                   "size")},
                                  "got": got})
    for group in ("rules", "gap"):
        rs = [r for r in out["rows"] if r["group"] == group]
        n = len(rs)
        g = {"n": n}
        for k in ("target", "room", "attributes"):
            g[k] = sum(r[k] for r in rs)
        g["wrong_room"] = sum(r["wrong_room"] for r in rs)
        # Review: attributes are read from the words taken as the target, so a
        # wrong target can still carry the right colours. Reported beside R4
        # (whose definition is 3.176's P4, frozen), not instead of it.
        g["attributes_with_target"] = sum(r["attributes"] and r["target"] for r in rs)
        kinds = defaultdict(lambda: {"n": 0, "target": 0, "room": 0, "attributes": 0})
        for r in rs:
            kk = kinds[r["kind"]]
            kk["n"] += 1
            for k in ("target", "room", "attributes"):
                kk[k] += r[k]
        g["by_kind"] = dict(kinds)
        out[group] = g
    rules = out["rules"]
    plain = [r for r in out["rows"] if r["kind"] == "plain" and r["group"] == "rules"]
    verdict = {}
    for bar, (k, share) in BARS.items():
        rate = rules[k] / rules["n"]
        verdict[bar] = {"metric": k, "got": f"{rules[k]} of {rules['n']}", "rate": round(rate, 4),
                        "bar": f">= {share:.0%}", "met": rate >= share}
    verdict["R3"] = {"metric": "a room given where none was said, or the wrong room",
                     "got": rules["wrong_room"], "bar": f"<= {R3_MAX}",
                     "met": rules["wrong_room"] <= R3_MAX}
    p_ok = sum(r["target"] and r["room"] and r["attributes"] for r in plain)
    verdict["R5"] = {"metric": "plain rows exact", "got": f"{p_ok} of {len(plain)}",
                     "bar": "100%", "met": p_ok == len(plain)}
    out["verdict"] = dict(sorted(verdict.items()))
    return out


def main():
    """Writes score.json once: the result is the first full score (the
    section's rule), so an existing file is never replaced; a later run goes
    to a path of its own. The tree's state is recorded beside the commit."""
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_name("score.json")
    if out.exists():
        raise SystemExit(f"{out} exists: 3.181's score is its first run; write elsewhere")
    sys.path.insert(0, str(ROOT))
    from brain.request import parse
    rows = [json.loads(line) for line in SET.read_text().splitlines() if line.strip()]
    result = score(rows, parse)
    head = subprocess.run(["git", "rev-parse", "--short=8", "HEAD"], cwd=ROOT,
                          capture_output=True, text=True).stdout.strip()
    parser = subprocess.run(["git", "log", "-1", "--format=%h", "--", "brain/request.py"],
                            cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "brain/", "evaluations/"],
                                cwd=ROOT, capture_output=True, text=True).stdout.strip())
    result = {"commit": head, "parser_last_changed": parser, "tree_dirty": dirty, **result}
    out.write_text(json.dumps(result, indent=1, ensure_ascii=False) + "\n")
    print(json.dumps(result["verdict"], indent=1))
    print(f"misses: {len(result['misses'])} (rules {sum(m['group'] == 'rules' for m in result['misses'])})")


if __name__ == "__main__":
    main()
