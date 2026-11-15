# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.184.B: score a parser once on the frozen blind set with 3.184's P1-P4.

    python evaluations/request-blind-3184/score.py <repo-root-with-the-parser> [out.json]

The parser is imported from `<root>/brain/request.py` (a worktree at the
frozen commit), never from this checkout, so the commit scored is the one
named. Prints and writes the score, the parser's path, sha256 and git commit,
and every miss with its sentence. The set is checked against its pin first."""

import hashlib
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SET = HERE / "requests.jsonl"
SET_SHA256 = "0fe77d505a7619885b8b590df9e0869dd8879cebafad1277fb26d107e37cbb9f"
BARS = {"P1": 99, "P2": 105, "P3": 1, "P4": 105}       # of 110; P3 at most


def load_parser(root: Path):
    path = root / "brain" / "request.py"
    spec = importlib.util.spec_from_file_location("request_under_test", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod          # dataclasses look their module up here
    spec.loader.exec_module(mod)
    return mod, path


def e_row(r):
    return r["expect"]


def score(root):
    root = Path(root).resolve()
    data = SET.read_bytes()
    set_sha = hashlib.sha256(data).hexdigest()
    if set_sha != SET_SHA256:                 # review: not an assert (python -O)
        raise SystemExit(f"the set is not the frozen one: {set_sha}")
    rows = [json.loads(line) for line in data.decode().splitlines()]
    git = lambda *a: subprocess.run(["git", "-C", str(root), *a],  # noqa: E731
                                    capture_output=True, text=True)
    head = git("rev-parse", "HEAD")
    if head.returncode != 0:
        raise SystemExit(f"{root} is not a git worktree: {head.stderr.strip()}")
    if git("status", "--porcelain", "brain/request.py").stdout.strip():
        raise SystemExit("brain/request.py differs from its commit: score a clean worktree")
    commit = head.stdout.strip()
    mod, path = load_parser(root)
    n = {"P1": 0, "P2": 0, "P3": 0, "P4": 0}
    misses = []
    for r in rows:
        e = r["expect"]
        try:
            got = mod.parse(r["text"], r["names"])
        except Exception as e:  # noqa: BLE001 -- review: a raise is a miss, the run goes on
            misses.append({"id": r["id"], "kind": r["kind"], "text": r["text"],
                           "names": r["names"], "expect": e_row(r), "got": None,
                           "wrong": ["target", "room", "attributes"], "error": repr(e)})
            continue
        room = dict(got.room_hint) if got.room_hint else None
        got_d = {"target": got.target, "room": room,
                 "colours": sorted(set(got.colours or ())), "size": got.size}
        ok_t = got.target == e["target"]
        ok_r = room == e["room"]
        wrong_room = room is not None and room != e["room"]
        ok_a = got_d["colours"] == sorted(set(e["colours"])) and got.size == e["size"]
        n["P1"] += ok_t
        n["P2"] += ok_r
        n["P3"] += wrong_room
        n["P4"] += ok_a
        if not (ok_t and ok_r and ok_a):
            misses.append({"id": r["id"], "kind": r["kind"], "text": r["text"],
                           "names": r["names"], "expect": e, "got": got_d,
                           "wrong": [k for k, ok in (("target", ok_t), ("room", ok_r),
                                                     ("attributes", ok_a)) if not ok]})
    met = {"P1": n["P1"] >= BARS["P1"], "P2": n["P2"] >= BARS["P2"],
           "P3": n["P3"] <= BARS["P3"], "P4": n["P4"] >= BARS["P4"]}
    return {"parser": "brain/request.py", "parser_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "commit": commit, "set_sha256": set_sha, "rows": len(rows),
            "counts": n, "bars": BARS, "met": met, "misses": misses}


if __name__ == "__main__":
    out = score(sys.argv[1])
    text = json.dumps(out, indent=1)
    if len(sys.argv) > 2:
        with open(sys.argv[2], "x") as f:      # review: a first score is never overwritten
            f.write(text + "\n")
    print(json.dumps({k: out[k] for k in ("commit", "counts", "met")}), f"misses={len(out['misses'])}")
