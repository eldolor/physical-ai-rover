# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/fast-map-3130/mutations.py -- 3.130 criterion 2: each planted
bug in `brain/frontier.py`, one at a time, must fail at least one fast-tier
test (`FILES`). The bugs are planted in a scratch copy of the tracked files
(as they stand in the working tree), never in this checkout, so a killed run
leaves nothing behind. Heavy work (CLAUDE.md 3c):

    python tools/desktop_lock.py shared --why "3.130 mutations" -- \
        python evaluations/fast-map-3130/mutations.py <out.json>
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
FILES = ["tests/test_frontier_3130.py", "tests/test_frontier_fast.py", "tests/test_frontier.py"]

# (name, the text in brain/frontier.py, its replacement): the criterion's
# (a)-(h), then one for each other piece 3.130 wrote.
MUTATIONS = [
    ("a: BFS neighbour order changed",
     "    offs = np.array([dy * pw + dx for dx, dy in _OFFSETS], dtype=np.int64)\n"
     "    cost = np.array([_DIAG if dx and dy else 1.0 for dx, dy in _OFFSETS])\n",
     "    offs = np.array([dy * pw + dx for dx, dy in _OFFSETS[::-1]], dtype=np.int64)\n"
     "    cost = np.array([_DIAG if dx and dy else 1.0 for dx, dy in _OFFSETS[::-1]])\n"),
    ("b: a later occurrence kept in reach's level step",
     "    none = np.iinfo(np.int64).max\n", "    none = -1\n"),
    ("b2: (with b) the running maximum",
     "        np.minimum.at(stamp, cand, at)\n", "        np.maximum.at(stamp, cand, at)\n"),
    ("d: _split's start cell changed",
     "    start = min(group, key=lambda c: (c[0] + c[1], c))\n",
     "    start = max(group, key=lambda c: (c[0] + c[1], c))\n"),
    ("d2: _split's piece order reversed",
     "    return [order[i:i + max_cells] for i in range(0, len(order), max_cells)]\n",
     "    return [order[i:i + max_cells] for i in range(0, len(order), max_cells)][::-1]\n"),
    ("e: one seen cell dropped from the unseen mask",
     "        unseen_mask[xy[on, 1], xy[on, 0]] = False\n",
     "        unseen_mask[xy[on, 1][1:], xy[on, 0][1:]] = False\n"),
    ("f: the ray step changed by half a cell",
     "            d += res * 0.5\n", "            d += res\n"),
    ("g: a tie in _ranked broken the other way",
     "    cand = [(int(i % w), int(i // w)) for i in r.order[idx].tolist()]\n",
     "    cand = [(int(i % w), int(i // w)) for i in r.order[idx[::-1]].tolist()]\n"),
    ("c: clusters' order reversed",
     "    for p in flat:\n        if not member[p]:\n",
     "    for p in flat[::-1]:\n        if not member[p]:\n"),
    ("h: components joined 4-connected only",
     "    lo = np.searchsorted(sy * key + ex, (sy + 1) * key + sx, side=\"left\")\n",
     "    lo = np.searchsorted(sy * key + ex, (sy + 1) * key + sx, side=\"right\")\n"),
    ("h2: (with h) ... and the down-right diagonal too",
     "    hi = np.searchsorted(sy * key + sx, (sy + 1) * key + ex, side=\"right\")\n",
     "    hi = np.searchsorted(sy * key + sx, (sy + 1) * key + ex, side=\"left\")\n"),
    ("i: a blocked ray walks on into the next stretch",
     "            live = live[kept[:, -1] & ~hit[pair[live]]]\n",
     "            live = live[~hit[pair[live]]]\n"),
    ("j: _Occupied.at one cell short",
     "        return int(np.maximum(np.abs(oy + y0 - y), np.abs(ox + x0 - x)).min())\n",
     "        return int(np.maximum(np.abs(oy + y0 - y), np.abs(ox + x0 - x)).min()) - 1\n"),
    ("k2: _shortlist drops the cells at its least",
     "    short = sq <= least * (1 + 1e-9) + 1e-12\n",
     "    short = sq > least * (1 + 1e-9) + 1e-12\n"),
    ("k: _shortlist's least taken as the most (equivalent: Python's min decides)",
     "    least = np.repeat(np.minimum.reduceat(sq, at), np.diff(np.append(at, k.size)))\n",
     "    least = np.repeat(np.maximum.reduceat(sq, at), np.diff(np.append(at, k.size)))\n"),
    ("m: _split's walk visits neighbours row first",
     "    offs = [dx + dy * w for dx in (-1, 0, 1) for dy in (-1, 0, 1)]\n"
     "    s = (start[0] - lo_x) + (start[1] - lo_y) * w\n",
     "    offs = [dx + dy * w for dy in (-1, 0, 1) for dx in (-1, 0, 1)]\n"
     "    s = (start[0] - lo_x) + (start[1] - lo_y) * w\n"),
    ("o: the box's second block dropped",
     "    return a[..., :out] | a[..., n - k:n - k + out]\n",
     "    return a[..., :out]\n"),
    ("p: components' union keeps the higher root",
     "        if i < j:\n            parent[j] = i\n        elif j < i:\n            parent[i] = j\n",
     "        if i < j:\n            parent[i] = j\n        elif j < i:\n            parent[j] = i\n"),
]
# k only widens the shortlist that Python's own `min` then decides, so no
# answer can change: an equivalent mutant, kept to show it, not counted.
EQUIVALENT = {"k: _shortlist's least taken as the most (equivalent: Python's min decides)"}
# b alone is a bug (a stamp reset to -1 never wins a minimum); b2 needs b's
# reset to keep the latest occurrence, so it runs with b applied.
TOGETHER = {"b2: (with b) the running maximum": "b: a later occurrence kept in reach's level step",
            "h2: (with h) ... and the down-right diagonal too": "h: components joined 4-connected only"}


def copy_tree(dest: str) -> None:
    """The tracked files, as they stand in the working tree, into `dest`."""
    names = subprocess.run(["git", "ls-files", "-z"], cwd=ROOT, check=True,
                           capture_output=True).stdout.decode().split("\0")
    for name in filter(None, names):
        src = os.path.join(ROOT, name)
        if os.path.isfile(src):
            os.makedirs(os.path.dirname(os.path.join(dest, name)) or dest, exist_ok=True)
            shutil.copy2(src, os.path.join(dest, name))


def main(out_path: str) -> None:
    rows = []
    with tempfile.TemporaryDirectory(prefix="mutations-3130-") as tree:
        copy_tree(tree)
        target = os.path.join(tree, "brain", "frontier.py")
        original = open(target).read()
        env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
        for name, old, new in MUTATIONS:
            text = original
            for n, o, nw in MUTATIONS:
                if n == name or TOGETHER.get(name) == n:
                    if text.count(o) != 1:
                        raise SystemExit(f"{n}: its text is not in the file exactly once")
                    text = text.replace(o, nw)
            with open(target, "w") as f:
                f.write(text)
            p = subprocess.run([sys.executable, "-m", "pytest", "-x", "-q", "-p", "no:cacheprovider",
                                *FILES], cwd=tree, env=env, capture_output=True, text=True,
                               timeout=1800)
            tail = p.stdout.strip().splitlines()[-1] if p.stdout.strip() else p.stderr[-200:]
            rows.append({"mutation": name, "caught": p.returncode != 0, "pytest": tail})
            print(f"{'caught ' if p.returncode else 'MISSED '} {name}: {tail}", flush=True)
    with open(out_path, "w") as f:
        f.write(json.dumps({"files": FILES, "mutations": rows,
                            "equivalent": sorted(EQUIVALENT),
                            "missed": [r["mutation"] for r in rows
                                       if not r["caught"] and r["mutation"] not in EQUIVALENT]},
                           indent=1) + "\n")


if __name__ == "__main__":
    main(sys.argv[1])
