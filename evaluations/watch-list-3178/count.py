# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/watch-list-3178/count.py

3.178.A: what the recorded walks already hold of the watch list, counted
from files committed to this repo -- no frames, no models, no network.

    .venv/bin/python evaluations/watch-list-3178/count.py            # print
    .venv/bin/python evaluations/watch-list-3178/count.py --write    # counts.json

Two sources, and what each can and cannot say:

* **The labelled set** (3.92's 21 walks, 2,261 frames), as 3.177.A pinned
  it in `evaluations/visual-promise-3177/set.json` (one sha256 a frame,
  one hash over the set). A walk's `labels.json` flags only ITS target, so
  the human reference holds a list item only where a walk's target is
  one. Counted: walks and frames whose target is a list item.
* **3.46's stored-object records** (`evaluations/inventory-346/memory-*.json`,
  the same 2,261 frames). Each frame's `label` is the prompt-free YOLOE
  name of the ONE stored box that scored highest for that walk's own
  target -- not a census of the frame. Counted: frames where that name is
  a list item's name. These are a detector's words, unadjudicated, and
  biased toward boxes that look like the walk's target; they are reported
  as names that surfaced, never as items present.

The list is 3.178's default (keys, wallet, phone, glasses, remote). A
name counts for an item only on an exact match to one of the item's
names below (case and surrounding space ignored): "phone charger cable"
and "phone box" are not phones. Near names (a list word inside a longer
name) are listed apart, uncounted, so the choice can be read.
"""

from __future__ import annotations

import hashlib
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.parent.parent
SET = ROOT / "evaluations" / "visual-promise-3177" / "set.json"
# 3.177.A's pin of 3.92's set (its `common.PINNED`, 2026-11-12).
PINNED = "00d12dbc44f6495a927a2a58d53887fc58c9e468e5774d65a8d90288d364a2d6"
MEMORY = [ROOT / "evaluations" / "inventory-346" / f
          for f in ("memory-all.json", "memory-heldout-all.json",
                    "memory-objects.json", "memory-heldout-objects.json")]
OUT = HERE / "counts.json"

# 3.178's default list and each item's names, fixed before counting.
LIST = {
    "keys": ("key", "keys", "car key", "house key", "key ring", "keyring",
             "keychain", "key chain", "a set of keys"),
    "wallet": ("wallet", "purse", "billfold", "coin purse", "card holder"),
    "phone": ("phone", "cellphone", "cell phone", "mobile phone",
              "smartphone", "smart phone", "iphone", "telephone"),
    "glasses": ("glasses", "eyeglasses", "spectacles", "sunglasses",
                "reading glasses", "eyewear"),
    "remote": ("remote", "remote control", "tv remote", "television remote"),
}
# A near name holds one of these words, ending a word ("payphone" is near,
# "keyboard" is not); reported, never counted.
_WORDS = re.compile(r"(keys?|wallet|purse|phone|glasses|spectacles|eyewear|remote)\b", re.I)


def item_of(name: str):
    """The list item `name` exactly names, or None."""
    n = " ".join((name or "").lower().split())
    for item, names in LIST.items():
        if n in names:
            return item
    return None


def set_hash(rows) -> str:
    """3.177's rule: one sha256 over (key, visible, the JPEG's sha256), in order."""
    h = hashlib.sha256()
    for r in rows:
        h.update(f"{r['key']}\t{int(r['visible'])}\t{r['sha256']}\n".encode())
    return h.hexdigest()


def labelled(path: Path = SET) -> dict:
    """The human reference: walks and frames whose target is a list item."""
    doc = json.loads(path.read_text())
    rows = doc["frames"]
    got = set_hash(rows)
    if got != PINNED:
        raise SystemExit(f"{path} is not 3.177's pinned set: {got} != {PINNED}")
    walks, targets = {}, {}
    for r in rows:
        walk = r["key"].split("/")[0]
        walks.setdefault(walk, r["target"])
        targets.setdefault(r["target"], [0, 0])
        targets[r["target"]][0] += 1
        targets[r["target"]][1] += bool(r["visible"])
    per_item = {item: {"walks": 0, "frames": 0, "visible": 0} for item in LIST}
    for target, (n, vis) in targets.items():
        item = item_of(target)
        if item:
            per_item[item]["frames"] += n
            per_item[item]["visible"] += vis
            per_item[item]["walks"] += sum(1 for t in walks.values() if t == target)
    return {"set_hash": got, "walks": len(walks), "frames": len(rows),
            "targets": {t: {"frames": n, "visible": v} for t, (n, v) in sorted(targets.items())},
            "per_item": per_item}


def surfaced(keys: set, paths=MEMORY) -> dict:
    """3.46's per-frame names that are a list item's, per records file."""
    out = {}
    for p in paths:
        doc = json.loads(p.read_text())
        recs = doc["records"]
        seen = {f"{r['walk']}/{r['frame']}" for r in recs}
        if not seen <= keys:
            raise SystemExit(f"{p.name} holds frames outside the pinned set: "
                             f"{sorted(seen - keys)[:3]}")
        exact = {item: [] for item in LIST}
        near = {}
        for r in recs:
            label = r.get("label") or ""
            item = item_of(label)
            key = f"{r['walk']}/{r['frame']}"
            if item:
                exact[item].append(key)
            elif _WORDS.search(label):
                near.setdefault(label, []).append(key)
        out[p.name] = {"variant": doc["config"].get("variant"),
                       "frames": len(recs),
                       "exact": {k: sorted(v) for k, v in exact.items()},
                       "near": {k: len(v) for k, v in sorted(near.items())}}
    return out


def count() -> dict:
    ref = labelled()
    keys = {r["key"] for r in json.loads(SET.read_text())["frames"]}
    return {"list": {k: list(v) for k, v in LIST.items()},
            "labelled": ref, "surfaced": surfaced(keys)}


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    d = count()
    text = json.dumps(d, indent=1, sort_keys=True) + "\n"
    if "--write" in argv:
        OUT.write_text(text)
    ref = d["labelled"]
    print(f"labelled set {ref['set_hash'][:12]}: {ref['walks']} walks, {ref['frames']} frames")
    for item, c in ref["per_item"].items():
        print(f"  {item:8s} target of {c['walks']} walks, {c['frames']} frames, {c['visible']} visible")
    for name, s in d["surfaced"].items():
        hits = {k: len(v) for k, v in s["exact"].items() if v}
        print(f"  {name} ({s['variant']}, {s['frames']} frames): exact {hits or 'none'}; near {s['near'] or 'none'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
