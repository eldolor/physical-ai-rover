# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/visual-promise-3177/frames.py

3.177.A criterion 1: pin 3.92's frame set. Writes `set.json` (each frame's
key, target, label and its JPEG's sha256; no image) and one hash over it.
With `--check`, recomputes from the recordings and fails on any difference.
An existing `set.json` is never overwritten without `--force`.

    .venv/bin/python evaluations/visual-promise-3177/frames.py [--check | --force]
"""

import hashlib
import json
import sys

from common import RECORDINGS, SET, set_hash, walks


def build() -> dict:
    rows = []
    for walk, target, frames in walks():
        for frame, visible in frames:
            data = (RECORDINGS / walk / frame).read_bytes()
            rows.append({"key": f"{walk}/{frame}", "target": target, "visible": bool(visible),
                         "sha256": hashlib.sha256(data).hexdigest()})
    return {"frames": rows, "hash": set_hash(rows), "n": len(rows),
            "visible": sum(r["visible"] for r in rows), "walks": len({r["key"].split("/")[0] for r in rows})}


def main():
    d = build()
    if "--check" in sys.argv:
        old = json.loads(SET.read_text())
        if old["hash"] != d["hash"]:
            old_walks = {r["key"].split("/")[0] for r in old["frames"]}
            new_walks = {r["key"].split("/")[0] for r in d["frames"]}
            raise SystemExit(f"the set changed: {old['hash']} -> {d['hash']}; walks added "
                             f"{sorted(new_walks - old_walks)}, gone {sorted(old_walks - new_walks)}")
        print("set unchanged", d["hash"])
        return
    if SET.exists() and "--force" not in sys.argv:
        raise SystemExit("set.json exists (the pinned set); --check it, or --force")
    SET.write_text(json.dumps(d, indent=0) + "\n")
    print(d["n"], "frames,", d["visible"], "in view,", d["walks"], "walks; hash", d["hash"])


if __name__ == "__main__":
    main()
