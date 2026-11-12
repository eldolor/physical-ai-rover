# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.177.A shared: 3.92's labelled frames, pinned, and the six objects' words."""

import hashlib
import json
import os
from pathlib import Path

HERE = Path(__file__).parent
RECORDINGS = Path(os.environ.get("PICAR_RECORDINGS", "/home/anshu/vision-picar-data/recordings"))
SET = HERE / "set.json"

# "burgundy backpack" and "red backpack" are one bag (3.92's OBJECT).
OBJECT = {"burgundy backpack": "red backpack"}


def obj(target: str) -> str:
    t = target.strip().lower()
    return OBJECT.get(t, t)


def walks():
    """[(walk, description, [(frame, visible), ...])] in a fixed order."""
    out = []
    for labels in sorted(RECORDINGS.glob("*/labels.json")):
        d = json.loads(labels.read_text())
        out.append((d["walk"], d["description"], sorted(d["target_visible_labels"].items())))
    return out


def set_hash(rows) -> str:
    """One sha256 over the set's rows (key, visible, the JPEG's sha256), in order."""
    h = hashlib.sha256()
    for r in rows:
        h.update(f"{r['key']}\t{int(r['visible'])}\t{r['sha256']}\n".encode())
    return h.hexdigest()


def load_set() -> dict:
    """`set.json`, refused if its rows no longer hash to its recorded hash."""
    d = json.loads(SET.read_text())
    if set_hash(d["frames"]) != d["hash"]:
        raise SystemExit("set.json's rows do not match its hash")
    return d


def texts(d: dict) -> dict:
    """object -> the words it is scored with: its first walk's description."""
    out = {}
    for r in d["frames"]:
        out.setdefault(obj(r["target"]), r["target"])
    return out


def done(out: Path) -> set:
    if not out.exists():
        return set()
    return {json.loads(l)["key"] for l in out.read_text().splitlines() if l.strip()}
