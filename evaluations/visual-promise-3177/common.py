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
# Criterion 1: the set the plan pinned (3.177.A, 2026-11-12).
PINNED = "00d12dbc44f6495a927a2a58d53887fc58c9e468e5774d65a8d90288d364a2d6"

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
    """`set.json`, refused unless its rows hash to the pinned hash."""
    d = json.loads(SET.read_text())
    if set_hash(d["frames"]) != d["hash"] or d["hash"] != PINNED:
        raise SystemExit("set.json is not the pinned set")
    return d


def texts(d: dict) -> dict:
    """object -> the words it stands for in another object's control: its
    first walk's description. A walk's own score always uses its own words."""
    out = {}
    for r in d["frames"]:
        out.setdefault(obj(r["target"]), r["target"])
    return out


def all_texts(d: dict) -> list:
    """Every distinct description in the set: each frame is scored against all."""
    return sorted({r["target"] for r in d["frames"]})


def rows(path: Path) -> dict:
    """key -> row; a torn last line (a killed run) is dropped, not fatal."""
    out = {}
    lines = [l for l in path.read_text().splitlines() if l.strip()] if path.exists() else []
    for i, line in enumerate(lines):
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            if i == len(lines) - 1:
                break
            raise
        out[r["key"]] = r
    return out
