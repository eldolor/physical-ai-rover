# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
brain/last_seen.py -- 3.175.C: "where did you last see it?", answered from
what the brain already keeps, without a mission.

Two sources, both on the robot's own disk:

* the chain head (3.50.D5): the newest `Inventory.report()` on the saved map,
  every landmark the chain still carries, with its belief and `seen_seq`;
* 3.162's reports (`brain/what_i_saw.py`): which mission saw what, when, and a
  thumbnail of it.

`answer()` is pure: it reads only the dicts it is handed, so the route that
calls it moves nothing and asks the robot nothing.

**The place is the one a find would go to first.** The head's rows that name
the target (the found rule's match: the name, lower case, inside the label),
believed at least `REPORT_BELIEF`, newest sighting first (`seen_seq`; a row
without one last), then by belief -- the order `MissionRunner._recall_carried`
hands explore (3.175.B). `tests/test_last_seen.py` pins the two against each
other, and 3.175.C's C1 measures it on real chains.

`status`:

* `seen` -- `place` is set;
* `doubtful` -- the chain still carries rows of that name, but all under
  `REPORT_BELIEF`: the robot looked there since and did not see it (3.175.B's
  stale rule), so a find would not go there first. `place` is null; the best
  row is `doubtful`;
* `not_seen` -- no row of that name;
* `no_memory` -- no chain head kept at all.

`when` is the newest kept report on the head's saved map with a row naming the
target within `MERGE_M` of the place: its `mission_id`, `ended` and the row's
`thumb`. Null when that report has been pruned (the store keeps 50), or for a
place carried from a mission whose report is gone.
"""

from __future__ import annotations

import math
import re
from typing import Iterable, Optional

from brain.inventory import CARRY_MIN_BELIEF, MERGE_M, REPORT_BELIEF, _seen_seq, carry_seq_of

MAX_NAME_CHARS = 80
SEEN, DOUBTFUL, NOT_SEEN, NO_MEMORY = "seen", "doubtful", "not_seen", "no_memory"


class NameRefused(ValueError):
    """A name that is blank or too long."""


def clean_name(name) -> str:
    """The name as the found rule reads a target: lower case, stripped, inner
    whitespace one space. Blank or over MAX_NAME_CHARS is refused."""
    text = re.sub(r"\s+", " ", str(name or "")).strip().lower()
    if not text:
        raise NameRefused("a name is needed")
    if len(text) > MAX_NAME_CHARS:
        raise NameRefused(f"a name is at most {MAX_NAME_CHARS} characters")
    return text


def names(name: str, label) -> bool:
    """The found rule (`MissionMemory.names_target`): the name in the label."""
    return bool(name) and name in str(label).lower()


def _num(v) -> Optional[float]:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    f = float(v)
    return f if math.isfinite(f) else None


def _rows(name: str, head: dict) -> list:
    """The head's rows naming `name`, as a mission would carry them (belief at
    least CARRY_MIN_BELIEF, a finite place)."""
    seq = carry_seq_of(head)
    out = []
    for d in list(head.get("reported") or []) + list(head.get("candidates") or []):
        # As `Inventory.seed` reads a row: float() on each number, so what a
        # mission carries is what is answered (review: one parse rule).
        try:
            label = str(d["label"]).strip().lower()
            x, y, b = float(d["x_m"]), float(d["y_m"]), float(d["belief"])
        except (KeyError, TypeError, ValueError):
            continue
        if not label or not all(map(math.isfinite, (x, y, b))) or b < CARRY_MIN_BELIEF:
            continue
        if not names(name, label):
            continue
        out.append({"label": label, "x_m": x, "y_m": y, "belief": b,
                    "room": str(d.get("room") or "unknown"),
                    "seen_seq": _seen_seq(d.get("seen_seq"), seq)})
    return out


def recall_order(rows: Iterable[dict]) -> list:
    """`_recall_carried`'s order over rows at or above REPORT_BELIEF: newest
    `seen_seq` first (None last), then belief. Stable, so ties keep the
    head's order, as `sorted` keeps the inventory's."""
    return sorted((r for r in rows if r["belief"] >= REPORT_BELIEF),
                  key=lambda r: (-(r["seen_seq"] if r["seen_seq"] is not None else -1),
                                 -r["belief"]))


def _when(name: str, place: dict, frame_id, reports: Iterable[dict]) -> Optional[dict]:
    """The newest kept report on `frame_id` with a row naming `name` within
    MERGE_M of `place`. `reports` newest first."""
    for rep in reports or []:
        if not isinstance(rep, dict) or rep.get("frame_id") != frame_id:
            continue
        best = None
        for row in rep.get("landmarks") or []:
            if not isinstance(row, dict) or not names(name, row.get("label")):
                continue
            x, y = _num(row.get("x_m")), _num(row.get("y_m"))
            if x is None or y is None:
                continue
            d = math.hypot(x - place["x_m"], y - place["y_m"])
            if d <= MERGE_M and (best is None or d < best[0]):
                best = (d, row)
        if best is not None:
            thumb = best[1].get("thumb")
            return {"mission_id": rep.get("mission_id"), "ended": _num(rep.get("ended")),
                    "saved_at": rep.get("saved_at"), "job": rep.get("job"),
                    "target": rep.get("target"),
                    "thumb": thumb if isinstance(thumb, str) else None,
                    "room": best[1].get("room")}
    return None


def _public(r: dict) -> dict:
    return {"label": r["label"], "x_m": round(r["x_m"], 3), "y_m": round(r["y_m"], 3),
            "belief": round(r["belief"], 3), "room": r["room"], "seen_seq": r["seen_seq"]}


def answer(name, head: Optional[dict], reports: Iterable[dict] = (), source: str = "saved") -> dict:
    """Where `name` was last seen, from the chain `head` and the kept 3.162
    `reports` (newest first; read only as far as `when` needs, so a
    generator is read lazily). Raises NameRefused for a blank or long name."""
    name = clean_name(name)
    out = {"name": name, "status": NO_MEMORY, "place": None, "places": [], "doubtful": None,
           "when": None, "frame_id": None, "source": source, "carry_seq": None,
           "report_belief": REPORT_BELIEF}
    if not isinstance(head, dict):
        return out
    out["frame_id"] = head.get("frame_id")
    out["carry_seq"] = carry_seq_of(head)
    rows = _rows(name, head)
    order = recall_order(rows)
    if order:
        out["status"] = SEEN
        out["places"] = [_public(r) for r in order]
        out["place"] = out["places"][0]
        out["when"] = _when(name, order[0], out["frame_id"], reports)
    elif rows:
        out["status"] = DOUBTFUL
        best = max(rows, key=lambda r: r["belief"])
        out["doubtful"] = _public(best)
        out["when"] = _when(name, best, out["frame_id"], reports)
    else:
        out["status"] = NOT_SEEN
    return out


def saved_head(store) -> Optional[dict]:
    """The chain head on the newest saved map: the newest local report that
    names a `frame_id` (by file time; read newest first, stopping at the first
    that names one), then `store.latest_for` on that frame (lineage decides
    within it). None when nothing is saved. `store` is a
    `control.inventory_store.InventoryStore`; `latest_for`'s TimeoutError (a
    save still in flight) is the caller's."""
    import json
    import os
    from pathlib import Path
    d = Path(store.local_dir)
    try:
        listed = [d / n for n in os.listdir(d) if n.endswith(".json") and not n.startswith(".")]
    except FileNotFoundError:
        return None
    timed = []
    for p in listed:
        try:
            timed.append((p.stat().st_mtime, p.name, p))
        except OSError:
            continue
    for _, _, p in sorted(timed, reverse=True):
        try:
            frame = json.loads(p.read_text()).get("frame_id")
        except (OSError, ValueError, AttributeError):
            continue
        if isinstance(frame, str) and frame:
            return store.latest_for(frame)
    return None
