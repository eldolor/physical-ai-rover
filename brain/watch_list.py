# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
watch_list.py -- a few named valuables noticed on every mission
(PLAN-ros-alignment.md 3.178; the design is 3.178.A's, written before B
measured anything).

A search drives through most of a house and, on the car, perception asks
about one thing: the day's target. This is the short list beside it --
keys, wallet, phone, glasses, remote by default, at most `MAX_ITEMS` -- so
a mission also notices the things people actually lose, and the inventory
keeps where it saw them.

What lives here, and what does not:

* **The list itself** (`normalise`): short noun phrases, lower-cased,
  de-duplicated, at most `MAX_ITEMS`. `watched(items, target)` drops an
  item that names the day's target: that one is counted once, as the
  target.
* **Which label names an item** (`item_for`): the exact phrase, or a label
  whose head noun (its last word) is the item -- the simulator's "tv
  remote" is a remote. 3.178.A's counting rule is the same: "phone charger
  cable" and "phone box" are not phones.
* **A crop's list sighting** (`sighting`): the softmax over every text a
  crop was scored against (target, distractors, list), read at each list
  item. Including the target, the distractors and the other items makes
  keys and remote compete for one crop instead of both passing on it. The
  gate is `WATCH_GATE` (0.9), stricter than the target's 0.8: a wrong pin
  costs trust and, through recall, a drive. **The target's own probability
  is never computed here**: `brain.perceive.Candidate.probability` stays
  the softmax over the target and the distractors only, so a list cannot
  move a crop's P(target).
* **The report's `watch` block** (`watch_block`): each item's pins -- a
  reported landmark (belief >= `REPORT_BELIEF`) whose label names the item.
  That block is what "where did you last see it" (3.175) and recall
  (3.172.C) read.

No model is loaded here; `brain.perceive` holds the models and calls
`sighting` per crop.
"""
from __future__ import annotations

import math
import re
from typing import Iterable, Optional, Sequence

DEFAULT_WATCH_LIST = ("keys", "wallet", "phone", "glasses", "remote")
MAX_ITEMS = 8
MAX_ITEM_CHARS = 40
# 3.178.A: fixed before measuring, not tuned on the frames that judge it.
WATCH_GATE = 0.9
_SPACE = re.compile(r"\s+")


def _clean(text: str) -> str:
    return _SPACE.sub(" ", str(text)).strip().lower()


def normalise(items: Optional[Iterable]) -> tuple:
    """The list as the brain holds it. Raises ValueError on an entry that is
    not a non-empty string or longer than `MAX_ITEM_CHARS`, or on more than
    `MAX_ITEMS` distinct entries. None is the default list; an empty list
    is a list switched off."""
    if items is None:
        return DEFAULT_WATCH_LIST
    if isinstance(items, (str, bytes)):
        raise ValueError("watch_list must be a list of names, not one string")
    out: list = []
    for raw in items:
        if not isinstance(raw, str):
            raise ValueError(f"watch_list entries must be strings; got {raw!r}")
        item = _clean(raw)
        if not item:
            raise ValueError("watch_list entries must not be empty")
        if len(item) > MAX_ITEM_CHARS:
            raise ValueError(f"watch_list entry {item!r} is longer than {MAX_ITEM_CHARS}")
        if item not in out:
            out.append(item)
    if len(out) > MAX_ITEMS:
        raise ValueError(f"watch_list holds at most {MAX_ITEMS} names; got {len(out)}")
    return tuple(out)


def item_for(label: Optional[str], items: Sequence[str]) -> Optional[str]:
    """The list item `label` names, or None. Exact first, then the head noun
    (the label's last word) -- never a word elsewhere in the label."""
    if not label:
        return None
    text = _clean(label)
    if text in items:
        return text
    head = text.rsplit(" ", 1)[-1]
    return head if head in items else None


def watched(items: Sequence[str], target: Optional[str]) -> tuple:
    """`items` less any item the day's target names, by the found rule's own
    test (`MissionMemory.names_target`: the target inside the name) or the
    other way round ("remote" in "remote control"): that thing is the
    target, counted once, as the target."""
    if not target:
        return tuple(items)
    t = _clean(target)
    return tuple(i for i in items if t not in i and i not in t)


def for_target(items: Optional[Iterable], target: str) -> tuple:
    """The list a pipeline uses for `target`: `normalise`d (a lone string is
    refused, never split into letters) less the target; () when empty."""
    if not items:
        return ()
    return watched(normalise(items), target)


def softmax(scores: Sequence[float], scale: float) -> list:
    xs = [s * scale for s in scores]
    top = max(xs)
    exps = [math.exp(x - top) for x in xs]
    total = sum(exps)
    return [e / total for e in exps]


def sighting(scores: Sequence[float], items: Sequence[str], scale: float,
             gate: float = WATCH_GATE, skip: Sequence[int] = ()) -> Optional[tuple]:
    """`(item, probability)` for one crop, or None. `scores` are the crop's
    cosines against every text it was scored on, with the list's texts LAST
    and in `items`' order; the probability is the softmax over all of them
    but the indices in `skip` (a distractor that repeats a list item, which
    would halve that item's mass). At a gate above 0.5 at most one item can
    pass."""
    n = len(items)
    if n == 0 or len(scores) <= n:
        return None
    kept = [s for k, s in enumerate(scores) if k not in skip]
    probs = softmax(kept, scale)[-n:]
    k = max(range(n), key=lambda i: probs[i])
    return (items[k], probs[k]) if probs[k] >= gate else None


def watch_block(report: dict, items: Sequence[str], target: Optional[str] = None,
                report_belief: Optional[float] = None) -> dict:
    """The mission report's `watch` block: for each watched item (less the
    target), its pins -- reported landmarks whose label names it -- with
    position, room, belief and the step last seen."""
    if report_belief is None:
        from brain.inventory import REPORT_BELIEF as report_belief
    items = watched(items, target)
    pins: dict = {i: [] for i in items}
    for lm in (report or {}).get("reported", []) or []:
        item = item_for(lm.get("label"), items)
        if item is None or float(lm.get("belief") or 0.0) < report_belief:
            continue
        pins[item].append({k: lm.get(k) for k in
                           ("id", "label", "x_m", "y_m", "room", "belief", "hits",
                            "last_step", "carried")})
    for rows in pins.values():
        rows.sort(key=lambda r: -(r["belief"] or 0.0))
    return {"items": list(items), "gate": WATCH_GATE, "pins": pins,
            "pinned": sum(1 for rows in pins.values() if rows)}
