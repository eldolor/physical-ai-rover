# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.181: the blind request set for the plain-words find
(`docs/plans/ros-alignment/3.181-request-blind-set.md`).

The set was written by a session that had never opened `brain/request.py`,
its tests or 3.176's request set, from 3.176's written rules only, and frozen
here before the parser was run on it. The pin makes any later edit to a row
fail the suite: a changed set is a new set, scored afresh and said so.
"""

import hashlib
import json
from pathlib import Path

import pytest

SET = Path(__file__).resolve().parents[1] / "evaluations" / "request-3181" / "blind_set.jsonl"
PINNED_SHA256 = "4426a2af5df0c13531cfc2f0d0ca720db4edb516833d4ab4486f6297ce7d4fcb"
ROOM_TYPES = {"kitchen", "living room", "bedroom", "bathroom", "dining room", "office",
              "laundry room", "garage", "hallway"}


def rows():
    return [json.loads(line) for line in SET.read_text().splitlines() if line.strip()]


def test_the_set_is_frozen():
    assert hashlib.sha256(SET.read_bytes()).hexdigest() == PINNED_SHA256, (
        "3.181's blind set changed after it was frozen: a changed set is a new set "
        "(write it down in the section before scoring it)")


def test_the_set_has_its_shape():
    rs = rows()
    assert len(rs) == 100
    assert sum(r["group"] == "rules" for r in rs) == 92
    assert sum(r["group"] == "gap" for r in rs) == 8
    assert sum(r["kind"] == "plain" for r in rs) == 8
    assert len({r["id"] for r in rs}) == len(rs)
    assert len({r["text"] for r in rs}) == len(rs)


@pytest.mark.parametrize("row", rows(), ids=lambda r: r["id"])
def test_each_row_says_a_parse_3176_can_give(row):
    assert set(row) == {"id", "group", "kind", "text", "names", "target", "room",
                        "colours", "size"}
    assert row["group"] in ("rules", "gap")
    assert row["target"] is None or (row["target"] == row["target"].lower()
                                     and row["target"].strip() == row["target"])
    room = row["room"]
    if room is not None:
        assert len(room) == 1
        if "type" in room:
            assert room["type"] in ROOM_TYPES
        else:
            assert room["name"] in row["names"]
    assert row["size"] in (None, "small", "large")
    assert row["colours"] == sorted(set(row["colours"]))
    if row["kind"] == "plain":
        assert row["target"] == row["text"] and room is None and not row["colours"]


# ---------- the scorer, on a fake parser (before the real one is scored) ----------

def _load_scorer():
    import importlib.util
    path = SET.with_name("score.py")
    spec = importlib.util.spec_from_file_location("score_3181", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_SCORER = _load_scorer()


def _scorer():
    return _SCORER


class _Req:
    def __init__(self, target, room, colours, size):
        self.target, self.room_hint, self.colours, self.size = target, room, tuple(colours), size


def _oracle(rs, change=None):
    by_text = {r["text"]: r for r in rs}

    def parse(text, names=()):
        r = dict(by_text[text])
        if change:
            r = change(r) or r
        return _Req(r["target"], r["room"], r["colours"], r["size"])
    return parse


def test_a_parser_that_gives_every_expected_parse_meets_every_bar():
    out = _scorer().score(rows(), _oracle(rows()))
    assert all(v["met"] for v in out["verdict"].values()), out["verdict"]
    assert out["misses"] == [] and out["rules"]["n"] == 92 and out["gap"]["n"] == 8


def test_a_wrong_room_counts_against_r2_and_r3_and_is_listed():
    def change(r):
        if r["id"] == "b041":                        # a rules row with a room
            return {**r, "room": {"type": "garage"}}
    rs = rows()
    assert rs[40]["id"] == "b041" and rs[40]["room"] is not None
    out = _scorer().score(rs, _oracle(rs, change))
    assert out["rules"]["room"] == 91 and out["rules"]["wrong_room"] == 1
    assert out["verdict"]["R3"]["met"]               # one is within the bar
    assert [m["id"] for m in out["misses"]] == ["b041"]
    assert out["misses"][0]["failed"] == ["room"]


def test_a_missed_room_is_not_a_wrong_room():
    def change(r):
        if r["room"] is not None and r["group"] == "rules":
            return {**r, "room": None}
    out = _scorer().score(rows(), _oracle(rows(), change))
    assert out["rules"]["wrong_room"] == 0 and not out["verdict"]["R2"]["met"]


def test_colours_compare_as_a_set():
    def change(r):
        return {**r, "colours": list(reversed(r["colours"]))}
    out = _scorer().score(rows(), _oracle(rows(), change))
    assert out["rules"]["attributes"] == 92


# ---------- the score, pinned as a floor (3.181.A's one run, 2026-11-15) ----------

def test_the_parser_scores_no_worse_than_its_blind_run():
    """3.181.A scored 3.176.B's parser once on this set: R1 82/92 (FAILED,
    bar 90%), R2 91/92, R3 0, R4 92/92, R5 8/8. Pinned as floors, so a
    regression fails here while a parser that reads more passes; a parser
    changed to fit THIS set no longer has a blind score (the section says so)."""
    from brain.request import parse
    out = _scorer().score(rows(), parse)
    r = out["rules"]
    assert r["target"] >= 82 and r["room"] >= 91 and r["attributes"] >= 92, r
    assert r["wrong_room"] == 0
    assert out["verdict"]["R5"]["met"]
    # Review: by row, not only by count -- a row that passed in the blind run
    # still passes (a fix elsewhere cannot hide a new miss).
    missed = {m["id"] for m in out["misses"] if m["group"] == "rules"}
    assert missed <= BLIND_RULES_MISSES, sorted(missed - BLIND_RULES_MISSES)


BLIND_RULES_MISSES = {"b011", "b013", "b020", "b025", "b026", "b027", "b029", "b030",
                      "b056", "b073"}


def test_the_recorded_score_is_the_blind_run():
    """score.json is the one run the section reports; the scorer refuses to
    overwrite it."""
    d = json.loads(SET.with_name("score.json").read_text())
    assert d["commit"] == "93d493f2" and d["parser_last_changed"] == "196528ab"
    assert {m["id"] for m in d["misses"] if m["group"] == "rules"} == BLIND_RULES_MISSES
    assert d["rules"]["target"] == 82 and d["rules"]["room"] == 91


def test_the_room_types_are_3176s():
    """Written blind from 3.176's table; checked against the parser's own
    vocabulary only after the freeze."""
    import brain.request as req
    types = {v for v in vars(req).values() if isinstance(v, (set, frozenset, tuple, list))
             and ROOM_TYPES <= set(v) and set(v) <= ROOM_TYPES}
    assert types, "brain/request.py's room types are not the set this file names"
