# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.184: the second blind request set is frozen. Its sha256 is pinned, and so
is its shape. It is written from 3.176.A's rules alone, without reading the
parser, its tests or the other request sets, and it is scored once, after the
parser fix lands (docs/plans/ros-alignment/3.184-request-blind-set-2.md).
This test does not import the parser: it pins the set, not a score."""

import hashlib
import json
from pathlib import Path

SET = Path(__file__).resolve().parents[1] / "evaluations/request-blind-3184/requests.jsonl"
SHA256 = "0fe77d505a7619885b8b590df9e0869dd8879cebafad1277fb26d107e37cbb9f"

# 3.176.A's closed room vocabulary (its canonical types), copied from the
# section's table, not imported, so this file reads nothing of the parser.
TYPES = {"kitchen", "living room", "bedroom", "bathroom", "dining room", "office",
         "laundry room", "garage", "hallway"}
COLOURS = {"red", "blue", "green", "black", "white", "grey", "yellow", "brown", "pink",
           "purple", "orange"}
KINDS = {"plain", "polite", "indirect", "room_last", "room_first", "synonym", "relative",
         "attributes", "room_in_target", "compound", "typo", "no_hint", "names",
         "no_target", "two_targets", "case"}


def _rows():
    return [json.loads(line) for line in SET.read_text().splitlines()]


def test_the_set_is_frozen():
    assert hashlib.sha256(SET.read_bytes()).hexdigest() == SHA256


def test_its_shape():
    rows = _rows()
    assert len(rows) == 110
    assert [r["id"] for r in rows] == [f"b{i:03d}" for i in range(1, 111)]
    assert {r["kind"] for r in rows} == KINDS
    assert len({r["text"] for r in rows}) == len(rows)          # no sentence twice
    for r in rows:
        assert set(r) == {"id", "kind", "text", "names", "expect"}, r["id"]
        e = r["expect"]
        assert set(e) == {"target", "room", "colours", "size"}, r["id"]
        t = e["target"]
        assert t is None or (t == t.lower() and t == t.strip() and t), r["id"]
        room = e["room"]
        assert room is None or room in ({"type": x} for x in TYPES) or (
            set(room) == {"name"} and room["name"] in r["names"]), r["id"]
        assert e["colours"] == sorted(set(e["colours"])) and set(e["colours"]) <= COLOURS, r["id"]
        assert e["size"] in (None, "small", "large"), r["id"]
        if t is None:
            assert e["colours"] == [] and e["size"] is None, r["id"]


def test_rooms_right_and_absent_both_present():
    rows = _rows()
    with_room = sum(r["expect"]["room"] is not None for r in rows)
    assert 50 <= with_room <= 70 and len(rows) - with_room >= 40
