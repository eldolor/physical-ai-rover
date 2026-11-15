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
