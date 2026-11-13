# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
3.176.A: the request set is frozen before any parser exists.

`evaluations/request-3176/requests.jsonl` is the written set 3.176.B's rule
parser and 3.176.D's model arm are scored on (the section file, "3.176.A").
Its bytes are pinned by sha256: a changed sentence or answer after the parser
is built would turn a test into a description (CLAUDE.md section 7). A change
is an amendment, written into the section file first, with this hash.

The shape checks hold the set to A's design: every expected room is a
vocabulary type or one of that request's map names, every attribute is a
canonical word, and both splits cover every kind.
"""
import hashlib
import json
import re
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SET = ROOT / "evaluations" / "request-3176" / "requests.jsonl"
SHA256 = "6fcb2742c56d1ff1e7d7e394e86e4e6c590761924132b29457c51874e063c07d"

# 3.176.A's vocabulary (the section file, "The room vocabulary"): the four
# types the shipped prior knows, then the others a label or a name can carry.
ROOM_TYPES = {"kitchen", "living room", "bedroom", "bathroom",
              "dining room", "office", "laundry room", "garage", "hallway"}
# The vocabulary's words for each type (the section file's table).
WORDS = {"kitchen": {"kitchen", "kitchenette"},
         "living room": {"living room", "lounge", "family room", "sitting room", "den"},
         "bedroom": {"bedroom", "master bedroom", "guest room", "spare room", "nursery"},
         "bathroom": {"bathroom", "restroom", "washroom", "powder room", "loo"},
         "dining room": {"dining room", "dining area"},
         "office": {"office", "home office", "study"},
         "laundry room": {"laundry room", "laundry", "utility room"},
         "garage": {"garage"},
         "hallway": {"hallway", "hall", "corridor", "entryway", "foyer"}}
COLOURS = {"red", "orange", "yellow", "green", "blue", "purple", "pink",
           "brown", "black", "white", "grey", "silver", "gold"}
SIZES = {"small", "large"}
KINDS = {"plain", "request_verb", "question", "attributes", "hint_type",
         "hint_synonym", "hint_cue", "hint_in_furniture", "location_not_room",
         "room_word_in_target", "compound", "ambiguous", "punctuation", "name_hint",
         "no_target", "two_targets", "absent_room"}


def _words(text: str) -> list:
    """Lower-case word tokens, an apostrophe kept inside a word ("ben's")."""
    return re.findall(r"[a-z0-9]+(?:'[a-z]+)?", text.lower())


def _said(phrase: str, text: str) -> bool:
    """`phrase`'s words appear in `text` in a row, as whole words."""
    want, have = _words(phrase), _words(text)
    return any(have[i:i + len(want)] == want for i in range(len(have) - len(want) + 1))


def _rows() -> list:
    return [json.loads(line) for line in SET.read_text(encoding="utf-8").splitlines()]


def test_the_set_is_the_pinned_one():
    assert hashlib.sha256(SET.read_bytes()).hexdigest() == SHA256


def test_every_row_has_the_designed_shape():
    rows = _rows()
    assert [r["id"] for r in rows] == [f"r{i:03d}" for i in range(1, len(rows) + 1)]
    for r in rows:
        assert set(r) == {"id", "split", "kind", "text", "names", "expect"}, r["id"]
        assert r["split"] in ("dev", "test") and r["kind"] in KINDS, r["id"]
        e = r["expect"]
        assert set(e) == {"target", "room", "colours", "size"}, r["id"]
        # No object, or two: no target (one search finds one thing; never a guess).
        assert (e["target"] is None) == (r["kind"] in ("no_target", "two_targets")), r["id"]
        if e["target"] is not None:
            # The target is what was said: lower case, its words in a row in the text.
            assert e["target"] == e["target"].strip().lower(), r["id"]
            assert _said(e["target"], r["text"]), r["id"]
        room = e["room"]
        if room is not None:
            assert len(room) == 1, r["id"]
            if "type" in room:
                assert room["type"] in ROOM_TYPES, r["id"]
                # A person's name wins over a type: a row whose text says one
                # of its map's names expects that name (or no room), never a type.
                assert not any(_said(n, r["text"]) for n in r["names"]), r["id"]
            else:
                assert room["name"] in r["names"], r["id"]
        if r["kind"] == "absent_room":
            # A room the house lacks: read as said (the parser does not know
            # the house), a type none of the map's names is a word for.
            assert room is not None and "type" in room, r["id"]
            assert not {n.lower() for n in r["names"]} & WORDS[room["type"]], r["id"]
        assert set(e["colours"]) <= COLOURS and e["size"] in SIZES | {None}, r["id"]


def test_a_plain_target_is_its_own_text():
    """The 0-difference class: a bare target reads back as typed."""
    for r in _rows():
        if r["kind"] == "plain":
            assert r["expect"]["target"] == r["text"] and r["expect"]["room"] is None, r["id"]


def test_the_vocabulary_names_every_type():
    assert set(WORDS) == ROOM_TYPES and all(t in WORDS[t] for t in ROOM_TYPES)


def test_both_splits_cover_every_kind_with_and_without_a_hint():
    rows = _rows()
    for split in ("dev", "test"):
        part = [r for r in rows if r["split"] == split]
        assert {r["kind"] for r in part} == KINDS, split
        hinted = Counter(r["expect"]["room"] is not None for r in part)
        assert hinted[True] >= 25 and hinted[False] >= 25, (split, hinted)
        assert sum("name" in (r["expect"]["room"] or {}) for r in part) >= 3, split
