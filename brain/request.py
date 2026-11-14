# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
brain/request.py

3.176: a find asked in plain words. One sentence ("my water bottle, probably
in the kitchen") becomes what the mission runs: the target the identity check
is asked about, the detector's prompts, the colour and size words, and a room
hint that only orders where explore looks (`docs/plans/ros-alignment/
3.176-plain-language-request.md`, 3.176.A's design).

* `parse(text, names=()) -> Request` -- rule-based, no model, no I/O: it works
  with every connection refused (3.49). `names` are the rooms a person named
  on the saved map (3.160); a name the sentence says wins over a type.
* `ROOM_WORDS` -- the closed room vocabulary: canonical type -> the words read
  as it. A word outside it is not a room and is never guessed into one.
* `canonical_room(word)` -- a room word or a label's type -> its canonical
  type, or None (the room policy matches a hint against labelled rooms
  through it: a room a person named "den" is a living room).
* `check_hint(hint, names)` -- a `room_hint` as `/mission/start` takes it,
  validated; raises ValueError.

Never a guess: two rooms, a negation, a misspelling or an unknown room give no
hint, and no object (or two) gives no target -- a start with no target is
refused, because one search finds one thing.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Optional, Sequence

# The vocabulary (3.176.A's table). The first four are the shipped room
# prior's types (ProcTHOR's); the others carry a hint only through a layer
# that names them (a person's names, a cloud label).
ROOM_WORDS = {
    "kitchen": ("kitchen", "kitchenette"),
    "living room": ("living room", "lounge", "family room", "sitting room", "den"),
    "bedroom": ("bedroom", "master bedroom", "guest room", "spare room", "nursery"),
    "bathroom": ("bathroom", "restroom", "washroom", "powder room", "loo"),
    "dining room": ("dining room", "dining area"),
    "office": ("office", "home office", "study"),
    "laundry room": ("laundry room", "laundry", "utility room"),
    "garage": ("garage",),
    "hallway": ("hallway", "hall", "corridor", "entryway", "foyer"),
}
ROOM_TYPES = tuple(ROOM_WORDS)
_WORD_TYPE = {w: t for t, ws in ROOM_WORDS.items() for w in ws}
# Longest phrase first, so "master bedroom" is read before "bedroom".
_ROOM_PHRASES = sorted(((tuple(w.split()), t) for w, t in _WORD_TYPE.items()),
                       key=lambda p: -len(p[0]))

COLOURS = {"red": "red", "orange": "orange", "yellow": "yellow", "green": "green",
           "blue": "blue", "purple": "purple", "pink": "pink", "brown": "brown",
           "black": "black", "white": "white", "grey": "grey", "gray": "grey",
           "silver": "silver", "gold": "gold"}
SIZES = {"small": "small", "little": "small", "tiny": "small", "mini": "small",
         "big": "large", "large": "large", "huge": "large"}

# What a sentence may open with before the thing (longest first).
_LEADS = sorted((tuple(p.split()) for p in (
    "find", "please find", "can you find", "could you find", "can you please find",
    "help me find", "go find", "find me", "look for", "search for", "locate",
    "where is", "where's", "where are", "where did i leave", "where did i put",
    "have you seen", "can you see", "i lost", "i've lost", "i can't find",
    "i cannot find", "i left", "is", "are", "check", "look in", "look", "go to",
    "try", "please")), key=lambda p: -len(p))
_OWNERS = {"my", "the", "a", "an", "our", "your", "his", "her", "their", "some",
           "this", "that", "these", "those"}
# A location word ends the thing's words; what follows may name a room.
_LOCATION = {"in", "on", "near", "by", "under", "behind", "beside", "inside", "at",
             "around", "underneath", "within"}
_LOCATION_PAIRS = {("next", "to"), ("close", "to")}
_PLACE = {"upstairs", "downstairs", "anywhere", "somewhere", "everywhere"}
_VERBS = {"is", "are", "was", "were", "it's", "they're"}
_FILLER = {"please", "probably", "maybe", "perhaps"}
_PUNCT = {",", ".", ";", ":", "!", "?", "(", ")", "-", "--"}
_NOT = {"not", "isn't", "aren't", "wasn't", "weren't", "never", "no"}
_NO_THING = {"it", "them", "something", "anything", "everything", "one", "that",
             "this", "thing", "stuff"}
_JOINS = {"and", "or"}

_TOKEN = re.compile(r"[a-z0-9]+(?:'[a-z]+)?|--|[-,.;:!?()]")


def _tokens(text: str) -> list:
    text = (text or "").lower().replace("’", "'")
    text = re.sub(r"[–—]", "-", text)
    return _TOKEN.findall(text)


def canonical_room(word: Optional[str]) -> Optional[str]:
    """A room word, or a label's type ('bedroom 2' already cut to 'bedroom'),
    -> its canonical vocabulary type, or None."""
    if not word:
        return None
    return _WORD_TYPE.get(" ".join(_tokens(word)))


@dataclass(frozen=True)
class Request:
    """What a sentence was understood as. `target` None: no single object
    was named (nothing to start). `room_hint`: {"type": t}, {"name": n} or
    None. `unused`: the words read past, shown so a wrong reading is seen."""
    text: str
    target: Optional[str]
    room_hint: Optional[dict] = None
    colours: tuple = ()
    size: Optional[str] = None
    unused: str = ""
    detector_prompts: tuple = field(default=())

    def to_dict(self) -> dict:
        return {"text": self.text, "target": self.target, "room_hint": self.room_hint,
                "attributes": {"colours": list(self.colours), "size": self.size},
                "detector_prompts": list(self.detector_prompts), "unused": self.unused}


def _strip_lead(toks: list) -> list:
    """Drop the request's opening words ("find", "where is", ...), repeatedly
    ("please find", "find: ...")."""
    changed = True
    while changed and toks:
        changed = False
        while toks and toks[0] in _PUNCT:
            toks, changed = toks[1:], True
        for lead in _LEADS:
            if tuple(toks[:len(lead)]) == lead:
                toks, changed = toks[len(lead):], True
                break
    return toks


def _thing_end(toks: list) -> int:
    """Index of the first token that ends the thing's words."""
    for i, t in enumerate(toks):
        if (t in _LOCATION or t in _PUNCT or t in _PLACE or t in _VERBS
                or t in _FILLER or tuple(toks[i:i + 2]) in _LOCATION_PAIRS
                or tuple(toks[i:i + 2]) == ("for", "me")):
            return i
    return len(toks)


def _owners_off(toks: list) -> list:
    while toks and (toks[0] in _OWNERS or toks[0].endswith("'s") or toks[0].endswith("s'")):
        toks = toks[1:]
    return toks


def _one_thing(words: list) -> Optional[str]:
    """The thing's words as one target, or None for none or two ("keys and
    wallet"; colour words joined by "and" are one thing)."""
    if not words or all(w in _NO_THING for w in words):
        return None
    if any(w in _JOINS for w in words):
        cut = max(i for i, w in enumerate(words) if w in _JOINS)
        left = [w for w in words[:cut] if w not in _JOINS]
        if not (left and all(w in COLOURS for w in left)):
            return None
    return " ".join(words)


def _rooms_in(toks: list, names: Sequence[str]) -> list:
    """[(start index, hint)] for every room these tokens name: a person's
    name first (whole, without case), then the vocabulary, longest first."""
    found, used = [], set()
    for n in sorted(names, key=lambda s: -len(_tokens(s))):
        want = tuple(_tokens(n))
        if not want:
            continue
        for i in range(len(toks) - len(want) + 1):
            span = set(range(i, i + len(want)))
            if tuple(toks[i:i + len(want)]) == want and not span & used:
                found.append((i, {"name": n}))
                used |= span
    for phrase, t in _ROOM_PHRASES:
        for i in range(len(toks) - len(phrase) + 1):
            span = set(range(i, i + len(phrase)))
            if tuple(toks[i:i + len(phrase)]) == phrase and not span & used:
                found.append((i, {"type": t}))
                used |= span
    return sorted(found, key=lambda f: f[0])


def _hint(toks: list, names: Sequence[str]) -> Optional[dict]:
    """One room named, not negated, or None (never a guess)."""
    rooms = _rooms_in(toks, names)
    distinct = {tuple(sorted(h.items())) for _, h in rooms}
    if len(distinct) != 1:
        return None
    first = rooms[0][0]
    if any(t in _NOT or t.endswith("n't") for t in toks[:first]):
        return None
    return rooms[0][1]


def parse(text: str, names: Sequence[str] = ()) -> Request:
    """One sentence -> a `Request` (3.176.A's rules). `names`: the saved
    map's room names (a person's, 3.160), matched whole and without case."""
    names = [n for n in (names or ()) if isinstance(n, str) and n.strip()]
    toks = _strip_lead(_tokens(text))
    end = _thing_end(toks)
    thing, rest = _owners_off(toks[:end]), toks[end:]
    target = _one_thing(thing)
    room_words = rest
    if target is not None and not rest:
        # "check the bedroom", "go to the living room": the words are a room,
        # not a thing to find -- no target; the room is still read.
        alone = _rooms_in(thing, names)
        if alone and alone[0][0] == 0 and _covers(thing, alone[0][1], names):
            target, room_words = None, thing
    elif target is None:
        room_words = toks
    hint = _hint(room_words, names)
    colours = tuple(COLOURS[w] for w in (target or "").split() if w in COLOURS)
    sizes = [SIZES[w] for w in (target or "").split() if w in SIZES]
    unused = " ".join(t for t in rest if t not in _PUNCT)
    return Request(text=text, target=target, room_hint=hint, colours=colours,
                   size=sizes[0] if sizes else None, unused=unused,
                   detector_prompts=(target,) if target else ())


def _covers(words: list, hint: dict, names: Sequence[str]) -> bool:
    """The words are exactly that room's name or vocabulary word."""
    if "name" in hint:
        return tuple(words) == tuple(_tokens(hint["name"]))
    return _WORD_TYPE.get(" ".join(words)) == hint["type"]


MAX_HINT_NAME = 40


def check_hint(hint, names: Optional[Sequence[str]] = None) -> Optional[dict]:
    """A `room_hint` as `/mission/start` takes it -> the same, validated
    (a vocabulary type, or one of `names` when they are given), or None for
    None. Raises ValueError with the reason."""
    if hint is None:
        return None
    if not isinstance(hint, dict) or len(hint) != 1:
        raise ValueError('room_hint must be {"type": <room type>} or {"name": <room name>}')
    (key, value), = hint.items()
    if not isinstance(value, str) or not value.strip():
        raise ValueError("room_hint's value must be a non-empty string")
    if key == "type":
        if value not in ROOM_WORDS:
            raise ValueError(f"room_hint type {value!r} is not one of {', '.join(ROOM_TYPES)}")
        return {"type": value}
    if key == "name":
        if len(value) > MAX_HINT_NAME:
            raise ValueError(f"room_hint name is over {MAX_HINT_NAME} characters")
        if names is not None and value not in names:
            raise ValueError(f"room_hint name {value!r} is not a room named on this map")
        return {"name": value}
    raise ValueError('room_hint must be {"type": <room type>} or {"name": <room name>}')
