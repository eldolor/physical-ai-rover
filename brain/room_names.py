# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
brain/room_names.py

3.160: rooms a person names once, kept with the saved map.

A named room is GEOMETRY, not a cut's id: the room cut (`room_labels.segment()`)
renumbers whenever the map changes, so a name is stored with the room's cells
on a fixed STORE_RES_M grid in the saved map's frame, one file per `frame_id`
(3.50.D5: `map-<8 hex>`, the same across reloads and re-maps). Any cut is
named by overlap: a cut room takes the name of the stored room that covers at
least MATCH_FRACTION of it (the largest share wins).

* `RoomNames(dir)` -- the store: `load(frame_id)`, and the edits a person makes
  (`name`, `merge`, `split`, `clear`, `place`), each written whole, atomically
  and fsynced. Local only: never in git (`recordings/` is ignored), never
  uploaded -- a home's floor plan.
* `name_cells(m, rooms)` -- the person's name for each map cell ("" where none):
  a cell inside a person's room carries its name whatever the cut does, so a
  re-cut that merges two rooms a person split never moves a name.
* `match(m, seg, rooms)` -- {cut room id: index into rooms}: which stored room
  a cut room IS (for the editor: what a tap edits); `placed(m, rooms)` -- which
  stored rooms still lie on the map's floor.
* `named_at(rooms, x, y)` -- the person's name for the room at a point, or "".

The directory is `ROOM_NAMES_DIR` (default `recordings/room-names` under the
repo). With no `frame_id` (no saved map; the simulator's own world) there is
nothing to key a name to, and every edit is refused.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import tempfile
import threading
import time
import uuid
from pathlib import Path
from typing import Optional

import logging

import numpy as np

logger = logging.getLogger(__name__)

STORE_RES_M = 0.10
MATCH_FRACTION = 0.60
MAX_ROOMS = 64
MAX_NAME = 40
HISTORY_KEPT = 200
FRAME_ID_RE = re.compile(r"[A-Za-z0-9_.:-]{1,64}")
STORED_RE = re.compile(r"s([0-9]{1,4})")
MAX_KEYS = 2_000_000             # cells a file may hold in all (a damaged file is refused)
MAX_CELL = 100_000               # |x|, |y| in store cells: 10 km from the map's origin
DEFAULT_DIR = Path(__file__).resolve().parents[1] / "recordings" / "room-names"


class NamesRefused(ValueError):
    """An edit that cannot be made as asked (bad input, an unknown room)."""


def default_dir() -> Path:
    return Path(os.environ.get("ROOM_NAMES_DIR") or DEFAULT_DIR)


def clean_name(name) -> str:
    """A person's name for a room: printable, trimmed, at most MAX_NAME."""
    s = " ".join(str(name or "").split())
    s = "".join(ch for ch in s if ch.isprintable())
    if len(s) > MAX_NAME:
        raise NamesRefused(f"a room name is at most {MAX_NAME} characters")
    return s


# ---------- geometry ----------

def _key(x: float, y: float) -> tuple:
    return (math.floor(x / STORE_RES_M), math.floor(y / STORE_RES_M))


def cut_cells(m: dict, seg: np.ndarray) -> dict:
    """{cut room id: set of store keys} -- each map cell's centre on the store grid."""
    res = float(m["resolution_m"])
    ox, oy = float(m["origin_x_m"]), float(m["origin_y_m"])
    out: dict = {}
    ys, xs = np.nonzero(seg >= 0)
    if not len(xs):
        return out
    kx = np.floor((ox + (xs + 0.5) * res) / STORE_RES_M).astype(int)
    ky = np.floor((oy + (ys + 0.5) * res) / STORE_RES_M).astype(int)
    for r, a, b in zip(seg[ys, xs].tolist(), kx.tolist(), ky.tolist()):
        out.setdefault(int(r), set()).add((a, b))
    return out


def to_runs(keys) -> list:
    """A set of (ix, iy) as [[iy, ix0, ix1], ...]: runs along x, inclusive."""
    runs = []
    for iy in sorted({k[1] for k in keys}):
        xs = sorted(k[0] for k in keys if k[1] == iy)
        start = prev = xs[0]
        for x in xs[1:]:
            if x != prev + 1:
                runs.append([iy, start, prev])
                start = x
            prev = x
        runs.append([iy, start, prev])
    return runs


def from_runs(runs) -> set:
    """Inverse of to_runs. Raises ValueError on runs no edit could have
    written (more than MAX_KEYS cells, a run backwards): a damaged file."""
    total = 0
    for iy, a, b in runs:
        if int(b) < int(a):
            raise ValueError("a run ends before it starts")
        if max(abs(int(iy)), abs(int(a)), abs(int(b))) > MAX_CELL:
            raise ValueError("a run far outside any house")
        total += int(b) - int(a) + 1
    if total > MAX_KEYS:
        raise ValueError(f"{total} cells: more than any map holds")
    return {(x, int(iy)) for iy, a, b in runs for x in range(int(a), int(b) + 1)}


def match(m: dict, seg: np.ndarray, rooms: list, cells: Optional[dict] = None) -> dict:
    """{cut room id: index into `rooms`} where that stored room covers at
    least MATCH_FRACTION of the cut room (the largest share wins)."""
    cells = cells if cells is not None else cut_cells(m, seg)
    sets = [r["_keys"] if "_keys" in r else from_runs(r["runs"]) for r in rooms]
    out = {}
    for rid, ks in cells.items():
        best, share = None, 0.0
        for i, s in enumerate(sets):
            f = len(ks & s) / len(ks)
            if f >= MATCH_FRACTION and f > share:
                best, share = i, f
        if best is not None:
            out[rid] = best
    return out


def owner_grid(m: dict, rooms: list, named_only: bool = False) -> np.ndarray:
    """The index of the stored room each of the map's cells lies in, as a
    (height, width) int array (-1 in none; the first room wins where two
    overlap). Vectorised: the rooms are painted on the store grid once, and
    the map's cells look themselves up there."""
    w, h, res = int(m["width"]), int(m["height"]), float(m["resolution_m"])
    out = np.full((h, w), -1, dtype=np.int32)
    keys = [(i, r["_keys"]) for i, r in enumerate(rooms) if r["_keys"] and (r.get("name") or not named_only)]
    if not keys or not w or not h:
        return out
    xs = [k[0] for _, ks in keys for k in ks]
    ys = [k[1] for _, ks in keys for k in ks]
    x0, y0 = min(xs), min(ys)
    paint = np.full((max(ys) - y0 + 1, max(xs) - x0 + 1), -1, dtype=np.int32)
    for i, ks in reversed(keys):              # the first room painted last: it wins
        a = np.array(list(ks), dtype=np.int64)
        paint[a[:, 1] - y0, a[:, 0] - x0] = i
    ox, oy = float(m["origin_x_m"]), float(m["origin_y_m"])
    kx = np.floor((ox + (np.arange(w) + 0.5) * res) / STORE_RES_M).astype(np.int64) - x0
    ky = np.floor((oy + (np.arange(h) + 0.5) * res) / STORE_RES_M).astype(np.int64) - y0
    okx = (kx >= 0) & (kx < paint.shape[1])
    oky = (ky >= 0) & (ky < paint.shape[0])
    sub = paint[np.clip(ky, 0, paint.shape[0] - 1)][:, np.clip(kx, 0, paint.shape[1] - 1)]
    out[:] = np.where(oky[:, None] & okx[None, :], sub, -1)
    return out


def owner_cells(m: dict, rooms: list, named_only: bool = False) -> list:
    """owner_grid() as a row-major list."""
    return owner_grid(m, rooms, named_only).ravel().tolist()


def name_cells(m: dict, rooms: list) -> list:
    """The person's name for each of the map's cells, row-major ("" off any
    named room). Free and unknown cells alike: the layer is per cell."""
    return [rooms[i]["name"] if i >= 0 else "" for i in owner_cells(m, rooms, named_only=True)]


PLACED_FRACTION = 0.40


def placed(m: dict, rooms: list) -> list:
    """For each stored room: does the map still have its floor (at least
    PLACED_FRACTION of its cells free)? A re-map that moved the map under a
    room leaves it unplaced until a person places it (never guessed)."""
    w, h, res = int(m["width"]), int(m["height"]), float(m["resolution_m"])
    ox, oy = float(m["origin_x_m"]), float(m["origin_y_m"])
    cells = m["cells"]
    out = []
    for r in rooms:
        keys, free = r["_keys"], 0
        for kx, ky in keys:
            x = int(((kx + 0.5) * STORE_RES_M - ox) // res)
            y = int(((ky + 0.5) * STORE_RES_M - oy) // res)
            free += 0 <= x < w and 0 <= y < h and cells[y * w + x] == 0
        out.append(bool(keys) and free / len(keys) >= PLACED_FRACTION)
    return out


def named_at(rooms: list, x: float, y: float) -> str:
    k = _key(x, y)
    for r in rooms:
        if r.get("name") and k in r["_keys"]:
            return r["name"]
    return ""


# ---------- the store ----------

class RoomNames:
    """One file per saved map. Reads are cached by the file's mtime; every
    edit is read-modify-write under one lock and replaces the file whole."""

    def __init__(self, directory=None):
        self.dir = Path(directory) if directory else default_dir()
        self._lock = threading.Lock()
        self._cache: dict = {}            # frame_id -> (mtime_ns, doc)

    def _path(self, frame_id: str) -> Path:
        if not isinstance(frame_id, str) or not FRAME_ID_RE.fullmatch(frame_id):
            raise NamesRefused("rooms are named on a saved map: no frame_id yet "
                               "(survey first; names are kept with the saved map)")
        return self.dir / f"{frame_id}.json"

    def load(self, frame_id: Optional[str]) -> dict:
        """The doc for a saved map; an empty one when nothing is stored. Each
        room carries `_keys` (its cells as a set) for the readers; never saved.
        A file that exists but cannot be read as one is `unreadable` (no
        rooms): readers go on without names, and every edit is refused, so
        the file is never overwritten with less than it held (review)."""
        empty = {"version": 1, "frame_id": frame_id, "rooms": [], "history": [], "rev": 0}
        if not frame_id:
            return empty
        try:
            p = self._path(frame_id)
            mt = p.stat().st_mtime_ns
        except (NamesRefused, OSError):
            return empty
        hit = self._cache.get(frame_id)
        if hit and hit[0] == mt:
            return hit[1]
        try:
            doc = json.loads(p.read_text())
            if not isinstance(doc, dict) or not isinstance(doc.get("rooms", []), list):
                raise ValueError("not a room-names file")
            for r in doc.get("rooms", []):
                r["_keys"] = from_runs(r.get("runs", []))
            doc.setdefault("rev", len(doc.get("history", [])))
        except Exception as e:  # noqa: BLE001 -- any damage: no names, and no edits
            logger.warning("room names file %s unreadable: %s", p, e)
            return dict(empty, unreadable=str(e)[:200])
        self._cache[frame_id] = (mt, doc)
        return doc

    def rooms(self, frame_id: Optional[str]) -> list:
        return self.load(frame_id)["rooms"]

    def _save(self, frame_id: str, doc: dict) -> None:
        p = self._path(frame_id)
        p.parent.mkdir(parents=True, exist_ok=True)
        out = {**doc, "rooms": [{k: v for k, v in r.items() if k != "_keys"} for r in doc["rooms"]],
               "history": doc.get("history", [])[-HISTORY_KEPT:]}
        fd, tmp = tempfile.mkstemp(prefix=f".{frame_id}.", dir=str(p.parent))
        try:
            with os.fdopen(fd, "w") as fh:
                json.dump(out, fh, indent=1)
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp, p)
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
        try:
            dfd = os.open(str(p.parent), os.O_RDONLY)
            try:
                os.fsync(dfd)
            finally:
                os.close(dfd)
        except OSError:
            pass
        self._cache.pop(frame_id, None)

    def edit(self, frame_id: str, m: dict, seg: np.ndarray, op: str, rev=None, **kw) -> dict:
        """Apply one person's edit against THIS cut (`m`, `seg`), save, and
        return the new doc. Ops:
        A target (`room`, `other`) is a cut room's id (an int) or a stored
        room as "s<index>" (a piece a person split off, which no cut room is).
        * name  {room, name}      -- name the room
        * merge {room, other}     -- one room from two (the larger keeps its name)
        * split {room, line}      -- two rooms either side of `line` [[x, y], [x, y]] (metres)
        * clear {room}            -- forget the person's room there
        * place {index, room}     -- put stored room `index` (unplaced after a re-map) on cut room `room`
        """
        self._path(frame_id)
        with self._lock:
            cur = self.load(frame_id)
            if cur.get("unreadable"):
                raise NamesRefused("the saved room names can't be read, so nothing is changed: "
                                   f"{cur['unreadable']}")
            if rev is not None and rev != cur.get("rev", 0):
                raise NamesRefused("the room names changed since this list was drawn: reload")
            doc = json.loads(json.dumps({k: v for k, v in cur.items()}, default=lambda o: None))
            rooms = doc["rooms"]
            for r in rooms:
                r["_keys"] = from_runs(r.get("runs", []))
            cells = cut_cells(m, seg)
            # Only rooms on this map can be what a cut room is (the view hides
            # the others: an edit must agree with what the person saw).
            on_map = placed(m, rooms) if rooms else []
            live = [i for i, ok in enumerate(on_map) if ok]
            matched = {r: live[j] for r, j in match(m, seg, [rooms[i] for i in live], cells).items()}

            def is_stored(t) -> bool:
                return isinstance(t, str) and bool(STORED_RE.fullmatch(t))

            def cut(room) -> set:
                if is_stored(room):
                    i = int(room[1:])
                    if i >= len(rooms):
                        raise NamesRefused(f"no stored room {room!r} (reload)")
                    if not on_map[i] and op != "clear":   # a lost room may still be forgotten
                        raise NamesRefused("that room isn't on this map: place it first")
                    return rooms[i]["_keys"]
                if not isinstance(room, int) or room not in cells:
                    raise NamesRefused(f"no room {room!r} in this map's cut (the map changed: reload)")
                return cells[room]

            def stored(room):            # the stored room a target is, or None
                if is_stored(room):
                    return rooms[int(room[1:])]
                i = matched.get(room)
                return rooms[i] if i is not None else None

            def put(keys: set, name: str, base=None):
                r = base if base is not None else {"id": uuid.uuid4().hex[:8], "by": "person"}
                r.update({"name": name, "runs": to_runs(keys), "_keys": keys,
                          "set_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")})
                if base is None:
                    if len(rooms) >= MAX_ROOMS:
                        raise NamesRefused(f"at most {MAX_ROOMS} rooms a map")
                    rooms.append(r)
                return r

            room = kw.get("room")
            if op == "name":
                name = clean_name(kw.get("name"))
                if not name:
                    raise NamesRefused("a name is needed (to forget one, clear it)")
                kw["name"] = name
                keys, s = cut(room), stored(room)
                put(s["_keys"] | keys if s else keys, name, s)
            elif op == "merge":
                other = kw.get("other")
                a, b = cut(room), cut(other)
                sa, sb = stored(room), stored(other)
                if other == room or (sa is not None and sa is sb):
                    raise NamesRefused("merge needs two different rooms")
                ka, kb = (sa["_keys"] if sa else a), (sb["_keys"] if sb else b)
                big, small = (sa, sb) if len(ka) >= len(kb) else (sb, sa)
                name = (big or {}).get("name") or (small or {}).get("name") or ""
                if small is not None and small is not big:
                    rooms.remove(small)
                put(ka | kb, name, big if big is not None else None)
            elif op == "split":
                line = kw.get("line")
                try:
                    (x0, y0), (x1, y1) = [(float(p[0]), float(p[1])) for p in line]
                except Exception:  # noqa: BLE001 -- any malformed line is the caller's
                    raise NamesRefused("split needs a line [[x, y], [x, y]] in metres")
                if not all(map(math.isfinite, (x0, y0, x1, y1))) or (x0, y0) == (x1, y1):
                    raise NamesRefused("split needs a line of two different points")
                s = stored(room)
                keys = s["_keys"] if s else cut(room)
                side = lambda k: ((x1 - x0) * ((k[1] + 0.5) * STORE_RES_M - y0)  # noqa: E731
                                  - (y1 - y0) * ((k[0] + 0.5) * STORE_RES_M - x0)) >= 0
                left = {k for k in keys if side(k)}
                right = keys - left
                if not left or not right:
                    raise NamesRefused("the line does not cross that room")
                big, small = (left, right) if len(left) >= len(right) else (right, left)
                put(big, (s or {}).get("name", ""), s)
                put(small, "")
            elif op == "clear":
                cut(room)
                s = stored(room)
                if s is not None:
                    rooms.remove(s)
            elif op == "place":
                i = kw.get("index")
                if isinstance(i, bool) or not isinstance(i, int) or not 0 <= i < len(rooms):
                    raise NamesRefused(f"no stored room {i!r}")
                if is_stored(room):
                    raise NamesRefused("place a stored room on a room of the cut")
                keys, s = cut(room), stored(room)
                if s is not None and s is not rooms[i]:
                    raise NamesRefused("that room already has a person's name: clear it first")
                put(keys, rooms[i].get("name", ""), rooms[i])
            else:
                raise NamesRefused(f"unknown edit {op!r}")
            doc["rev"] = cur.get("rev", 0) + 1
            doc["history"] = doc.get("history", []) + [
                {"at": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "op": op,
                 **{k: v for k, v in kw.items() if k in ("room", "other", "name", "index")}}]
            self._save(frame_id, doc)
            return self.load(frame_id)


_DEFAULT: Optional[RoomNames] = None


def default_store() -> RoomNames:
    """The process's store, at `ROOM_NAMES_DIR` as read the first time."""
    global _DEFAULT
    if _DEFAULT is None:
        _DEFAULT = RoomNames()
    return _DEFAULT


# ---------- the twin's editor (3.160.B: here, so control/ imports no numpy) ----------

def runs(a) -> list:
    """A per-cell list as [value, count, value, count, ...] (most of a map
    is one value: the reply stays small over the tunnel)."""
    a = np.asarray(a).ravel()
    if not a.size:
        return []
    edges = np.flatnonzero(np.diff(a)) + 1
    starts = np.concatenate(([0], edges))
    counts = np.diff(np.concatenate((starts, [a.size])))
    return np.column_stack((a[starts], counts)).ravel().tolist()


def cut_hash(seg) -> str:
    return hashlib.sha256(np.ascontiguousarray(seg, dtype=np.int32).tobytes()).hexdigest()[:16]


def editor_view(store: "RoomNames", m: dict, frame_id: str, seg=None, seeds=()) -> dict:
    """What the editor draws: the cut with the person's rooms over it, one
    row per room (`s<n>` a person's room on this map, `c<n>` an unnamed cut
    room with the model's `suggestion` from `seeds` [x, y, name]), the stored
    rooms a re-map left unplaced, and the `cut_hash` and `rev` an edit names.
    `cut` and `owner` are run-length lists."""
    from brain.room_labels import segment     # a cycle at import time otherwise
    if seg is None:
        seg, _ = segment(m)
    doc = store.load(frame_id)
    if doc.get("unreadable"):
        return {"closed": "The saved room names can't be read, so they can't be changed here.",
                "rooms": []}
    rooms = doc["rooms"]
    on_map = placed(m, rooms) if rooms else []
    shown = [r if on_map[i] else {"_keys": set(), "name": r.get("name", "")}
             for i, r in enumerate(rooms)]
    own = owner_grid(m, shown).ravel()
    free = np.asarray(m["cells"]) == 0
    flat = seg.ravel()
    res, ox, oy, width = m["resolution_m"], m["origin_x_m"], m["origin_y_m"], m["width"]

    def centre(idx):
        return (round(float(ox + ((idx % width).mean() + 0.5) * res), 2),
                round(float(oy + ((idx // width).mean() + 0.5) * res), 2))
    out = []
    for i, r in enumerate(rooms):
        idx = np.flatnonzero((own == i) & free) if on_map[i] else np.array([], dtype=np.int64)
        if idx.size:
            x, y = centre(idx)
            out.append({"key": f"s{i}", "name": r.get("name", ""), "source": "person",
                        "x_m": x, "y_m": y, "cells": int(idx.size)})
    order = np.argsort(flat, kind="stable")
    vals, first = np.unique(flat[order], return_index=True)
    bounds = list(first) + [flat.size]
    for n, rid in enumerate(vals.tolist()):
        if rid < 0:
            continue
        idx = order[bounds[n]:bounds[n + 1]]
        if (own[idx] >= 0).sum() >= 0.6 * idx.size:
            continue                  # a person's room covers it
        votes: dict = {}
        for sx, sy, name in seeds:
            cx, cy = int((sx - ox) // res), int((sy - oy) // res)
            if 0 <= cx < width and 0 <= cy < m["height"] and flat[cy * width + cx] == rid:
                votes[name] = votes.get(name, 0) + 1
        x, y = centre(idx)
        sug = max(votes, key=votes.get) if votes else ""
        out.append({"key": f"c{rid}", "name": "", "source": "model" if sug else "none",
                    "suggestion": sug, "x_m": x, "y_m": y, "cells": int(idx.size)})
    return {"rev": doc.get("rev", 0), "cut_hash": cut_hash(seg),
            "map": {k: m.get(k) for k in ("map_id", "map_version", "width", "height",
                                          "resolution_m", "origin_x_m", "origin_y_m")},
            "cut": runs(flat), "owner": runs(own), "rooms": out,
            "unplaced": [{"key": f"s{i}", "index": i, "name": r.get("name", "")}
                         for i, r in enumerate(rooms) if not on_map[i] and r.get("name")]}
