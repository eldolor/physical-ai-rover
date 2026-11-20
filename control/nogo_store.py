# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
control/nogo_store.py -- 3.161 (D2): the brain keeps a person's no-go zones,
one local file per saved map.

`<NOGO_DIR>/<frame_id>.json` (default `recordings/nogo` under the repo,
which git ignores), written whole, atomically and fsynced, as 3.160's room
names are. Local only: never in git, never uploaded -- a home's floor plan.
No expiry: a zone stays until a person deletes it (D1).

    {"version": 1, "frame_id": "map-...", "rev": n, "zones": [zone, ...]}

`rev` counts edits; an edit may name the `rev` it saw and is refused if the
file moved since (another tab's edit). With no `frame_id` (no saved map, the
simulator's own world) nothing can be keyed, and every edit is refused.
"""

from __future__ import annotations

import json
import logging
import os
import re
import tempfile
import threading
from pathlib import Path
from typing import Optional

from world import nogo

logger = logging.getLogger(__name__)

FRAME_ID_RE = re.compile(r"[A-Za-z0-9_.:-]{1,64}")
MAX_FILE_BYTES = 256_000
DEFAULT_DIR = Path(__file__).resolve().parents[1] / "recordings" / "nogo"
CLOSED_NO_FRAME = "Survey first: zones are kept with the saved map."
# 3.161.B part 2 (E3, E4): a zone over the dock, in the operator's words.
DOCK_REFUSED = ("That zone covers the robot's dock, so it couldn't get home. "
                "Draw it clear of the dock.")
DOCK_CONFLICT = ("This zone covers the robot's dock, so the robot ignores it. "
                 "Delete it, or draw it clear of the dock.")


class NoGoRefused(ValueError):
    """An edit that cannot be made as asked."""


def dock_leg(dock: dict) -> tuple:
    """E1: the trip home's last leg, (staging point, dock), in the house frame.
    `dock` is `brain/dock.py`'s {x_m, y_m, theta}."""
    from brain.dock import staging_point
    return staging_point(dock), (float(dock["x_m"]), float(dock["y_m"]))


def dock_conflicts(zones, dock: Optional[dict]) -> list:
    """E1: the ids of the zones that cover the dock (within DOCK_CLEAR_M of
    its last leg). No dock known: none."""
    if not dock:
        return []
    a, b = dock_leg(dock)
    return [z["id"] for z in zones or ()
            if nogo.segment_distance(z, a, b) <= nogo.DOCK_CLEAR_M + 1e-9]


def split_for_dock(zones, dock: Optional[dict]) -> tuple:
    """E4: (the zones in force, the ids left out for covering the dock)."""
    zones = list(zones or ())
    out = set(dock_conflicts(zones, dock))
    return [z for z in zones if z["id"] not in out], [z["id"] for z in zones if z["id"] in out]


def dock_report(zones, dock: Optional[dict]) -> list:
    """E4/E6: GET /mission/nogo's `dock_conflicts`."""
    return [{"id": i, "reason": DOCK_CONFLICT} for i in dock_conflicts(zones, dock)]


def default_dir() -> Path:
    return Path(os.environ.get("NOGO_DIR") or DEFAULT_DIR)


class NoGoStore:
    def __init__(self, directory=None):
        self.dir = Path(directory) if directory else default_dir()
        self._lock = threading.Lock()

    def _path(self, frame_id) -> Path:
        if not isinstance(frame_id, str) or not FRAME_ID_RE.fullmatch(frame_id):
            raise NoGoRefused(CLOSED_NO_FRAME)
        return self.dir / f"{frame_id}.json"

    def load(self, frame_id: Optional[str]) -> dict:
        """The doc for a saved map; an empty one when nothing is stored. A
        file that exists but cannot be read is `unreadable` (no zones), and
        every edit is refused so it is never overwritten with less."""
        empty = {"version": 1, "frame_id": frame_id, "rev": 0, "zones": []}
        if not frame_id:
            return empty
        try:
            p = self._path(frame_id)
        except NoGoRefused:
            return empty
        try:
            if p.stat().st_size > MAX_FILE_BYTES:
                raise ValueError("larger than any set of zones")
            doc = json.loads(p.read_text())
        except FileNotFoundError:
            return empty
        except Exception as e:  # noqa: BLE001 -- any damage: no zones, and no edits
            logger.warning("no-go file %s unreadable: %s", p, e)
            return dict(empty, unreadable=str(e)[:200])
        try:
            if not isinstance(doc, dict):
                raise ValueError("not a no-go file")
            zones = nogo.clean_zones(doc.get("zones", []))
            rev = doc.get("rev", 0)
            if isinstance(rev, bool) or not isinstance(rev, int) or rev < 0:
                raise ValueError("a bad rev")
        except (ValueError, nogo.ZoneRefused) as e:
            logger.warning("no-go file %s unreadable: %s", p, e)
            return dict(empty, unreadable=str(e)[:200])
        return {"version": 1, "frame_id": frame_id, "rev": rev, "zones": zones}

    def zones(self, frame_id: Optional[str]) -> list:
        return self.load(frame_id)["zones"]

    def _save(self, frame_id: str, doc: dict) -> None:
        p = self._path(frame_id)
        p.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix=f".{frame_id}.", dir=str(p.parent))
        try:
            with os.fdopen(fd, "w") as fh:
                json.dump(doc, fh, indent=1)
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

    def edit(self, frame_id: Optional[str], op: str, rev=None, zone=None, zone_id=None,
             dock: Optional[dict] = None) -> dict:
        """One person's edit, saved; returns the new doc.
        * add    {zone}     -- a rectangle (or a wall from `wall_from_line`)
        * delete {zone_id}  -- remove one
        * clear             -- remove all
        Raises NoGoRefused (no saved map, an unreadable file, a stale rev,
        a bad zone, too many, an unknown id, or -- with `dock` known -- an add
        that covers the dock, E3)."""
        self._path(frame_id)
        with self._lock:
            doc = self.load(frame_id)
            if doc.get("unreadable"):
                raise NoGoRefused("the zones file is damaged: nothing is changed "
                                  f"({doc['unreadable']})")
            if rev is not None and rev != doc["rev"]:
                raise NoGoRefused("the zones changed since they were drawn: reload")
            zones = list(doc["zones"])
            if op == "add":
                if len(zones) >= nogo.MAX_ZONES:
                    raise NoGoRefused(f"at most {nogo.MAX_ZONES} zones a map")
                try:
                    z = nogo.clean_zone({k: v for k, v in (zone or {}).items()
                                         if k in ("x0", "y0", "x1", "y1", "kind")})
                except nogo.ZoneRefused as e:
                    raise NoGoRefused(str(e)) from None
                if dock_conflicts([z], dock):
                    raise NoGoRefused(DOCK_REFUSED)
                zones.append(z)
            elif op == "delete":
                kept = [z for z in zones if z["id"] != zone_id]
                if len(kept) == len(zones):
                    raise NoGoRefused("no such zone: reload")
                zones = kept
            elif op == "clear":
                zones = []
            else:
                raise NoGoRefused(f"unknown edit {op!r}")
            new = {"version": 1, "frame_id": frame_id, "rev": doc["rev"] + 1, "zones": zones}
            self._save(frame_id, new)
            return new


_DEFAULT: Optional[NoGoStore] = None


def default_store() -> NoGoStore:
    """The process's store, at `NOGO_DIR` as read the first time."""
    global _DEFAULT
    if _DEFAULT is None:
        _DEFAULT = NoGoStore()
    return _DEFAULT


def push(base_url: str, secret: Optional[str], frame_id: Optional[str], zones: list,
         timeout: float = 5.0, client=None) -> dict:
    """D2: the whole set for the saved map the robot is on, to the robot
    server (`PUT /world/nogo`), which passes it on to the bridge. Its answer,
    or {"accepted": False, "detail": why} -- never raises: the store keeps
    the zones, and the next start or edit sends them again."""
    import httpx
    body = {"frame_id": frame_id, "zones": [dict(z) for z in zones]}
    headers = {"x-app-secret": secret} if secret else {}
    try:
        if client is not None:
            r = client.put("/world/nogo", json=body, headers=headers)
        else:
            r = httpx.put(base_url.rstrip("/") + "/world/nogo", json=body, headers=headers,
                          timeout=timeout)
        if r.status_code in (404, 405, 501):
            return {"accepted": False, "detail": "the robot server predates no-go zones"}
        r.raise_for_status()
        return r.json()
    except Exception as e:  # noqa: BLE001 -- see the docstring
        logger.warning("no-go zones not sent to the robot server: %s", e)
        return {"accepted": False, "detail": str(e)[:200]}
