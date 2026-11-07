# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
brain/what_i_saw.py

3.162: what a mission saw, kept on the robot for the person to check -- the
frame its arrival's identity check judged, a thumbnail of each thing it
placed on the map, and the rooms.

One directory per mission, `<WHAT_I_SAW_DIR>/<mission_id>/` (default
`recordings/what-i-saw`, ignored by git):

* `report.json` -- the end state, job, target, times, the saved map's
  `frame_id`, the rooms entered, the found frame's record (its SHA-256 as
  the identity check was sent it, its box, whether it was confirmed), and
  one row per landmark (label, position, room, belief, looks, thumbnail);
* `found.jpg` -- the judged frame, byte for byte as sent (FOUND_MAX_BYTES;
  a larger one is stored re-encoded and says so: its hash stays the sent
  frame's);
* `thumbs/<n>.jpg` -- the best look of each landmark, cropped to its box,
  THUMB_PX on the long side, at most THUMB_MAX_BYTES; at most MAX_THUMBS,
  the highest belief kept.

Bounded: a mission at most MISSION_MAX_BYTES, the store at most KEEP
missions and STORE_MAX_BYTES, the oldest deleted first. A mission is written
whole (built in a temporary directory, then renamed in) and deleted whole.
Local only: frames never leave the robot except to the person's own phone
through the brain's authenticated route; 3.46's S3 inventory carries labels
and positions, no image.

`ReportWriter` saves on a thread of its own: a mission's end never waits on
a disk, and a write that fails is logged once and never fails a mission
(3.99's rule).
"""

from __future__ import annotations

import hashlib
import io
import json
import logging
import os
import queue
import re
import shutil
import tempfile
import threading
import time
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

MAX_THUMBS = 60
THUMB_PX = 160
THUMB_MAX_BYTES = 12_000
FOUND_MAX_BYTES = 150_000
MISSION_MAX_BYTES = 1_000_000
KEEP = 50
STORE_MAX_BYTES = 50_000_000
MAX_ROWS = 200                   # landmark rows in report.json (the likeliest first)
REPORT_MAX_BYTES = 20_000        # report.json's share of a mission's bytes
ORPHAN_S = 600.0                 # a hidden temp or half-deleted directory older than this is swept
ID_RE = re.compile(r"[A-Za-z0-9_.-]{1,80}")
THUMB_RE = re.compile(r"thumbs/([0-9]{1,3})\.jpg")
DEFAULT_DIR = Path(__file__).resolve().parents[1] / "recordings" / "what-i-saw"


class ReportRefused(ValueError):
    """A request for a report that is not there, or a name that is not one."""


def default_dir() -> Path:
    return Path(os.environ.get("WHAT_I_SAW_DIR") or DEFAULT_DIR)


def _jpeg(img, limit: int) -> bytes:
    """`img` as a JPEG of at most `limit` bytes: quality stepped down, then
    the image halved, until it fits."""
    while True:
        for q in (80, 65, 50, 35, 25):
            buf = io.BytesIO()
            img.save(buf, "JPEG", quality=q)
            if buf.tell() <= limit:
                return buf.getvalue()
        if max(img.size) <= 16:
            return buf.getvalue()
        img = img.resize((max(1, img.size[0] // 2), max(1, img.size[1] // 2)))


def thumbnail(jpeg: bytes, box=None) -> bytes:
    """The look cropped to `box` (fractions of the image: x0, y0, x1, y1,
    grown by a tenth each side), THUMB_PX on its long side, a JPEG of at most
    THUMB_MAX_BYTES."""
    from PIL import Image                     # only where a report is written
    img = Image.open(io.BytesIO(jpeg)).convert("RGB")
    if box:
        w, h = img.size
        x0, y0, x1, y1 = (float(v) for v in box)
        gx, gy = 0.1 * (x1 - x0), 0.1 * (y1 - y0)
        crop = (max(0, int((x0 - gx) * w)), max(0, int((y0 - gy) * h)),
                min(w, int((x1 + gx) * w) + 1), min(h, int((y1 + gy) * h) + 1))
        if crop[2] > crop[0] and crop[3] > crop[1]:
            img = img.crop(crop)
    img.thumbnail((THUMB_PX, THUMB_PX))
    return _jpeg(img, THUMB_MAX_BYTES)


def _clean(v):
    """A value as JSON can hold it: numpy numbers as plain ones, NaN and
    infinities as None, anything else unknown as its string."""
    if isinstance(v, dict):
        return {str(k): _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    if isinstance(v, bool) or v is None or isinstance(v, str):
        return v
    try:
        f = float(v)
    except (TypeError, ValueError):
        return str(v)
    if f != f or f in (float("inf"), float("-inf")):
        return None
    return int(f) if isinstance(v, int) or (hasattr(v, "dtype") and "int" in str(v.dtype)) else f


def _fsync_dir(d: Path) -> None:
    try:
        fd = os.open(str(d), os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError:
        pass


def _write(path: Path, data: bytes) -> None:
    with open(path, "wb") as fh:
        fh.write(data)
        fh.flush()
        os.fsync(fh.fileno())


def _found_bytes(jpeg: bytes) -> tuple:
    """(bytes to store, re-encoded?) for the judged frame: as sent when it
    is a JPEG of at most FOUND_MAX_BYTES, else re-encoded (the hash recorded
    is always the sent frame's)."""
    if len(jpeg) <= FOUND_MAX_BYTES and jpeg[:3] == b"\xff\xd8\xff":
        return jpeg, False
    from PIL import Image
    return _jpeg(Image.open(io.BytesIO(jpeg)).convert("RGB"), FOUND_MAX_BYTES), True


def _size(d: Path) -> int:
    return sum(p.stat().st_size for p in d.rglob("*") if p.is_file())


_LOCKS: dict = {}
_LOCKS_LOCK = threading.Lock()


def _lock_for(d: Path) -> threading.Lock:
    """One lock per directory in this process: the writer's store and the
    routes' store never prune and delete over each other (review)."""
    key = str(d.resolve()) if d.exists() else str(d.absolute())
    with _LOCKS_LOCK:
        return _LOCKS.setdefault(key, threading.Lock())


class ReportStore:
    def __init__(self, directory=None):
        self.dir = Path(directory) if directory else default_dir()
        self._lock = _lock_for(self.dir)

    def _sweep(self) -> None:
        """Hidden directories a crashed save or a failed delete left (a kill,
        a power cut): older than ORPHAN_S, removed. Never a live save's."""
        now = time.time()
        for p in self.dir.iterdir():
            if p.is_dir() and p.name.startswith("."):
                try:
                    if now - p.stat().st_mtime > ORPHAN_S:
                        shutil.rmtree(p)
                except OSError as e:
                    logger.warning("could not sweep %s: %s", p, e)

    def _next_seq(self) -> int:
        """A number every save takes, one more than any report holds: the
        store's order, by which it prunes -- not the clock, which a board
        with no RTC can set back."""
        top = 0
        for p in self.dir.iterdir():
            if p.is_dir() and not p.name.startswith("."):
                try:
                    top = max(top, int(json.loads((p / "report.json").read_text()).get("seq") or 0))
                except (OSError, ValueError, TypeError):
                    continue
        return top + 1

    def _mission_dir(self, mission_id: str) -> Path:
        if not isinstance(mission_id, str) or not ID_RE.fullmatch(mission_id) or mission_id[0] == ".":
            raise ReportRefused(f"no report {mission_id!r}")
        return self.dir / mission_id

    # ---- writing ----

    def save(self, mission_id: str, report: dict, found: Optional[dict] = None,
             looks: Optional[list] = None) -> dict:
        """Write one mission's report whole. `found`: {"jpeg", "box",
        "confirmed"} -- the frame the identity check was sent; `looks`: one
        per landmark, {"label", "x_m", "y_m", "room", "belief", "looks",
        "jpeg", "box"}. Returns the saved report.json."""
        final = self._mission_dir(mission_id)
        rep = _clean({k: v for k, v in (report or {}).items() if k not in ("found", "landmarks")})
        rep["mission_id"] = mission_id
        rep["saved_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        self.dir.mkdir(parents=True, exist_ok=True)
        tmp = Path(tempfile.mkdtemp(prefix=f".{mission_id}.", dir=str(self.dir)))
        try:
            budget = MISSION_MAX_BYTES - REPORT_MAX_BYTES
            rep["found"] = None
            if found and found.get("jpeg"):
                raw = found["jpeg"]
                try:
                    data, resized = _found_bytes(raw)
                except Exception as e:  # noqa: BLE001 -- a damaged frame loses itself only
                    logger.warning("report %s: the judged frame not kept (%s)", mission_id, e)
                    data = None
                if data is not None:
                    _write(tmp / "found.jpg", data)
                    budget -= len(data)
                    rep["found"] = _clean({"file": "found.jpg",
                                           "sha256": hashlib.sha256(raw).hexdigest(),
                                           "box": found.get("box"),
                                           "confirmed": bool(found.get("confirmed")),
                                           "resized": resized})
            rows = sorted(looks or [], key=lambda r: -(_clean(r.get("belief")) or 0.0))[:MAX_ROWS]
            marks = []
            (tmp / "thumbs").mkdir()
            for i, r in enumerate(rows):
                row = _clean({k: r.get(k) for k in ("label", "x_m", "y_m", "room", "belief", "looks")})
                row["thumb"] = None
                if r.get("jpeg") and len([m for m in marks if m["thumb"]]) < MAX_THUMBS:
                    try:
                        t = thumbnail(r["jpeg"], r.get("box"))
                    except Exception as e:  # noqa: BLE001 -- a bad frame loses its thumbnail only
                        logger.warning("report %s: no thumbnail for %s (%s)", mission_id, row["label"], e)
                        t = None
                    if t is not None and len(t) <= budget:
                        name = f"thumbs/{len([m for m in marks if m['thumb']])}.jpg"
                        _write(tmp / name, t)
                        budget -= len(t)
                        row["thumb"] = name
                marks.append(row)
            with self._lock:
                rep["seq"] = self._next_seq()
                rep["landmarks"] = marks
                body = json.dumps(rep, separators=(",", ":"), allow_nan=False).encode()
                while len(body) > REPORT_MAX_BYTES and rep["landmarks"]:
                    # Over its share: the least likely rows go (their thumbnails too).
                    gone = rep["landmarks"].pop()
                    if gone.get("thumb"):
                        (tmp / gone["thumb"]).unlink(missing_ok=True)
                    body = json.dumps(rep, separators=(",", ":"), allow_nan=False).encode()
                _write(tmp / "report.json", body)
                _fsync_dir(tmp / "thumbs")
                _fsync_dir(tmp)
                aside = None
                if final.exists():
                    # Old copy aside first, the new one in, then the old removed:
                    # never a moment with neither (review).
                    aside = final.with_name(f".{final.name}.replaced.{time.monotonic_ns()}")
                    os.replace(final, aside)
                os.replace(tmp, final)
                _fsync_dir(self.dir)
                if aside is not None:
                    shutil.rmtree(aside, ignore_errors=True)
                try:
                    self._sweep()
                    self._prune(keep=final)
                except Exception as e:  # noqa: BLE001 -- the report is saved; pruning tries again next time
                    logger.warning("report store not pruned: %s", e)
        except BaseException:
            shutil.rmtree(tmp, ignore_errors=True)
            raise
        return rep

    def _remove(self, d: Path, strict: bool = False) -> None:
        """Whole or not at all: renamed aside first, so no reader sees half.
        `strict` (a person's delete): a remove that cannot finish raises, so
        it is never reported done with frames left on disk."""
        gone = d.with_name(f".{d.name}.deleting.{os.getpid()}.{time.monotonic_ns()}")
        os.replace(d, gone)
        if strict:
            shutil.rmtree(gone)
        else:
            shutil.rmtree(gone, ignore_errors=True)

    def _seq_of(self, p: Path) -> int:
        try:
            return int(json.loads((p / "report.json").read_text()).get("seq") or 0)
        except (OSError, ValueError, TypeError):
            return 0

    def _prune(self, keep: Optional[Path] = None) -> None:
        """Oldest first by the store's own sequence, never `keep` (the report
        just written), until within KEEP and STORE_MAX_BYTES."""
        missions = sorted((p for p in self.dir.iterdir() if p.is_dir() and not p.name.startswith(".")),
                          key=self._seq_of)
        sizes = {p: _size(p) for p in missions}
        total = sum(sizes.values())
        for old in [p for p in missions if p != keep]:
            if len(missions) <= KEEP and total <= STORE_MAX_BYTES:
                break
            missions.remove(old)
            total -= sizes[old]
            self._remove(old)

    def delete(self, mission_id: str) -> bool:
        d = self._mission_dir(mission_id)
        with self._lock:
            if not d.is_dir():
                return False
            self._remove(d, strict=True)
        return True

    # ---- reading ----

    def list(self) -> list:
        """Newest first: one line a mission (no images)."""
        out = []
        if not self.dir.is_dir():
            return out
        for p in self.dir.iterdir():
            if not p.is_dir() or p.name.startswith("."):
                continue
            try:
                rep = json.loads((p / "report.json").read_text())
            except (OSError, ValueError):
                continue
            out.append({"mission_id": p.name, "job": rep.get("job"), "target": rep.get("target"),
                        "outcome": rep.get("outcome"), "end_reason": rep.get("end_reason"),
                        "step": rep.get("step"), "ended": rep.get("ended"),
                        "found": bool(rep.get("found")),
                        "things": len(rep.get("landmarks") or []),
                        "saved_at": rep.get("saved_at"), "seq": rep.get("seq") or 0})
        return sorted(out, key=lambda r: r["seq"], reverse=True)

    def get(self, mission_id: str) -> dict:
        try:
            return json.loads((self._mission_dir(mission_id) / "report.json").read_text())
        except (OSError, ValueError):
            raise ReportRefused(f"no report {mission_id!r}")

    def image(self, mission_id: str, name: str) -> bytes:
        """`found.jpg` or `thumbs/<n>.jpg` of a report, and nothing else."""
        if name != "found.jpg" and not THUMB_RE.fullmatch(name or ""):
            raise ReportRefused(f"no image {name!r}")
        p = self._mission_dir(mission_id) / name
        try:
            return p.read_bytes()
        except OSError:
            raise ReportRefused(f"no image {name!r} in {mission_id!r}")


class ReportWriter:
    """Saves reports on a thread of its own. `submit()` never blocks the
    mission and never raises; a failed save is logged (once per mission)."""

    def __init__(self, store: Optional[ReportStore] = None):
        self.store = store or ReportStore()     # the routes' store shares its lock (one per directory)
        # Two at most: each holds its frames (the judged one and every look's)
        # in the brain's memory until written; the board's floor is 0.5 GB.
        self._q: "queue.Queue" = queue.Queue(maxsize=2)
        self._t = threading.Thread(target=self._run, daemon=True, name="mission-report-writer")
        self._t.start()
        self.saved = 0
        self.failed = 0

    def submit(self, mission_id: str, report: dict, found=None, looks=None) -> bool:
        try:
            self._q.put_nowait((mission_id, report, found, looks))
            return True
        except queue.Full:
            logger.warning("report %s not saved: the writer is behind", mission_id)
            self.failed += 1
            return False

    def _run(self) -> None:
        while True:
            item = self._q.get()
            if item is None:
                return
            try:
                self.store.save(*item)
                self.saved += 1
            except Exception as e:  # noqa: BLE001 -- never the mission's failure
                self.failed += 1
                logger.warning("report %s not saved: %s", item[0], e)
            finally:
                self._q.task_done()

    def flush(self, timeout: float = 10.0) -> bool:
        """Wait until every submitted report is written (tests, shutdown)."""
        end = time.monotonic() + timeout
        while self._q.unfinished_tasks and time.monotonic() < end:
            time.sleep(0.01)
        return not self._q.unfinished_tasks
