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


def _found_bytes(jpeg: bytes) -> tuple:
    """(bytes to store, re-encoded?) for the judged frame."""
    if len(jpeg) <= FOUND_MAX_BYTES:
        return jpeg, False
    from PIL import Image
    return _jpeg(Image.open(io.BytesIO(jpeg)).convert("RGB"), FOUND_MAX_BYTES), True


def _size(d: Path) -> int:
    return sum(p.stat().st_size for p in d.rglob("*") if p.is_file())


class ReportStore:
    def __init__(self, directory=None):
        self.dir = Path(directory) if directory else default_dir()
        self._lock = threading.Lock()

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
        rep = {k: v for k, v in (report or {}).items() if k not in ("found", "landmarks")}
        rep["mission_id"] = mission_id
        rep["saved_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        self.dir.mkdir(parents=True, exist_ok=True)
        tmp = Path(tempfile.mkdtemp(prefix=f".{mission_id}.", dir=str(self.dir)))
        try:
            budget = MISSION_MAX_BYTES - 20_000          # room for report.json
            if found and found.get("jpeg"):
                raw = found["jpeg"]
                data, resized = _found_bytes(raw)
                (tmp / "found.jpg").write_bytes(data)
                budget -= len(data)
                rep["found"] = {"file": "found.jpg", "sha256": hashlib.sha256(raw).hexdigest(),
                                "box": found.get("box"), "confirmed": bool(found.get("confirmed")),
                                "resized": resized}
            else:
                rep["found"] = None
            rows = sorted(looks or [], key=lambda r: -(r.get("belief") or 0.0))
            marks = []
            (tmp / "thumbs").mkdir()
            for i, r in enumerate(rows):
                row = {k: r.get(k) for k in ("label", "x_m", "y_m", "room", "belief", "looks")}
                row["thumb"] = None
                if r.get("jpeg") and len([m for m in marks if m["thumb"]]) < MAX_THUMBS:
                    try:
                        t = thumbnail(r["jpeg"], r.get("box"))
                    except Exception as e:  # noqa: BLE001 -- a bad frame loses its thumbnail only
                        logger.warning("report %s: no thumbnail for %s (%s)", mission_id, row["label"], e)
                        t = None
                    if t is not None and len(t) <= budget:
                        name = f"thumbs/{len([m for m in marks if m['thumb']])}.jpg"
                        (tmp / name).write_bytes(t)
                        budget -= len(t)
                        row["thumb"] = name
                marks.append(row)
            rep["landmarks"] = marks
            with open(tmp / "report.json", "w") as fh:
                json.dump(rep, fh, indent=1)
                fh.flush()
                os.fsync(fh.fileno())
            with self._lock:
                if final.exists():
                    self._remove(final)
                os.replace(tmp, final)
                self._prune()
        except BaseException:
            shutil.rmtree(tmp, ignore_errors=True)
            raise
        return rep

    def _remove(self, d: Path) -> None:
        """Whole or not at all: renamed aside first, so no reader sees half."""
        gone = d.with_name(f".{d.name}.deleting.{os.getpid()}.{time.monotonic_ns()}")
        os.replace(d, gone)
        shutil.rmtree(gone, ignore_errors=True)

    def _prune(self) -> None:
        missions = sorted((p for p in self.dir.iterdir() if p.is_dir() and not p.name.startswith(".")),
                          key=lambda p: (p / "report.json").stat().st_mtime
                          if (p / "report.json").exists() else 0.0)
        sizes = {p: _size(p) for p in missions}
        total = sum(sizes.values())
        while missions and (len(missions) > KEEP or total > STORE_MAX_BYTES):
            old = missions.pop(0)
            total -= sizes[old]
            self._remove(old)

    def delete(self, mission_id: str) -> bool:
        d = self._mission_dir(mission_id)
        with self._lock:
            if not d.is_dir():
                return False
            self._remove(d)
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
                        "outcome": rep.get("outcome"), "ended": rep.get("ended"),
                        "found": bool(rep.get("found")),
                        "things": len(rep.get("landmarks") or []),
                        "saved_at": rep.get("saved_at")})
        return sorted(out, key=lambda r: r.get("saved_at") or "", reverse=True)

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
        self.store = store or ReportStore()
        self._q: "queue.Queue" = queue.Queue(maxsize=8)
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
