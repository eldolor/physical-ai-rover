"""Where a mission's object inventory goes when the mission ends
(PLAN-ros-alignment.md 3.46 D).

Always to a local file, and to S3 when a bucket is configured -- decided by
the user 2026-10-07. The bucket is the recordings bucket
(`cloudformation/recordings-s3.yaml`: private, encrypted, versioned), under
its own prefix:

    <local_dir>/<mission_id>.json
    s3://<bucket>/<prefix>/<robot>/<mission_id>.json

What is stored is `brain.inventory.Inventory.report()`: labels, map
positions, beliefs and votes -- **never an image**. It is still a list of
what is in someone's home, which is why it goes to the private bucket and
nowhere else.

**A failure here never fails a mission.** The mission is over when this
runs; `save()` records what went wrong in its return value and the log, and
the robot is already stopped either way.
"""
from __future__ import annotations

import contextlib
import json
import logging
import os
import re
import socket
import threading
import time
from pathlib import Path
from typing import Optional

from brain.inventory import carry_seq_of

logger = logging.getLogger(__name__)

_SAFE = re.compile(r"[^a-z0-9._-]+")


def _fsync(fd: int) -> None:
    """`os.fsync`, named here so a test can fail it for this module only."""
    os.fsync(fd)


def _fsync_dir(path: Path) -> None:
    """Make a rename in `path` durable. Best effort: not every platform can
    open a directory for fsync, and the file itself is already on disk."""
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        _fsync(fd)
    except OSError:
        pass
    finally:
        os.close(fd)


STALE_TMP_S = 3600
_IN_FLIGHT: set = set()          # paths of this process's saves being written now
_IN_FLIGHT_LOCK = threading.Lock()

# 3.50.D5 (review 1): local saves handed to a thread and not yet written, per
# directory -- module-wide, since the brain builds a store per mission. A
# lookup waits for them (up to PENDING_WAIT_S), so a mission started while
# the last one's report is still being written carries that report.
PENDING_WAIT_S = 5.0      # milliseconds normally; slack for a slow fsync on the board
_PENDING: dict = {}
_PENDING_COND = threading.Condition()


def _sweep_stale_tmp(directory: Path) -> None:
    """A save cut off by a crash or a power loss leaves `.<name>.tmp`
    behind (no handler runs), and mission ids are unique, so nothing ever
    overwrites it. Remove those over an hour old -- never one of this
    process's saves in progress (named, so a clock jump at boot cannot
    make a live one look old). Best effort: a sweep never fails a save."""
    with contextlib.suppress(OSError):
        cutoff = time.time() - STALE_TMP_S
        with _IN_FLIGHT_LOCK:
            busy = set(_IN_FLIGHT)
        for tmp in directory.glob(".*.json.tmp"):
            if str(tmp) in busy:
                continue
            with contextlib.suppress(OSError):
                if tmp.stat().st_mtime < cutoff:
                    tmp.unlink()


def _slug(text: str) -> str:
    return _SAFE.sub("-", str(text).lower()).strip("-")[:60] or "x"


class InventoryStore:
    def __init__(self, local_dir: str, bucket: str = "", prefix: str = "inventory",
                 robot: Optional[str] = None, client=None):
        self.local_dir = Path(local_dir)
        self.bucket = bucket or ""
        self.prefix = prefix.strip("/") or "inventory"
        self.robot = _slug(robot or socket.gethostname())
        self._client = client

    def _s3(self):
        if self._client is None:
            import boto3
            self._client = boto3.client("s3")
        return self._client

    def save_async(self, mission_id: str, report: dict) -> threading.Thread:
        """`save()` on a daemon thread (the mission's end never waits on an
        upload), counted as pending until its local file is written, so
        `latest_for()` can wait for it (3.50.D5)."""
        key = str(self.local_dir.resolve())
        with _PENDING_COND:
            _PENDING[key] = _PENDING.get(key, 0) + 1

        fired = []

        def done():
            # Once: when the local file is settled, or when the save ends
            # without getting that far (an exception) -- whichever is first.
            with _PENDING_COND:
                if fired:
                    return
                fired.append(True)
                _PENDING[key] -= 1
                if not _PENDING[key]:
                    del _PENDING[key]
                _PENDING_COND.notify_all()

        t = threading.Thread(target=self.save, args=(mission_id, report),
                             kwargs={"_local_done": done}, daemon=True)
        try:
            t.start()
        except BaseException:
            done()
            raise
        return t

    def save(self, mission_id: str, report: dict, _local_done=None) -> dict:
        try:
            return self._save(mission_id, report, _local_done)
        finally:
            if _local_done is not None:
                _local_done()             # no-op if the local step already said so

    def _save(self, mission_id: str, report: dict, _local_done=None) -> dict:
        name = f"{_slug(mission_id)}.json"
        body = json.dumps(report, indent=1, sort_keys=True)
        out = {"local": None, "s3": None, "errors": []}
        try:
            self.local_dir.mkdir(parents=True, exist_ok=True)
            _sweep_stale_tmp(self.local_dir)
            path = self.local_dir / name
            # 3.55: write then rename, so a brain that exits mid-save (the
            # save runs on a daemon thread) leaves no half-written file.
            tmp = path.with_name(f".{name}.tmp")
            with _IN_FLIGHT_LOCK:
                _IN_FLIGHT.add(str(tmp))
            try:
                with open(tmp, "w") as f:
                    f.write(body)
                    f.flush()
                    # Durable, not only atomic: the Jetson can lose power
                    # seconds after a mission, and a rename to a new name
                    # can then surface as an empty file.
                    _fsync(f.fileno())
                os.replace(tmp, path)
                _fsync_dir(self.local_dir)    # ...and the rename itself
            except BaseException:
                with contextlib.suppress(OSError):   # never hide the real error
                    tmp.unlink(missing_ok=True)
                raise
            finally:
                with _IN_FLIGHT_LOCK:
                    _IN_FLIGHT.discard(str(tmp))
            out["local"] = str(path)
        except OSError as exc:
            out["errors"].append(f"local: {exc}")
        if _local_done is not None:
            _local_done()                 # the local file is settled: lookups may go on
            _local_done = None
        if self.bucket:
            key = f"{self.prefix}/{self.robot}/{name}"
            try:
                self._s3().put_object(Bucket=self.bucket, Key=key, Body=body.encode(),
                                      ContentType="application/json",
                                      ServerSideEncryption="AES256")
                out["s3"] = f"s3://{self.bucket}/{key}"
            except Exception as exc:  # noqa: BLE001 -- never fail a mission
                out["errors"].append(f"s3: {exc}")
        for err in out["errors"]:
            logger.warning("inventory %s not saved: %s", mission_id, err)
        return out

    def latest_for(self, frame_id: Optional[str]) -> Optional[dict]:
        """3.50.D5: the head of the chain on this saved map (its `frame_id`):
        the report with the highest `carry_seq` -- each mission's is the one
        it carried plus one, so lineage decides, not a clock (review 1) --
        and between equals the newest file. None if there is none. What a
        new mission carries over; each report holds what its mission carried
        plus what it saw, so only the head is needed.

        Waits up to `PENDING_WAIT_S` for local saves still in flight here
        (`save_async`); past that it raises `TimeoutError`. Local only: S3
        is never read back. A file that does not name the frame is not
        parsed; files that cannot be read, decoded or parsed (named in a
        warning), other frames and hidden `.tmp` saves are skipped. A
        directory that cannot be listed raises: a failed lookup, not
        "nothing saved"."""
        if not frame_id:
            return None
        key = str(self.local_dir.resolve())
        with _PENDING_COND:
            if not _PENDING_COND.wait_for(lambda: not _PENDING.get(key), PENDING_WAIT_S):
                # Read around it and this mission would tie the save still
                # being written (review 2): a failed lookup, so it detaches.
                raise TimeoutError(f"an inventory save still in flight after "
                                   f"{PENDING_WAIT_S:.0f} s")
        try:
            listed = [p for p in self.local_dir.glob("*.json") if not p.name.startswith(".")]
        except FileNotFoundError:
            return None
        best, best_key = None, None
        needle = json.dumps(frame_id)
        for path in listed:
            try:
                mtime = path.stat().st_mtime
                text = path.read_text()
            except (FileNotFoundError, IsADirectoryError):
                continue                  # removed since it was listed, or not a file
            except (OSError, UnicodeDecodeError) as e:
                # Skipped, with its name (review 4): if it was the head, the
                # chain forks and loses that one mission's own sightings;
                # failing instead would stop all carrying until someone
                # removed the file. The lesser harm.
                logger.warning("inventory: %s skipped (unreadable): %s", path.name, e)
                continue
            if needle not in text:
                continue
            try:
                report = json.loads(text)
            except ValueError as e:
                logger.warning("inventory: %s skipped (not JSON): %s", path.name, e)
                continue
            if not isinstance(report, dict) or report.get("frame_id") != frame_id:
                continue
            rank = (carry_seq_of(report), mtime, path.name)
            if best_key is None or rank > best_key:
                best, best_key = report, rank
        return best


def inventory_store_from_config(config: dict) -> InventoryStore:
    return InventoryStore(
        local_dir=config.get("inventory_dir", "recordings/inventory"),
        bucket=config.get("inventory_bucket", ""),
        prefix=config.get("inventory_prefix", "inventory"),
    )
