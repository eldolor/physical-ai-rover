# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
control/scheduler.py -- 3.163.B: missions and surveys on a schedule
(`docs/plans/ros-alignment/3.163-scheduled-missions.md`, D1-D5 and B's
amendments).

* `validate()`: a person's schedules (D1): at most `MAX_SCHEDULES`, each
  `{id, job: find|survey, target (a find's), days (0 = Monday), at (HH:MM,
  local time), enabled}`, plus one master switch.
* `ScheduleStore`: `<schedule_dir>/schedules.json` (written whole: a temporary
  file, fsynced, renamed in) and `<schedule_dir>/runs.jsonl` (one line per due
  time's end, flushed and fsynced). A home's routine: local only, never
  uploaded or committed.
* `Scheduler`: one `tick()` every `TICK_S`. A due time (date, id) is tried at
  every tick until it starts or `GRACE_S` runs out (D2). A `firing` line is
  written before the start is asked, so a due time never fires twice, even
  across a restart or a clock stepped back. A due time no tick saw inside its
  grace (the brain was down) ends `missed_down`.

The checks and the start belong to the brain server (`check`, `start`): this
module only decides when to ask. It never lifts a person's Stop, and a start
is the brain's ordinary one (D4). It vetoes nothing: the robot server decides
as it does for a person's start.
"""

import asyncio
import json
import logging
import os
import re
from datetime import datetime, time as dtime, timedelta
from pathlib import Path
from typing import Awaitable, Callable, Optional

logger = logging.getLogger(__name__)

TICK_S = 10.0
GRACE_S = 600.0
LOOKBACK_S = 86400.0        # how far back a restart looks for due times it missed
MAX_SCHEDULES = 10
JOBS = ("find", "survey")
ID_RE = re.compile(r"^[a-z0-9][a-z0-9-]{0,31}$")
AT_RE = re.compile(r"^([01]\d|2[0-3]):([0-5]\d)$")
TARGET_MAX = 80
RUNS_SHOWN = 10

# What a due time ends as.
FIRING, FIRED, SKIPPED, MISSED_DOWN = "firing", "fired", "skipped", "missed_down"
FINAL = (FIRING, FIRED, SKIPPED, MISSED_DOWN)

# Why a due time is skipped (D3, in its order; B's amendment 9 adds the last two).
DISABLED = "disabled"
NOT_ALLOWED_HERE = "not_allowed_here"
MISSION_RUNNING = "mission_running"
BUSY = "busy"                       # mission_running still, when the grace ran out
PERSON_STOP_LATCHED = "person_stop_latched"
PERSON_HOLDING = "person_holding"
NOT_DOCKED = "not_docked"
BATTERY_NOT_NORMAL = "battery_not_normal"
ROS_NOT_READY = "ros_not_ready"
ROBOT_UNREACHABLE = "robot_unreachable"
START_REFUSED = "start_refused"

CLOCK_FILE_ENV = "SCHEDULE_CLOCK_FILE"


class SkipStart(Exception):
    """A start the brain refused, with the reason it ends under."""

    def __init__(self, reason: str, detail: str = ""):
        super().__init__(detail or reason)
        self.reason = reason
        self.detail = detail


def default_clock() -> datetime:
    """The local wall time. Only with `SCHEDULE_CLOCK_FILE` set (an
    accelerated week's harness, never a unit) is it moved on by the seconds
    that file holds, read at every call."""
    now = datetime.now()
    path = os.environ.get(CLOCK_FILE_ENV)
    if path:
        try:
            now += timedelta(seconds=float(Path(path).read_text().strip() or 0.0))
        except (OSError, ValueError) as e:
            logger.warning("schedule clock file unreadable: %s", e)
    return now


def validate(body) -> dict:
    """The schedules as a person set them, or ValueError naming the fault."""
    if not isinstance(body, dict):
        raise ValueError("schedules: an object with `enabled` and `schedules`")
    unknown = set(body) - {"enabled", "schedules"}
    if unknown:
        raise ValueError(f"schedules: unknown field(s) {sorted(unknown)}")
    rows = body.get("schedules", [])
    if not isinstance(rows, list) or len(rows) > MAX_SCHEDULES:
        raise ValueError(f"schedules: a list of at most {MAX_SCHEDULES}")
    out, ids = [], set()
    for i, s in enumerate(rows):
        if not isinstance(s, dict):
            raise ValueError(f"schedule {i}: an object")
        extra = set(s) - {"id", "job", "target", "days", "at", "enabled"}
        if extra:
            raise ValueError(f"schedule {i}: unknown field(s) {sorted(extra)}")
        sid = s.get("id")
        if not isinstance(sid, str) or not ID_RE.match(sid) or sid in ids:
            raise ValueError(f"schedule {i}: `id` must be unique, a-z, 0-9 and '-'")
        ids.add(sid)
        job = s.get("job")
        if job not in JOBS:
            raise ValueError(f"schedule {sid}: `job` is find or survey")
        target = s.get("target")
        if job == "find":
            if not isinstance(target, str) or not target.strip() or len(target) > TARGET_MAX:
                raise ValueError(f"schedule {sid}: a find needs a target (1-{TARGET_MAX} characters)")
            target = target.strip()
        elif target not in (None, ""):
            raise ValueError(f"schedule {sid}: a survey has no target")
        else:
            target = None
        days = s.get("days")
        if (not isinstance(days, list) or not days
                or any(not isinstance(d, int) or isinstance(d, bool) or not 0 <= d <= 6 for d in days)):
            raise ValueError(f"schedule {sid}: `days` is a non-empty list of 0 (Monday) to 6")
        at = s.get("at")
        if not isinstance(at, str) or not AT_RE.match(at):
            raise ValueError(f"schedule {sid}: `at` is HH:MM")
        enabled = s.get("enabled", True)
        if not isinstance(enabled, bool):
            raise ValueError(f"schedule {sid}: `enabled` is true or false")
        out.append({"id": sid, "job": job, "target": target, "days": sorted(set(days)),
                    "at": at, "enabled": enabled})
    enabled = body.get("enabled", False)
    if not isinstance(enabled, bool):
        raise ValueError("schedules: `enabled` is true or false")
    return {"enabled": enabled, "schedules": out}


def _same(a: dict, b: dict) -> bool:
    return all(a.get(k) == b.get(k) for k in ("job", "target", "days", "at"))


class ScheduleStore:
    """The two files under `directory`."""

    def __init__(self, directory):
        self.dir = Path(directory)
        self.path = self.dir / "schedules.json"
        self.runs_path = self.dir / "runs.jsonl"

    def load(self) -> dict:
        try:
            body = json.loads(self.path.read_text())
        except FileNotFoundError:
            return {"enabled": False, "schedules": []}
        except (OSError, ValueError) as e:
            # Never a start from a file that cannot be read: off, and said.
            logger.error("schedules unreadable (%s): read as off", e)
            return {"enabled": False, "schedules": [], "unreadable": str(e)}
        return body

    def save(self, body: dict, now: datetime) -> dict:
        """`body` (already validated) written whole. A schedule whose time or
        job changed counts from now, so no due time before it is ever owed."""
        old = {s["id"]: s for s in self.load().get("schedules", [])}
        rows = []
        for s in body["schedules"]:
            was = old.get(s["id"])
            since = was["since"] if was is not None and _same(was, s) and "since" in was else now.isoformat()
            rows.append({**s, "since": since})
        out = {"enabled": body["enabled"], "schedules": rows}
        self.dir.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_name(self.path.name + ".tmp")
        with open(tmp, "w") as f:
            f.write(json.dumps(out, indent=1) + "\n")
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.path)
        fd = os.open(self.dir, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
        return out

    def append(self, line: dict) -> None:
        self.dir.mkdir(parents=True, exist_ok=True)
        with open(self.runs_path, "a") as f:
            f.write(json.dumps(line) + "\n")
            f.flush()
            os.fsync(f.fileno())

    def lines(self) -> list:
        try:
            text = self.runs_path.read_text()
        except FileNotFoundError:
            return []
        out = []
        for raw in text.splitlines():
            try:
                out.append(json.loads(raw))
            except ValueError:
                continue        # a torn last line: the due time is tried again
        return out

    def finals(self) -> dict:
        return {(x["due"], x["id"]): x for x in self.lines() if x.get("outcome") in FINAL}


def due_times(s: dict, since: datetime, until: datetime):
    """`s`'s due times in (since, until], oldest first."""
    hh, mm = (int(v) for v in s["at"].split(":"))
    day = since.date()
    while day <= until.date():
        if day.weekday() in s["days"]:
            due = datetime.combine(day, dtime(hh, mm))
            if since < due <= until:
                yield due
        day += timedelta(days=1)


class Scheduler:
    """When to ask for a start. `check(state, schedule)` answers a skip
    reason or None; `start(schedule)` answers the mission's id or raises
    `SkipStart`. Both are the brain server's, and awaited."""

    def __init__(self, store: ScheduleStore,
                 check: Callable[[dict, dict], Awaitable[Optional[str]]],
                 start: Callable[[dict], Awaitable[Optional[str]]],
                 clock: Callable[[], datetime] = default_clock,
                 grace_s: float = GRACE_S, lookback_s: float = LOOKBACK_S,
                 before: Optional[Callable[[], Awaitable[None]]] = None):
        self.store, self.check, self.start, self.clock = store, check, start, clock
        self.before = before        # awaited first at every tick (the brain notes the dock)
        self.grace_s, self.lookback_s = grace_s, lookback_s
        self.pending: dict = {}     # (due, id) -> the reason it is waiting on
        self._lock = asyncio.Lock()

    async def tick(self) -> list:
        """One pass. Returns the lines it wrote."""
        async with self._lock:
            if self.before is not None:
                try:
                    await self.before()
                except Exception:  # noqa: BLE001 -- a note that failed costs nothing here
                    logger.exception("schedule tick's first step failed")
            now = self.clock()
            state = self.store.load()
            finals = self.store.finals()
            wrote = []
            for s in state.get("schedules", []):
                try:
                    since = datetime.fromisoformat(s.get("since") or now.isoformat())
                except ValueError:
                    since = now
                since = max(since, now - timedelta(seconds=self.lookback_s))
                for due in due_times(s, since, now):
                    key = (due.isoformat(), s["id"])
                    if key in finals:
                        continue
                    line = await self._try(state, s, key, now - due)
                    if line is not None:
                        finals[key] = line
                        wrote.append(line)
            return wrote

    async def _try(self, state: dict, s: dict, key: tuple, age: timedelta) -> Optional[dict]:
        base = {"due": key[0], "id": s["id"], "job": s["job"], "target": s["target"]}
        if age.total_seconds() > self.grace_s:
            waited = self.pending.pop(key, None)
            if waited is None:
                return self._end({**base, "outcome": MISSED_DOWN})
            return self._end({**base, "outcome": SKIPPED,
                              "reason": BUSY if waited == MISSION_RUNNING else waited})
        reason = await self.check(state, s)
        if reason is not None:
            self.pending[key] = reason
            return None
        self.pending.pop(key, None)
        self._end({**base, "outcome": FIRING})
        try:
            mission_id = await self.start(s)
        except SkipStart as e:
            return self._end({**base, "outcome": SKIPPED, "reason": e.reason, "detail": e.detail[:300]})
        except Exception as e:  # noqa: BLE001 -- a start that raised is a skip on the record
            logger.exception("scheduled start %s failed", s["id"])
            return self._end({**base, "outcome": SKIPPED, "reason": START_REFUSED, "detail": str(e)[:300]})
        return self._end({**base, "outcome": FIRED, "mission_id": mission_id})

    def _end(self, line: dict) -> dict:
        line = {**line, "at": self.clock().isoformat(timespec="seconds")}
        self.store.append(line)
        logger.info("schedule %s due %s: %s %s", line["id"], line["due"], line["outcome"],
                    line.get("reason") or line.get("mission_id") or "")
        return line

    def view(self) -> dict:
        """What the twin shows: the schedules, the last due times' ends, the
        next run and what each due time now waits on."""
        now = self.clock()
        state = self.store.load()
        lines = [x for x in self.store.lines() if x.get("outcome") in (FIRED, SKIPPED, MISSED_DOWN)]
        return {"enabled": bool(state.get("enabled")),
                "schedules": [{k: v for k, v in s.items() if k != "since"}
                              for s in state.get("schedules", [])],
                "runs": lines[-RUNS_SHOWN:][::-1],
                "next": next_run(state, now),
                "waiting": [{"due": d, "id": i, "reason": r} for (d, i), r in sorted(self.pending.items())],
                "unreadable": state.get("unreadable")}


def next_run(state: dict, now: datetime) -> Optional[dict]:
    """The next due time of an enabled schedule, the master switch on."""
    if not state.get("enabled"):
        return None
    best = None
    for s in state.get("schedules", []):
        if not s.get("enabled"):
            continue
        for due in due_times(s, now, now + timedelta(days=8)):
            if best is None or due < best[0]:
                best = (due, s)
            break
    if best is None:
        return None
    due, s = best
    return {"id": s["id"], "job": s["job"], "target": s["target"], "at": due.isoformat(),
            "in_s": round((due - now).total_seconds())}
