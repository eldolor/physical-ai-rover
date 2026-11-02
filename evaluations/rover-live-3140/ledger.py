# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/rover-live-3140/ledger.py

3.140.A: the ledger every run, verdict, go and halt is written to, and the
gates that read it (criterion A3). Plain functions over a list of events, so
tests drive them without a board.

Events, one JSON line each (append-only, fsynced, under an flock):

    open    a run starts: row, run, surface, motion, room, sha, checklist
    close   the run's after-check: surprise (rule 6), note
    score   a verdict: met | FAILED (| decided, L8 only), evidence, by
    go      the user's go to continue past a FAILED L3+ row (criterion 2)
    halt    a surprise ended the session (rule 6)
    resume  the user lifts a halt
"""

from __future__ import annotations

import fcntl
import json
import os
import time
from pathlib import Path
from typing import Iterable, Optional

SURFACES = ("none", "stand", "floor")
MOTIONS = ("verb", "wheels", "goal", "mission")
VERDICTS = ("met", "FAILED", "decided")
TRUTH = ("grid", "none")
# Rows of these levels that FAIL stop every later floor run until fixed
# (criterion 2); a FAILED row above them needs only the user's go.
HARD_LEVELS = (0, 1, 2)


class Refused(RuntimeError):
    pass


# ------------------------------------------------------------------ storage

def read(path: Path) -> list:
    p = Path(path)
    if not p.exists():
        return []
    out = []
    for n, line in enumerate(p.read_text().splitlines(), 1):
        if line.strip():
            try:
                out.append(json.loads(line))
            except ValueError as e:
                # A cut line is a broken record, never skipped quietly.
                raise Refused(f"{p}:{n}: not JSON ({e}); repair the ledger by hand") from None
    return out


def append(path: Path, event: dict) -> dict:
    ev = {"t": time.strftime("%Y-%m-%dT%H:%M:%S%z"), **event}
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "a") as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        f.write(json.dumps(ev) + "\n")
        f.flush()
        os.fsync(f.fileno())
    return ev


# ------------------------------------------------------------------- state

def latest_verdict(events: Iterable[dict], row: str) -> Optional[str]:
    v = None
    for e in events:
        if e["kind"] == "score" and e["row"] == row:
            v = e["verdict"]
    return v


def has_go(events: list, row: str) -> bool:
    """A go counts only if it came after the row's latest FAILED."""
    last_fail = go = -1
    for i, e in enumerate(events):
        if e.get("row") != row:
            continue
        if e["kind"] == "score" and e["verdict"] == "FAILED":
            last_fail = i
        elif e["kind"] == "go":
            go = i
    return last_fail >= 0 and go > last_fail


def halted(events: list) -> Optional[dict]:
    h = None
    for e in events:
        if e["kind"] == "halt":
            h = e
        elif e["kind"] == "resume":
            h = None
    return h


def open_runs(events: list) -> list:
    opened = {(e["row"], e["run"]): e for e in events if e["kind"] == "open"}
    for e in events:
        if e["kind"] == "close":
            opened.pop((e["row"], e["run"]), None)
    return list(opened.values())


def run_event(events: list, row: str, run: str, kind: str) -> Optional[dict]:
    for e in events:
        if e["kind"] == kind and e["row"] == row and e.get("run") == run:
            return e
    return None


# ------------------------------------------------------------------- gates

def check_checklist(surface: str, c: dict) -> list:
    """Rules 1-5, signed before the run (criterion 6); rule 6 is `close`."""
    out = []
    if not (c.get("signed_by") or "").strip():
        out.append("checklist unsigned (--signed-by: the person at the car)")
    if surface == "stand" and not c.get("on_stand"):
        out.append("rule 1: a stand run needs the chassis on the stand, wheels clear (--on-stand)")
    if surface == "floor":
        if not (c.get("stop_person") or "").strip():
            out.append("rule 2: a floor run needs a named person on STOP (--stop-person)")
        if not c.get("power_switch"):
            out.append("rule 2: a floor run needs the power switch in reach (--power-switch)")
    if surface != "none":
        if not c.get("safety_on"):
            out.append("rule 3: safety, the watchdog and the heartbeat on (--safety-on)")
        if c.get("truth") not in TRUTH:
            out.append("rule 4: say how distances are judged (--truth grid|none)")
        if not c.get("speed_default"):
            out.append("rule 5: speeds at today's defaults (--speed-default)")
    return out


def stand_done(events: list, path: str) -> bool:
    """Rule 1: an earlier stand run of this motion path, closed with no
    surprise and not scored FAILED."""
    for e in events:
        if e["kind"] != "open" or e["surface"] != "stand" or path not in e.get("motion", ()):
            continue
        c = run_event(events, e["row"], e["run"], "close")
        if c is None or c.get("surprise"):
            continue
        scores = [s for s in events if s["kind"] == "score" and s["row"] == e["row"]
                  and s.get("run") == e["run"]]
        if scores and scores[-1]["verdict"] == "FAILED":
            continue
        return True
    return False


def floor_gate(events: list, rows: dict, row) -> list:
    """Criterion 2, for a floor run of `row`."""
    out = []
    for r in rows.values():
        if r.decision or r.level not in HARD_LEVELS or r.id == row.id:
            continue
        if latest_verdict(events, r.id) == "FAILED":
            out.append(f"criterion 2: {r.id} FAILED stops every floor run but its own re-run until fixed")
    for r in rows.values():
        if r.decision or r.level >= row.level:
            continue
        v = latest_verdict(events, r.id)
        if v == "met":
            continue
        if v == "FAILED" and r.level not in HARD_LEVELS and has_go(events, r.id):
            continue
        if v == "FAILED" and r.level in HARD_LEVELS:
            continue      # already named above
        out.append(f"criterion 2: L{row.level} on the floor waits on {r.id} "
                   f"({v or 'not run'}{'' if v != 'FAILED' else ', no go from the user'})")
    return out


def refusals(events: list, rows: dict, row_id: str, surface: str, motion: list,
             checklist: dict, on_dev: bool, board: bool) -> list:
    """Every reason a run may not start; [] lets it start. Read before any
    board call (A3)."""
    row = rows.get(row_id)
    if row is None:
        return [f"{row_id}: no such row in the section file"]
    out = []
    if row.decision:
        out.append(f"{row_id} is an L8 decision: it is scored, never run")
    if surface not in SURFACES:
        out.append(f"surface {surface!r}: one of {SURFACES}")
    bad = [m for m in motion if m not in MOTIONS]
    if bad:
        out.append(f"motion {bad}: each one of {MOTIONS}")
    if surface == "none" and motion:
        out.append("motion named for a run off the car's wheels (surface none)")
    if surface == "floor" and not motion:
        out.append("a floor run names its motion paths (--motion)")
    if surface != "none" and not board:
        out.append("a stand or floor run goes through the board (--cmd)")
    h = halted(events)
    if h:
        out.append(f"rule 6: the session halted at {h['row']} run {h.get('run')} ({h.get('note')}); "
                   f"the user's `resume` lifts it")
    op = open_runs(events)
    if op:
        out.append("an open run must be closed first: "
                   + ", ".join(f"{e['row']} {e['run']}" for e in op))
    if row.reported and not row.amended_bar:
        out.append(f"criterion 3: {row_id} is *reported* and amendment 1 has no bar for it")
    if not on_dev:
        out.append("criterion 3: the commit under test is not on origin/dev")
    out += check_checklist(surface, checklist)
    if surface == "floor":
        for m in motion:
            if m in MOTIONS and not stand_done(events, m):
                out.append(f"rule 1: no stand run of motion path {m!r} yet")
        out += floor_gate(events, rows, row)
    return out


def score_refusals(events: list, rows: dict, row_id: str, run: Optional[str], verdict: str) -> list:
    row = rows.get(row_id)
    if row is None:
        return [f"{row_id}: no such row"]
    if verdict not in VERDICTS:
        return [f"verdict {verdict!r}: one of {VERDICTS}"]
    if (verdict == "decided") != row.decision:
        return [f"{row_id}: " + ("an L8 decision is `decided`" if row.decision else
                                 "`decided` is only for L8")]
    if row.decision:
        return []
    if run is None:
        return ["a run is scored by its run id"]
    if run_event(events, row_id, run, "open") is None:
        return [f"{row_id} has no run {run}"]
    if run_event(events, row_id, run, "close") is None:
        return [f"{row_id} run {run} is not closed (rule 6's after-check comes first)"]
    if verdict == "met" and row.reported and not row.amended_bar:
        return [f"criterion 3: {row_id} has no bar to be met"]
    return []


def go_refusals(events: list, rows: dict, row_id: str) -> list:
    row = rows.get(row_id)
    if row is None:
        return [f"{row_id}: no such row"]
    if row.level in HARD_LEVELS:
        return [f"criterion 2: a FAILED L{row.level} row is fixed, not passed by a go"]
    if latest_verdict(events, row_id) != "FAILED":
        return [f"{row_id} is not FAILED"]
    return []
