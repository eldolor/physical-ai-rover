# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/rover-live-3140/run.py

3.140's row runner on the desktop (3.140.A). One row, one run at a time:

    python evaluations/rover-live-3140/run.py check          # A1, A2, A6: rows, coverage, bars
    python evaluations/rover-live-3140/run.py status         # every row's latest verdict, open runs, halts
    python evaluations/rover-live-3140/run.py run L2.3 --surface floor --motion verb \\
        --room kitchen --signed-by NAME --stop-person NAME --power-switch --safety-on \\
        --truth grid --speed-default --cmd '<board command>' [--timed] [--sha REV]
    python evaluations/rover-live-3140/run.py run L0.1 --surface none --room hall \\
        --signed-by NAME --value holes_mm=86x58 --note '...'             # a manual row: no board, no lock
    python evaluations/rover-live-3140/run.py close L2.3 <run> --surprise no [--note ...]
    python evaluations/rover-live-3140/run.py score L2.3 <run> --verdict met|FAILED --evidence '...' --by NAME
    python evaluations/rover-live-3140/run.py go L5.1 --by NAME --note '...'    # the user's go past a FAILED L3+ row
    python evaluations/rover-live-3140/run.py resume --by NAME --note '...'     # the user lifts a rule-6 halt

A board run reuses 3.136.A's runner, not a copy: `boardrun.locked()` (the
desktop lock, exclusive with `--timed`, then `board_lock.py run --why`, no
wait) and `run.board_run()` with `bench_dir="bench-3140"` (the commit pushed
to the board, telemetry, the command, fetch, the board put back as found).
The gates (`ledger.refusals`) run before the locks and again under them, and
a stand or floor run reads the robot server's `/health` first: `mode` must
be `hardware` (A3 g). The runner never drives the motors: `--cmd` is the
brain's HTTP API or a measuring tool, written by the step that owns the row.

Records (A4): `<row>/<run>/record.json` here, beside the fetched board
directory; images, video and maps stay out of git (`.gitignore`).
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PERF = ROOT / "evaluations" / "jetson-perf-3136"
sys.path.insert(0, str(HERE))
sys.path.append(str(PERF))   # after HERE: this file is `run`, 3.136's is loaded by path

import ledger as lg  # noqa: E402
import rows as rw  # noqa: E402

PY = sys.executable

def _common_dir() -> Path:
    """The repository's shared `.git`: one ledger for every worktree on this
    machine, so a halt or a FAILED recorded in one session's worktree gates
    the next session's runs (`plan-claims/` lives there for the same
    reason)."""
    out = subprocess.run(["git", "rev-parse", "--git-common-dir"], cwd=HERE,
                         capture_output=True, text=True, check=True).stdout.strip()
    return (HERE / out).resolve()


# The gates read LEDGER; every append is copied to MIRROR, the committed copy
# beside the records (criterion 4). A machine with no LEDGER yet starts from
# the mirror it checked out.
LEDGER = _common_dir() / "rover-live-3140" / "ledger.jsonl"
MIRROR = HERE / "ledger.jsonl"
BENCH_DIR = "bench-3140"
ROBOT_PORT = 8000            # picar-stack's robot server on the board


# ---------------------------------------------------------------- the commit

def resolve(sha: str) -> str:
    return subprocess.run(["git", "rev-parse", "--verify", f"{sha}^{{commit}}"], cwd=ROOT,
                          capture_output=True, text=True, check=True).stdout.strip()


def on_dev(sha: str) -> bool:
    # origin/dev as last fetched; `run` fetches it first.
    return subprocess.run(["git", "merge-base", "--is-ancestor", sha, "origin/dev"],
                          cwd=ROOT, capture_output=True).returncode == 0


def fetch_dev() -> None:
    subprocess.run(["git", "fetch", "-q", "origin", "dev"], cwd=ROOT, capture_output=True, timeout=120)


# ------------------------------------------------------------- the board run

def perf_board_run():
    """3.136.A's `run.board_run`, loaded by path under its own name."""
    spec = importlib.util.spec_from_file_location("perf_run_3136", PERF / "run.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.board_run


def health_cmd(port: int, name: str) -> str:
    # /health is unauthenticated on purpose (CLAUDE.md section 6).
    return f"curl -s -m 5 localhost:{port}/health > $OUT/{name} 2>/dev/null"


def wrap(cmd: str, port: int = ROBOT_PORT) -> str:
    """The row's command between two `/health` reads; the run's exit code is
    the command's own. A subshell, so an `exit` in the command ends only it."""
    return (f"{health_cmd(port, 'health_before.json')}; "
            f"( {cmd}\n) > $OUT/row.log 2>&1; rc=$?; echo $rc > $OUT/row_exit; "
            f"{health_cmd(port, 'health_after.json')}; (exit $rc)")


def board_health(run, port: int = ROBOT_PORT) -> tuple:
    """(health dict or None, raw) from the robot server's /health on the board."""
    raw = run(f"curl -s -m 5 localhost:{port}/health", check=False)
    try:
        h = json.loads(raw)
        return (h if isinstance(h, dict) else None), raw
    except ValueError:
        return None, raw


def health_refusals(h, sha: str) -> list:
    """A3 g: the car's robot server, in hardware mode, running the commit
    under test (criterion 3: the code that moves the car is the scored one)."""
    if h is None:
        return ["A3 g: no /health from the board's robot server"]
    out = []
    if h.get("mode") != "hardware":
        out.append(f"A3 g: the board's /health mode is {h.get('mode')!r}, not 'hardware'")
    rev = str((h.get("identity") or {}).get("git_revision") or "")
    if len(rev) < 7 or not sha.startswith(rev):
        out.append(f"A3 g: the robot server runs {rev or 'an unknown revision'!r}, "
                   f"not the commit under test {sha[:12]}")
    return out


def _json(p: Path):
    try:
        return json.loads(p.read_text())
    except (OSError, ValueError):
        return None


def execute(spec: dict, *, ledger_path: Path = LEDGER, rows: dict = None, run=None,
            board_run=None, on_dev_fn=on_dev, records: Path = HERE, mirror: Path = MIRROR) -> int:
    """A run, inside the locks for a board run. Re-reads the gates (the state
    may have changed while it waited) and checks /health under the ledger's
    lock, writes the `open` event, runs, writes the record. Returns the
    command's exit, or 3 when refused."""
    rows = rows if rows is not None else rw.load_plan()
    board = bool(spec.get("cmd"))
    if board:
        import boardrun as br  # noqa: E402  (3.136.A's)
        run = run or br.ssh
        if board_run is None:
            board_run = perf_board_run()
        br.require_board_lock()
    out = records / spec["row"] / spec["run"]
    rec = {"row": spec["row"], "run": spec["run"], "sha": spec["sha"], "room": spec["room"],
           "surface": spec["surface"], "motion": spec["motion"], "checklist": spec["checklist"],
           "cmd": spec.get("cmd"), "timed": spec.get("timed", False), "note": spec.get("note"),
           "values": spec.get("values", {}), "bar": rows[spec["row"]].bar,
           "amended_bar": rows[spec["row"]].amended_bar}
    # The gates' read and the `open` append under one lock: two terminals
    # never both pass "no open run".
    with lg.locked(ledger_path, mirror):
        events = lg.read(ledger_path)
        why = lg.refusals(events, rows, spec["row"], spec["surface"], spec["motion"],
                          spec["checklist"], on_dev_fn(spec["sha"]), board)
        if not why and board:
            # Every board command, `none` included: a command can move the car
            # whatever the declared surface, so the car must be the scored one.
            h, raw = board_health(run, spec.get("port", ROBOT_PORT))
            why = health_refusals(h, spec["sha"])
            rec["health_gate"] = h
        if why:
            print("refused:\n  " + "\n  ".join(why))
            return 3
        lg.append(ledger_path, {"kind": "open", "row": spec["row"], "run": spec["run"],
                                "surface": spec["surface"], "motion": spec["motion"],
                                "room": spec["room"], "sha": spec["sha"],
                                "checklist": spec["checklist"], "board": board},
                  mirror=mirror)
    out.mkdir(parents=True, exist_ok=True)
    rc = 0
    try:
        if board:
            brec = board_run(f"3140-{spec['row']}", wrap(spec["cmd"], spec.get("port", ROBOT_PORT)),
                             out, sha=spec["sha"], allow_sudo=False, stop_twin=False, run=run,
                             bench_dir=BENCH_DIR)
            rec.update(brec)
            rec["board_sha"] = brec.get("sha")   # the board's checkout of it
            rec["sha"] = spec["sha"]
            b = out / "board"
            rec["health_before"] = _json(b / "health_before.json")
            rec["health_after"] = _json(b / "health_after.json")
            ex = (b / "row_exit")
            rec["row_exit"] = int(ex.read_text().strip()) if ex.exists() and ex.read_text().strip() else None
            rc = rec["row_exit"] if rec["row_exit"] is not None else 1
        for src in spec.get("logs", []):
            dst = out / "logs" / Path(src).name
            dst.parent.mkdir(exist_ok=True)
            dst.write_bytes(Path(src).read_bytes())
            rec.setdefault("logs", []).append(str(dst.relative_to(out)))
    except BaseException as e:
        # The run stays open for rule 6's after-check (the car may have
        # moved before the error); the error is on the ledger, so the run
        # can never be scored met or count as a stand run.
        rec["error"] = f"{type(e).__name__}: {e}"
        lg.append(ledger_path, {"kind": "error", "row": spec["row"], "run": spec["run"],
                                "error": rec["error"]}, mirror=mirror)
        raise
    finally:
        # boardrun writes its own record.json first (as found, restore,
        # left_as_found); on an error before it returned, keep all of it.
        base = _json(out / "record.json") if not rec.get("as_found") else None
        if isinstance(base, dict):
            rec = {**base, "board_sha": base.get("sha"), **rec}
        (out / "record.json").write_text(json.dumps(rec, indent=1))
    print(f"{spec['row']} run {spec['run']}: record {out / 'record.json'}; exit {rc}")
    print(f"next: run.py close {spec['row']} {spec['run']} --surprise no|yes  (rule 6)")
    return rc


# ------------------------------------------------------------------ commands

def _checklist(a) -> dict:
    return {"signed_by": a.signed_by, "on_stand": a.on_stand, "stop_person": a.stop_person,
            "power_switch": a.power_switch, "safety_on": a.safety_on, "truth": a.truth,
            "speed_default": a.speed_default}


def cmd_run(a, *, ledger_path=LEDGER, rows=None, locked=None) -> int:
    rows = rows if rows is not None else rw.load_plan()
    fetch_dev()
    sha = resolve(a.sha)
    motion = [m for m in (a.motion or "").split(",") if m]
    logs = [Path(p).resolve() for p in a.log]
    missing = [str(p) for p in logs if not p.is_file()]
    names = [p.name for p in logs]
    if missing or len(set(names)) != len(names):
        print(f"refused: --log: missing {missing}" if missing else
              f"refused: --log: two logs share a name {names}")
        return 3
    bad = [v for v in a.value if "=" not in v or not v.split("=", 1)[0].strip()]
    if bad:
        print(f"refused: --value is k=v: {bad}")
        return 3
    values = dict(v.split("=", 1) for v in a.value)
    spec = {"row": a.row, "run": time.strftime("%Y%m%d-%H%M%S"), "sha": sha, "room": a.room,
            "surface": a.surface, "motion": motion, "checklist": _checklist(a), "cmd": a.cmd,
            "timed": a.timed, "note": a.note, "values": values,
            "logs": [str(p) for p in logs], "port": a.robot_port}
    why = lg.refusals(lg.read(ledger_path, MIRROR), rows, a.row, a.surface, motion, spec["checklist"],
                      on_dev(sha), bool(a.cmd))
    if why:
        print("refused:\n  " + "\n  ".join(why))
        return 3
    if not a.cmd:
        return execute(spec, ledger_path=ledger_path, rows=rows)
    if locked is None:
        import boardrun as br  # noqa: E402
        locked = br.locked
    with tempfile.NamedTemporaryFile("w", suffix=".json", prefix="3140-", delete=False) as f:
        json.dump(spec, f)
    try:
        return locked([PY, str(Path(__file__).resolve()), "_run", f.name],
                      f"3.140 {a.row} ({a.surface})",
                      desktop_mode="exclusive" if a.timed else "shared", desktop_wait=a.wait)
    finally:
        os.unlink(f.name)


def cmd_close(a, *, ledger_path=LEDGER, mirror=MIRROR) -> int:
    with lg.locked(ledger_path, mirror):
        return _close(a, ledger_path, mirror)


def _close(a, ledger_path, mirror) -> int:
    events = lg.read(ledger_path)
    if lg.run_event(events, a.row, a.run, "open") is None:
        print(f"refused: {a.row} has no run {a.run}")
        return 3
    if lg.run_event(events, a.row, a.run, "close") is not None:
        print(f"refused: {a.row} run {a.run} is already closed")
        return 3
    surprise = a.surprise == "yes"
    if surprise and not (a.note or "").strip():
        print("refused: rule 6: say what happened (--note)")
        return 3
    lg.append(ledger_path, {"kind": "close", "row": a.row, "run": a.run, "surprise": surprise,
                            "note": a.note}, mirror=mirror)
    if surprise:
        # Rule 6: the row is FAILED and the session ends, until the user's resume.
        lg.append(ledger_path, {"kind": "score", "row": a.row, "run": a.run, "verdict": "FAILED",
                                "evidence": f"rule 6: {a.note}", "by": "rule 6"}, mirror=mirror)
        lg.append(ledger_path, {"kind": "halt", "row": a.row, "run": a.run, "note": a.note}, mirror=mirror)
        print(f"{a.row} FAILED (rule 6); the session is halted until `resume` by the user")
    return 0


def cmd_score(a, *, ledger_path=LEDGER, rows=None, mirror=MIRROR) -> int:
    rows = rows if rows is not None else rw.load_plan()
    with lg.locked(ledger_path, mirror):
        return _score(a, ledger_path, rows, mirror)


def _score(a, ledger_path, rows, mirror) -> int:
    events = lg.read(ledger_path)
    why = lg.score_refusals(events, rows, a.row, a.run, a.verdict)
    if not (a.evidence or "").strip():
        why.append("a verdict cites its evidence (--evidence)")
    if not (a.by or "").strip():
        why.append("a verdict names who scored it (--by)")
    if why:
        print("refused:\n  " + "\n  ".join(why))
        return 3
    lg.append(ledger_path, {"kind": "score", "row": a.row, "run": a.run, "verdict": a.verdict,
                            "evidence": a.evidence, "by": a.by}, mirror=mirror)
    return 0


def cmd_go(a, *, ledger_path=LEDGER, rows=None, mirror=MIRROR) -> int:
    rows = rows if rows is not None else rw.load_plan()
    with lg.locked(ledger_path, mirror):
        return _go(a, ledger_path, rows, mirror)


def _go(a, ledger_path, rows, mirror) -> int:
    why = lg.go_refusals(lg.read(ledger_path), rows, a.row)
    if not (a.by or "").strip() or not (a.note or "").strip():
        why.append("the user's go names who gave it and their words (--by, --note)")
    if why:
        print("refused:\n  " + "\n  ".join(why))
        return 3
    lg.append(ledger_path, {"kind": "go", "row": a.row, "by": a.by, "note": a.note}, mirror=mirror)
    return 0


def cmd_resume(a, *, ledger_path=LEDGER, mirror=MIRROR) -> int:
    with lg.locked(ledger_path, mirror):
        return _resume(a, ledger_path, mirror)


def _resume(a, ledger_path, mirror) -> int:
    if lg.halted(lg.read(ledger_path)) is None:
        print("refused: the session is not halted")
        return 3
    if not (a.by or "").strip() or not (a.note or "").strip():
        print("refused: a resume names the user and their words (--by, --note)")
        return 3
    lg.append(ledger_path, {"kind": "resume", "by": a.by, "note": a.note}, mirror=mirror)
    return 0


def cmd_check(rows=None) -> int:
    try:
        rows = rows if rows is not None else rw.load_plan()
    except rw.RowsError as e:
        print(f"rows: {e}")
        return 1
    problems = [f"criterion 3: *reported* {r} has no bar in amendment 1" for r in rw.missing_bars(rows)]
    problems += [f"criterion 1: {p}" for p in rw.coverage(rw.PLAN.read_text(), rw.INDEX.read_text(), rows)]
    levels = {}
    for r in rows.values():
        levels[r.level] = levels.get(r.level, 0) + 1
    print(f"{len(rows)} rows: " + ", ".join(f"L{k} {v}" for k, v in sorted(levels.items())))
    print("\n".join(problems) if problems else "check: OK")
    return 1 if problems else 0


def cmd_status(*, ledger_path=LEDGER, rows=None) -> int:
    rows = rows if rows is not None else rw.load_plan()
    events = lg.read(ledger_path)
    for r in rows.values():
        v = lg.latest_verdict(events, r.id) or "-"
        if v == "FAILED" and lg.has_go(events, r.id):
            v += " (go)"
        print(f"{r.id:6} {v:12} {r.check[:70]}")
    for e in lg.open_runs(events):
        print(f"OPEN: {e['row']} {e['run']} (close it: rule 6)")
    h = lg.halted(events)
    if h:
        print(f"HALTED at {h['row']} {h.get('run')}: {h.get('note')}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check")
    sub.add_parser("coverage")
    sub.add_parser("status")
    r = sub.add_parser("run")
    r.add_argument("row")
    r.add_argument("--surface", required=True, choices=lg.SURFACES)
    r.add_argument("--motion", default="", help="comma list of " + ",".join(lg.MOTIONS))
    r.add_argument("--room", required=True)
    r.add_argument("--cmd", help="the board command (run in ~/bench-3140, $OUT set); none: a manual row")
    r.add_argument("--sha", default="HEAD")
    r.add_argument("--timed", action="store_true", help="the exclusive desktop lock")
    r.add_argument("--wait", action="store_true", help="wait for the desktop lock")
    r.add_argument("--robot-port", type=int, default=ROBOT_PORT)
    r.add_argument("--note")
    r.add_argument("--value", action="append", default=[], help="k=v, a manual row's reading")
    r.add_argument("--log", action="append", default=[], help="a local text log to keep with the record")
    r.add_argument("--signed-by", default="")
    r.add_argument("--on-stand", action="store_true")
    r.add_argument("--stop-person", default="")
    r.add_argument("--power-switch", action="store_true")
    r.add_argument("--safety-on", action="store_true")
    r.add_argument("--truth", choices=lg.TRUTH)
    r.add_argument("--speed-default", action="store_true")
    i = sub.add_parser("_run")
    i.add_argument("spec")
    c = sub.add_parser("close")
    c.add_argument("row")
    c.add_argument("run")
    c.add_argument("--surprise", required=True, choices=("no", "yes"))
    c.add_argument("--note")
    s = sub.add_parser("score")
    s.add_argument("row")
    s.add_argument("run", nargs="?")
    s.add_argument("--verdict", required=True, choices=lg.VERDICTS)
    s.add_argument("--evidence", default="")
    s.add_argument("--by", default="")
    g = sub.add_parser("go")
    g.add_argument("row")
    g.add_argument("--by", default="")
    g.add_argument("--note", default="")
    u = sub.add_parser("resume")
    u.add_argument("--by", default="")
    u.add_argument("--note", default="")
    a = ap.parse_args(argv)
    if a.cmd == "check":
        return cmd_check()
    if a.cmd == "coverage":
        problems = rw.coverage(rw.PLAN.read_text(), rw.INDEX.read_text())
        print("\n".join(problems) if problems else "coverage: every section 3.1-3.139 accounted for")
        return 1 if problems else 0
    if a.cmd == "status":
        return cmd_status()
    if a.cmd == "run":
        return cmd_run(a)
    if a.cmd == "_run":
        return execute(json.loads(Path(a.spec).read_text()))
    if a.cmd == "close":
        return cmd_close(a)
    if a.cmd == "score":
        return cmd_score(a)
    if a.cmd == "go":
        return cmd_go(a)
    if a.cmd == "resume":
        return cmd_resume(a)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
