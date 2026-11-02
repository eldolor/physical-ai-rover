# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.140.A's harness against a fake ledger and a fake board (no ssh, no locks):

    .venv/bin/python -m pytest evaluations/rover-live-3140/test_harness.py -q
"""

import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import ledger as lg  # noqa: E402
import rows as rw  # noqa: E402
import run as runner  # noqa: E402

ROWS = rw.load_plan()
SHA = "a" * 40
STAND_OK = {"signed_by": "U", "on_stand": True, "safety_on": True, "truth": "none",
            "speed_default": True}
FLOOR_OK = {"signed_by": "U", "stop_person": "U", "power_switch": True, "safety_on": True,
            "truth": "grid", "speed_default": True}
NONE_OK = {"signed_by": "U"}


def ev(kind, row=None, run=None, **kw):
    e = {"kind": kind, **kw}
    if row:
        e["row"] = row
    if run:
        e["run"] = run
    return e


def passed(*row_ids):
    """Every named row run off the car, closed and met."""
    out = []
    for i, r in enumerate(row_ids):
        out += [ev("open", r, f"p{i}", surface="none", motion=[]), ev("close", r, f"p{i}", surprise=False),
                ev("score", r, f"p{i}", verdict="met")]
    return out


def stand(path, run="s1", verdict=None, surprise=False):
    out = [ev("open", "L1.3", run, surface="stand", motion=[path]),
           ev("close", "L1.3", run, surprise=surprise)]
    if verdict:
        out.append(ev("score", "L1.3", run, verdict=verdict))
    return out


def level_ids(*levels):
    return [r.id for r in ROWS.values() if r.level in levels]


def floor(events, row="L2.3", motion=("verb",), checklist=FLOOR_OK, on_dev=True):
    return lg.refusals(events, ROWS, row, "floor", list(motion), checklist, on_dev, True)


# ------------------------------------------------------------- A1, A2, A6

def test_the_rows_come_from_the_section_file():
    counts = {lv: len(level_ids(lv)) for lv in range(9)}
    assert counts == {0: 12, 1: 9, 2: 13, 3: 12, 4: 11, 5: 15, 6: 5, 7: 7, 8: 10}
    assert rw.missing_bars(ROWS) == []
    assert ROWS["L5.1"].reported and "8" in ROWS["L5.1"].amended_bar
    assert ROWS["L8.1"].decision


@pytest.mark.parametrize("table, err", [
    ("### L2 -- x\n| L2.1 | a | b | c |\n| L2.1 | a | b | c |\n", "twice"),
    ("### L2 -- x\n| L2.1 | a | b | c |\n| L2.3 | a | b | c |\n", "out of order"),
    ("### L2 -- x\n| L3.1 | a | b | c |\n", "under L2"),
    ("### L2 -- x\n| L2.1 | a | b | *reported* |\n## Amendment 1 (x)\n| L2.9 | bar |\n", "do not exist"),
])
def test_a_broken_table_is_refused(table, err):
    with pytest.raises(rw.RowsError, match=err):
        rw.load(table)


def test_a_reported_row_without_an_amendment_bar_is_named():
    rows = rw.load("### L2 -- x\n| L2.1 | a | b | *reported* |\n| L2.2 | a | b | 1 cm |\n")
    assert rw.missing_bars(rows) == ["L2.1"]


def test_coverage_of_the_real_plan_is_complete():
    assert rw.coverage(rw.PLAN.read_text(), rw.INDEX.read_text(), ROWS) == []


def test_coverage_names_what_is_missing():
    text = rw.PLAN.read_text()
    index = rw.INDEX.read_text() + "| [3.141](3.141-x.md) | a later section |\n"
    assert rw.coverage(text, index, ROWS) == ["3.141: in the index, not in the coverage table"]
    broken = text.replace("| 3.34 | L2.1 |", "| 3.34 | L2.99 |")
    assert rw.coverage(broken, rw.INDEX.read_text(), ROWS) == ["3.34: names L2.99, which does not exist"]
    dropped = text.replace("| 3.38-3.40 SLAM | L3.3 |", "| 3.38-3.39 SLAM | L3.3 |")
    assert rw.coverage(dropped, rw.INDEX.read_text(), ROWS) == [
        "3.40: in the index, not in the coverage table"]


def test_check_command_passes_on_the_plan(capsys):
    assert runner.cmd_check() == 0
    assert "check: OK" in capsys.readouterr().out


# ------------------------------------------------------------------ A3 gates

def test_a_good_floor_run_starts():
    events = passed(*level_ids(0, 1)) + stand("verb")
    assert floor(events) == []


def test_a_levels_in_order():
    events = passed(*level_ids(0)) + stand("verb")       # L1 not run
    why = floor(events)
    assert any("L2 on the floor waits on L1.1 (not run)" in w for w in why)
    # A FAILED L3 row with the user's go lets L4 on the floor; without it, not.
    base = passed(*level_ids(0, 1, 2)) + passed(*[r for r in level_ids(3) if r != "L3.5"]) + stand("verb")
    failed = base + [ev("open", "L3.5", "f", surface="none", motion=[]), ev("close", "L3.5", "f", surprise=False),
                     ev("score", "L3.5", "f", verdict="FAILED")]
    assert any("L3.5 (FAILED, no go" in w for w in floor(failed, row="L4.1"))
    assert floor(failed + [ev("go", "L3.5")], row="L4.1") == []
    # A go given before the latest FAILED does not count.
    again = failed + [ev("go", "L3.5"), ev("score", "L3.5", "f", verdict="FAILED")]
    assert any("L3.5" in w for w in floor(again, row="L4.1"))


def test_a_failed_hard_row_stops_every_floor_run_but_its_own():
    events = passed(*level_ids(0, 1)) + stand("verb") + [ev("score", "L2.3", "x", verdict="FAILED")]
    assert any("L2.3 FAILED stops every floor run" in w for w in floor(events, row="L2.4"))
    assert floor(events, row="L2.3") == []            # its own re-run, the fix's proof
    assert any("L2.3 FAILED" in w for w in floor(events, row="L1.3"))
    assert lg.go_refusals(events, ROWS, "L2.3") == [
        "criterion 2: a FAILED L2 row is fixed, not passed by a go"]


@pytest.mark.parametrize("stand_events, ok", [
    ([], False),
    (stand("goal"), False),                                # another path
    (stand("verb", surprise=True), False),                 # a surprise on the stand
    (stand("verb", verdict="FAILED"), False),
    ([ev("open", "L1.3", "s1", surface="stand", motion=["verb"])], False),   # never closed
    (stand("verb"), True),
    (stand("verb", verdict="met"), True),
])
def test_b_stand_first(stand_events, ok):
    events = passed(*level_ids(0, 1)) + stand_events
    rule1 = [w for w in floor(events) if w.startswith("rule 1")]
    assert (rule1 == []) == ok


def test_b_every_named_path_needs_its_stand_run():
    events = passed(*level_ids(0, 1)) + stand("verb")
    assert floor(events, motion=("verb", "goal")) == ["rule 1: no stand run of motion path 'goal' yet"]


def test_c_a_reported_row_without_a_bar_is_refused():
    rows = rw.load("### L0 -- x\n| L0.1 | a | b | *reported* |\n")
    why = lg.refusals([], rows, "L0.1", "none", [], NONE_OK, True, False)
    assert why == ["criterion 3: L0.1 is *reported* and amendment 1 has no bar for it"]
    assert lg.score_refusals([ev("open", "L0.1", "r"), ev("close", "L0.1", "r")], rows, "L0.1", "r",
                             "met") == ["criterion 3: L0.1 has no bar to be met"]


def test_d_the_commit_must_be_on_dev():
    events = passed(*level_ids(0, 1)) + stand("verb")
    assert floor(events, on_dev=False) == ["criterion 3: the commit under test is not on origin/dev"]


@pytest.mark.parametrize("surface, drop, rule", [
    ("floor", "signed_by", "checklist unsigned"),
    ("floor", "stop_person", "rule 2"),
    ("floor", "power_switch", "rule 2"),
    ("floor", "safety_on", "rule 3"),
    ("floor", "truth", "rule 4"),
    ("floor", "speed_default", "rule 5"),
    ("stand", "on_stand", "rule 1"),
])
def test_e_the_checklist_must_be_signed(surface, drop, rule):
    full = dict(FLOOR_OK if surface == "floor" else STAND_OK)
    assert lg.check_checklist(surface, full) == []
    full.pop(drop)
    why = lg.check_checklist(surface, full)
    assert len(why) == 1 and why[0].startswith(rule)


def test_e_an_unsigned_run_is_refused_by_the_gates():
    events = passed(*level_ids(0, 1)) + stand("verb")
    assert floor(events, checklist={**FLOOR_OK, "stop_person": ""}) == [
        "rule 2: a floor run needs a named person on STOP (--stop-person)"]
    assert lg.refusals([], ROWS, "L4.2", "none", [], {}, True, False) == [
        "checklist unsigned (--signed-by: the person at the car)"]


def test_f_a_surprise_halts_until_the_users_resume(tmp_path):
    led = tmp_path / "ledger.jsonl"
    for e in passed(*level_ids(0, 1)) + stand("verb"):
        lg.append(led, e)
    lg.append(led, ev("open", "L2.3", "r1", surface="floor", motion=["verb"]))
    assert runner.cmd_close(SimpleNamespace(row="L2.3", run="r1", surprise="yes", note=""),
                            ledger_path=led) == 3                       # a surprise is described
    assert runner.cmd_close(SimpleNamespace(row="L2.3", run="r1", surprise="yes", note="touched the jamb"),
                            ledger_path=led) == 0
    events = lg.read(led)
    assert lg.latest_verdict(events, "L2.3") == "FAILED"
    assert any("rule 6: the session halted" in w for w in floor(events, row="L2.3"))
    none = lg.refusals(events, ROWS, "L4.2", "none", [], NONE_OK, True, False)
    assert any("rule 6" in w for w in none)                            # every run, not only floor
    assert runner.cmd_resume(SimpleNamespace(by="U", note=""), ledger_path=led) == 3
    assert runner.cmd_resume(SimpleNamespace(by="U", note="jamb taped; go on"), ledger_path=led) == 0
    events = lg.read(led)
    assert lg.halted(events) is None
    assert not any("rule 6" in w for w in floor(events, row="L2.3"))


def test_an_open_run_blocks_the_next():
    events = passed(*level_ids(0, 1)) + stand("verb") + [ev("open", "L2.4", "r9", surface="floor", motion=["verb"])]
    assert floor(events) == ["an open run must be closed first: L2.4 r9"]


@pytest.mark.parametrize("surface, motion, board, part", [
    ("none", ["verb"], False, "motion named for a run off"),
    ("floor", [], True, "names its motion paths"),
    ("stand", ["verb"], False, "goes through the board"),
    ("floor", ["drive"], True, "motion ['drive']"),
])
def test_run_shapes_refused(surface, motion, board, part):
    checklist = FLOOR_OK if surface == "floor" else STAND_OK
    why = lg.refusals(passed(*level_ids(0, 1)) + stand("verb"), ROWS, "L2.3", surface, motion,
                      checklist, True, board)
    assert any(part in w for w in why), why


def test_decisions_are_scored_never_run():
    assert lg.refusals([], ROWS, "L8.1", "none", [], NONE_OK, True, False)[0].startswith("L8.1 is an L8")
    assert lg.score_refusals([], ROWS, "L8.1", None, "decided") == []
    assert lg.score_refusals([], ROWS, "L2.1", None, "decided")[0].endswith("`decided` is only for L8")


def test_a_run_is_scored_only_once_closed():
    events = [ev("open", "L0.1", "r")]
    assert "not closed" in lg.score_refusals(events, ROWS, "L0.1", "r", "met")[0]
    assert lg.score_refusals(events + [ev("close", "L0.1", "r")], ROWS, "L0.1", "r", "met") == []


# ---------------------------------------------------- execute, A3 g, A4, A5

class FakeBoard:
    def __init__(self, mode="hardware"):
        self.mode = mode
        self.calls = []

    def ssh(self, cmd, check=True, timeout=120.0, input=None):
        self.calls.append(cmd)
        if "/health" in cmd:
            return json.dumps({"status": "ok", "mode": self.mode})
        return ""

    def board_run(self, label, cmd, out, *, sha, allow_sudo, stop_twin, run, bench_dir):
        self.board = {"label": label, "cmd": cmd, "sha": sha, "bench_dir": bench_dir,
                      "allow_sudo": allow_sudo, "stop_twin": stop_twin}
        b = out / "board"
        b.mkdir(parents=True)
        (b / "health_before.json").write_text(json.dumps({"mode": self.mode, "wheel_loop": {"late": 0}}))
        (b / "health_after.json").write_text(json.dumps({"mode": self.mode, "wheel_loop": {"late": 0}}))
        (b / "row_exit").write_text("0\n")
        return {"label": label, "sha": "board-checkout", "left_as_found": True, "as_found": {}}


def spec(row="L2.3", surface="floor", motion=("verb",), cmd="python -m tools.x", checklist=FLOOR_OK):
    return {"row": row, "run": "20261102-120000", "sha": SHA, "room": "hall", "surface": surface,
            "motion": list(motion), "checklist": checklist, "cmd": cmd, "timed": False, "note": None,
            "values": {}, "logs": [], "port": 8000}


def seeded(tmp_path):
    led = tmp_path / "ledger.jsonl"
    for e in passed(*level_ids(0, 1)) + stand("verb"):
        lg.append(led, e)
    return led


def test_a_board_run_is_recorded(tmp_path, monkeypatch):
    monkeypatch.setenv("PICAR_BOARD_TOKEN", "tok")
    led, fb = seeded(tmp_path), FakeBoard()
    rc = runner.execute(spec(), ledger_path=led, rows=ROWS, run=fb.ssh, board_run=fb.board_run,
                        on_dev_fn=lambda s: True, records=tmp_path)
    assert rc == 0
    assert fb.board["bench_dir"] == "bench-3140" and fb.board["sha"] == SHA
    assert fb.board["allow_sudo"] is False and fb.board["stop_twin"] is False
    assert "health_before.json" in fb.board["cmd"] and "health_after.json" in fb.board["cmd"]
    rec = json.loads((tmp_path / "L2.3" / "20261102-120000" / "record.json").read_text())
    for k in ("sha", "room", "surface", "motion", "checklist", "health_before", "health_after",
              "left_as_found", "as_found", "bar"):
        assert k in rec, k
    assert rec["sha"] == SHA and rec["health_before"]["mode"] == "hardware" and rec["row_exit"] == 0
    opened = lg.open_runs(lg.read(led))
    assert [(e["row"], e["run"]) for e in opened] == [("L2.3", "20261102-120000")]


def test_g_a_board_not_in_hardware_mode_is_refused(tmp_path, monkeypatch):
    monkeypatch.setenv("PICAR_BOARD_TOKEN", "tok")
    led, fb = seeded(tmp_path), FakeBoard(mode="sim")
    rc = runner.execute(spec(), ledger_path=led, rows=ROWS, run=fb.ssh, board_run=fb.board_run,
                        on_dev_fn=lambda s: True, records=tmp_path)
    assert rc == 3 and not hasattr(fb, "board")
    assert lg.open_runs(lg.read(led)) == []


def test_the_gates_are_read_again_under_the_locks(tmp_path, monkeypatch):
    monkeypatch.setenv("PICAR_BOARD_TOKEN", "tok")
    led, fb = seeded(tmp_path), FakeBoard()
    lg.append(led, ev("halt", "L2.4", "x", note="contact"))      # landed while the run waited
    assert runner.execute(spec(), ledger_path=led, rows=ROWS, run=fb.ssh, board_run=fb.board_run,
                          on_dev_fn=lambda s: True, records=tmp_path) == 3
    assert fb.calls == [] and not hasattr(fb, "board")


def test_a_board_run_needs_the_board_lock(tmp_path, monkeypatch):
    monkeypatch.delenv("PICAR_BOARD_TOKEN", raising=False)
    fb = FakeBoard()
    with pytest.raises(Exception, match="board_lock"):
        runner.execute(spec(), ledger_path=seeded(tmp_path), rows=ROWS, run=fb.ssh,
                       board_run=fb.board_run, on_dev_fn=lambda s: True, records=tmp_path)
    assert fb.calls == []


def _args(**kw):
    base = dict(row="L2.3", surface="floor", motion="verb", room="hall", cmd="x", sha="HEAD",
                timed=False, wait=False, robot_port=8000, note=None, value=[], log=[],
                signed_by="U", on_stand=False, stop_person="U", power_switch=True, safety_on=True,
                truth="grid", speed_default=True)
    base.update(kw)
    return SimpleNamespace(**base)


@pytest.mark.parametrize("timed, mode", [(True, "exclusive"), (False, "shared")])
def test_a5_a_board_run_goes_through_both_locks(tmp_path, monkeypatch, timed, mode):
    monkeypatch.setattr(runner, "fetch_dev", lambda: None)
    monkeypatch.setattr(runner, "on_dev", lambda s: True)
    seen = {}

    def locked(argv, why, desktop_mode, desktop_wait):
        seen.update(argv=argv, why=why, mode=desktop_mode)
        seen["spec"] = json.loads(Path(argv[-1]).read_text())
        return 0

    assert runner.cmd_run(_args(timed=timed), ledger_path=seeded(tmp_path), rows=ROWS, locked=locked) == 0
    assert seen["mode"] == mode and seen["argv"][-2] == "_run" and "3.140 L2.3" in seen["why"]
    assert seen["spec"]["sha"] == runner.resolve("HEAD")
    assert not Path(seen["argv"][-1]).exists()          # the spec file is cleaned up


def test_a5_refused_before_any_lock(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, "fetch_dev", lambda: None)
    monkeypatch.setattr(runner, "on_dev", lambda s: False)
    called = []
    assert runner.cmd_run(_args(), ledger_path=seeded(tmp_path), rows=ROWS,
                          locked=lambda *a, **k: called.append(1)) == 3
    assert called == []


def test_a5_a_manual_row_takes_no_lock_and_keeps_its_readings(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, "fetch_dev", lambda: None)
    monkeypatch.setattr(runner, "on_dev", lambda s: True)
    monkeypatch.setattr(runner, "HERE", tmp_path)
    log = tmp_path / "calipers.txt"
    log.write_text("86.1 x 58.0\n")
    led = tmp_path / "ledger.jsonl"
    a = _args(row="L0.1", surface="none", motion="", cmd=None, value=["holes_mm=86.1x58.0"], log=[str(log)])
    called = []
    orig = runner.execute
    monkeypatch.setattr(runner, "execute",
                        lambda s, ledger_path, rows: orig(s, ledger_path=ledger_path, rows=rows,
                                                          on_dev_fn=lambda x: True, records=tmp_path))
    assert runner.cmd_run(a, ledger_path=led, rows=ROWS, locked=lambda *x, **k: called.append(1)) == 0
    assert called == []
    [d] = list((tmp_path / "L0.1").iterdir())
    rec = json.loads((d / "record.json").read_text())
    assert rec["values"] == {"holes_mm": "86.1x58.0"} and rec["logs"] == ["logs/calipers.txt"]
    assert (d / "logs" / "calipers.txt").read_text() == "86.1 x 58.0\n"


def test_score_go_and_close_record_who_and_why(tmp_path):
    led = tmp_path / "ledger.jsonl"
    lg.append(led, ev("open", "L5.1", "r", surface="none", motion=[]))
    assert runner.cmd_score(SimpleNamespace(row="L5.1", run="r", verdict="met", evidence="", by="C"),
                            ledger_path=led, rows=ROWS) == 3            # not closed, no evidence
    assert runner.cmd_close(SimpleNamespace(row="L5.1", run="r", surprise="no", note=None),
                            ledger_path=led) == 0
    assert runner.cmd_close(SimpleNamespace(row="L5.1", run="r", surprise="no", note=None),
                            ledger_path=led) == 3                       # once
    assert runner.cmd_score(SimpleNamespace(row="L5.1", run="r", verdict="FAILED", evidence="6/10", by="C"),
                            ledger_path=led, rows=ROWS) == 0
    assert runner.cmd_go(SimpleNamespace(row="L5.1", by="", note=""), ledger_path=led, rows=ROWS) == 3
    assert runner.cmd_go(SimpleNamespace(row="L5.1", by="user", note="carry on"),
                         ledger_path=led, rows=ROWS) == 0
    assert lg.has_go(lg.read(led), "L5.1")


def test_a_cut_ledger_line_is_an_error_not_skipped(tmp_path):
    led = tmp_path / "ledger.jsonl"
    led.write_text('{"kind": "open"\n')
    with pytest.raises(lg.Refused, match="not JSON"):
        lg.read(led)


# ------------------------------------------------------------- A4: no images

@pytest.mark.parametrize("name, ignored", [
    ("L4.1/r/board/frame_0001.jpg", True), ("L6.1/r/twin.png", True), ("L1.4/r/grid.mp4", True),
    ("L3.8/r/board/maps/v3/current.json", True), ("L3.8/r/board/house.posegraph", True),
    ("L3.8/r/board/house.data", True), ("L3.8/r/board/map.pgm", True),
    ("L2.3/r/record.json", False), ("L2.3/r/board/row.log", False), ("ledger.jsonl", False),
])
def test_images_and_maps_never_enter_git(name, ignored):
    r = subprocess.run(["git", "check-ignore", "-q", str(HERE / name)], cwd=HERE)
    assert (r.returncode == 0) == ignored
