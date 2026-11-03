# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.135.A's harness against made-up truth logs and a fake board (no
stack, no ssh, no locks):

    .venv/bin/python -m pytest evaluations/jetson-fmea-3135/test_fmea_harness.py -q
"""

import copy
import importlib.util
import json
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import fmea  # noqa: E402
import measure as m  # noqa: E402
import plant_bugs as plant  # noqa: E402
import stack  # noqa: E402

# This section's run.py by path: 3.136's is also `run`, and it imports it.
_spec = importlib.util.spec_from_file_location("fmea_run", HERE / "run.py")
fmea_run = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fmea_run)
br, perf_run = fmea_run.br, fmea_run.perf_run


def log(samples):
    """(t, x, sp) tuples -> truth rows, 1 m of open floor around each."""
    return [{"t": t, "x": x, "y": 0.0, "th": 0.0, "sp": sp, "gap_cm": 50.0} for t, x, sp in samples]


def dense(*spans):
    """(t0, t1, sp) spans -> a log sampled every 20 ms, as the recorder writes."""
    out = []
    for t0, t1, sp in spans:
        n = round((t1 - t0) / 0.02)
        out += [(round(t0 + i * 0.02, 3), 0.0, sp) for i in range(n)]
    return log(out)


def test_wheels_zero_is_the_first_zero_that_holds_half_a_second():
    rows = dense((0.0, 1.0, [0.3, 0.3]), (1.0, 1.2, [0.0, 0.0]), (1.2, 1.4, [0.3, 0.3]),
                 (1.4, 2.5, [0.0, 0.0]), (2.5, 3.0, [0.1, -0.1]))
    assert m.wheels_zero_s(rows, 0.4) == 1.0           # 1.0 was undone at 1.2; 1.4 holds
    assert m.wheels_zero_s(rows, 2.5) is None           # a turn later is not zero
    assert m.wheels_zero_s([r for r in rows if r["t"] < 1.8], 0.4) is None   # log ends first
    holey = dense((0.0, 0.2, [0, 0])) + dense((1.0, 1.1, [0, 0]))
    assert m.wheels_zero_s(holey, 0.0) is None           # a hold across a 0.8 s hole is no hold
    assert m.is_forward([0.3, 0.3]) and not m.is_forward([0.1032, -0.1032])
    assert m.forward_at(rows, 0.7) is True and m.forward_at(rows, 1.1) is False
    assert m.forward_at(rows, -1) is None


def test_travel_contacts_and_holes():
    rows = log([(0.0, 0.0, [0, 0]), (0.1, 0.01, [0, 0]), (0.2, 0.03, [0, 0]), (0.9, 0.05, [0, 0])])
    assert m.travel_cm(rows, 0.05) == 4.0
    assert m.travel_cm(rows, 0.0, 0.15) == 1.0
    rows[2]["gap_cm"] = 0.4
    assert m.contacts(rows) == 1 and m.contacts(rows, 0.5) == 0
    assert m.gaps_s(rows, 0.0, 1.0) == 0.7
    assert m.gaps_s(rows, 0.0, 3.0) == 2.1                # a log that stops early has a hole
    assert m.gaps_s(rows, 5.0, 6.0) == float("inf")
    assert m.nonzero_after(log([(1, 0, [0.1, 0.1]), (2, 0, [0, 0])]), 0, 3) == 1


def test_truth_reader_skips_a_line_cut_by_the_kill(tmp_path):
    p = tmp_path / "truth.jsonl"
    p.write_text('{"t": 2, "x": 0, "y": 0, "th": 0, "sp": [0, 0], "gap_cm": 9}\n'
                 '{"t": 1, "x": 0, "y": 0, "th": 0, "sp": [0, 0], "gap_cm": 9}\n{"t": 3, "x"')
    assert [r["t"] for r in m.read_truth(p)] == [1, 2]


GOOD = {"fault_at": 10.0, "forward_at_fault": True, "truth_hole_s": 0.02, "gap_unknown": 0, "contacts": 0, "wheels_zero_s": 0.3,
        "travel_cm": 5.0, "travel_stale_cm": 0.5, "drive_after_cont": 0,
        "no_feedback_refusals": 1, "outcome": "failed",
        # 3.135.B's goal rows
        "end_reason": "ros_unavailable: the bridge is gone", "last_refusal": {"reason": "ros_unavailable"},
        "recovery_outcome": "found", "stop_code": 200, "stop_answer_s": 0.05,
        "wheels_zero_from_stop_s": 0.0, "drive_after_unpause": 0, "driver_at_fault": "ros"}


@pytest.mark.parametrize("row", sorted(fmea.ROWS))
def test_a_run_inside_every_bar_passes(row):
    assert fmea.judge(row, GOOD) == {"pass": True, "valid": True, "reasons": []}


@pytest.mark.parametrize("row, change, reason", [
    ("F6", {"wheels_zero_s": 1.2}, "bar 1.1"),
    ("F6", {"wheels_zero_s": None}, "after None s"),
    ("F6", {"travel_cm": 15.1}, "bar 15"),
    ("F7", {"drive_after_cont": 3}, "after SIGCONT"),
    ("F8", {"wheels_zero_s": 1.7}, "bar 1.6"),
    ("F8", {"outcome": "found"}, "ended found"),
    ("F9", {"travel_stale_cm": 2.1}, "bar 2"),
    ("F10", {"no_feedback_refusals": 0}, "no_feedback"),
    ("F13", {"wheels_zero_s": 1.11}, "bar 1.1"),
    ("F13", {"contacts": 2}, "2 contact"),
    ("F9", {"truth_hole_s": 0.5}, "hole"),
    ("F6", {"fault_at": None}, "never injected"),
    ("F13", {"forward_at_fault": False}, "invalid"),
    ("F8", {"gap_unknown": 3}, "contact unjudged"),
    ("F8", {"gap_unknown": None}, "contact unjudged"),
    ("F4", {"wheels_zero_s": 0.51}, "bar 0.5"),
    ("F4", {"travel_cm": 5.1}, "bar 5"),
    ("F4", {"end_reason": "time limit"}, "not failed naming ROS"),
    ("F4", {"outcome": "exhausted"}, "not failed naming ROS"),
    ("F4", {"last_refusal": {"reason": "watchdog"}}, "not ros_unavailable"),
    ("F4", {"recovery_outcome": "out_of_budget"}, "not found"),
    ("F5", {"stop_answer_s": 1.2}, "within 1.0"),
    ("F5", {"stop_code": "TimeoutError: x"}, "within 1.0"),
    ("F5", {"wheels_zero_from_stop_s": None}, "bar 0.5"),
    ("F5", {"drive_after_unpause": 4}, "after unpause"),
    ("F6g", {"travel_cm": 15.1}, "bar 15"),
    ("F7g", {"drive_after_cont": 1}, "after SIGCONT"),
    ("F4", {"driver_at_fault": "brain"}, "not nav2"),
])
def test_each_broken_bar_fails_its_row(row, change, reason):
    v = fmea.judge(row, {**GOOD, **change})
    assert not v["pass"] and any(reason in r for r in v["reasons"]), v


def test_every_row_names_a_known_plant():
    assert {r["plant"] for r in fmea.ROWS.values()} <= stack.ROBOT_PLANTS | stack.BODY_PLANTS
    with pytest.raises(stack.StackError):
        stack.Stack(Path("/nonexistent-never-made"), plant="nope")


def test_the_stack_environment_carries_nothing_of_the_shell(tmp_path, monkeypatch):
    for k in ("APP_SHARED_SECRET", "VISION_URL", "METRICS_URL", "INVENTORY_BUCKET", "LOCAL_VLM_URL",
              "WALKS_SECRET", "AWS_PROFILE"):
        monkeypatch.setenv(k, "leak")
    env = stack.Stack(tmp_path, plant="watchdog,heartbeat").env
    assert "leak" not in env.values()
    assert env["VISION_URL"] == f"http://127.0.0.1:{stack.CLOSED_PORT}"
    assert env["INVENTORY_BUCKET"] == "" and "METRICS_URL" not in env
    assert env["APP_SHARED_SECRET"] == env["ROBOT_SHARED_SECRET"] and len(env["APP_SHARED_SECRET"]) == 32


def test_plant_summary_catches_only_a_planted_run_that_fails_cleanly():
    ok = {"verdict": {"pass": True, "valid": True, "reasons": []}, "measures": {"wheels_zero_s": 1.0}}
    bad = {"verdict": {"pass": False, "valid": True, "reasons": ["x"]},
           "measures": {"wheels_zero_s": None}}
    s = plant.summarise("F13", ok, bad)
    assert s["caught"] and s["discriminates"] and s["base"]["wheels_zero_s"] == 1.0
    assert not plant.summarise("F13", ok, ok)["caught"]
    assert not plant.summarise("F13", ok, {**bad, "error": "stack died"})["caught"]
    s = plant.summarise("F9", bad, bad)                  # caught, but the base fails too
    assert s["caught"] and not s["discriminates"]
    invalid = {"verdict": {"pass": False, "valid": False, "reasons": ["anything"]}, "measures": {}}
    assert not plant.summarise("F13", ok, invalid)["caught"]


# ------------------------------------------------- the board, faked (criterion 5)

FOUND = {"units": {u: {"active": "active", "enabled": "enabled"} for u in br.UNITS},
         "hold_flag": None, "vlm_env": {"sha256": "ab", "mode": "600"}, "containers": "",
         "listening": "8000 8001 8005 8080 ", "power_mode": "NV Power Mode: 15W",
         "mem_available_mb": 2500, "oom_kill": 0, "uptime_s": 1.0, "epoch": 1.0}


class FakeBoard:
    def __init__(self):
        self.state, self.cmds = copy.deepcopy(FOUND), []

    def __call__(self, cmd, check=True, timeout=0, input=None):
        self.cmds.append(cmd)
        if "systemctl is-active" in cmd and "units=" in cmd:
            return json.dumps(self.state)
        if "echo $?" in cmd:
            return "0\n"
        if "lease_boardrun.py lease" in cmd:          # 3.136.F's lease: its pid, alive
            return "4242\nalive 99\n"
        return "watching\n" if "lease.stop" in cmd else ""


class FakeHeartbeat:
    def __init__(self, path):
        pass

    def start(self):
        pass

    def stop(self):
        pass


class FakeSampler:
    def __init__(self, out, run):
        self.desktop_load = [[1, 0.1, 0.1, 0.1]]

    def start(self):
        pass

    def stop(self):
        pass


def test_board_run_uses_this_sections_checkout_and_leaves_the_board_as_found(monkeypatch, tmp_path):
    monkeypatch.setenv("PICAR_BOARD_TOKEN", "t0k")
    b, pushed = FakeBoard(), []

    def push(sha, run, bench_dir="bench-3136"):
        pushed.append(bench_dir)
        return "f" * 40

    rec = perf_run.board_run("fmea-smoke", fmea_run.SMOKE_CMD, tmp_path, run=b, push=push,
                             fetch=lambda *a: None, sampler_cls=FakeSampler, bench_dir="bench-3135",
                             heartbeat_cls=FakeHeartbeat)
    assert pushed == ["bench-3135"] and rec["board_out"].startswith("bench-3135/out-fmea-smoke-")
    cmd = next(c for c in b.cmds if "fmea.py F13" in c)
    assert cmd.startswith("cd ~/bench-3135 &&") and "PICAR_BOARD_TOKEN=t0k" in cmd
    assert rec["left_as_found"] and rec["as_found_differences"] == []
    assert not any("sudo" in c for c in b.cmds)
    s = fmea_run.smoke_summary(rec, tmp_path)
    assert s["left_as_found"] and s["fmea_exit"] is None        # nothing fetched: nothing claimed


def test_3136s_default_checkout_is_unchanged(monkeypatch, tmp_path):
    monkeypatch.setenv("PICAR_BOARD_TOKEN", "t0k")
    b = FakeBoard()
    rec = perf_run.board_run("x", "true", tmp_path, run=b, push=lambda s, r, bench_dir: "f" * 40,
                             fetch=lambda *a: None, sampler_cls=FakeSampler, heartbeat_cls=FakeHeartbeat)
    assert rec["board_out"].startswith("bench-3136/") and br.BENCH_DIR == "bench-3136"


def test_push_checkout_refuses_a_bench_dir_that_is_not_a_plain_name():
    for bad in ("../x", "a b", "x;rm", "", "-x", "vision-picar-service", "vision-picar", "bench-a/b", "bench-378", "bench-3135x"):
        with pytest.raises(br.BoardError):
            br.push_checkout("HEAD", run=lambda *a, **k: "", bench_dir=bad)


# ------------------------------------------------- 3.135.B: rows.py, amendment 4

import rows  # noqa: E402


def _run(i, ok, valid=True, error=None, why=()):
    return {"i": i, "error": error, "verdict": {"pass": ok, "valid": valid, "reasons": list(why)}}


def test_a_row_is_met_only_with_every_scored_run_passing():
    assert rows.verdict([_run(i, True) for i in range(1, 6)], 5)["state"] == "met"
    v = rows.verdict([_run(1, True), _run(2, False, why=["moved 3 cm"]), *[_run(i, True) for i in (3, 4, 5)]], 5)
    assert v["state"] == "FAILED" and v["runs_failed"] == 1 and v["reasons"] == ["moved 3 cm"]


def test_errored_and_invalid_runs_are_replaced_not_scored():
    recs = [_run(1, False, valid=False), _run(2, True, error="StackError: x"),
            *[_run(i, True) for i in range(3, 8)]]
    v = rows.verdict(recs, 5)
    assert v["state"] == "met" and v["runs_scored"] == 5 and v["runs_total"] == 7
    assert [n["i"] for n in v["not_scored"]] == [1, 2]
    # an errored run's verdict says fail, but it is not a guard's failure
    assert rows.verdict([_run(1, False, error="boom")] + [_run(i, True) for i in range(2, 4)], 5)["state"] == "incomplete"


def test_runs_past_the_wanted_count_are_not_scored():
    recs = [_run(i, True) for i in range(1, 6)] + [_run(6, False)]
    assert rows.verdict(recs, 5)["state"] == "met"


def test_the_row_command_runs_rows_py_with_its_plant():
    c = fmea_run.row_cmd("F9", 5, "lidar_fresh")
    assert "rows.py F9 --out $OUT/rows --runs 5 --plant lidar_fresh" in c
    assert "--house" not in c and "--plant" not in fmea_run.row_cmd("F8", 5)


# ------------------------------------------------------- 3.135.B: cloud rows

import cloud_rows  # noqa: E402

CLOUD_GOOD = {"outcome": "arrived_unconfirmed", "contacts": 0, "gap_unknown": 0, "truth_hole_s": 0.02,
              "late_confirmation": {"state": "refused"}, "drive_after_end": 0, "max_open_paid": 1,
              "park_s": 9.0, "travel_mission_cm": 0.0, "mission_s": 70.0}


def test_cloud_cases_pass_inside_their_bars():
    for row in ("F11a", "F11b", "F12a"):
        assert cloud_rows.judge(row, CLOUD_GOOD)["pass"], row
    for row in ("F11c", "F12b"):
        assert cloud_rows.judge(row, {**CLOUD_GOOD, "outcome": "failed"})["pass"], row


@pytest.mark.parametrize("row, change, reason", [
    ("F11a", {"outcome": "failed"}, "not arrived_unconfirmed"),
    ("F11a", {"outcome": "found"}, "not arrived_unconfirmed"),
    ("F11b", {"outcome": "found"}, "no cloud yes"),
    ("F11b", {"late_confirmation": None}, "no late confirmation"),
    ("F11b", {"drive_after_end": 2}, "after the mission's end"),
    ("F11c", {"outcome": "failed", "travel_mission_cm": 3.0}, "bar 0.5"),
    ("F12a", {"max_open_paid": 2}, "open at once"),
    ("F12a", {"park_s": 31.0}, "bar 30"),
    ("F12b", {"outcome": "failed", "mission_s": 301.0}, "bar 300"),
    ("F12b", {"outcome": "out_of_budget"}, "not failed"),
    ("F11a", {"contacts": 1}, "contact"),
])
def test_each_broken_cloud_bar_fails(row, change, reason):
    v = cloud_rows.judge(row, {**CLOUD_GOOD, **change})
    assert not v["pass"] and any(reason in r for r in v["reasons"]), v


def test_a_cloud_case_that_never_ended_is_not_scored():
    assert cloud_rows.judge("F11a", {"outcome": None})["valid"] is False


def test_rows_py_finds_both_kinds_of_row():
    assert rows.runner("F9").__name__ == "fmea" and rows.runner("F12a").__name__ == "cloud_rows"
    assert {r["plant"] for r in cloud_rows.ROWS.values()} - {""} <= stack.BRAIN_PLANTS


# ------------------------------------------------------- 3.135.B: the clock row

import sys_rows  # noqa: E402

CLOCK_GOOD = {"outcome": "found", "stepped": True, "watchdog_after_step": 0, "brain_503_lines": 0,
              "contacts": 0, "gap_unknown": 0, "truth_hole_s": 0.02}


def test_the_clock_cases_and_their_bars():
    assert sys_rows.judge("F16p", CLOCK_GOOD)["pass"] and sys_rows.judge("F16c", CLOCK_GOOD)["pass"]
    for change, reason in (({"watchdog_after_step": 1}, "watchdog stops"),
                           ({"brain_503_lines": 2}, "503/stale"),
                           ({"outcome": "out_of_budget"}, "out_of_budget"),
                           ({"outcome": "exhausted"}, "the control's found"),
                           ({"stepped": False}, "not stepped")):
        v = sys_rows.judge("F16m", {**CLOCK_GOOD, **change})
        assert not v["pass"] and any(reason in r for r in v["reasons"]), (change, v)
    assert sys_rows.judge("F16m", {**CLOCK_GOOD, "stepped": False})["valid"] is False
    assert rows.runner("F16p").__name__ == "sys_rows"
    assert {r["plant"] for r in sys_rows.ROWS.values()} - {""} <= stack.ROBOT_PLANTS | stack.BRAIN_PLANTS


# --------------------------------------------------------- 3.135.B: VE rows

import vlm_rows  # noqa: E402

VLM_GOOD = {"fault_at": 1.0, "forward_at_fault": True, "driver_at_fault": "ros", "question_hit": True,
            "contacts": 0, "gap_unknown": 0, "truth_hole_s": 0.02, "oom_kills": 1, "killed_ve": True,
            "alive_after": {"robot": True, "brain": True, "ros": True}, "late_ticks": 0,
            "ve_back_s": 45.0, "next_answer": True, "failed_in_watch": False,
            "wheels_zero_from_stop_s": 0.06, "cancelled_by": "stop", "paid_after_stop": 0}


@pytest.mark.parametrize("row, outcome", [("F2", "found"), ("F3f", "stopped_by_harness"), ("F3a", "found"),
                                          ("F3r", "arrived_unconfirmed"), ("F17g", "cancelled"),
                                          ("F17q", "cancelled"), ("F17a", "cancelled")])
def test_ve_cases_pass_inside_their_bars(row, outcome):
    assert vlm_rows.judge(row, {**VLM_GOOD, "outcome": outcome}) == {"pass": True, "valid": True, "reasons": []}


@pytest.mark.parametrize("row, change, reason", [
    ("F2", {"oom_kills": 2}, "bar 1"), ("F2", {"killed_ve": False}, "not VE"),
    ("F2", {"alive_after": {"robot": True, "brain": False, "ros": True}}, "alive after"),
    ("F2", {"late_ticks": 1}, "late wheel"), ("F2", {"outcome": "running"}, "ended running"),
    ("F3f", {"ve_back_s": 61.0}, "bar 60"), ("F3f", {"next_answer": False}, "not answered"),
    ("F3f", {"failed_in_watch": True}, "failed after the kill"),
    ("F3a", {"outcome": "out_of_budget"}, "not found"), ("F3r", {"outcome": "found"}, "not arrived_unconfirmed"),
    ("F17g", {"wheels_zero_from_stop_s": 0.21}, "bar 0.2"),
    ("F17q", {"outcome": "running", "cancelled_by": None}, "not cancelled by stop"),
    ("F17a", {"paid_after_stop": 1}, "paid calls after"),
])
def test_each_broken_ve_bar_fails(row, change, reason):
    base = {**VLM_GOOD, "outcome": {"F2": "found", "F3f": "x", "F3a": "found", "F3r": "arrived_unconfirmed"}.get(row, "cancelled")}
    v = vlm_rows.judge(row, {**base, **change})
    assert not v["pass"] and any(reason in r for r in v["reasons"]), v


def test_a_ve_fault_off_its_moment_is_invalid():
    assert vlm_rows.judge("F3a", {**VLM_GOOD, "question_hit": False, "outcome": "found"})["valid"] is False
    assert vlm_rows.judge("F17g", {**VLM_GOOD, "driver_at_fault": "brain", "outcome": "cancelled"})["valid"] is False
    assert rows.runner("F17q").__name__ == "vlm_rows"
    assert {r["plant"] for r in vlm_rows.ROWS.values()} - {"ve_oom0"} <= stack.ROBOT_PLANTS | stack.BRAIN_PLANTS


def test_the_full_disk_case_and_its_bars():
    ok = {"outcome": "found", "write_fail_lines": 2, "mission_s": 120.0, "late_ticks": 0,
          "contacts": 0, "gap_unknown": 0, "truth_hole_s": 0.02}
    assert sys_rows.judge("F15", ok)["pass"]
    assert sys_rows.judge("F15", {**ok, "write_fail_lines": 0})["valid"] is False
    assert not sys_rows.judge("F15", {**ok, "write_fail_lines": 9})["pass"]       # per frame, not per minute
    assert not sys_rows.judge("F15", {**ok, "late_ticks": 1})["pass"]
    assert not sys_rows.judge("F15", {**ok, "outcome": None})["pass"]
