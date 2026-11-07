# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.136.C's scorer (`lat_score.py`): the readings amendment 7 defines.

    pytest evaluations/jetson-perf-3136/test_lat.py -q
"""

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import lat_score as ls  # noqa: E402


def truth(sps, t0=100.0, dt=0.02):
    return [{"t": round(t0 + i * dt, 4), "sp": sp, "gap_cm": 20.0} for i, sp in enumerate(sps)]


def test_forward_zero_reads_a_pivot_as_stopped():
    # moving, then nav2's pivot in place (equal and opposite): forward zero
    rows = truth([[0.2, 0.2]] * 5 + [[-0.01, 0.01]] * 40)
    assert ls.forward_zero_s(rows, 100.0) == 0.1
    # the strict reading never sees it zero
    assert ls.m.wheels_zero_s(rows, 100.0) is None


def test_forward_zero_needs_the_hold():
    rows = truth([[0.2, 0.2]] * 5 + [[0, 0]] * 10 + [[0.2, 0.2]] * 5 + [[0, 0]] * 40)
    assert ls.forward_zero_s(rows, 100.0) == 0.4


def jl(path: Path, rows):
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))


def test_nav_excuses_a_goal_ended_in_an_acts_span(tmp_path):
    jl(tmp_path / "pace.jsonl", [
        {"k": "nav", "t": 10.0, "goals": [["a" * 12, 2], ["b" * 12, 2], ["c" * 12, 2]]},
        {"k": "nav", "t": 12.0, "goals": [["a" * 12, 6]]},      # aborted inside the drop's span
        {"k": "nav", "t": 30.0, "goals": [["b" * 12, 6]]},      # aborted long after any act
        {"k": "nav", "t": 31.0, "goals": [["c" * 12, 4]]}])
    jl(tmp_path / "person.jsonl", [{"k": "drop", "t": 11.0, "until": 13.5}])
    jl(tmp_path / "explore_sends.jsonl", [{"sent": [["a" * 32, "approach"], ["b" * 32, "view"],
                                                    ["c" * 32, "frontier"]]}])
    n = ls.nav(tmp_path)
    assert n["person"] == 1 and n["aborted_other"] == 1 and n["succeeded"] == 1


def test_p5_reads_the_first_scan_that_shows_the_object(tmp_path):
    s = tmp_path / "s001"
    s.mkdir()
    jl(s / "truth.jsonl", truth([[0.2, 0.2]] * 10 + [[0.0, 0.0]] * 40, t0=100.0))
    jl(s / "lat_shim.jsonl", [{"k": "scan", "t": 100.05, "fwd_m": 1.2},
                              {"k": "scan", "t": 100.15, "fwd_m": 1.1},     # swept before the drop
                              {"k": "scan", "t": 100.18, "fwd_m": 0.3}])
    jl(tmp_path / "p5.jsonl", [{"tag": "base", "stack": 1,
                                "drop": {"t": 100.1, "T_cm": 12.0}}])
    t = ls.p5(tmp_path)["base"]["trials"][0]
    assert t["drop_to_scan_s"] == 0.08
    assert t["scan_to_zero_s"] == 0.02 and t["drop_to_zero_s"] == 0.1
    assert t["clear_before"] is True and t["contacts"] == 0


def test_a_missing_reading_is_the_worst_not_dropped():
    s = ls.summary([0.01] * 39 + [None], 99)
    assert s["n_measured"] == 39 and s["p99_ms"] == "inf"


def test_amendment_9_counts_an_approach_abort_as_a_frontier_one(tmp_path):
    jl(tmp_path / "pace.jsonl", [
        {"k": "nav", "t": 10.0, "goals": [["a" * 12, 2], ["c" * 12, 2]]},
        {"k": "nav", "t": 20.0, "goals": [["a" * 12, 6], ["c" * 12, 4]]}])
    jl(tmp_path / "explore_sends.jsonl", [{"sent": [["a" * 32, "approach"], ["c" * 32, "frontier"]]}])
    assert ls.nav(tmp_path)["aborted_other"] == 1                      # runs 1-2: as written
    n = ls.nav(tmp_path, approach_as_frontier=True)
    assert n["aborted_other"] == 0 and n["aborted_frontier"] == 1


def test_amendment_9_applies_only_to_runs_after_2026_11_07(tmp_path):
    (tmp_path / "stacks.jsonl").write_text("")
    (tmp_path / "run_start").write_text("1794036000.0\n")             # run 2's day
    assert ls.validity(tmp_path)["amendment_9"] is False
    (tmp_path / "run_start").write_text("1795000000.0\n")             # the week of 11-23
    assert ls.validity(tmp_path)["amendment_9"] is True
