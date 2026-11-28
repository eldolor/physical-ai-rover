# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_scan_stamp_truth.py

PLAN-ros-alignment.md 3.236.A criterion 4: `/scan`'s `stamp_unix` against
GROUND TRUTH under the fake lidar's real timing -- a 3 s version of
`evaluations/scan-stamp-3236/measure.py` (the emitter's packets on a real
pty, the real driver's reader thread, the robot server's real `/scan`
route). 3.221 pinned the capture stamp with an injected clock; this pins it
on the clock the car runs on, so a regression back to request stamps (the
error nav2 and slam_toolbox were handed before 3.221: 50-150 ms) fails here.

The revolution each reply served is read from its ranges (the emitter casts
each revolution at its own distance), never from the stamp, so a stamp a
whole revolution off fails too (review, 3.236.A).

Bars are looser than the 60 s run's (criterion 1: max 1.98 ms on
2026-11-28) so a loaded xdist worker does not fail them by scheduling: the
age is judged on polls that started on time, and the count of revolutions
asks for half the polls.
"""

import importlib.util
import os
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("scan_stamp_3236",
                                               ROOT / "evaluations/scan-stamp-3236/measure.py")
measure = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(measure)


def _bars(s):
    """Criterion 4's bars; returns the first one failed, or None."""
    if s["unusable"] or s["not_capture"]:
        return "unusable or not capture"
    if s["distinct_revolutions"] < 15 or not s["one_stamp_per_revolution"]:
        return "revolutions"
    err = s["err_ms"]
    if abs(err["median"]) > 10.0 or err["max_abs"] > 50.0:
        return "error"
    if s["age_ms"]["on_time"] < 15 or s["age_ms"]["on_time_outside_0_250"]:
        return "age"
    if s["request_stamp_err_ms"]["median"] < 3.0 * max(abs(err["median"]), 1.0):
        return "contrast"
    return None


def test_the_scan_stamp_is_the_capture_time_on_ground_truth(tmp_path):
    s = measure.run(seconds=3.0, seed=3, tmp_dir=tmp_path)["summary"]
    assert _bars(s) is None, s


def test_a_stamp_a_whole_revolution_stale_is_caught(tmp_path, monkeypatch):
    """Red check: the previous revolution's stamp sent with this scan reads
    as ~100 ms off and fails the bars (before the review's fix it read ~0)."""
    import robot.lidar_ld19 as ld
    real = ld.Ld19Scanner.get_scan

    def stale(self, max_range_m=None):
        out = real(self, max_range_m)
        if "capture_unix" in out:
            out["capture_unix"] -= 1.0 / measure.RATE_HZ
        return out

    monkeypatch.setattr(ld.Ld19Scanner, "get_scan", stale)
    s = measure.run(seconds=2.0, seed=3, tmp_dir=tmp_path)["summary"]
    assert s["err_ms"]["median"] == pytest.approx(-100.0, abs=10.0), s["err_ms"]
    assert _bars(s) == "error"


def test_the_measure_restores_the_simulated_body(tmp_path, monkeypatch):
    """The harness swaps the simulated body's scan for the driver's and
    clears the variables that change the app; a later test in the same
    worker must get both back exactly: set ones set, absent ones absent."""
    from sim.mock_robot import MockRobot
    monkeypatch.setenv("ROBOT_LIDAR_SELF_BOX_CM", "-7.1,3.2,-10.7,8.5")
    monkeypatch.delenv("ROUTE_PREFIX", raising=False)
    before = MockRobot.get_scan
    measure.run(seconds=0.3, tmp_dir=tmp_path)
    assert MockRobot.get_scan is before
    assert os.environ["ROBOT_LIDAR_SELF_BOX_CM"] == "-7.1,3.2,-10.7,8.5"
    assert "ROUTE_PREFIX" not in os.environ
