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

Bars are looser than the 60 s run's (criterion 1: median 1.1 ms, max 2.6 ms
on 2026-11-28) so a loaded xdist worker does not fail them by scheduling.
"""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("scan_stamp_3236",
                                               ROOT / "evaluations/scan-stamp-3236/measure.py")
measure = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(measure)


def test_the_scan_stamp_is_the_capture_time_on_ground_truth(tmp_path):
    res = measure.run(seconds=3.0, seed=3, tmp_dir=tmp_path)
    s = res["summary"]
    assert s["unusable"] == 0 and s["not_capture"] == 0, s
    assert s["distinct_revolutions"] >= 25, s
    assert s["one_stamp_per_revolution"], s
    err = s["err_ms"]
    assert abs(err["median"]) <= 10.0 and err["max_abs"] <= 50.0, err
    # The age handed to the bridge is honest: never in the future, never stale.
    assert s["age_ms"]["outside_0_250"] == 0, s["age_ms"]
    # The request stamp (before 3.221) would be at least 3x further off.
    req = s["request_stamp_err_ms"]["median"]
    assert req >= 3.0 * max(abs(err["median"]), 1.0), (req, err)


def test_the_measure_restores_the_simulated_body(tmp_path, monkeypatch):
    """The harness swaps the simulated body's scan for the driver's and
    clears three variables; a later test in the same worker must get both
    back."""
    from sim.mock_robot import MockRobot
    monkeypatch.setenv("ROBOT_LIDAR_SELF_BOX_CM", "-7.1,3.2,-10.7,8.5")
    before = MockRobot.get_scan
    measure.run(seconds=0.3, tmp_dir=tmp_path)
    assert MockRobot.get_scan is before
    import os
    assert os.environ["ROBOT_LIDAR_SELF_BOX_CM"] == "-7.1,3.2,-10.7,8.5"
