# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.161.B's numbers, pinned (docs/plans/ros-alignment/3.161-no-go-zones-on-twin.md
Z1-Z5, Z10). The sweeps themselves are evaluations/nogo-3161/sweep.py; their
summary is evaluations/nogo-3161/summary.json (score.py). This file holds the
summary to the bars as recorded, and re-runs a slice of the sweep through the
real path so a regression in explore's zone check fails the suite."""

import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SUMMARY = json.loads((ROOT / "evaluations/nogo-3161/summary.json").read_text())
_spec = importlib.util.spec_from_file_location("nogo_sweep", ROOT / "evaluations/nogo-3161/sweep.py")


@pytest.fixture(scope="module")
def sweep():
    mod = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(mod)
    return mod


def test_z1_no_goal_into_a_zone():
    z1 = SUMMARY["Z1"]
    assert z1["goals"] > 2000
    assert z1["in_zone"] == 0 and z1["in_zone_grown_012"] == 0


def test_z2_no_entry_under_autonomy():
    assert SUMMARY["Z2"]["entries"] == 0


def test_z3_every_robot_started_in_a_zone_leaves_and_the_zone_returns():
    z3 = SUMMARY["Z3"]
    assert z3["n"] == 10 and z3["left"] == 10 and z3["restored_within_030_of_clearing"] == 10


def test_z5_survey_coverage_outside_zones_within_a_point():
    assert SUMMARY["Z5_survey"]["houses"] == 12
    assert SUMMARY["Z5_survey"]["worst_points_below"] >= -1.0


def test_z5_find_rate_as_recorded():
    """Recorded, not met: the find rate is not equal (the plan's 3.161.B
    record). Pinned so the number cannot drift unseen."""
    f = SUMMARY["Z5_find"]
    assert (f["found_zones"], f["found_none"], f["n"]) == (82, 87, 96)
    assert f["median_ratio"] is not None and abs(f["median_ratio"] - 1) <= 0.05


def test_z10_no_zones_is_devs_mission_for_mission():
    z10 = SUMMARY["Z10"]
    assert z10["compared"] == 96 and z10["identical"] == 96, z10["differ"]


@pytest.mark.parametrize("house,seed", [("scaled_house", 0), ("procthor:val:0", 1)])
def test_a_slice_of_the_sweep_sends_no_goal_into_a_zone(sweep, house, seed):
    r = sweep.run(house, seed, "zones")
    assert r["outcome"] != "exception", r.get("tb")
    assert r["goals"] > 0 and len(r["zones"]) == 3
    assert r["goals_in_grown"] == [] and r["entries"] == []


def test_a_slice_of_the_leave_sweep(sweep):
    r = sweep._leave("procthor:val:0")
    restored = [e for e in r["suspend_events"] if e[0] == "restored"]
    assert r["left"] and restored and restored[0][2] <= 0.60


def test_the_live_run_as_recorded():
    live = SUMMARY["live"]
    assert live["z6_refused"] == 10 and live["goals_in_zone"] == []
    assert live["mission"]["outcome"] == "found" and live["mission"]["entries"] == []
    assert live["bridge_after_draw"]["zones"] == 3 and live["bridge_after_mission"]["zones"] == 3
    assert live["store_after_mission"] == 3
    z3 = live["z3"]
    assert z3["left_after_s"] is not None and z3["restored"]["clear_m"] <= 0.60
