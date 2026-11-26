# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_lidar_self_box.py

PLAN-ros-alignment.md 3.218: the Rover's D500 sees the pan-tilt mast right
behind it (2.3-14 cm, the rear quarter, every scan). A measured self box in
the safety layer lets FORWARD ignore those returns; REVERSE stays refused,
as `astern_occluded`, because the mast hides what is behind. Criteria 1-6
are here; 7 is on the Rover. The recorded returns are
`evaluations/lidar-self-box-3218/selfhits-20261126.json` (only those under
30 cm; never the room).
"""

import json
import math
import random
from pathlib import Path

import pytest

from robot.safety import (FOOTPRINT_LENGTH_M, FOOTPRINT_WIDTH_M, LIDAR_X_M,
                          SELF_BOX_EDGE_MIN_CM, SafetyController, self_box)

ROVER_BOX = (-7.1, 3.2, -10.7, 8.5)
RECORD = Path(__file__).resolve().parents[1] / "evaluations/lidar-self-box-3218/selfhits-20261126.json"


class _Body:
    """A body with a lidar scan and nothing else, wheels reporting (so a
    blind astern reads as blind, not as a phone walk)."""

    def __init__(self, ranges):
        self.ranges = ranges

    def get_scan(self, max_range_m=None):
        return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
                "range_min_m": 0.02, "range_max_m": 12.0, "ranges_m": list(self.ranges)}

    def get_wheel_state(self):
        return {"usable": True}

    def get_depth_grid(self):
        return {"usable": False}

    def get_distance(self):
        return 0.0


def _recorded(which):
    rows = json.loads(RECORD.read_text())["under_30cm"]
    ranges = [None] * 360
    for a, v in rows.items():
        ranges[int(a) + 180] = v[which] / 100.0
    return ranges


def _body_point(r_m, deg):
    a = math.radians(deg)
    return r_m * 100 * math.cos(a) + LIDAR_X_M * 100, -r_m * 100 * math.sin(a)


# ---- criterion 1: every recorded self return is inside the box --------------

@pytest.mark.parametrize("which", ["median_cm", "max_cm"])
def test_every_recorded_self_return_is_inside_the_box(which):
    x0, x1, y0, y1 = ROVER_BOX
    ranges = _recorded(which)
    hits = [(i - 180, r) for i, r in enumerate(ranges) if r is not None]
    assert len(hits) == 92
    for deg, r in hits:
        x, y = _body_point(r, deg)
        assert x0 <= x <= x1 and y0 <= y <= y1, (deg, r, x, y)
    safety = SafetyController(_Body(ranges), self_box_cm=ROVER_BOX)
    assert safety.footprint_clearance(+1) == (None, "scan_footprint_no_target")
    # Without the box the same scan blocks every forward move (the bug).
    assert SafetyController(_Body(ranges)).footprint_clearance(+1) == (0.0, "scan_footprint")


# ---- criterion 2: the box is inside the outline -----------------------------

def test_the_rover_box_is_inside_the_outline_by_the_margin():
    hl, hw = FOOTPRINT_LENGTH_M * 50, FOOTPRINT_WIDTH_M * 50
    x0, x1, y0, y1 = self_box(ROVER_BOX)
    assert min(hl + x0, hl - x1, hw + y0, hw - y1) >= SELF_BOX_EDGE_MIN_CM


@pytest.mark.parametrize("bad", [(-7.1, 3.2, -11.2, 8.5), (-12.5, 0, 0, 1), (1, 0, 0, 1),
                                 (0, 1, 0, float("nan")), (0, 1, 2), "a,b,c,d"])
def test_a_box_outside_the_outline_or_malformed_is_refused(bad):
    with pytest.raises(ValueError):
        SafetyController(_Body([None] * 360), self_box_cm=bad)


def test_the_box_reads_from_a_string_and_unset_is_none():
    assert self_box("-7.1,3.2,-10.7,8.5") == ROVER_BOX
    assert self_box(None) is None and self_box("") is None


# ---- criterion 3: nothing else changes --------------------------------------

def test_scans_with_nothing_in_the_box_give_identical_answers():
    rng = random.Random(3218)
    x0, x1, y0, y1 = ROVER_BOX
    for trial in range(10_000):
        ranges = []
        for deg in range(-180, 180):
            r = None if rng.random() < 0.3 else rng.choice(
                (rng.uniform(0.02, 0.6), rng.uniform(0.6, 4.0)))
            if r is not None:
                x, y = _body_point(r, deg)
                if x0 <= x <= x1 and y0 <= y <= y1:
                    r = None
            ranges.append(r)
        plain = SafetyController(_Body(ranges))
        boxed = SafetyController(_Body(ranges), self_box_cm=ROVER_BOX)
        for d in (+1, -1):
            assert plain.footprint_clearance(d) == boxed.footprint_clearance(d), (trial, d)
        assert plain.reverse_clearance() == boxed.reverse_clearance(), trial
        omega = rng.choice((-1.2, -0.4, 0.4, 1.2))
        assert plain.pivot_scale(omega) == boxed.pivot_scale(omega), trial


# ---- criterion 4: reverse stays refused, as what it is ----------------------

def test_reverse_is_refused_as_occluded_while_the_mast_is_seen():
    safety = SafetyController(_Body(_recorded("median_cm")), self_box_cm=ROVER_BOX)
    assert safety.reverse_clearance() == (0.0, "astern_occluded")


# ---- criterion 5: turns are not blocked by the mast -------------------------

@pytest.mark.parametrize("box", [None, ROVER_BOX])
def test_turns_are_not_blocked_by_the_mast(box):
    safety = SafetyController(_Body(_recorded("max_cm")), self_box_cm=box)
    for omega in (-1.2, -0.4, 0.4, 1.2):
        assert safety.pivot_scale(omega)[0] == 1.0


# ---- a real obstacle ahead still stops a forward move ------------------------

def test_an_object_ahead_still_blocks_forward_with_the_box():
    ranges = _recorded("median_cm")
    ranges[180] = (FOOTPRINT_LENGTH_M * 50 - LIDAR_X_M * 100 + 10) / 100   # 10 cm past the bumper
    safety = SafetyController(_Body(ranges), self_box_cm=ROVER_BOX)
    cm, source = safety.footprint_clearance(+1)
    assert source == "scan_footprint" and cm == pytest.approx(10.0, abs=0.2)
