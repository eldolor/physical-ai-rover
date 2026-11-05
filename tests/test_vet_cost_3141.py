# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.141.E criterion 2: the vet with its scan arithmetic in numpy gives
exactly dev's answers -- the frozen `tests/vet_reference.py` (dev's
corridor loop on top of `tests/pivot_reference.py`'s turn guard) -- and does
its per-scan work once."""

import math
import random

import robot.safety as safety
from robot.safety import SafetyController
from tests.test_pivot_3141 import Body, case, scan_of
from tests.vet_reference import VetReference


def answers(cls, ranges, missed, omega, lr):
    body = Body(ranges, missed, lr)
    s = cls(body)
    scan = scan_of(ranges)
    return (s.vet_wheel_velocity(*lr), s.forward_clearance(), s.reverse_clearance(),
            s.footprint_clearance(+1, scan), s.footprint_clearance(-1, scan),
            s.pivot_scale(omega, scan), s.pivot_blocked(omega, scan))


def corridor_case(rng):
    """Returns placed ON the corridor's own edges (the front edge, the side
    margin, the outline's width), and just either side of them: a beam is
    chosen first, then the range that puts its point at the edge."""
    ranges = [None] * 360
    hl, hw = safety.FOOTPRINT_LENGTH_M * 50.0, safety.FOOTPRINT_WIDTH_M * 50.0
    off = safety.LIDAR_X_M * 100.0
    for _ in range(rng.randrange(1, 8)):
        i = rng.randrange(360)
        a = math.radians(-180.0 + i)
        c, sn = math.cos(a), math.sin(a)
        nudge = rng.choice([0.0, 1e-9, -1e-9, rng.uniform(-3, 3)])
        if rng.random() < 0.5 and abs(c) > 0.2:      # x on the front (or back) edge
            target = rng.choice([hl, -hl]) + nudge
            r = (target - off) / (100.0 * c)
        elif abs(sn) > 0.2:                          # |y| on a side edge
            target = rng.choice([hw, hw + safety.FOOTPRINT_SIDE_MARGIN_CM]) + nudge
            r = target / (100.0 * abs(sn))
        else:
            r = rng.uniform(0.05, 0.6)
        if r > 0:
            ranges[i] = r
    return ranges, 0.0, rng.uniform(-2, 2), (rng.uniform(-8, 8), rng.uniform(-8, 8))


def test_corridor_rooms_equals_the_scalar_rule():
    import numpy as np
    rng = random.Random(1)
    hl, hw = safety.FOOTPRINT_LENGTH_M * 50.0, safety.FOOTPRINT_WIDTH_M * 50.0
    along = [rng.choice([hl, 0.0, -1e-12, rng.uniform(-50, 50)]) + rng.choice([0, 1e-12, -1e-12])
             for _ in range(5000)]
    lateral = [abs(rng.choice([hw, hw + safety.FOOTPRINT_SIDE_MARGIN_CM, rng.uniform(0, 40)])
                   + rng.choice([0, 1e-12, -1e-12])) for _ in range(5000)]
    got = safety._corridor_rooms(np.array(along), np.array(lateral)).tolist()
    want = [safety._corridor_room(a, b) for a, b in zip(along, lateral)]
    assert [math.inf if w is None else w for w in want] == got


def test_a_scan_without_ranges_still_raises():
    import pytest
    s = SafetyController(Body([None] * 360, 0.0))
    with pytest.raises((KeyError, TypeError)):
        s.footprint_clearance(+1, {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0})


def test_same_answers_as_dev():
    rng = random.Random(3141)
    for _ in range(1500):
        c = case(rng)
        assert answers(VetReference, *c) == answers(SafetyController, *c), c
    for _ in range(1500):
        c = corridor_case(rng)
        assert answers(VetReference, *c) == answers(SafetyController, *c), c


def test_the_scan_is_converted_once_per_scan(monkeypatch):
    calls = []
    real = safety.scan_points_cm

    def spy(scan):
        calls.append(id(scan))
        return real(scan)
    monkeypatch.setattr(safety, "scan_points_cm", spy)
    rng = random.Random(7)
    for _ in range(50):
        ranges, missed, omega, lr = case(rng)
        calls.clear()
        SafetyController(Body(ranges, missed, lr)).vet_wheel_velocity(*lr)
        assert len(calls) <= 1, f"{len(calls)} conversions of one scan in one vet"
