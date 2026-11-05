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
    """Returns placed on the corridor's own edges: the front edge, the side
    margin, inside the outline, behind."""
    ranges = [None] * 360
    hl, hw = safety.FOOTPRINT_LENGTH_M * 50.0, safety.FOOTPRINT_WIDTH_M * 50.0
    for _ in range(rng.randrange(1, 8)):
        x = rng.choice([hl, hl + 1e-9, hl - 1e-9, rng.uniform(-60, 60)])
        y = rng.choice([hw + safety.FOOTPRINT_SIDE_MARGIN_CM, hw, rng.uniform(-30, 30)])
        x -= safety.LIDAR_X_M * 100.0
        r = math.hypot(x, y) / 100.0
        a = round(math.degrees(math.atan2(-y, x)))
        ranges[(a + 180) % 360] = r
    return ranges, 0.0, rng.uniform(-2, 2), (rng.uniform(-8, 8), rng.uniform(-8, 8))


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
