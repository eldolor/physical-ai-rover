# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.141 criterion 1: `pivot_scale()`'s search with its per-scan work done
once (`robot/safety.py` `_PivotRows`) gives exactly dev's answers -- the
frozen `tests/pivot_reference.py` -- on random scans around the chassis,
and the comparison catches planted bugs. The 200,000-case run is
`evaluations/wheel-loop-3141/equivalence.py`, on these same cases."""

import math
import random

import numpy as np
import pytest

import robot.safety as safety
from robot.safety import PIVOT_MARGIN_CM, SafetyController
from tests.pivot_reference import ReferenceSafety

R, TRACK = 0.04, 0.17


class Body:
    def __init__(self, ranges, missed, lr=(0.0, 0.0)):
        self.ranges, self.missed, self.lr = ranges, missed, lr

    def get_wheel_state(self):
        return {"usable": True, "wheel_radius_m": R, "track_width_m": TRACK,
                "left": {"velocity_rad_s": self.lr[0]}, "right": {"velocity_rad_s": self.lr[1]}}

    def get_scan(self, max_range_m=None):
        return scan_of(self.ranges)

    def get_depth_grid(self):
        return {"usable": False}

    def sensor_age_s(self):
        return 0.08

    def turn_since_scan(self):
        return self.missed


def scan_of(ranges):
    return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
            "ranges_m": list(ranges)}


def _box(half_x, half_y):
    out = []
    for i in range(360):
        a = math.radians(-180 + i)
        c, s = math.cos(a), math.sin(a)
        t = min((half_x - 0.04) / c if c > 1e-9 else math.inf,
                (-half_x - 0.04) / c if c < -1e-9 else math.inf,
                half_y / abs(s) if abs(s) > 1e-9 else math.inf)
        out.append(round(t, 3) if t < 8 else None)
    return out


def case(rng: random.Random):
    """One random scan, aged arc and command: rooms from tight (the chassis
    pressed into a corner) to open, clutter near the body, gaps, and
    returns placed at the chassis' margin and at PIVOT_NEAR_CM's edge."""
    hx = rng.choice([rng.uniform(0.127, 0.16), rng.uniform(0.15, 0.30), rng.uniform(0.15, 0.30),
                     rng.uniform(0.3, 3.0)])
    hy = rng.choice([rng.uniform(0.116, 0.15), rng.uniform(0.14, 0.30), rng.uniform(0.14, 0.30),
                     rng.uniform(0.3, 3.0)])
    ranges = _box(hx, hy)
    hl, hw = safety.FOOTPRINT_LENGTH_M * 50.0, safety.FOOTPRINT_WIDTH_M * 50.0
    for _ in range(rng.randrange(0, 6)):
        # A return a chosen distance from the chassis outline: on the
        # margin, on the near edge, or anywhere close.
        i = rng.randrange(360)
        gap = rng.choice([PIVOT_MARGIN_CM, safety.PIVOT_NEAR_CM, rng.uniform(0.0, 25.0),
                          PIVOT_MARGIN_CM + rng.uniform(0.0, 3.0), PIVOT_MARGIN_CM + rng.uniform(0.0, 3.0)])
        gap += rng.choice([0.0, 1e-9, -1e-9, rng.uniform(-0.05, 0.05)])
        a = math.radians(-180 + i)
        # Walk out from the lidar until the distance to the rectangle is `gap`.
        lo, hi = 0.0, 1.0
        for _ in range(60):
            mid = (lo + hi) / 2
            x = mid * 100 * math.cos(a) + safety.LIDAR_X_M * 100
            y = -mid * 100 * math.sin(a)
            d = math.hypot(max(abs(x) - hl, 0.0), max(abs(y) - hw, 0.0))
            lo, hi = (mid, hi) if d < gap else (lo, mid)
        ranges[i] = hi
    for _ in range(rng.randrange(0, 30)):
        ranges[rng.randrange(360)] = None
    if rng.random() < 0.1:
        ranges[rng.randrange(360)] = rng.choice([math.nan, math.inf])
    missed = 0.0 if rng.random() < 0.4 else rng.uniform(-0.2, 0.2)
    omega = rng.choice([1, -1]) * rng.choice([rng.uniform(0.001, 0.4), rng.uniform(0.4, 4.0)])
    v = rng.choice([0.0, 0.0, rng.uniform(-0.3, 0.3)])
    half = omega * TRACK / 2.0
    lr = ((v - half) / R, (v + half) / R)
    return ranges, missed, omega, lr


def answers(cls, ranges, missed, omega, lr):
    body = Body(ranges, missed, lr)
    s = cls(body)
    scan = scan_of(ranges)
    return (s.pivot_scale(omega, scan), s.pivot_blocked(omega, scan),
            s.pivot_blocked(omega, scan, exact=True), s.pivot_blocked(omega * 0.37, scan, exact=True),
            s.vet_wheel_velocity(*lr))


def differences(n: int, seed: int) -> int:
    rng = random.Random(seed)
    bad = 0
    for _ in range(n):
        c = case(rng)
        if answers(ReferenceSafety, *c) != answers(SafetyController, *c):
            bad += 1
    return bad


def test_same_answers_as_dev():
    assert differences(1000, seed=3141) == 0


def test_cases_reach_the_search():
    """The cases are not all trivial: many turns are clamped, scaled or
    blocked at the first returns, and many pass."""
    rng = random.Random(3141)
    kinds = {"free": 0, "scaled": 0, "stopped": 0}
    for _ in range(600):
        ranges, missed, omega, lr = case(rng)
        scale, _ = SafetyController(Body(ranges, missed, lr)).pivot_scale(omega, scan_of(ranges))
        kinds["free" if scale == 1.0 else "stopped" if scale == 0.0 else "scaled"] += 1
    assert min(kinds.values()) >= 50, kinds


def test_no_scan_and_no_turn_unchanged():
    body = Body([None] * 360, 0.0)
    for cls in (ReferenceSafety, SafetyController):
        s = cls(body)
        assert s.pivot_scale(0.0, scan_of([0.1] * 360)) == (1.0, None)
        assert s.pivot_scale(1.0, {"usable": False}) == (1.0, None)
        assert s.pivot_blocked(1.0, scan_of([None] * 360)) is None


# Planted bugs in the one-pass rows (3.141.E): each must be caught.

class _ArcOffByOne(safety._PivotRows):
    def __init__(self, scan, missed):
        super().__init__(scan, missed * PIVOT_AGE_SAMPLES_SHORT)


PIVOT_AGE_SAMPLES_SHORT = (safety.PIVOT_AGE_SAMPLES - 1) / safety.PIVOT_AGE_SAMPLES


class _DropsNearestRows(safety._PivotRows):
    def __init__(self, scan, missed):
        super().__init__(scan, missed)
        keep = self._nows > 2.0
        self._xs, self._ys, self._nows = self._xs[keep], self._ys[keep], self._nows[keep]


class _FirstChunkClosesOnEqual(safety._PivotRows):
    def first_blocking(self, c, s):
        for x, y, now in zip(self._xs.tolist(), self._ys.tolist(), self._nows.tolist()):
            after = self._dist(c * x - s * y, s * x + c * y)
            if after < PIVOT_MARGIN_CM + 0.05 and after < now - 1e-6:
                return after
        return None


class _ReachWithoutTheChassis(safety._PivotRows):
    def __init__(self, scan, missed):
        super().__init__(scan, missed)
        keep = np.hypot(self._xs, self._ys) <= math.hypot(self._hl, self._hw) + 1.0
        self._xs, self._ys, self._nows = self._xs[keep], self._ys[keep], self._nows[keep]


class _LaterChunksFirst(safety._PivotRows):
    """Rows out of scan order."""
    def __init__(self, scan, missed):
        super().__init__(scan, missed)
        self._xs, self._ys, self._nows = self._xs[::-1].copy(), self._ys[::-1].copy(), \
            self._nows[::-1].copy()


def _first_beams_case():
    """A return at the margin's edge in the scan's first beam (dead
    astern, -180 deg), so the first chunk's exact path decides: a turn
    that would bring the rear edge onto it is scaled."""
    ranges = [3.0] * 360
    ranges[0] = (safety.FOOTPRINT_LENGTH_M / 2 + safety.LIDAR_X_M) + (PIVOT_MARGIN_CM + 0.02) / 100
    ranges[1] = ranges[0] + 0.001
    return ranges, 0.0, 2.5, (-5.3125, 5.3125)


def test_first_chunk_decides_a_turn():
    c = _first_beams_case()
    assert answers(ReferenceSafety, *c) == answers(SafetyController, *c)


@pytest.mark.parametrize("mutant", [_ArcOffByOne, _DropsNearestRows, _LaterChunksFirst,
                                    _FirstChunkClosesOnEqual, _ReachWithoutTheChassis],
                         ids=lambda m: m.__name__)
def test_planted_bugs_are_caught(monkeypatch, mutant):
    monkeypatch.setattr(safety, "_PivotRows", mutant)
    assert differences(300, seed=3141) > 0


def test_rows_of_another_scan_are_not_used():
    near, far = [0.14] * 360, [3.0] * 360
    s = SafetyController(Body(near, 0.0))
    stale = safety._PivotRows(scan_of(far), 0.0)
    assert s.pivot_blocked(3.0, scan_of(near), _rows=stale) is not None


def test_first_chunk_mutant_caught_on_the_first_beams(monkeypatch):
    c = _first_beams_case()
    before = answers(SafetyController, *c)
    monkeypatch.setattr(safety, "_PivotRows", _FirstChunkClosesOnEqual)
    assert answers(SafetyController, *c) != before


def test_a_stale_points_cache_is_caught(monkeypatch):
    """3.141.E: points kept for a scan whose contents changed would be a
    stale answer; a cache keyed without the ranges is caught."""
    cls = safety._ScanPoints

    def of(scan):
        key = (scan.get("angle_min_deg"), scan.get("angle_increment_deg"))
        last = cls._last
        if last is not None and last[0] == key:
            return last[1]
        pts = list(safety.scan_points_cm(scan))
        arr = np.array(pts, dtype=float).reshape(-1, 2)
        val = (pts, arr[:, 0], arr[:, 1])
        cls._last = (key, val)
        return val
    monkeypatch.setattr(cls, "of", staticmethod(of))
    monkeypatch.setattr(cls, "_last", None)
    assert differences(300, seed=3141) > 0
