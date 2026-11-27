# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.225: the home trip's battery reserve reads the pack, not the motors'
sag. Criteria 1-2 and 5-6 (`docs/plans/ros-alignment/3.225-battery-sag-reserve.md`;
criterion 7 is in `tests/test_battery_ladder.py`)."""
import json
from pathlib import Path

import pytest

from brain.dock import K, REST_WINDOW_S, HomeBudget

TRACES = Path(__file__).resolve().parents[1] / "evaluations/battery-sag-3225/survey-traces-20261126.json"
FLOOR_V = 10.5
DOCK = {"x_m": 0.0, "y_m": 0.0, "theta": 90.0, "source": "start"}
POSE = {"usable": True, "x_m": 2.0, "y_m": 0.0}     # away from the dock: a reserve is possible


class FixedTrip(HomeBudget):
    """`HomeBudget` with the trip home's time held at the run's own value,
    so only the battery half is under test."""

    def __init__(self, t_home_s: float):
        super().__init__(dock=DOCK)
        self._t = t_home_s

    def t_home(self, L: float) -> float:
        return self._t


def first_battery_reserve(samples, t_home_s, until_s=None):
    """Replay (t, v) or (t, v, at_rest) as the runner does (each reading
    observed, then the reserve asked); the first battery reserve as
    (t, reason), or None."""
    b = FixedTrip(t_home_s)
    for t, v, *rest in samples:
        if until_s is not None and t > until_s:
            break
        b.observe_battery(v, t, rest[0] if rest else None)
        why = b.reserve_reason(lambda: None, POSE, t, None, v, FLOOR_V)
        if why and why.startswith("battery"):
            return t, why
    return None


def search_samples(run):
    """The search's samples and the one the end was decided on (the first
    row the trace shows returning: the trace is 5 s, the decision earlier)."""
    rows = run["samples"]
    n = next(i for i, r in enumerate(rows) if r[2] != "search")
    return [(t, v) for t, v, _ in rows[:n + 1]]


@pytest.mark.parametrize("name", ["survey_2009", "survey_2015"])
def test_tonights_surveys_would_not_have_ended(name):
    """Criterion 1: 3.140's two surveys (2026-11-26) ended on one sagged
    sample read as a 22 V/h drain; at the run's own T_home no battery
    reserve now, at any sample of the search."""
    run = json.loads(TRACES.read_text())[name]
    assert first_battery_reserve(search_samples(run), run["t_home_s"]) is None


def rested(t):
    return 11.2 - 0.002 * t


def test_a_real_drain_still_ends_the_search_on_time():
    """Criterion 2: a pack falling 0.002 V/s, read every 12 s with a 0.5 V
    sag on alternate readings. The reserve fires no later than
    REST_WINDOW_S + 12 s after the noise-free prediction crosses 10.6 V,
    and not before it crosses 10.7 V."""
    samples = [(t, round(rested(t) - (0.5 if i % 2 else 0.0), 3))
               for i, t in enumerate(range(0, 1200, 12))]
    t_home = 60.0

    def crosses(level):
        return next(t / 10 for t in range(12000) if rested(t / 10) - K * 0.002 * t_home < level)

    hit = first_battery_reserve(samples, t_home)
    assert hit is not None
    assert crosses(10.7) <= hit[0] <= crosses(10.6) + REST_WINDOW_S + 12, (hit, crosses(10.6))


def test_rested_then_driving_without_a_pause_is_not_a_drain():
    """Criterion 2, the known limit: rested 11.8 V for 30 s, then every
    reading 0.4 V low (driving), the true drain 0.0006 V/s: no reserve
    within 1,200 s."""
    samples = [(t, round(11.8 - 0.0006 * t - (0.4 if t >= 30 else 0.0), 3))
               for t in range(0, 1200, 12)]
    assert first_battery_reserve(samples, 60.0) is None


def test_the_rested_voltage_is_the_windows_highest_reading():
    b = HomeBudget(DOCK)
    assert b.rested_v() is None
    b.observe_battery(11.8, 0.0)
    b.observe_battery(11.13, 5.0)                   # a sagged reading
    assert b.rested_v() == 11.8
    b.observe_battery(11.5, 70.0)                   # 11.8 has left the window
    assert b.rested_v() == 11.5


def test_driving_without_a_pause_is_not_a_drain():
    """Criterion 5b (amendment 2; 5 as written was mis-posed: its own pack
    reaches the reserve at ~910 s). Readings at rest at 11.2 V for 10 s,
    then every reading 0.5 V low and not at rest, the true drain 0.0006
    V/s: no reserve before the noise-free prediction crosses 10.65 V, one
    within 12 s after it crosses 10.6 V. The window rule alone (no
    `at_rest`) ended this at 114 s."""
    samples = [(t, round(11.2 - 0.0006 * t - (0.5 if t > 10 else 0.0), 3), t <= 10)
               for t in range(0, 1200, 6)]

    def crosses(level):
        return next(t / 10 for t in range(12000)
                    if 11.2 - 0.0006 * t / 10 - K * 0.0006 * 60.0 < level)

    hit = first_battery_reserve(samples, 60.0)
    assert hit is not None
    assert crosses(10.65) <= hit[0] <= crosses(10.6) + 12, (hit, crosses(10.6))


def test_a_real_drain_at_rest_ends_the_search_within_one_sample():
    """Criterion 6: criterion 2's pack with its sagged readings marked not
    at rest: the reserve fires within 12 s after the noise-free prediction
    crosses 10.6 V, never before it crosses 10.65 V."""
    samples = [(t, round(rested(t) - (0.5 if i % 2 else 0.0), 3), not i % 2)
               for i, t in enumerate(range(0, 1200, 12))]

    def crosses(level):
        return next(t / 10 for t in range(12000) if rested(t / 10) - K * 0.002 * 60.0 < level)

    hit = first_battery_reserve(samples, 60.0)
    assert hit is not None
    assert crosses(10.65) <= hit[0] <= crosses(10.6) + 12, (hit, crosses(10.6))


def test_the_rested_voltage_at_rest_is_projected_to_now():
    """Amendment 1: the latest reading at rest, carried forward by the
    drain; the window ends at the time asked, not the last sample."""
    b = HomeBudget(DOCK)
    b.observe_battery(11.6, 0.0, True)
    b.observe_battery(11.0, 30.0, False)            # a sag: never counted
    b.observe_battery(11.48, 60.0, True)
    assert b.drain_v_s(60.0) == pytest.approx(0.002)
    assert b.rested_v(90.0) == pytest.approx(11.48 - 0.002 * 30)


def test_a_fresh_rate_after_a_charge_is_measured_not_inherited():
    """Review 1: the recharge test passes on its prior rate by chance;
    here the new span's rate differs from the prior one."""
    b = HomeBudget(DOCK)
    b.observe_battery(12.0, 0.0)
    b.observe_battery(11.4, 60.0)
    assert b.drain_v_s() == pytest.approx(0.01)
    b.recharged()
    b.observe_battery(12.2, 100.0)
    b.observe_battery(12.1, 200.0)
    assert b.drain_v_s() == pytest.approx(0.001)


def test_a_real_drain_while_driving_without_a_pause_still_ends_the_search():
    """Review 2 (both reviewers): rested once at 11.2 V, then driving with no
    pause while the pack really falls 0.002 V/s, every reading 0.5 V low and
    not at rest. Loaded readings measured against loaded ones see the drain:
    the reserve fires before the rested pack is predicted under the floor."""
    def true_v(t):
        return 11.2 - 0.002 * t
    samples = [(t, round(true_v(t) - (0.5 if t > 10 else 0.0), 3), t <= 10)
               for t in range(0, 1500, 6)]
    hit = first_battery_reserve(samples, 60.0)
    assert hit is not None
    assert true_v(hit[0]) - K * 0.002 * 60.0 >= 10.5, hit


def test_a_body_that_does_not_say_windows_on_its_last_reading():
    """Review 2: with no `at_rest`, a repeated sagged value is no new time:
    the window ends at the last distinct reading, not at the clock."""
    b = HomeBudget(DOCK)
    b.observe_battery(11.80, 0.0)
    b.observe_battery(11.13, 5.0)
    b.observe_battery(11.13, 80.0)                  # the same measurement: dropped
    assert b.rested_v(80.0) == 11.80
