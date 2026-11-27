# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.225: the home trip's battery reserve reads the pack, not the motors'
sag. Criteria 1-2 (`docs/plans/ros-alignment/3.225-battery-sag-reserve.md`)."""
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
    """Replay (t, v) as the runner does (each reading observed, then the
    reserve asked); the first battery reserve as (t, reason), or None."""
    b = FixedTrip(t_home_s)
    for t, v in samples:
        if until_s is not None and t > until_s:
            break
        b.observe_battery(v, t)
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
