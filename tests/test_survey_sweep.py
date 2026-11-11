# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_survey_sweep.py -- 3.70 S1/S2's sim numbers, pinned (about 80 s
per mode; in tools/plan_section.py's SLOW_FILES).

The 9 surveys of `tests/survey_sweep.py` (3 spread starts in each named
house, FakeNav for nav2). Recorded 2026-10-13 in 3.70's "Records: S1-S3":
* every survey ends `exhausted` with coverage >= 95% (S1, met);
* `rooms`: label calls median 10, max 19 -- never past the cap of 20;
  rooms >= 95% labelled 22 of 90 (S2 FAILED; pinned as no regression);
* `spacing` (amendment 1: asked again through every doorway, labels stop
  at doorways): calls median 29, max 44; rooms >= 95% labelled 46 of 90
  (36 before; S2 still FAILED, pinned as no regression).
"""

import statistics

import pytest

from tests import survey_sweep as sw


@pytest.fixture(scope="module", params=["rooms", "spacing"])
def rows(request):
    return request.param, [sw.run(h, st, mode=request.param)
                           for h in sw.HOUSES for st in sw.starts(h)]


def test_every_sim_survey_ends_exhausted_with_the_house_mapped(rows):
    _mode, rs = rows
    assert [r["outcome"] for r in rs] == ["exhausted"] * 9, rs
    assert all(r["coverage"] >= 0.95 for r in rs), [r["coverage"] for r in rs]


def test_label_calls_and_where_labels_land_do_not_regress(rows):
    mode, rs = rows
    calls = [r["label_calls"] for r in rs]
    good = sum(v >= 0.95 for r in rs for v in r["per_room"].values())
    if mode == "rooms":
        # 3.173.B amendment 3 (the orchestrator, 2026-11-11): median 10 -> 11 with the
        # small-room regions (off 10/19, good 22; on 11/18, good 24).
        assert statistics.median(calls) <= 11 and max(calls) <= 20, calls
        assert good >= 22, good
    else:
        # 3.173.B (the orchestrator, 2026-11-10): the median was 29, the small-room goal's
        # arm off exactly on it (off 29/44, good 46; on 31/44, good 48): the two calls are
        # home_first_floor's pantry and laundry, newly labelled, and calls are free under
        # the default `frame` source. A guard on cost, not a 3.173 bar.
        # Amendment 3 (the orchestrator, 2026-11-11): the SECOND raise in 3.173, 29 -> 31
        # -> 34, max 44 -> 45 (off 29/44, good 46; on 34/45, good 45). Good rooms 46 -> 45
        # is a regression accepted for now, recorded in 3.173's file, not hidden.
        assert statistics.median(calls) <= 34 and max(calls) <= 45, calls
        assert good >= 45, good
