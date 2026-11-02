# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.130 criterion 1's sweep (slow tier, SLOW_FILES): every `reach`,
`find_frontiers` and `find_view_gaps` call explore makes in in-process
missions in the four simulator houses and 20 ProcTHOR houses gives exactly
`tests/frontier_dev.py`'s answer, and the six live SLAM maps (20 poses each,
the camera's seen cells from 8, 30 and 100 poses) give exactly the frozen
pure-Python reference's (`frontier_compare.differences`)."""
import random

import pytest

import brain.explore as E
from sim.maps import procthor
from tests import frontier_compare as fc
from tests import frontier_dev as dev
from tests.test_explore import _mission


@pytest.mark.parametrize("path", fc.MAPS, ids=lambda p: p.name.split(".")[0])
def test_live_slam_maps_with_larger_seen_sets_give_the_old_answers(path):
    rng = random.Random(f"3130:{path.name}")
    m = fc.load(path)
    seens = (fc.seen_set(m, 8, rng), fc.seen_set(m, 30, rng), fc.seen_set(m, 100, rng))
    bad = []
    for i, (x, y) in enumerate(fc.poses(m, 20, rng)):
        bad += [(i, d) for d in fc.differences(m, x, y, fc.LEAVE, seens)]
    assert bad == []


HOUSES = ["starter_house", "scaled_house", "home_first_floor", "complex_house"] + [
    f"procthor:val:{i}" for i in procthor.usable()[:20]]


@pytest.mark.parametrize("house", HOUSES)
def test_every_call_a_mission_makes_gives_devs_answer(house, monkeypatch):
    """Each call's answer is compared with dev's as it is made, so the
    mission runs on the new code throughout."""
    calls = {"reach": 0, "find_frontiers": 0, "find_view_gaps": 0}
    bad = []
    real = {name: getattr(E, name) for name in calls}

    def spy(name):
        def call(m, x, y, *a, **kw):
            got = real[name](m, x, y, *a, **kw)
            if "reach_map" in kw:                   # dev's own reach, same pose
                kw = {**kw, "reach_map": None}
            want = getattr(dev, name)(m, x, y, *a, **kw)
            if name == "reach":
                same = (got.order.tolist(), got.dist.tolist(), got.start, got.need) == \
                    (want.order.tolist(), want.dist.tolist(), want.start, want.need)
            else:
                same = [vars(v) for v in got] == [vars(v) for v in want]
            if not same:
                bad.append((name, calls[name]))
            calls[name] += 1
            return got
        return call
    for name in calls:
        monkeypatch.setattr(E, name, spy(name))
    _mission(house=house, max_steps=60)
    assert calls["reach"] and calls["find_frontiers"], calls
    assert bad == []
