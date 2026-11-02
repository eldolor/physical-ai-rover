# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.131 criterion 1's sweep (slow tier, SLOW_FILES): every `room_features()`
call explore makes under `explore_goal_policy: rooms` in in-process missions
in the four simulator houses and 20 ProcTHOR houses -- each handed the
choice's `goal_context()` -- gives exactly the frozen pre-3.131 answer
(`tests/room_features_reference.py`)."""
import pytest

import brain.goal_policy as G
from tests import room_features_reference as ref
from tests.test_explore import _mission
from tests.test_frontier_3130_sweep import HOUSES


@pytest.mark.parametrize("house", HOUSES)
def test_every_room_features_call_a_mission_makes_gives_the_old_answer(house, monkeypatch):
    calls, with_ctx, bad = [0], [0], []
    real = G.room_features

    def spy(m, pose, goals, *, trail, types, prior, ctx=None):
        got = real(m, pose, goals, trail=trail, types=types, prior=prior, ctx=ctx)
        want = ref.room_features(m, pose, goals, trail=list(trail), types=types, prior=prior)
        if got != want:
            bad.append(calls[0])
        calls[0] += 1
        with_ctx[0] += ctx is not None
        return got
    monkeypatch.setattr(G, "room_features", spy)
    _mission(house=house, max_steps=60, explore_goal_policy="rooms")
    assert calls[0] and with_ctx[0] == calls[0], (calls, with_ctx)
    assert bad == []
