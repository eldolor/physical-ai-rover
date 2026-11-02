# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.131 criterion 1 (fast tier): `room_features()` and `coarse()` handed a
choice's `goal_context()` -- explore's one map conversion a choice -- give
exactly the frozen pre-3.131 answers (`tests/room_features_reference.py`),
as do the new functions without a context: on 3.96's benchmark (the six live
SLAM maps, 20 poses, every candidate a choice would score) and on random
grids. The slow sweep (`test_room_features_ctx_sweep.py`) covers every call
explore makes in 24 in-process missions."""
import random

import numpy as np
import pytest

import brain.goal_policy as G
from brain import frontier as fr
from tests import frontier_compare as fc
from tests import room_features_reference as ref
from world.interface import CELL_FREE

PRIOR = {"kitchen": 0.3, "bedroom": 0.1, "office": 0.0}
TYPES = (None, "kitchen", "bedroom", "office")


def _three_ways(m, pose, goals, trail, types, prior):
    """The frozen answer, the new one without a context, and with one."""
    kw = dict(trail=trail, types=types, prior=prior)
    return (ref.room_features(m, pose, goals, **kw),
            G.room_features(m, pose, goals, **kw),
            G.room_features(m, pose, goals, ctx=G.goal_context(m), **kw))


def _trail(m, x, y, n=30):
    """The 30 free cells nearest the pose, as 3.96's benchmark draws the
    brain's trail (`time_choice.trail_near`)."""
    w, res, ox, oy = m["width"], m["resolution_m"], m["origin_x_m"], m["origin_y_m"]
    free = np.flatnonzero(np.asarray(m["cells"]) == CELL_FREE)
    px, py = ox + (free % w + 0.5) * res, oy + (free // w + 0.5) * res
    near = np.argsort((px - x) ** 2 + (py - y) ** 2, kind="stable")[:n]
    return list(zip(px[near].tolist(), py[near].tolist()))


@pytest.mark.parametrize("path", fc.MAPS, ids=lambda p: p.name.split(".")[0])
def test_the_benchmark_maps_give_the_old_room_features(path):
    rng = random.Random(f"3131:{path.name}")
    m = fc.load(path)
    want = ref.coarse(m)
    assert G.coarse(m) == want
    assert G.coarse(m, cells=G.goal_context(m)["cells"]) == want
    for x, y in fc.poses(m, 20, rng):
        r = fr.reach(m, x, y, clearance_m=fc.LEAVE)
        goals = [f.goal for f in fr.find_frontiers(m, x, y, clearance_m=fc.LEAVE, reach_map=r)]
        goals += [g.goal for g in fr.find_view_gaps(m, x, y, set(), clearance_m=fc.LEAVE,
                                                    reach_map=r)]
        if not goals:
            continue
        types = [rng.choice(TYPES) for _ in goals]
        pose = {"x_m": x, "y_m": y, "heading_deg": rng.uniform(0, 360)}
        a, b, c = _three_ways(m, pose, goals, _trail(m, x, y), types, PRIOR)
        assert a == b == c, (x, y)


def _random_map(rng):
    res = rng.choice((0.05, 0.10, 0.15, 0.30, 0.60))
    w, h = rng.randint(1, 90), rng.randint(1, 90)
    vals = rng.choice(((-1, 0, 100), (-1, 0, 100, 50, 7), (-1, 0, 100, 255), (0,), (-1,)))
    return {"width": w, "height": h, "resolution_m": res,
            "origin_x_m": rng.uniform(-3, 3), "origin_y_m": rng.uniform(-3, 3),
            "cells": [rng.choice(vals) for _ in range(w * h)]}


def test_random_grids_give_the_old_room_features():
    rng = random.Random(3131)
    for i in range(200):
        m = _random_map(rng)
        assert G.coarse(m) == ref.coarse(m), i
        assert G.coarse(m, cells=G.goal_context(m)["cells"]) == ref.coarse(m), i
        ox, oy, W, H = m["origin_x_m"], m["origin_y_m"], m["width"] * m["resolution_m"], \
            m["height"] * m["resolution_m"]
        goals = [(ox + rng.uniform(-1, W + 1), oy + rng.uniform(-1, H + 1))
                 for _ in range(rng.randint(1, 12))]
        trail = [(ox + rng.uniform(0, W), oy + rng.uniform(0, H)) for _ in range(rng.randint(0, 30))]
        types = [rng.choice(TYPES) for _ in goals]
        pose = {"x_m": goals[0][0], "y_m": goals[0][1]}
        a, b, c = _three_ways(m, pose, goals, trail, types, PRIOR)
        assert a == b == c, i


def test_a_context_from_another_map_is_refused():
    m = fc.load(fc.MAPS[0])
    other = {**m, "cells": list(m["cells"])}        # equal, but another object
    with pytest.raises(ValueError):
        G.room_features(m, {"x_m": 0.0, "y_m": 0.0}, [(0.0, 0.0)], trail=[], types=[None],
                        prior={}, ctx=G.goal_context(other))


def test_the_policy_hands_its_context_through(monkeypatch):
    """`RoomGoalPolicy.extend(ctx=...)` reaches `room_features()` (explore's
    path) and changes no answer."""
    m = fc.load(fc.MAPS[1])
    x, y = fc.poses(m, 1, random.Random(1))[0]
    goals = [f.goal for f in fr.find_frontiers(m, x, y, clearance_m=fc.LEAVE)]
    pose = {"x_m": x, "y_m": y, "heading_deg": 0.0}
    feats = [[0.0] * len(G.FEATURES) for _ in goals]
    pol = G.goal_policy_for("rooms")
    target = "knife"
    kw = dict(trail=_trail(m, x, y), types=["kitchen"] * len(goals), target=target)
    seen = []
    real = G.room_features

    def spy(*a, **k):
        seen.append(k.get("ctx"))
        return real(*a, **k)
    monkeypatch.setattr(G, "room_features", spy)
    ctx = G.goal_context(m)
    with_ctx = pol.extend(m, pose, goals, feats, ctx=ctx, **kw)
    without = pol.extend(m, pose, goals, feats, **kw)
    assert seen == [ctx, None]
    assert with_ctx == without
