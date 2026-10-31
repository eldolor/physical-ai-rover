"""3.130: `brain/frontier.py`'s map work gives exactly the answers of
`tests/frontier_dev.py` (dev's copy at the section's fork, itself held to the
pure-Python `tests/frontier_reference.py` by 3.77's tests) -- on live SLAM
maps with the camera's seen cells, and on random small maps, where ties and
odd shapes are common. The slow sweep is `test_frontier_3130_sweep.py`."""
import random

import numpy as np
import pytest

from brain import frontier as new
from tests import frontier_compare as fc
from tests import frontier_dev as dev
from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN


def answers(fr, m, x, y, clearance, seen):
    r = fr.reach(m, x, y, clearance)
    return ([(int(i), float(d)) for i, d in zip(r.order, r.dist)], r.start, r.need,
            [vars(f) for f in fr.find_frontiers(m, x, y, clearance_m=clearance, reach_map=r)],
            [vars(g) for g in fr.find_view_gaps(m, x, y, seen, clearance_m=clearance,
                                                reach_map=r)])


def random_map(rng: random.Random) -> dict:
    """Rooms of free floor with walls, clutter and unknown, 8-60 cells a
    side, at 0.05 or 0.1 m: small enough that ties are common."""
    w, h = rng.randint(8, 60), rng.randint(8, 60)
    g = np.full((h, w), CELL_UNKNOWN, dtype=np.int8)
    for _ in range(rng.randint(1, 5)):              # rooms
        x0, y0 = rng.randrange(w), rng.randrange(h)
        g[y0:y0 + rng.randint(3, h), x0:x0 + rng.randint(3, w)] = CELL_FREE
    occ, unk = rng.random() * 0.08, rng.random() * 0.05
    noise = np.array([[rng.random() for _ in range(w)] for _ in range(h)])
    g[noise < occ] = CELL_OCCUPIED
    g[(noise >= occ) & (noise < occ + unk)] = CELL_UNKNOWN
    if rng.random() < 0.5:                          # a wall with a gap
        x = rng.randrange(w)
        g[:, x] = CELL_OCCUPIED
        g[rng.randrange(h):rng.randrange(h) + 4, x] = CELL_FREE
    return {"usable": True, "resolution_m": rng.choice((0.05, 0.1)), "width": w, "height": h,
            "origin_x_m": rng.uniform(-3, 3), "origin_y_m": rng.uniform(-3, 3),
            "cells": g.ravel().tolist()}


def test_live_maps_with_the_cameras_seen_cells_give_devs_answers():
    rng = random.Random(3130)
    for path in fc.MAPS[:3]:
        m = fc.load(path)
        seens = (set(), fc.seen_set(m, 8, rng), fc.seen_set(m, 30, rng))
        for x, y in fc.poses(m, 4, rng):
            for seen in seens:
                assert answers(new, m, x, y, fc.LEAVE, seen) == \
                    answers(dev, m, x, y, fc.LEAVE, seen)


def test_random_small_maps_give_devs_answers():
    rng = random.Random(31301)
    checked = 0
    for _ in range(300):
        m = random_map(rng)
        free = [i for i, v in enumerate(m["cells"]) if v == CELL_FREE]
        if not free:
            continue
        for i in rng.sample(free, min(3, len(free))):
            x, y = fc.centre(m, i)
            seen = {(rng.randrange(-2, m["width"] + 2), rng.randrange(-2, m["height"] + 2))
                    for _ in range(rng.randrange(0, len(m["cells"])))}
            clearance = rng.choice((0.05, 0.1, 0.22, fc.LEAVE))
            assert answers(new, m, x, y, clearance, seen) == answers(dev, m, x, y, clearance, seen)
            checked += 1
    assert checked > 600


def test_the_nearest_occupied_cell_is_read_as_before():
    rng = np.random.default_rng(3130)
    for _ in range(2000):
        h, w = rng.integers(1, 30, 2)
        cells = np.where(rng.random((h, w)) < rng.random() * 0.3,
                         CELL_OCCUPIED, CELL_FREE).astype(np.int8)
        a, b = new._Occupied(cells), dev._Occupied(cells)
        for k in range(0, 9):
            assert np.array_equal(a.clear(k), b.clear(k))
        for _ in range(5):
            x, y, cap = int(rng.integers(0, w)), int(rng.integers(0, h)), int(rng.integers(-1, 12))
            assert a.at(x, y, cap) == b.at(x, y, cap)


@pytest.mark.parametrize("chunk", [1, 7, new._SIGHT_CHUNK, 1000])
def test_rays_see_a_patch_as_before_whatever_the_stretch(chunk, monkeypatch):
    """`_Sight.sees` walks a stretch at a time; the answer is the whole
    rays' answer, at any stretch length."""
    m = fc.load(fc.MAPS[2])
    grid = new._grid(m)
    labels, count = new._components(grid == CELL_FREE)
    old_labels, old_count = dev._components(grid == CELL_FREE)
    a, b = new._Sight(m, grid, labels), dev._Sight(m, grid, old_labels)
    rng = random.Random(chunk)
    monkeypatch.setattr(new, "_SIGHT_CHUNK", chunk)
    for _ in range(40):
        pairs = [(*fc.centre(m, rng.randrange(len(m["cells"]))), rng.uniform(-400, 400))
                 for _ in range(rng.randint(1, 12))]
        i = rng.randrange(len(m["cells"]))
        c, oc = int(labels.ravel()[i]), int(old_labels.ravel()[i])
        assert a.sees(c, pairs) == b.sees(oc, pairs)
    assert a.sees(0, []) == []


def test_frontier_pieces_keep_their_cells_order():
    """`_split` walks a long frontier in dev's order: pieces are compared as
    lists, at the default cut and at others."""
    rng = random.Random(7)
    for path in fc.MAPS:
        m = fc.load(path)
        for group in dev._clusters(dev.frontier_cells(m)):
            for most in (int(new.MAX_FRONTIER_M / m["resolution_m"]), 1, rng.randint(2, 40)):
                assert new._split(group, most) == dev._split(group, most)


def test_components_label_as_dev_did_on_random_masks():
    rng = np.random.default_rng(31303)
    for _ in range(2000):
        h, w = (int(v) for v in rng.integers(1, 40, 2))
        mask = rng.random((h, w)) < rng.random()
        a, b = new._components(mask), dev._components(mask)
        assert a[1] == b[1] and np.array_equal(a[0], b[0])
    for path in fc.MAPS[:2]:
        grid = new._grid(fc.load(path))
        for mask in (grid == CELL_FREE, grid == CELL_UNKNOWN):
            a, b = new._components(mask), dev._components(mask)
            assert a[1] == b[1] and np.array_equal(a[0], b[0])


def test_box_dilation_is_devs_on_random_stacks():
    rng = np.random.default_rng(31302)
    for _ in range(1500):
        p, h, w = (int(v) for v in rng.integers(1, 30, 3))
        mark = rng.random((min(p, 6), h, w)) < rng.random() * 0.2
        r = int(rng.integers(0, 20))
        assert np.array_equal(new._box_any_stack(mark, r), dev._box_any_stack(mark, r))


def test_a_map_with_no_frontier_or_no_free_floor_answers_as_before():
    rng = random.Random(2)
    for fill in (CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN):
        m = {"usable": True, "resolution_m": 0.05, "width": 12, "height": 9,
             "origin_x_m": 0.0, "origin_y_m": 0.0, "cells": [fill] * 108}
        for _ in range(3):
            x, y = rng.uniform(0, 0.6), rng.uniform(0, 0.45)
            assert answers(new, m, x, y, 0.05, set()) == answers(dev, m, x, y, 0.05, set())
