# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.165: a landmark's points are bounded, and the bound keeps its position
and its reach (docs/plans/ros-alignment/3.165-inventory-landmarks-bounded.md)."""
import random

import pytest

from brain import inventory as inv
from brain.inventory import Inventory
from tests.test_inventory import _pose, _scan


def _look(i, label, x_robot, y_robot, rng_m, bearing=0.0, h=0.0):
    i.observe([{"label": label, "bearing_deg": bearing}], _pose(x_robot, y_robot, h),
              _scan({bearing: rng_m}))


def test_500_looks_from_one_spot_hold_a_bounded_number_of_points():
    """Red on dev: one point per look, 500 of them."""
    i = Inventory()
    for _ in range(500):
        _look(i, "sofa", 0.0, 0.0, 2.0)
    (lm,) = i.landmarks
    assert len(lm.points) <= inv.MAX_POINTS
    assert lm.as_dict()["points"] == 500          # the report still counts observations


def test_a_long_object_seen_along_its_length_stays_bounded_and_keeps_its_reach():
    """A 3 m sofa seen from 600 places along its front: members bounded, the
    position still the mean of every observation, and both ends still link."""
    i, rng, xs = Inventory(), random.Random(5), []
    for _ in range(600):
        x = rng.uniform(-1.5, 1.5)
        _look(i, "sofa", x, 0.0, 2.0)
        xs.append(x)
    (lm,) = i.landmarks
    assert len(lm.points) <= inv.MAX_POINTS
    assert lm.x == pytest.approx(sum(xs) / len(xs), abs=1e-9)
    assert lm.y == pytest.approx(-2.0, abs=1e-9)
    # A sighting just past either end, within MERGE_M of the true end, joins it.
    for end in (-1.5 - inv.MERGE_M * 0.8, 1.5 + inv.MERGE_M * 0.8):
        _look(i, "sofa", end, 0.0, 2.0)
        assert len(i.landmarks) == 1


def test_a_fold_keeps_the_weighted_mean():
    i = Inventory()
    _look(i, "mug", 0.0, 0.0, 2.0)
    _look(i, "mug", 0.0, 0.0, 2.0 + inv.FOLD_M / 2)       # folds into the first
    (lm,) = i.landmarks
    assert len(lm.points) == 1 and lm.points[0][3] == 2
    assert lm.y == pytest.approx(-(2.0 + 2.0 + inv.FOLD_M / 2) / 2, abs=1e-9)
    assert lm.points[0][1] == pytest.approx(lm.y, abs=1e-9)


def test_a_point_further_than_the_fold_is_its_own_member():
    i = Inventory()
    _look(i, "mug", 0.0, 0.0, 2.0)
    _look(i, "mug", 0.0, 0.0, 2.0 + inv.FOLD_M * 2)
    (lm,) = i.landmarks
    assert len(lm.points) == 2


def test_another_labels_point_never_folds_into_a_member():
    """A relabel join (within RELABEL_M) is a vote; its point stays its own."""
    i = Inventory()
    _look(i, "mug", 0.0, 0.0, 2.0)
    _look(i, "cup", 0.0, 0.0, 2.0 + inv.FOLD_M / 2)
    (lm,) = i.landmarks
    assert sorted(p[2] for p in lm.points) == ["cup", "mug"]


def test_a_merge_past_the_cap_folds_down_and_keeps_the_mean():
    """Two landmarks bridged by a point each bring their members; the union
    is folded back to the cap and the position is the mean of all."""
    i, rng, xs = Inventory(), random.Random(9), []
    for lo, hi in ((-1.4, -0.4), (0.4, 1.4)):         # ends 0.4 m from the bridge
        for _ in range(200):
            x = rng.uniform(lo, hi)
            _look(i, "bed", x, 0.0, 2.0)
            xs.append(x)
    assert len(i.landmarks) == 2
    _look(i, "bed", 0.0, 0.0, 2.0)
    xs.append(0.0)
    (lm,) = i.landmarks
    assert len(lm.points) <= inv.MAX_POINTS
    assert lm.x == pytest.approx(sum(xs) / len(xs), abs=1e-9)
    assert lm.as_dict()["points"] == len(xs)


def test_with_no_two_of_one_label_the_closest_fold_under_the_heavier():
    lm = inv.Landmark(id=1, room="r", first_step=0, points=[(0.0, 0.0, "a")])
    lm.add(0.0, 0.0, "a")                        # folds: weight 2 at the origin
    lm.add(0.5, 0.0, "b")
    assert len(lm.points) == 2
    assert lm._fold_closest_any()
    assert len(lm.points) == 1 and lm.points[0][2] == "a" and lm.points[0][3] == 3
    assert lm.points[0][0] == pytest.approx(0.5 / 3)
    assert lm.x == pytest.approx(0.5 / 3)


def test_the_cap_holds_with_alternating_labels():
    lm = inv.Landmark(id=1, room="r", first_step=0, points=[(0.0, 0.0, "a")])
    for k in range(1, 3 * inv.MAX_POINTS):
        lm.add(k * 0.2, 0.0, "a" if k % 2 == 0 else "b")
    assert len(lm.points) <= inv.MAX_POINTS
    assert lm._n == 3 * inv.MAX_POINTS


def test_a_carried_point_never_folds_or_moves():
    """3.50.D5: a carried landmark's saved point stays where the earlier
    mission put it, however many sightings land on it."""
    i = Inventory()
    saved = {"frame_id": "f", "reported": [{"label": "stool", "x_m": 0.0, "y_m": -2.0,
                                            "belief": 0.9, "votes": {"stool": 3}}]}
    assert i.seed("f", saved) == 1
    for _ in range(100):
        i.observe([{"label": "stool", "bearing_deg": 0.0}],
                  {**_pose(0.0, 0.0, 0.0), "frame_id": "f"}, _scan({0.0: 2.01}))
    (lm,) = i.landmarks
    assert lm.points[0][:3] == [0.0, -2.0, "stool"] and lm.points[0][3] == 1
    assert len(lm.points) <= inv.MAX_POINTS
    assert lm.as_dict()["points"] == 101
