# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.175.B: a carried landmark looked at and not seen is stale for the chain
(held to `STALE_BELIEF`, so the next mission does not recall it), and recalls
go newest sighting first (`seen_seq`)
(docs/plans/ros-alignment/3.175-memory-when-things-move.md)."""

from brain.inventory import CARRY_MIN_BELIEF, REPORT_BELIEF, STALE_BELIEF, Inventory
from tests.test_inventory import _scan
from tests.test_map_frame_carry import SAVED, _pose, _session_one
from tests.test_recall_3172 import _report, _row, _runner


def _stool(inv):
    return next(lm for lm in inv.landmarks if lm.label == "stool")


def _seeded(seq=None):
    saved = _session_one()
    if seq is not None:
        saved["carry_seq"] = seq
    inv = Inventory()
    inv.seed(SAVED, saved)
    return inv


def test_one_empty_look_at_a_carried_landmark_makes_it_stale_for_the_chain():
    inv = _seeded()
    assert _stool(inv).belief >= REPORT_BELIEF
    # The stool is at (0, -2); heading 0 faces it. The detector ran and saw
    # nothing, and the scan reaches past it (not occluded).
    inv.observe([], _pose(y=0.5), _scan({0: 3.0}))
    lm = _stool(inv)
    assert CARRY_MIN_BELIEF <= lm.belief <= STALE_BELIEF + 1e-9 < REPORT_BELIEF
    nxt = Inventory()
    nxt.seed(SAVED, dict(inv.report(), mission_id="m2"))
    stool = _stool(nxt)
    assert stool.carried and stool.belief < REPORT_BELIEF     # carried, not recalled


def test_a_stale_landmark_seen_again_from_a_new_view_is_reported_again():
    inv = _seeded()
    inv.observe([], _pose(y=0.5), _scan({0: 3.0}))
    inv.observe([{"label": "stool", "bearing_deg": 0.0}], _pose(x=2.0, y=-2.0, h=270.0),
                _scan({0: 2.0}))
    assert _stool(inv).belief >= REPORT_BELIEF


def test_a_landmark_seen_this_mission_keeps_the_forgiveness():
    """Seen here first: one miss after is the detector dropping it (3.46)."""
    inv = _seeded()
    inv.observe([{"label": "stool", "bearing_deg": 0.0}], _pose(), _scan({0: 2.0}))
    before = _stool(inv).belief
    inv.observe([], _pose(y=0.5), _scan({0: 3.0}))
    assert _stool(inv).belief == before


def test_seen_seq_is_the_mission_that_last_saw_it():
    inv = _seeded(seq=4)                                     # this mission is 5
    assert {lm.label: lm.seen_seq for lm in inv.landmarks} == {"stool": 0, "box": 0}
    inv.observe([{"label": "stool", "bearing_deg": 0.0}], _pose(), _scan({0: 2.0}))
    rows = {d["label"]: d["seen_seq"] for d in inv.report()["reported"] + inv.report()["candidates"]}
    assert rows == {"stool": 5, "box": 0}


def test_a_report_that_does_not_say_carries_none():
    saved = _session_one()
    for d in saved["reported"] + saved["candidates"]:
        d.pop("seen_seq", None)
    saved["candidates"][0]["seen_seq"] = "x"
    inv = Inventory()
    inv.seed(SAVED, saved)
    assert {lm.seen_seq for lm in inv.landmarks} == {None}


def test_recalls_go_newest_sighting_first_then_by_belief():
    runner, _, _ = _runner("washer")
    rows = [dict(_row("washer", 1.0, 1.0, 0.98), seen_seq=1),
            dict(_row("washer", 2.0, 2.0, 0.85), seen_seq=3),
            _row("washer", 3.0, 3.0, 0.99),                  # says nothing: last
            dict(_row("washer", 4.0, 4.0, 0.90), seen_seq=3)]
    runner.inventory.seed("f1", _report(rows))
    assert runner._recall_carried() == 3
    assert runner.agent.recall == [(4.0, 4.0), (2.0, 2.0), (1.0, 1.0)]
