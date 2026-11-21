# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.190.D: criteria 14-15's live record, pinned (amendment 4's bars).

Reads `evaluations/room-prior-3190/live/live.json` (the find on the live
stack, 2026-11-21) and `shot_cloud.json` (criterion 15's `cloud` line again
after the wording change), and re-derives each verdict from the rows, so an
edit to the record that changes a verdict fails here. No browser, no stack.
"""

import json

from brain.request import canonical_room
from tests.conftest import REPO_ROOT

LIVE = REPO_ROOT / "evaluations" / "room-prior-3190" / "live"
PRICE_IN, PRICE_OUT = 4.40e-6, 22e-6


def _rec():
    return json.loads((LIVE / "live.json").read_text())


def test_the_source_is_cloud_then_cache():
    rec = _rec()
    assert rec["run1"]["room_prior"]["source"] == "cloud"
    assert rec["run2"]["room_prior"]["source"] == "cache"
    assert rec["run1"]["room_prior"]["rates"] == rec["run2"]["room_prior"]["rates"]
    assert rec["run1"]["top_type"] == rec["run2"]["top_type"] == "kitchen"


def test_both_finds_found_with_no_contact():
    rec = _rec()
    for run in ("run1", "run2"):
        assert rec[run]["outcome"] == "found", run
        assert rec[run]["contacts"] == 0, run
        assert rec[run]["docked"] is True, run


def test_the_first_goal_bar_failed():
    """Criterion 14's last clause, as recorded: the first goal of both runs
    was a frontier in the dining room, not a room of the top type."""
    rec = _rec()
    for run in ("run1", "run2"):
        g = rec[run]["first_goal"]
        assert g["kind"] == "frontier" and g["room"] == "dining room", run
        assert canonical_room(g["room"]) != rec[run]["top_type"], run
        assert rec[run]["first_goal_in_top"] is False, run


def test_one_paid_call_for_the_two_runs_and_one_for_the_screenshot():
    rec, shot = _rec(), json.loads((LIVE / "shot_cloud.json").read_text())
    for r, n in ((rec, 1), (shot, 1)):
        assert len(r["calls"]) == n
        assert all(c["status"] == 200 and c["stop"] == "end_turn" for c in r["calls"])
        usd = sum(c["in"] * PRICE_IN + c["out"] * PRICE_OUT for c in r["calls"])
        assert round(usd, 5) == r["spend"]["usd"]
    assert rec["spend"]["usd"] + shot["spend"]["usd"] < 0.01


def test_the_screenshots_show_their_lines():
    rec, shot = _rec(), json.loads((LIVE / "shot_cloud.json").read_text())
    assert rec["none_shot"]["ok"] and rec["none_shot"]["source"] == "none"
    assert shot["shot"]["ok"] and shot["shot"]["source"] == "cloud"
    assert shot["shot"]["text"] == "Likeliest for the refrigerator: the kitchen (asked online)."
    # Each record names the picture that shows its line.
    assert rec["run1"]["shot"]["png"] == "twin-3190d-cloud-asrun.png"
    assert shot["shot"]["png"] == "twin-3190d-cloud.png"
    assert not any("/" in str(rec[r].get("run_dir", "")) for r in ("run1", "run2"))
    for png in ("twin-3190d-none.png", "twin-3190d-cloud.png", "twin-3190d-cloud-asrun.png"):
        assert (LIVE / png).stat().st_size > 10_000, png
