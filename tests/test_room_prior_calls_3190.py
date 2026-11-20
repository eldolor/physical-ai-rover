# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
3.190.B, pinned: the 40 fresh targets' replies from /room's model
(`evaluations/room-prior-3190/b/replies.json`, the brain's cache shape),
re-parsed and re-scored for criteria 4-6, and the total spent. Free: no call
is made. Each verdict is pinned as it came out.
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HERE = ROOT / "evaluations" / "room-prior-3190"
sys.path.insert(0, str(HERE))
import calls                                                      # noqa: E402
import targets                                                  # noqa: E402
from brain import room_prior                                    # noqa: E402

REPLIES = json.loads((HERE / "b" / "replies.json").read_text())
SCORED = json.loads((HERE / "b" / "score.json").read_text())
MODEL = "us.anthropic.claude-opus-5-5"

# The section file, "B results".
VERDICTS = {"criterion_4": (True, 40), "criterion_5": (True, 20), "criterion_6": (True, 39)}
SPENT_USD = 0.0741


def test_every_target_was_asked_once_of_the_room_model():
    assert calls.MODEL_ID == MODEL
    assert REPLIES[room_prior.SERVED] == MODEL
    got = REPLIES[MODEL]
    assert sorted(got) == sorted(room_prior.target_key(t) for t in targets.TARGETS)
    for e in got.values():
        assert set(e) == {"reply", "stop", "usage", "prior"}


def test_each_cached_prior_is_its_reply_parsed_by_the_brain():
    for e in REPLIES[MODEL].values():
        assert e["prior"] == room_prior.parse(e["reply"])


def test_the_replies_seed_a_brain_cache(tmp_path):
    p = tmp_path / "cache.json"
    p.write_text(json.dumps(REPLIES))
    hit = room_prior.cached("Spatula", room_prior.read_model(p), p)
    assert hit is not None and hit["prior"]["kitchen"] == 0.6


def test_the_recorded_score_is_what_the_replies_score():
    assert calls.score(REPLIES) == SCORED


def test_each_bar_is_pinned_as_it_came_out():
    assert SCORED["criterion_4"]["met"] == VERDICTS["criterion_4"][0]
    assert SCORED["criterion_4"]["valid"] == VERDICTS["criterion_4"][1]
    assert SCORED["criterion_5"]["met"] == VERDICTS["criterion_5"][0]
    assert SCORED["criterion_5"]["matched"] == VERDICTS["criterion_5"][1]
    assert SCORED["criterion_6"]["met"] == VERDICTS["criterion_6"][0]
    assert SCORED["criterion_6"]["informative"] == VERDICTS["criterion_6"][1]
    assert SCORED["criterion_6"]["flat"] == ["candle"]


def test_the_spend_stayed_under_the_cap():
    assert SCORED["spend"]["calls"] == 40
    assert SCORED["spend"]["usd"] == SPENT_USD
    assert SPENT_USD < calls.CAP_USD
