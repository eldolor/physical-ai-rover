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


# ---- criterion 7 (amendments 5-7): the same 40, then criterion 1's arm, under Fable 5.1 ----
import b7                                                       # noqa: E402

B7_REPLIES = json.loads((HERE / "b7" / "replies.json").read_text())
B7_SCORED = json.loads((HERE / "b7" / "score.json").read_text())
B7_ARMS = HERE / "b7" / "arms"
FABLE51 = "us.anthropic.claude-fable-5-1"
B7_SPENT_USD = 0.2269          # Fable 5.1's 49 calls (the section file)
B7_RUN_USD = 0.2315            # with the two refused-model pilots' one answer each


def test_criterion_7_asked_fable_51_of_its_pinned_region_at_its_billed_rate():
    assert b7.MODELS["fable51"] == (FABLE51, "us-east-1", (11.00, 55.00))
    got = B7_REPLIES[FABLE51]
    assert sorted(got) == sorted(room_prior.target_key(t) for t in b7.ASK_ORDER)
    for e in got.values():
        assert e["prior"] == room_prior.parse(e["reply"]) and e["stop"] == "end_turn"
    # Each refused model's pilot left exactly its one answered call.
    assert len(B7_REPLIES["us.anthropic.claude-sonnet-5-5"]) == 1
    assert len(B7_REPLIES["us.anthropic.claude-fable-5-2"]) == 1


def test_criterion_7_scores_4_to_6_as_recorded():
    assert b7.score(B7_REPLIES, "fable51") == B7_SCORED
    assert (B7_SCORED["criterion_4"]["valid"], B7_SCORED["criterion_4"]["met"]) == (40, True)
    assert (B7_SCORED["criterion_5"]["matched"], B7_SCORED["criterion_5"]["met"]) == (20, True)
    assert (B7_SCORED["criterion_6"]["informative"], B7_SCORED["criterion_6"]["met"]) == (40, True)


def test_criterion_7_spend_stayed_under_the_cap():
    s = b7.spend(B7_REPLIES, "fable51")
    assert s["calls"] == 49 and round(s["usd"], 4) == B7_SPENT_USD
    assert round(s["usd_all"], 4) == B7_RUN_USD < b7.CAP_USD["fable51"]


def test_criterion_7_arm_fable_meets_a_and_fails_b():
    rows = {a: json.loads((B7_ARMS / f"{a}-R.json").read_text()) for a in b7.ARMS}
    rep = b7.score_arms(rows, "fable51")
    assert rep == json.loads((B7_ARMS / "score.json").read_text())
    assert rep["a"]["met"] is True
    assert rep["a"]["paired_challenger_minus_none"]["ci95"] == [0.0249, 0.1187]
    assert rep["b"]["met"] is False
    assert rep["b"]["paired_challenger_minus_opus"]["mean"] == -0.0296
    assert rep["b"]["paired_challenger_minus_opus"]["ci95"] == [-0.0533, -0.0067]
    assert rep["beats_opus"] is False


# ---- 3.190.G (amendment 8): the same comparison under Claude Sonnet 5 ----
SONNET5 = "us.anthropic.claude-sonnet-5"
G_SCORED = json.loads((HERE / "b7" / "score-sonnet5.json").read_text())
G_ARMS = HERE / "b7" / "arms-sonnet5"
G_SPENT_USD = 0.0470           # Sonnet 5's 49 calls (the section file)


def test_g_asked_sonnet_5_of_rooms_region_at_its_derived_rate():
    assert b7.MODELS["sonnet5"] == (SONNET5, "us-east-2", (2.20, 11.00))
    got = B7_REPLIES[SONNET5]
    assert sorted(got) == sorted(room_prior.target_key(t) for t in b7.ASK_ORDER)
    for e in got.values():
        assert e["prior"] == room_prior.parse(e["reply"]) and e["stop"] == "end_turn"


def test_g_scores_4_to_6_as_recorded():
    assert b7.score(B7_REPLIES, "sonnet5") == G_SCORED
    assert (G_SCORED["criterion_4"]["valid"], G_SCORED["criterion_4"]["met"]) == (40, True)
    assert (G_SCORED["criterion_5"]["matched"], G_SCORED["criterion_5"]["met"]) == (20, True)
    assert (G_SCORED["criterion_6"]["informative"], G_SCORED["criterion_6"]["met"]) == (40, True)


def test_g_spend_stayed_under_its_cap():
    s = b7.spend(B7_REPLIES, "sonnet5")
    assert s["calls"] == 49 and round(s["usd"], 4) == G_SPENT_USD
    assert round(s["usd_all"], 4) == G_SPENT_USD < b7.CAP_USD["sonnet5"]


def test_g_arm_sonnet_5_meets_a_and_ties_opus():
    rows = {a: json.loads((G_ARMS / f"{a}-R.json").read_text()) for a in b7.ARMS}
    rep = b7.score_arms(rows, "sonnet5")
    assert rep == json.loads((G_ARMS / "score.json").read_text())
    assert rep["a"]["met"] is True
    assert rep["a"]["paired_challenger_minus_none"]["ci95"] == [0.0322, 0.1339]
    assert rep["b"]["met"] is False
    assert rep["b"]["paired_challenger_minus_opus"]["mean"] == -0.0185
    assert rep["b"]["paired_challenger_minus_opus"]["ci95"] == [-0.0539, 0.0161]
    assert rep["beats_opus"] is False
    # The opus and none arms are amendment 7's, unchanged.
    fable = json.loads((B7_ARMS / "score.json").read_text())["summary"]
    assert rep["summary"]["opus"] == fable["opus"] and rep["summary"]["none"] == fable["none"]
