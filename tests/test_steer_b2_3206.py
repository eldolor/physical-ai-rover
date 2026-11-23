# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.206.B2 (amendment 4): the identity check's task list, what it reuses
from B1, how a row is read as a yes, and that a partial run is not judged
-- no AWS call is made."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("b2_3206", ROOT / "evaluations/steer-distill-3206/b2_bedrock.py")
b2 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(b2)


def test_tasks_are_the_manifest_once_unmirrored():
    ts = b2.tasks()
    frames = b2.mf.build()["frames"]
    assert len(ts) == 2268 and len({t["id"] for t in ts}) == 2268
    assert not any(t["mirror"] for t in ts)
    assert sum("I2261" in frames[t["key"]]["sets"] for t in ts) == 2261
    assert sum("taupe" in frames[t["key"]]["sets"] for t in ts) == 7


def test_b1s_unmirrored_rows_are_reused_not_bought_again():
    have = b2.reused()
    assert len(have) == 100 and not any(r["mirror"] for r in have.values())
    ask = b2.to_ask()
    assert len(ask) == 2168 and not {t["id"] for t in ask} & set(have)


def test_the_pilot_is_one_in_view_and_one_out_of_view_frame():
    frames = b2.mf.build()["frames"]
    seen, unseen = b2.pilot_tasks()
    assert frames[seen["key"]]["visible"] and not frames[unseen["key"]]["visible"]
    assert {seen["id"], unseen["id"]} <= {t["id"] for t in b2.to_ask()}


def test_a_box_is_a_yes_and_an_empty_list_a_no():
    t = b2.to_ask()[0]
    box = {**t, "status": "answered", "frac_x": 0.4}
    none = {**t, "status": "answered", "frac_x": None}
    assert b2.answers([box])[t["id"]]["yes"] is True
    assert b2.answers([none])[t["id"]]["yes"] is False
    assert t["id"] not in b2.answers([{**t, "status": "error"}])       # never read as a "no"


def test_a_partial_run_is_not_judged_but_b2_4_is_measured():
    out = b2.scores([])
    missing = 2168 - len(b2.pilot_rows())
    assert not out["coverage"]["whole"] and out["coverage"]["missing"] == missing
    assert not out["all_met"]
    assert not any(v for k, v in out["bars"].items() if not k.startswith("B2-4"))


def test_b2_4_counts_the_models_first_reply_only():
    ask = [t for t in b2.to_ask() if t["id"] not in b2.pilot_rows()]
    bad = [{**t, "status": "truncated"} for t in ask[:30]]              # 30 of 2,268: 1.3%
    later = [{**t, "status": "answered", "frac_x": None} for t in ask[:30]]
    rest = [{**t, "status": "answered", "frac_x": None} for t in ask[30:]]
    share = b2.first_try_share(bad + later + rest)
    assert abs(share - (2268 - 30) / 2268) < 1e-9 and share < b2.MIN_ANSWERED


def test_b2_4_is_not_failed_by_errors_or_by_frames_never_asked():
    ask = [t for t in b2.to_ask() if t["id"] not in b2.pilot_rows()]
    throttled = [{**t, "status": "error"} for t in ask[:30]]
    assert b2.first_try_share(throttled) == 1.0                         # a gap, not a bad reply
    assert b2.first_try_share([]) == 1.0
    assert len(b2.never_replied(throttled)) == len(ask)                 # so --final is refused
    answered = [{**t, "status": "answered", "frac_x": None} for t in ask]
    assert b2.never_replied(throttled + answered) == []


def test_the_pilot_gate_needs_a_box_on_the_in_view_frame(monkeypatch):
    seen, unseen = b2.pilot_ids()
    none = {seen["id"]: {"frac_x": None}, unseen["id"]: {"frac_x": None}}
    monkeypatch.setattr(b2, "pilot_rows", lambda: none)
    assert not b2.pilot_passed()                # two "no"s: the format may be unread
    monkeypatch.setattr(b2, "pilot_rows", lambda: {**none, seen["id"]: {"frac_x": 0.5}})
    assert b2.pilot_passed()
    monkeypatch.setattr(b2, "pilot_rows", lambda: {seen["id"]: {"frac_x": 0.5}})
    assert not b2.pilot_passed()                # one reply missing


def test_the_stop_leaves_room_for_every_call_in_flight():
    worst = b2.b1.cost([{"billed_unknown": True}])
    assert not b2.over_stop(b2.STOP_USD - b2.AT_ONCE * worst - 1e-9)
    assert b2.over_stop(b2.STOP_USD - b2.AT_ONCE * worst)
    assert b2.over_stop(b2.STOP_USD)


def test_a_refusal_before_the_model_ran_is_not_counted_as_spend():
    class Throttle(Exception):
        response = {"Error": {"Code": "ThrottlingException"}}

    class Timeout(Exception):
        pass
    assert not b2.billed(Throttle(), asked=True)
    assert b2.billed(Timeout(), asked=True)     # may have run: counted
    assert not b2.billed(Timeout(), asked=False)
