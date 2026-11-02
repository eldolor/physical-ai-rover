# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_bridge_hold.py

3.133 criteria 1 and 4: the ROS bridge names its own goal hold. Every
`_goal_refusal()` body carries `hold` -- `nav2_starting`, `slam_not_ready`
or null -- set only for a state that passes on its own, as the bridge knows
it. The robot server reads it (`world/bridge_faults.py` `hold_reason()`)
instead of guessing from the body's shape (3.127).
"""
import pathlib
import re
import types

import httpx
import pytest

from tests.test_nav2_ready_gate import _bridge, _Manager
from tests.test_review_376h1 import bridge_mod  # noqa: F401  (the fixture)
from world import bridge_faults

BRIDGE_SRC = (pathlib.Path(__file__).resolve().parents[1] / "service/slam/src/picar_bridge"
              / "picar_bridge/bridge.py")


def _refusal(b):
    code, body = b.send_goal({"x_m": 1.0, "y_m": 0.0})
    assert code == 503, (code, body)
    assert "hold" in body, body
    assert b.nav.futures == [], "nothing may be sent to nav2"
    return body


def _reason(body):
    """The bridge's own body through the robot server's reader (criterion 4)."""
    req = httpx.Request("POST", "http://bridge/goal")
    return bridge_faults.hold_reason(httpx.HTTPStatusError(
        "x", request=req, response=httpx.Response(503, json=body, request=req)))


def _raise_on_answer(m):
    def boom():
        raise RuntimeError("service call failed")
    m.calls[-1].result = boom
    m.calls[-1].complete(None)


def _give_up(b, bridge_mod):
    b._nav2_ask_since -= bridge_mod.NAV2_ASK_TIMEOUT_S + 0.1
    b._poll_nav2()


def _expect(b, hold):
    body = _refusal(b)
    assert body["hold"] == hold, body
    assert _reason(body) == hold, body
    return body


# ---------- (a) a nav2 that has not yet come up: a start ----------

def test_a_fresh_bridge_not_yet_asked_holds_as_starting(bridge_mod):
    """Red on dev 0aeba203: no `hold` key."""
    _expect(_bridge(bridge_mod, _Manager()), "nav2_starting")


def test_no_lifecycle_manager_yet_is_starting(bridge_mod):
    m = _Manager(up=False)
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    _expect(b, "nav2_starting")


def test_asks_in_flight_or_given_up_unanswered_are_starting(bridge_mod):
    """Humble serves is_active in the group running its start-up (3.105)."""
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    _expect(b, "nav2_starting")
    _give_up(b, bridge_mod)
    assert len(m.calls) == 2
    _expect(b, "nav2_starting")


def test_a_cold_start_holds_past_3127s_60_s(bridge_mod):
    """3.127 read a start past 60 s as ROS DOWN; the bridge's own bar is
    START_HOLD_MAX_S (300 s) from its start (review round 1)."""
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    b._nav2_inactive_since -= 600
    body = _expect(b, "nav2_starting")
    assert body["nav2"]["inactive_s"] >= 600


# ---------- (b) once active, never a start again ----------

def test_inactive_after_active_is_no_hold(bridge_mod):
    """A crash or reset; 3.127 read its first 60 s as `nav2_starting`."""
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    m.answer(True)
    b._poll_nav2()
    m.answer(False)
    body = _expect(b, None)
    assert body["nav2"]["inactive_s"] < 5, "the case 3.127 guessed as a start"


def test_unanswered_after_active_is_no_hold(bridge_mod):
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    m.answer(True)
    b._poll_nav2()
    _give_up(b, bridge_mod)
    _expect(b, None)


def test_manager_gone_after_active_is_no_hold(bridge_mod):
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    m.answer(True)
    m.up = False
    b._poll_nav2()
    _expect(b, None)


def test_a_flapping_manager_is_no_hold(bridge_mod):
    m = _Manager()
    b = _bridge(bridge_mod, m)
    for active in (True, False, True, False):
        b._poll_nav2()
        m.answer(active)
    _expect(b, None)


# ---------- (c) the manager answered failure before ever being active ----------

def test_answered_not_active_before_active_is_no_hold_for_good(bridge_mod):
    """The start-up returned and failed: it will not pass on its own."""
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    m.answer(False)
    _expect(b, None)
    b._poll_nav2()
    _give_up(b, bridge_mod)                      # later asks unanswered
    m.up = False
    b._poll_nav2()                               # and the manager gone
    _expect(b, None)


def test_a_raised_call_during_a_start_is_not_an_answer(bridge_mod):
    """Review round 2: a call that raises is the transport failing (a
    respawn, a cancelled future), not the manager answering -- the start
    goes on (within START_HOLD_MAX_S), and active still ends it."""
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    _raise_on_answer(m)
    body = _expect(b, "nav2_starting")
    assert "failed" in body["nav2"]["why"], body
    b._poll_nav2()
    m.answer(True)
    b._poll_nav2()
    _raise_on_answer(m)
    _expect(b, None)


def test_a_late_answer_to_a_given_up_ask_ends_nothing(bridge_mod):
    """An answer the bridge ignores (given up on) does not end the start."""
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    stale = m.calls[-1]
    _give_up(b, bridge_mod)
    stale.complete(types.SimpleNamespace(success=False, message=""))
    _expect(b, "nav2_starting")


def test_active_ends_the_hold_and_goals_go(bridge_mod):
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    m.answer(True)
    assert b.send_goal({"x_m": 1.0, "y_m": 0.0}) == (200, {"state": "pending"})


# ---------- (d) SLAM ----------

@pytest.mark.parametrize("state", ["starting", "checking", "remapping"])
def test_a_passing_slam_state_is_slam_not_ready(bridge_mod, state):
    """Red on dev 0aeba203: no `hold` key."""
    b = _bridge(bridge_mod, _Manager())
    b.slam_state = state
    body = _expect(b, "slam_not_ready")
    assert body["slam"] == state


def test_a_failed_slam_is_no_hold(bridge_mod):
    b = _bridge(bridge_mod, _Manager())
    b.slam_state = "failed"
    _expect(b, None)


def test_slam_hold_comes_first_even_while_nav2_starts(bridge_mod):
    b = _bridge(bridge_mod, _Manager())
    b.slam_state = "checking"
    _expect(b, "slam_not_ready")


def test_slam_passing_is_every_state_the_bridge_sets_but_ready_and_failed(bridge_mod):
    """A state the bridge adds or renames must be sorted in or out here."""
    states = set(re.findall(r'slam_state = "([a-z_]+)"', BRIDGE_SRC.read_text()))
    assert states - {"ready", "failed"} == set(bridge_mod.SLAM_PASSING), states
    assert {"ready", "failed"} <= states


# ---------- criterion 4: the names across the wall ----------

def test_the_bridges_hold_names_are_the_robot_servers(bridge_mod):
    """Every `hold` the bridge source can send is one the robot server
    reads, and the robot server reads no name the bridge never sends."""
    sent = {getattr(bridge_mod, n) for n in re.findall(r'"hold":\s*([A-Z0-9_]+)\b',
                                                        BRIDGE_SRC.read_text())}
    sent |= set(re.findall(r'"hold":\s*"([a-z_]+)"', BRIDGE_SRC.read_text()))
    sent.discard(None)
    assert sent == set(bridge_faults.HOLDS), sent
    # Thermos round 2: every quoted `hold` key in the source is one of the
    # dict-literal forms read above, so a name set another way
    # (body["hold"] = ...) fails here, not quietly. A local variable named
    # `hold` is not a key and is not counted (review round 3).
    src = BRIDGE_SRC.read_text()
    every = re.findall(r'["\']hold["\']', src)
    read = re.findall(r'"hold":\s*(?:[A-Z0-9_]+\b|"[a-z_]+"|None\b)', src)
    assert len(every) == len(read), (every, read)
    # Thermos round 3: a conditional's other arm is never a name of its own.
    arms = re.findall(r'"hold":[^\n]*?\belse\s+([^\s,})]+)', src)
    assert arms and set(arms) == {"None"}, arms


def test_the_no_server_503_says_no_hold(bridge_mod):
    """Review round 3: every 503 body carries `hold`; this one is a crash."""
    m = _Manager()
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    m.answer(True)
    b.nav.wait_for_server = lambda timeout_sec: False
    code, body = b.send_goal({"x_m": 1.0, "y_m": 0.0})
    assert code == 503 and "hold" in body and body["hold"] is None, body
    assert _reason(body) is None


def test_slam_bounded_by_start_is_within_slam_passing(bridge_mod):
    assert set(bridge_mod.SLAM_BOUNDED_BY_START) <= set(bridge_mod.SLAM_PASSING)
    assert {bridge_mod.HOLD_NAV2, bridge_mod.HOLD_SLAM} == set(bridge_faults.HOLDS)


# ---------- review round 1: a start that never ends is not a hold for ever ----------

def test_a_nav2_start_past_the_bridges_ceiling_is_no_hold(bridge_mod):
    """Both reviewers: a manager that dies before its first answer (no
    respawn) read `nav2_starting` for the bridge's life. Past
    START_HOLD_MAX_S from the bridge's own start it is an outage."""
    m = _Manager(up=False)
    b = _bridge(bridge_mod, m)
    b._poll_nav2()
    b._started_mono -= bridge_mod.START_HOLD_MAX_S - 1
    _expect(b, "nav2_starting")
    b._started_mono -= 2
    _expect(b, None)


def test_a_slam_start_past_the_bridges_ceiling_is_no_hold(bridge_mod):
    """Thermos: a fresh start waits for its first pose with no timeout."""
    b = _bridge(bridge_mod, _Manager())
    b.slam_state = "starting"
    b._started_mono -= bridge_mod.START_HOLD_MAX_S + 1
    _expect(b, None)


@pytest.mark.parametrize("state", ["checking", "remapping"])
def test_bounded_slam_states_hold_whatever_the_bridges_age(bridge_mod, state):
    """`checking` (the verdict's own timeout) and `remapping` (a re-map hours
    in, with REMAP_READY_TIMEOUT_S) end by themselves; the ceiling is a start's."""
    b = _bridge(bridge_mod, _Manager())
    b.slam_state = state
    b._started_mono -= bridge_mod.START_HOLD_MAX_S * 10
    _expect(b, "slam_not_ready")
