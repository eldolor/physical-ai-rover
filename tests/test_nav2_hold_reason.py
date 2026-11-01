"""
tests/test_nav2_hold_reason.py

3.127 criteria 1-2: a goal the bridge HOLDS (3.105: nav2's lifecycle not yet
active; 3.50.D3: slam_toolbox not `ready`) is refused by its own name --
`nav2_starting` / `slam_not_ready` -- not as the outage `ros_unavailable`,
and stays retryable, so explore still waits on it. Every other bridge error
keeps `ros_unavailable` and its old verdict.

3.133 criteria 2-3: the hold is the bridge's own `hold` field, never guessed
from the body's shape; a body with no `hold` (an older image) is
`ros_unavailable`. The bridge's side is tests/test_bridge_hold.py.
"""

import httpx
import pytest
from fastapi.testclient import TestClient

import robot.server as server
from tests.test_ros_goals import GoalBridge, world_with
from tests.test_ros_world import FakeTruth
from world import bridge_faults

NAV2_HELD = (503, {"error": "nav2 is not active: busy starting nav2, or hung",
                   "hold": "nav2_starting",
                   "nav2": {"active": False, "why": "busy starting nav2, or hung",
                            "inactive_s": 4.2}})
SLAM_HELD = (503, {"error": "SLAM is not ready (starting)", "hold": "slam_not_ready",
                   "slam": "starting"})


class _AnsweringBridge(GoalBridge):
    """POST /goal answers one fixed reply (or raises one transport error)."""

    def __init__(self, reply, **kw):
        super().__init__(**kw)
        self.reply = reply

    def handler(self, request):
        if request.url.path == "/goal" and request.method == "POST":
            if isinstance(self.reply, Exception):
                raise self.reply
            status, body = self.reply
            return httpx.Response(status, json=body)
        return super().handler(request)


def _post_goal(monkeypatch, reply):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    truth = FakeTruth(1.0, 1.0, 90.0)
    world, _ = world_with(truth=truth, bridge=_AnsweringBridge(
        reply, start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0}))
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    r = TestClient(server.create_app()).post("/world/goal", json={"x_m": 2.0, "y_m": 1.0})
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["accepted"] is False, body
    return body


# ---------- criterion 1: the robot server names the hold ----------

def test_a_goal_held_while_nav2_starts_is_nav2_starting(monkeypatch):
    """Red on dev 989ba9f8: `ros_unavailable`."""
    body = _post_goal(monkeypatch, NAV2_HELD)
    assert body["reason"] == "nav2_starting", body
    assert body["retryable"] is True and body["bridge_status"] == 503, body
    assert "nav2 is not active" in body["detail"], body


def test_a_goal_held_while_slam_is_not_ready_is_slam_not_ready(monkeypatch):
    """Red on dev 989ba9f8: `ros_unavailable`."""
    body = _post_goal(monkeypatch, SLAM_HELD)
    assert body["reason"] == "slam_not_ready", body
    assert body["retryable"] is True and body["bridge_status"] == 503, body
    assert "SLAM is not ready" in body["detail"], body


@pytest.mark.parametrize("reply, retryable, status", [
    ((503, {"error": "nav2 is not up (navigate_to_pose has no server)"}), True, 503),
    ((500, {"error": "boom", "nav2": {"active": False}}), True, 500),
    ((401, {"error": "unauthorised", "slam": "starting"}), False, 401),
    ((503, {"error": "nav2 is not active", "nav2": "not a dict"}), True, 503),
    (httpx.ConnectError("bridge gone"), True, None),
])
def test_every_other_bridge_error_stays_ros_unavailable(monkeypatch, reply, retryable, status):
    body = _post_goal(monkeypatch, reply)
    assert body["reason"] == "ros_unavailable", body
    assert body["retryable"] is retryable and body["bridge_status"] == status, body


def test_hold_reason_finds_no_hold_in_a_body_without_one():
    req = httpx.Request("POST", "http://bridge/goal")
    assert bridge_faults.hold_reason(_err(*NAV2_HELD)) == "nav2_starting"
    assert bridge_faults.hold_reason(_err(*SLAM_HELD)) == "slam_not_ready"
    assert bridge_faults.hold_reason(_err(503, {"error": "x"})) is None
    assert bridge_faults.hold_reason(_err(503, content=b"<html>proxy</html>")) is None
    assert bridge_faults.hold_reason(_err(503, ["nav2"])) is None
    assert bridge_faults.hold_reason(_err(502, {"nav2": {"active": False, "inactive_s": 1.0}})) is None
    assert bridge_faults.hold_reason(httpx.ConnectError("gone", request=req)) is None
    assert bridge_faults.hold_reason(ValueError("not httpx")) is None


def _err(status, body=None, content=None):
    req = httpx.Request("POST", "http://bridge/goal")
    resp = (httpx.Response(status, json=body, request=req) if content is None
            else httpx.Response(status, content=content, request=req))
    return httpx.HTTPStatusError("x", request=req, response=resp)


# ---------- 3.133: the bridge names the hold; the robot server reads it ----------

def test_a_slow_cold_start_the_bridge_holds_is_nav2_starting(monkeypatch):
    """3.133 criterion 2. Red on dev 0aeba203 (past 60 s read as ROS DOWN)."""
    body = _post_goal(monkeypatch, (503, {
        "error": "nav2 is not active: busy starting nav2, or hung", "hold": "nav2_starting",
        "nav2": {"active": False, "why": "busy starting nav2, or hung", "inactive_s": 300.0}}))
    assert body["reason"] == "nav2_starting", body
    assert body["retryable"] is True and body["bridge_status"] == 503, body


@pytest.mark.parametrize("reply", [
    # a nav2 that crashed 4 s ago: 3.127 guessed "starting" for 60 s
    (503, {"error": "nav2 is not active: nav2's nodes are not active", "hold": None,
           "nav2": {"active": False, "why": "nav2's nodes are not active", "inactive_s": 4.2}}),
    # a SLAM state the bridge does not call passing
    (503, {"error": "SLAM is not ready (starting)", "hold": None, "slam": "starting"}),
])
def test_what_the_bridge_does_not_call_a_hold_is_ros_unavailable(monkeypatch, reply):
    """3.133 criterion 2. Red on dev 0aeba203 (the body's shape read as a hold)."""
    body = _post_goal(monkeypatch, reply)
    assert body["reason"] == "ros_unavailable", body
    assert body["retryable"] is True and body["bridge_status"] == 503, body


@pytest.mark.parametrize("body", [
    {"error": "nav2 is not active", "nav2": {"active": False, "inactive_s": 4.2}},
    {"error": "SLAM is not ready (starting)", "slam": "starting"},
])
def test_an_older_bridge_without_hold_is_ros_unavailable(monkeypatch, body):
    """3.133 criterion 3, the deliberate fallback: an image built before 3.133
    sends no `hold`; it reads as before 3.127, never 3.127's guess. Still
    retryable (a 503), so explore waits exactly as before."""
    out = _post_goal(monkeypatch, (503, body))
    assert out["reason"] == "ros_unavailable", out
    assert out["retryable"] is True and out["bridge_status"] == 503, out


@pytest.mark.parametrize("err, reason", [
    (_err(503, {"error": "x", "hold": "slam_not_ready", "slam": "checking"}), "slam_not_ready"),
    (_err(503, {"error": "x", "hold": "nav2_starting"}), "nav2_starting"),
    (_err(503, {"error": "x", "hold": "nav2_restarting"}), None),    # unknown name
    (_err(503, {"error": "x", "hold": ["nav2_starting"]}), None),     # not a string
    (_err(503, {"error": "x", "hold": True}), None),
    (_err(502, {"error": "x", "hold": "nav2_starting"}), None),       # not a 503
    (_err(500, {"error": "x", "hold": "slam_not_ready"}), None),
])
def test_hold_reason_reads_only_a_known_hold_on_a_503(err, reason):
    assert bridge_faults.hold_reason(err) == reason
    assert set(bridge_faults.HOLDS) == {"nav2_starting", "slam_not_ready"}


# ---------- criterion 2: explore still waits on it ----------

def test_explore_waits_on_a_held_goal_and_fails_no_place():
    from brain.explore import ExploreAgent

    class _HeldNav:
        def set_goal(self, x_m, y_m):
            return {"accepted": False, "reason": "nav2_starting", "retryable": True,
                    "bridge_status": 503, "detail": "nav2 is not active"}

    agent = ExploreAgent.__new__(ExploreAgent)
    # Only what _send reads before the refusal is answered.
    agent.navigator = _HeldNav()
    agent._choice_t0 = None
    agent._attempts = []
    agent.events = []
    agent._event = lambda name, **kw: agent.events.append((name, kw))
    agent.goals_failed = 0
    action, moved, why = agent._send(2.0, 1.0, "frontier", now=0.0)
    assert (action, moved) == ("WAIT", False)
    assert why == "goal refused: nav2_starting"
    assert agent.goals_failed == 0
    assert agent.events == [("goal_refused", {"kind": "frontier", "goal": [2.0, 1.0],
                                              "reason": "nav2_starting"})]


# ---------- the contract across the wall: the bridge's own bodies ----------

from tests.test_review_376h1 import bridge_mod  # noqa: E402,F401  (the fixture)


def _as_error(code, body):
    req = httpx.Request("POST", "http://bridge/goal")
    return httpx.HTTPStatusError("x", request=req,
                                 response=httpx.Response(code, json=body, request=req))


def test_the_bridges_own_hold_bodies_read_as_their_holds(bridge_mod):  # noqa: F811
    """picar_bridge's real `_goal_refusal` replies (stubbed ROS), so a change
    to the bridge's body keys fails here rather than quietly reverting the
    twin to ROS DOWN."""
    from tests.test_nav2_ready_gate import _bridge, _Manager

    b = _bridge(bridge_mod, _Manager())          # SLAM ready, nav2 never answered
    code, body = b.send_goal({"x_m": 1.0, "y_m": 0.0})
    assert bridge_faults.hold_reason(_as_error(code, body)) == "nav2_starting", (code, body)

    b.slam_state = "starting"
    code, body = b.send_goal({"x_m": 1.0, "y_m": 0.0})
    assert bridge_faults.hold_reason(_as_error(code, body)) == "slam_not_ready", (code, body)
