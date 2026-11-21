# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.208: a transport failure on `POST /action` is a budgeted fault, and a
failed action is never re-sent.

3.202's chaos run (K7) showed one connection reset on the brain's
`POST /action` ending the mission `failed`: `RemoteRobot` raised
`RobotTransportError` and `MissionRunner._step_tick`'s catch-all ended the
mission at once. Now a request that got no HTTP answer is `RobotUnreachable`
(a `RobotTransportError`); that tick stops the car and records no step, and
`DEFAULT_MAX_ACTION_FAILURES` (3) consecutive such ticks end the mission
`failed` ("robot unreachable 3 times in a row: ..."). Nothing re-sends the
POST: the next tick decides again from fresh sensing. Every answer the
robot server gave (a veto, a preemption, a 4xx, a 5xx) keeps its meaning.

The robot here is a real `RemoteRobot` against an in-process
`robot/server.py` (the real path: MissionRunner -> agent ->
robot/safety.py -> RemoteRobot -> robot server -> MockRobot), with the
fault injected in the HTTP client: `before` loses the request before the
server sees it, `after` lets the server execute it and loses the answer.
"""

import random

import httpx
import pytest

from control.mission_runner import (
    ACTION_RETRY_S, DEFAULT_MAX_ACTION_FAILURES, FAILED, FOUND, MissionRunner,
)
from control.remote_robot import RemoteRobot, RobotTransportError, RobotUnreachable
from control.remote_world import RemoteWorld
from tests.conftest import ASGI_BASE_URL, asgi_client

BUDGET = 150
MOVES = {"FORWARD", "REVERSE", "LEFT", "RIGHT"}


class FaultyLink:
    """Stands in for the httpx client `RemoteRobot` uses. Every
    `POST /action` is numbered; `plan(n, body)` returns None (forward it),
    "before" (reset before the server sees it) or "after" (the server
    executes it, the answer is lost). `decision` is the id of the agent's
    decision in flight (set by `count_decisions`), so each POST that reached
    the server is filed under the decision that sent it."""

    def __init__(self, client, plan=None):
        self.client = client
        self.plan = plan or (lambda n, body: None)
        self.posts = 0
        self.reached = []          # (decision, action) per POST the server saw
        self.failed = []           # (n, action, how)
        self.decision = None
        self.wire = []             # every POST, in order: "STOP" for /stop, else the action

    def request(self, method, url, json=None, headers=None):
        if method == "POST" and url.endswith("/stop"):
            self.wire.append("STOP")
        if method == "POST" and url.endswith("/action"):
            self.wire.append((json or {}).get("action"))
            self.posts += 1
            action = (json or {}).get("action")
            how = self.plan(self.posts, json or {})
            if how == "before":
                self.failed.append((self.posts, action, how))
                raise httpx.RemoteProtocolError("Server disconnected without sending a response.")
            self.reached.append((self.decision, action))
            response = self.client.request(method, url, json=json, headers=headers)
            if how == "after":
                self.failed.append((self.posts, action, how))
                raise httpx.ReadError("[Errno 104] Connection reset by peer")
            return response
        return self.client.request(method, url, json=json, headers=headers)

    def close(self):
        self.client.close()


def stack(plan=None, house=None, monkeypatch=None):
    if house is not None:
        monkeypatch.setenv("SIM_MAP", house)
    from robot.server import create_app
    client = asgi_client(create_app())
    link = FaultyLink(client, plan)
    robot = RemoteRobot(ASGI_BASE_URL, client=link)
    world = RemoteWorld(ASGI_BASE_URL, client=client)
    return link, robot, world


def runner_for(robot, world, **kw):
    return MissionRunner(robot, target_object="red backpack", max_steps=BUDGET,
                         world=world, idle=lambda s: None, **kw)


def count_decisions(runner, link):
    """File every POST under the agent decision that sent it: each
    `check_and_execute` call is one decided action."""
    safety = runner.agent.safety
    real = safety.check_and_execute
    seq = {"n": 0}

    def wrapped(action, **kw):
        seq["n"] += 1
        link.decision = seq["n"]
        try:
            return real(action, **kw)
        finally:
            link.decision = None
    safety.check_and_execute = wrapped


def drive(runner, max_ticks=1000, seen=None):
    ticks = 0
    while runner.tick():
        if seen is not None:
            seen.append(runner.status())
        ticks += 1
        assert ticks < max_ticks, "runner never terminated"
    return runner.status()


def moves_reached_per_decision(link):
    per = {}
    for decision, action in link.reached:
        if decision is not None and action in MOVES:
            per[decision] = per.get(decision, 0) + 1
    return per


def nth_move(n, how):
    """Fail the n-th move POST (not a stop, not a look)."""
    count = {"moves": 0}

    def plan(i, body):
        if body.get("action") in MOVES:
            count["moves"] += 1
            if count["moves"] == n:
                return how
        return None
    return plan


@pytest.mark.parametrize("how", ["before", "after"])
def test_one_reset_on_post_action_does_not_end_the_mission(how):
    """K7, red before 3.208: one lost POST /action ended `failed`."""
    link, robot, world = stack(nth_move(5, how))
    runner = runner_for(robot, world)
    count_decisions(runner, link)
    runner.start()
    seen = []
    status = drive(runner, seen=seen)
    assert len(link.failed) == 1 and link.failed[0][2] == how
    assert status["outcome"] == FOUND, status["error"]
    counted = [s for s in seen if s["action_failures"] == 1]
    assert counted, "the failed action was never counted"
    assert any("robot unreachable 1/3" in line for line in counted[0]["log_tail"])
    assert status["action_failures"] == 0, "a completed tick resets the count"
    per = moves_reached_per_decision(link)
    assert per and max(per.values()) == 1, "an action reached the robot twice"


def test_the_failed_action_is_never_resent_and_the_car_stops():
    """The POST after the failed one is the failed tick's stop, never the
    failed verb again; every move that reached the robot belongs to its own
    decision."""
    link, robot, world = stack(nth_move(4, "before"))
    runner = runner_for(robot, world)
    count_decisions(runner, link)
    runner.start()
    status = drive(runner)
    assert status["outcome"] == FOUND, status["error"]
    i = [k for k, a in enumerate(link.wire) if a in MOVES][3]
    assert link.wire[i + 1] == "STOP", link.wire[i:i + 3]
    per = moves_reached_per_decision(link)
    assert max(per.values()) == 1
    assert sum(per.values()) == len([d for d, a in link.reached if a in MOVES])


def test_a_link_that_never_comes_back_ends_failed_after_three():
    link, robot, world = stack(lambda i, body: "before" if (
        body.get("action") in MOVES and link.posts >= 6) else None)
    waits = []
    runner = MissionRunner(robot, target_object="red backpack", max_steps=BUDGET,
                           world=world, idle=waits.append)
    runner.start()
    status = drive(runner)
    assert status["outcome"] == FAILED
    assert status["error"].startswith(
        f"robot unreachable {DEFAULT_MAX_ACTION_FAILURES} times in a row")
    assert "Server disconnected" in status["error"]
    move_failures = [f for f in link.failed if f[1] in MOVES]
    assert len(move_failures) == DEFAULT_MAX_ACTION_FAILURES, "exactly three failed ticks"
    assert waits.count(ACTION_RETRY_S) == DEFAULT_MAX_ACTION_FAILURES - 1
    first = link.failed[0][0]
    assert not [a for d, a in link.reached[first:] if a in MOVES], "a move after the first failure"


def test_the_budget_counts_consecutive_ticks_only():
    """Two failures, a good tick, two more: never three in a row."""
    bad = {3, 4, 6, 7}
    count = {"moves": 0}

    def plan(i, body):
        if body.get("action") in MOVES:
            count["moves"] += 1
            if count["moves"] in bad:
                return "after"
        return None
    link, robot, world = stack(plan)
    runner = runner_for(robot, world)
    runner.start()
    status = drive(runner)
    assert status["outcome"] == FOUND, status["error"]


def test_an_answer_the_robot_gave_still_ends_the_mission_at_once():
    """A 5xx (or any error status) is an answer, not a lost link: kept."""
    from robot.server import create_app
    client = asgi_client(create_app())

    class Refusing(FaultyLink):
        def request(self, method, url, json=None, headers=None):
            if method == "POST" and url.endswith("/action") and (json or {}).get("action") in MOVES:
                return httpx.Response(503, text="upstream gone",
                                      request=httpx.Request(method, url))
            return self.client.request(method, url, json=json, headers=headers)
    link = Refusing(client)
    robot = RemoteRobot(ASGI_BASE_URL, client=link)
    runner = runner_for(robot, RemoteWorld(ASGI_BASE_URL, client=client))
    runner.start()
    status = drive(runner)
    assert status["outcome"] == FAILED
    assert status["error"].startswith("step failed:") and "HTTP 503" in status["error"]
    assert status["action_failures"] == 0


def test_a_transport_failure_on_a_read_still_ends_the_mission_at_once():
    """Out of scope (recorded in 3.208): a lost GET is not budgeted here.
    The frontier policy reads its scan inside its vision_fn, so a lost scan
    is B3.2's vision failure, as before 3.208 -- never an action failure."""
    from robot.server import create_app
    client = asgi_client(create_app())

    class LostScan(FaultyLink):
        def request(self, method, url, json=None, headers=None):
            if method == "GET" and "/scan" in url:
                raise httpx.ReadError("[Errno 104] Connection reset by peer")
            return self.client.request(method, url, json=json, headers=headers)
    robot = RemoteRobot(ASGI_BASE_URL, client=LostScan(client))
    runner = runner_for(robot, RemoteWorld(ASGI_BASE_URL, client=client))
    runner.start()
    seen = []
    status = drive(runner, seen=seen)
    assert status["outcome"] == FAILED
    assert status["error"].startswith("vision unavailable 3 times in a row")
    assert "robot unreachable" not in status["error"]
    assert not any(s["action_failures"] for s in seen)


def test_a_stop_during_a_failed_action_is_unchanged():
    """stop() lands while the POST is in flight, then it fails: `cancelled`,
    not counted, and no second stop from the loser of the race."""
    holder = {}

    def plan(i, body):
        if body.get("action") in MOVES and not holder.get("done"):
            holder["done"] = True
            holder["at"] = len(link.wire)
            holder["runner"].stop()
            return "before"
        return None
    link, robot, world = stack(plan)
    runner = holder["runner"] = runner_for(robot, world)
    runner.start()
    drive(runner)
    status = runner.status()
    assert status["outcome"] == "cancelled"
    assert status["action_failures"] == 0
    assert link.wire[holder["at"]:].count("STOP") == 1, "only stop()'s own stop"


def test_unreachable_is_raised_only_for_a_request_with_no_answer():
    """RemoteRobot's own classification, unit level."""
    def handler(request):
        raise httpx.ConnectError("[Errno 111] Connection refused")
    robot = RemoteRobot("http://robot", client=httpx.Client(
        transport=httpx.MockTransport(handler), base_url="http://robot"))
    with pytest.raises(RobotUnreachable) as e:
        robot.drive_forward()
    assert isinstance(e.value, RobotTransportError), "old handlers still catch it"
    assert e.value.method == "POST" and e.value.path == "/action"

    def timeout(request):
        raise httpx.ReadTimeout("timed out")
    robot = RemoteRobot("http://robot", client=httpx.Client(
        transport=httpx.MockTransport(timeout), base_url="http://robot"))
    with pytest.raises(RobotUnreachable):
        robot.turn_left()

    def bad_url(request):
        raise httpx.UnsupportedProtocol("Request URL is missing an 'http://' or 'https://' protocol.")
    robot = RemoteRobot("http://robot", client=httpx.Client(
        transport=httpx.MockTransport(bad_url), base_url="http://robot"))
    with pytest.raises(RobotTransportError) as e:
        robot.drive_forward()
    assert not isinstance(e.value, RobotUnreachable), "our configuration is not the link"

    def answer_503(request):
        return httpx.Response(503, text="no")
    robot = RemoteRobot("http://robot", client=httpx.Client(
        transport=httpx.MockTransport(answer_503), base_url="http://robot"))
    with pytest.raises(RobotTransportError) as e:
        robot.drive_forward()
    assert not isinstance(e.value, RobotUnreachable), "an answer is not a lost link"


@pytest.mark.parametrize("seed", [1, 2])
def test_five_percent_resets_still_find_and_nothing_is_sent_twice(seed):
    """Criterion 4's pinned slice (the sweep is
    evaluations/action-budget-3208/sweep.py): 5% of POST /action lost,
    half before and half after the robot saw it."""
    rng = random.Random(seed)

    def plan(i, body):
        if rng.random() < 0.05:
            return rng.choice(["before", "after"])
        return None
    link, robot, world = stack(plan)
    runner = runner_for(robot, world)
    count_decisions(runner, link)
    runner.start()
    status = drive(runner)
    assert status["outcome"] == FOUND, status["error"]
    per = moves_reached_per_decision(link)
    assert max(per.values()) == 1
