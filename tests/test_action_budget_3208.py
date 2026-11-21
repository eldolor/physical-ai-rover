# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.208 (with amendment 1): a robot request that gets no answer is a
budgeted fault of the robot LINK, and a failed action is never re-sent.

3.202's chaos run (K7) showed one connection reset on the brain's
`POST /action` ending the mission `failed`: `RemoteRobot` raised
`RobotTransportError` and `MissionRunner._step_tick`'s catch-all ended it.
Now a request with no HTTP answer, on any route, is `RobotUnreachable`
(`WorldUnreachable` for `/world/*`), recorded in the robot's `LinkHealth`.
A step tick in which one went unanswered sends no move (`LinkLost` from the
halt gate, even when the lost read was swallowed), stops the car and
records no step; nothing re-sends anything, and the next tick decides again
from fresh sensing. The mission ends `failed` when no request has been
answered for `LINK_DEAD_S` and `LINK_MIN_FAILED_TICKS` failed in a row
("robot link lost"), or after `LINK_MAX_FAILED_TICKS` ("robot link
failing"). Every answer the robot server gave keeps its meaning.

The robot is a real `RemoteRobot` (and `RemoteWorld`, sharing its link)
against an in-process `robot/server.py`: the real path MissionRunner ->
agent -> robot/safety.py -> RemoteRobot -> robot server -> MockRobot. The
fault is injected in the HTTP client: `before` loses the request before the
server sees it, `after` lets the server act on it and loses the answer.
"""

import random

import httpx
import pytest

from control.mission_runner import (
    FAILED, FOUND, LINK_DEAD_S, LINK_FAILING, LINK_LOST, LINK_MAX_FAILED_TICKS,
    LINK_MIN_FAILED_TICKS, LINK_RETRY_S, MissionRunner,
)
from control.remote_robot import LinkHealth, RemoteRobot, RobotTransportError, RobotUnreachable
from control.remote_world import RemoteWorld, WorldTransportError, WorldUnreachable
from tests.conftest import ASGI_BASE_URL, asgi_client

BUDGET = 150
MOVES = {"FORWARD", "REVERSE", "LEFT", "RIGHT"}
REQUEST_TIMEOUT_S = 10.0


class FakeClock:
    """The runner's clock: moves only when the runner idles, or when a
    request is made to take time (a blackholed link's timeout)."""

    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def idle(self, s):
        self.t += s


class FaultyLink:
    """Stands in for the httpx client both remote clients use. Every request
    is numbered; `plan(n, method, path, body)` returns None (forward it),
    "before" (no answer; the server never sees it), "after" (the server
    acts on it; the answer is lost) or "timeout" (no answer after
    REQUEST_TIMEOUT_S on `clock`). `decision` is the id of the agent's
    decision in flight (`count_decisions`), so each POST that reached the
    server is filed under the decision that sent it."""

    def __init__(self, client, plan=None, clock=None):
        self.client = client
        self.plan = plan or (lambda n, method, path, body: None)
        self.clock = clock
        self.n = 0
        self.reached = []          # (decision, action) per POST /action the server saw
        self.failed = []           # (n, method, path, action, how)
        self.wire = []             # every POST in order: "STOP" for /stop, else the action
        self.decision = None

    def request(self, method, url, json=None, headers=None, **kw):
        self.n += 1
        path = url[len(ASGI_BASE_URL):] if url.startswith(ASGI_BASE_URL) else url
        path = path.split("?")[0]
        body = json or {}
        action = body.get("action") if path == "/action" else ("STOP" if path == "/stop" else None)
        if method == "POST":
            self.wire.append(action)
        how = self.plan(self.n, method, path, body)
        if how in ("before", "timeout"):
            self.failed.append((self.n, method, path, action, how))
            if how == "timeout":
                if self.clock is not None:
                    self.clock.idle(REQUEST_TIMEOUT_S)
                raise httpx.ReadTimeout("timed out")
            raise httpx.RemoteProtocolError("Server disconnected without sending a response.")
        if method == "POST" and path == "/action":
            self.reached.append((self.decision, action))
        response = self.client.request(method, url, json=json, headers=headers, **kw)
        if how == "after":
            self.failed.append((self.n, method, path, action, how))
            raise httpx.ReadError("[Errno 104] Connection reset by peer")
        return response

    def close(self):
        self.client.close()


def stack(plan=None, house=None, monkeypatch=None, clock=None):
    if house is not None:
        monkeypatch.setenv("SIM_MAP", house)
    from robot.server import create_app
    client = asgi_client(create_app())
    link = FaultyLink(client, plan, clock)
    robot = RemoteRobot(ASGI_BASE_URL, client=link)
    world = RemoteWorld(ASGI_BASE_URL, client=link, link=robot.link)
    return link, robot, world


def runner_for(robot, world, clock=None, **kw):
    clock = clock or FakeClock()
    return MissionRunner(robot, target_object="red backpack", max_steps=BUDGET,
                         world=world, clock=clock, idle=clock.idle, **kw)


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


def drive(runner, max_ticks=1500, seen=None):
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
    """Lose the n-th move POST (not a stop, not a look)."""
    count = {"moves": 0}

    def plan(i, method, path, body):
        if path == "/action" and body.get("action") in MOVES:
            count["moves"] += 1
            if count["moves"] == n:
                return how
        return None
    return plan


def all_seen(seen):
    return [line for s in seen for line in s["log_tail"]]


# -- criterion 1: K7, and the action never sent twice -----------------------

@pytest.mark.parametrize("how", ["before", "after"])
def test_one_reset_on_post_action_does_not_end_the_mission(how):
    """K7, red before 3.208: one lost POST /action ended `failed`."""
    link, robot, world = stack(nth_move(5, how))
    runner = runner_for(robot, world)
    count_decisions(runner, link)
    runner.start()
    seen = []
    status = drive(runner, seen=seen)
    assert len(link.failed) == 1 and link.failed[0][4] == how
    assert status["outcome"] == FOUND, status["error"]
    counted = [s for s in seen if s["link_failures"] == 1]
    assert counted, "the lost action was never counted"
    assert any(f"robot link 1/{LINK_MIN_FAILED_TICKS}" in line for line in counted[0]["log_tail"])
    assert status["link_failures"] == 0, "a completed tick resets the count"
    per = moves_reached_per_decision(link)
    assert per and max(per.values()) == 1, "an action reached the robot twice"


def test_the_failed_action_is_never_resent_and_the_car_stops():
    """The POST after the lost one is the failed tick's stop, never the
    lost verb again; every move that reached the robot is its own decision."""
    link, robot, world = stack(nth_move(4, "before"))
    runner = runner_for(robot, world)
    count_decisions(runner, link)
    runner.start()
    status = drive(runner)
    assert status["outcome"] == FOUND, status["error"]
    i = [k for k, a in enumerate(link.wire) if a in MOVES][3]
    assert link.wire[i + 1] == "STOP", link.wire[i:i + 3]
    assert max(moves_reached_per_decision(link).values()) == 1


# -- criterion 2': the link budget -------------------------------------------

def _dead_from_fifth_move(clock, how):
    state = {"dead": False, "moves": 0, "t0": None}

    def plan(i, method, path, body):
        if path == "/action" and body.get("action") in MOVES and not state["dead"]:
            state["moves"] += 1
            if state["moves"] == 5:
                state["dead"], state["t0"] = True, clock()
        return how if state["dead"] else None
    return state, plan


def test_a_dead_link_ends_link_lost_within_the_deadline_and_never_moves_again():
    """Every request reset from the 5th move on (the K6 model): ends
    `failed` "robot link lost" within LINK_DEAD_S + one request timeout of
    the first failure, with no move after it."""
    clock = FakeClock()
    state, plan = _dead_from_fifth_move(clock, "before")
    link, robot, world = stack(plan, clock=clock)
    runner = runner_for(robot, world, clock=clock)
    runner.start()
    status = drive(runner)
    assert status["outcome"] == FAILED
    assert status["error"].startswith(f"{LINK_LOST}: no answer for "), status["error"]
    assert LINK_DEAD_S <= clock() - state["t0"] <= LINK_DEAD_S + REQUEST_TIMEOUT_S
    # four moves reached the robot; the fifth was the first lost request
    assert len([a for _d, a in link.reached if a in MOVES]) == 4, "a move after the first failure"


def test_a_blackholed_link_ends_after_three_failed_ticks():
    """Each request times out: the silence passes LINK_DEAD_S at once, and
    the AND's three failed ticks are what the end waits on."""
    clock = FakeClock()
    state, plan = _dead_from_fifth_move(clock, "timeout")
    link, robot, world = stack(plan, clock=clock)
    runner = runner_for(robot, world, clock=clock)
    runner.start()
    status = drive(runner)
    assert status["outcome"] == FAILED
    assert status["error"].startswith(f"{LINK_LOST}: no answer for "), status["error"]
    assert f"{LINK_MIN_FAILED_TICKS} failed ticks in a row" in status["error"]
    # each failed tick: its first request times out, then its stop does
    bound = LINK_MIN_FAILED_TICKS * (2 * REQUEST_TIMEOUT_S + LINK_RETRY_S) + REQUEST_TIMEOUT_S
    assert clock() - state["t0"] <= bound


def test_reads_answered_and_every_post_lost_ends_link_failing_after_twenty():
    """A link that answers reads but loses every move never falls silent:
    the backstop ends it after exactly LINK_MAX_FAILED_TICKS."""
    state = {"posts": 0}

    def plan(i, method, path, body):
        if path == "/action":
            state["posts"] += 1
            return "before" if state["posts"] > 3 else None
        return None
    link, robot, world = stack(plan)
    runner = runner_for(robot, world)
    runner.start()
    status = drive(runner)
    assert status["outcome"] == FAILED
    assert status["error"].startswith(
        f"{LINK_FAILING}: {LINK_MAX_FAILED_TICKS} ticks in a row with an unanswered request"), \
        status["error"]
    assert len([f for f in link.failed if f[2] == "/action"]) == LINK_MAX_FAILED_TICKS


def test_the_budget_counts_consecutive_ticks_only():
    """Two lost, a good tick, two more lost: never three in a row."""
    bad = {3, 4, 6, 7}
    count = {"moves": 0}

    def plan(i, method, path, body):
        if path == "/action" and body.get("action") in MOVES:
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
    """A 503 is an answer, not a lost link: kept."""
    link, robot, world = stack()
    real = link.request

    def refusing(method, url, json=None, headers=None, **kw):
        if method == "POST" and url.endswith("/action") and (json or {}).get("action") in MOVES:
            return httpx.Response(503, text="upstream gone", request=httpx.Request(method, url))
        return real(method, url, json=json, headers=headers, **kw)
    link.request = refusing
    runner = runner_for(robot, world)
    runner.start()
    status = drive(runner)
    assert status["outcome"] == FAILED
    assert status["error"].startswith("step failed:") and "HTTP 503" in status["error"]
    assert status["link_failures"] == 0


def test_a_lost_scan_inside_vision_lands_in_the_link_budget_not_visions():
    """The frontier reads its scan inside its vision_fn: wrapped as
    VisionUnavailable, but the lost request makes it a link failure."""
    state = {"scans": 0}

    def plan(i, method, path, body):
        if path == "/scan":
            state["scans"] += 1
            if state["scans"] == 10:
                return "before"
        return None
    link, robot, world = stack(plan)
    runner = runner_for(robot, world)
    runner.start()
    seen = []
    status = drive(runner, seen=seen)
    assert status["outcome"] == FOUND, status["error"]
    assert any(s["link_failures"] == 1 for s in seen)
    assert not any(s["vision_failures"] for s in seen)


def test_a_lost_frame_lands_in_the_link_budget_not_the_cameras():
    state = {"frames": 0}

    def plan(i, method, path, body):
        if path == "/frame":
            state["frames"] += 1
            if state["frames"] in (4, 5):
                return "before"
        return None
    link, robot, world = stack(plan)
    runner = runner_for(robot, world)
    runner.start()
    seen = []
    status = drive(runner, seen=seen)
    assert status["outcome"] == FOUND, status["error"]
    assert max(s["link_failures"] for s in seen) == 2
    assert not any(s["camera_failures"] for s in seen)


# -- criterion 2'': no move from partial sensing ------------------------------

def test_a_swallowed_lost_read_still_stops_the_tick_from_moving():
    """`_guarded_vision` swallows a failed odometry read. Lose one: that
    tick sends no movement or pan POST at all -- the next POST is the stop."""
    state = {"odo": 0, "at": None}

    def plan(i, method, path, body):
        if path == "/odometry":
            state["odo"] += 1
            if state["odo"] == 6:
                state["at"] = len(link.wire)
                return "before"
        return None
    link, robot, world = stack(plan)
    runner = runner_for(robot, world)
    runner.start()
    seen = []
    status = drive(runner, seen=seen)
    assert status["outcome"] == FOUND, status["error"]
    assert link.wire[state["at"]] == "STOP", link.wire[state["at"]:state["at"] + 3]
    assert any("robot link 1/" in line and "refusing" in line for line in all_seen(seen))


def test_a_stop_during_a_failed_action_is_unchanged():
    """stop() lands while the POST is in flight, then it fails: `cancelled`,
    not counted, and no second stop from the loser of the race."""
    holder = {}

    def plan(i, method, path, body):
        if path == "/action" and body.get("action") in MOVES and not holder.get("done"):
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
    assert status["link_failures"] == 0
    assert link.wire[holder["at"]:].count("STOP") == 1, "only stop()'s own stop"


# -- the clients' classification ----------------------------------------------

def _client(handler):
    return httpx.Client(transport=httpx.MockTransport(handler), base_url="http://robot")


def test_unreachable_is_raised_only_for_a_request_with_no_answer():
    def refused(request):
        raise httpx.ConnectError("[Errno 111] Connection refused")
    robot = RemoteRobot("http://robot", client=_client(refused))
    with pytest.raises(RobotUnreachable) as e:
        robot.drive_forward()
    assert isinstance(e.value, RobotTransportError), "old handlers still catch it"
    assert (e.value.method, e.value.path) == ("POST", "/action")
    with pytest.raises(RobotUnreachable):
        robot.get_distance()
    assert robot.link.counts() == (0, 2)

    def timeout(request):
        raise httpx.ReadTimeout("timed out")
    with pytest.raises(RobotUnreachable):
        RemoteRobot("http://robot", client=_client(timeout)).turn_left()

    def bad_url(request):
        raise httpx.UnsupportedProtocol("Request URL is missing an 'http://' or 'https://' protocol.")
    robot = RemoteRobot("http://robot", client=_client(bad_url))
    with pytest.raises(RobotTransportError) as e:
        robot.drive_forward()
    assert not isinstance(e.value, RobotUnreachable), "our configuration is not the link"
    assert robot.link.counts() == (0, 0)

    def answer_503(request):
        return httpx.Response(503, text="no")
    robot = RemoteRobot("http://robot", client=_client(answer_503))
    with pytest.raises(RobotTransportError) as e:
        robot.drive_forward()
    assert not isinstance(e.value, RobotUnreachable), "an answer is not a lost link"
    assert robot.link.counts() == (1, 0)


def test_the_world_shares_the_robots_link():
    link = LinkHealth()

    def reset(request):
        raise httpx.ReadError("[Errno 104] Connection reset by peer")
    world = RemoteWorld("http://robot", client=_client(reset), link=link)
    with pytest.raises(WorldUnreachable) as e:
        world.get_pose()
    assert isinstance(e.value, WorldTransportError) and e.value.status is None
    assert link.counts() == (0, 1)

    def answer_500(request):
        return httpx.Response(500, text="no")
    world = RemoteWorld("http://robot", client=_client(answer_500), link=link)
    with pytest.raises(WorldTransportError) as e:
        world.get_pose()
    assert not isinstance(e.value, WorldUnreachable)
    assert link.counts() == (1, 1)


def test_the_brain_servers_world_shares_the_robots_link_only_on_its_server():
    from control.brain_server import remote_world_for
    robot = RemoteRobot("http://127.0.0.1:8000/")
    config = {"robot_url": "http://127.0.0.1:8000", "request_timeout_s": 10.0}
    assert remote_world_for(robot, config, None).link is robot.link
    elsewhere = remote_world_for(robot, {**config, "world_url": "http://bridge:9000"}, None)
    assert elsewhere.link is not robot.link


# -- criterion 4' pinned slices (the sweep is evaluations/action-budget-3208/) --

@pytest.mark.parametrize("model", ["all", "action"])
def test_five_percent_loss_still_finds_and_nothing_is_sent_twice(model):
    rng = random.Random(3)

    def plan(i, method, path, body):
        if model == "action" and path != "/action":
            return None
        if rng.random() < 0.05:
            return rng.choice(["before", "after"])
        return None
    link, robot, world = stack(plan)
    runner = runner_for(robot, world)
    count_decisions(runner, link)
    runner.start()
    status = drive(runner)
    assert link.failed, "no fault was injected"
    assert status["outcome"] == FOUND, status["error"]
    assert max(moves_reached_per_decision(link).values()) == 1
