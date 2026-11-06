# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_goal_lease.py

`PLAN-ros-alignment.md` 3.145 criterion 1: a nav2 goal the brain sent lives
only while the brain does. 3.135.B's F6g and F7g (the board, 5 of 5 each): a
brain killed or frozen with its explore goal driving left nav2 driving it on
to its end, 2.8-3.3 s and 17-19 cm, because nav2's wheel posts feed the
watchdog. Now the brain beats `POST /autonomy/heartbeat`; a goal it sends
holds a lease that lapses `watchdog_timeout_s` after its last beat, ending
the goal as a Stop does and latching autonomy out (`autonomy_lapsed`) until
a mission starts.

In process, against 3.82's fake bridge (tests/test_refused_goal_hold.py),
with the app's lifespan running so the watchdog loop does the judging.
"""

import threading
import time

import pytest

from control.mission_runner import FAILED
from control.remote_robot import RemoteRobot
from robot.interface import AUTONOMY_LAPSED, AutonomyLapsed
from tests.conftest import ASGI_BASE_URL
from tests.test_person_stop import _nav2
from tests.test_refused_goal_hold import _explore_over_http, _server_over

BRAIN = {"x-driver": "brain"}
LEASE_S = 1.0            # config/robot.yaml's watchdog_timeout_s
JUDGE_SLACK_S = 0.1      # the watchdog loop's period: criterion 1's bar
SCHED_SLACK_S = 0.25     # a busy test host's thread scheduling, beside it


@pytest.fixture
def live(monkeypatch):
    """The robot server over the fake bridge, its lifespan (watchdog) running."""
    client, bridge = _server_over(monkeypatch, None)
    with client:
        assert client.get("/health").json()["watchdog_timeout_s"] == LEASE_S
        yield client, bridge


def _beat(client, headers=BRAIN):
    r = client.post("/autonomy/heartbeat", headers=headers)
    assert r.status_code == 200, r.text
    return r.json()


def _goal(client, bridge, headers):
    r = client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}, headers=headers).json()
    assert r.get("accepted") is True, r
    assert bridge.goal and bridge.goal["state"] == "active", bridge.goal
    return r


def _lease(client):
    return client.get("/health").json()["goal_lease"]


def _active(bridge):
    return bool(bridge.goal and bridge.goal.get("state") == "active")


def _watch(client, bridge, seconds):
    """nav2 posting at 20 Hz for `seconds`; returns when it was first held
    at zero or its goal ended, or None if it drove the whole time."""
    tick, speed = _nav2(client, bridge)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        tick()
        if not _active(bridge) or speed() == 0.0:
            return time.monotonic()
        time.sleep(0.05)
    return None


def test_a_silent_brains_goal_ends_within_its_lease_and_latches_autonomy(live):
    """F6g/F7g in process: the brain's last beat, then silence. Before 3.145
    nav2 drove the goal on to its end; now it is held and cancelled one lease
    after the last beat, and the brain is refused by name until a start."""
    client, bridge = live
    _beat(client)
    last_beat = time.monotonic()
    _goal(client, bridge, BRAIN)
    assert _lease(client)["owner"] == "brain"
    tick, speed = _nav2(client, bridge)
    tick()
    assert speed() > 0.0, "the fake nav2 never drove -- the test proves nothing"
    held_at = _watch(client, bridge, 3.0)
    assert held_at is not None, "nav2 drove the silent brain's goal for 3 s"
    assert held_at - last_beat <= LEASE_S + JUDGE_SLACK_S + SCHED_SLACK_S, held_at - last_beat
    deadline = time.monotonic() + 2.0
    while _active(bridge) and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not _active(bridge), bridge.goal
    lease = _lease(client)
    assert lease["lapses"] == 1 and lease["owner"] is None, lease
    assert lease["last_lapse"]["owner"] == "brain", lease

    # The brain, thawed: its verbs and goals are refused by name...
    assert _beat(client)["lapsed"] is True
    v = client.post("/action", json={"action": "LOOK_LEFT"}, headers=BRAIN).json()
    assert v["executed"] is False and v["reason"] == AUTONOMY_LAPSED, v
    g = client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}, headers=BRAIN).json()
    assert g["accepted"] is False and g["reason"] == AUTONOMY_LAPSED, g
    # ...and RemoteRobot hears it as the end of the mission, not a veto.
    with pytest.raises(AutonomyLapsed):
        RemoteRobot(ASGI_BASE_URL, client=client).look_left()
    # A person still drives, and a mission start lifts the latch.
    p = client.post("/action", json={"action": "LOOK_RIGHT"},
                    headers={"x-driver": "twin-dpad"}).json()
    assert p["executed"] is True, p
    r = client.post("/autonomy/resume", headers=BRAIN).json()
    assert r["resumed"] is True and r["stopped_by"] == "brain", r
    assert _beat(client)["lapsed"] is False


def test_a_beating_brain_keeps_its_goal(live):
    """No false lapse: a brain beating every 0.25 s with no other command
    for 5 s (a long VLM call, say) keeps its goal driving."""
    client, bridge = live
    stop = threading.Event()

    def beats():
        while not stop.wait(0.25):
            _beat(client)

    _beat(client)
    _goal(client, bridge, BRAIN)
    t = threading.Thread(target=beats, daemon=True)
    t.start()
    try:
        assert _watch(client, bridge, 5.0) is None, "a beating brain's goal was ended"
    finally:
        stop.set()
        t.join()
    assert _lease(client)["lapses"] == 0
    # Released, not lapsed, when its navigator closes.
    r = client.delete("/autonomy/heartbeat", headers=BRAIN).json()
    assert r["released"] is True, r
    assert _watch(client, bridge, 1.5) is None
    assert _lease(client)["lapses"] == 0


def test_a_brain_that_never_beats_keeps_the_old_behaviour(live):
    """A brain older than 3.145 sends no beat: its goal is not leased."""
    client, bridge = live
    _goal(client, bridge, BRAIN)
    assert _lease(client)["owner"] is None
    assert _watch(client, bridge, 1.8) is None
    assert _lease(client)["lapses"] == 0


@pytest.mark.parametrize("headers", [{"x-driver": "twin-dpad"}, {}], ids=["person", "unnamed"])
def test_a_person_or_unnamed_goal_is_never_leased(live, headers):
    """The brain beat once and fell silent, but the goal is a person's tap
    or a script's: it is theirs, and nothing about the brain ends it."""
    client, bridge = live
    _beat(client)
    _goal(client, bridge, headers)
    assert _lease(client)["owner"] is None
    # A person's tap holds the robot as a press does, refusing nav2's posts
    # until it lapses: past that, past a lease too, the goal still drives.
    time.sleep(LEASE_S + 0.3)
    assert _active(bridge), bridge.goal
    assert _watch(client, bridge, 1.0) is None
    assert _lease(client)["lapses"] == 0


def test_a_brains_own_cancel_releases_its_lease(live):
    client, bridge = live
    _beat(client)
    _goal(client, bridge, BRAIN)
    client.delete("/world/goal", headers=BRAIN)
    assert _lease(client)["owner"] is None
    time.sleep(LEASE_S + 0.4)
    assert _lease(client)["lapses"] == 0


def test_a_persons_stop_latch_is_kept_over_a_lapse(live):
    """A person's latch set while the brain's goal still held its lease (a
    relatch after a refused start): the lapse ends the goal but the latch
    stays the person's -- the mission ends `cancelled`, theirs."""
    client, bridge = live
    _beat(client)
    _goal(client, bridge, BRAIN)
    assert client.post("/autonomy/relatch", json={"by": "twin-dpad"}).json()["latched"] is True
    assert _watch(client, bridge, 3.0) is not None
    v = client.post("/action", json={"action": "LOOK_LEFT"}, headers=BRAIN).json()
    assert v["reason"] == "person_stop", v
    assert _lease(client)["lapses"] == 1


def test_a_frozen_explore_mission_ends_failed_naming_the_lapse(live):
    """The brain side, over HTTP: explore's navigator beats from its first
    goal; frozen (its beats stop, as SIGSTOP stops them), the goal is ended;
    thawed, its next goal is refused and the mission ends `failed`, naming
    `autonomy_lapsed` -- never a person's `cancelled`."""
    client, bridge = live
    runner = _explore_over_http(client)
    nav = runner._navigator._nav
    runner.start()
    for _ in range(400):
        assert runner.tick(), runner.status()
        if _active(bridge):
            break
    assert _active(bridge), "explore never sent a goal"
    assert _lease(client)["owner"] == "brain", "explore's goal was not leased"
    nav._beats_stop.set()                        # frozen: no beat, no goodbye
    assert _watch(client, bridge, 3.0) is not None
    for _ in range(200):
        if not runner.tick():
            break
    status = runner.status()
    assert status["outcome"] == FAILED, status
    assert AUTONOMY_LAPSED in str(status.get("end_reason")), status
