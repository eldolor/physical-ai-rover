# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_watchdog_reason.py

3.164: the watchdog's record beside a Stop's or fault's reason. `last_refusal`
is replaced about a second after whatever really stopped the robot, so
`/health` now keeps `last_stop_reason` beside it, and says whether a silence
was a hung brain (`watchdog.trip`) or a mission that ended (calm).

A live `uvicorn` robot server with a real event loop, as
tests/test_watchdog_integration.py: the judging is the watchdog loop's.
"""

import os
import subprocess
import sys
import time

import httpx
import pytest
import yaml

from control.health import check_robot
from tests.conftest import REPO_ROOT, SERVER_START_TIMEOUT_S, free_port
from tests.test_goal_lease import _beat, _goal, live  # noqa: F401

WATCHDOG_S = 0.3
TICK_TIMEOUT_S = 2.0
TRIP_AFTER_S = 0.8 * TICK_TIMEOUT_S
SLACK_S = 0.5
BRAIN = {"x-driver": "brain"}
PERSON = {"x-driver": "twin-dpad"}


@pytest.fixture(scope="module")
def url(tmp_path_factory):
    base = yaml.safe_load((REPO_ROOT / "config" / "robot.yaml").read_text())
    base["safety"]["watchdog_timeout_s"] = WATCHDOG_S
    base.setdefault("brain", {})["tick_timeout_s"] = TICK_TIMEOUT_S
    base.setdefault("sim", {})["realtime"] = True
    config = tmp_path_factory.mktemp("cfg") / "robot.yaml"
    config.write_text(yaml.safe_dump(base))
    port = free_port()
    proc = subprocess.Popen(
        [sys.executable, "-m", "uvicorn", "robot.server:app",
         "--host", "127.0.0.1", "--port", str(port), "--log-level", "warning"],
        cwd=REPO_ROOT, env={**os.environ, "ROBOT_CONFIG_PATH": str(config)})
    base_url = f"http://127.0.0.1:{port}"
    try:
        deadline = time.monotonic() + SERVER_START_TIMEOUT_S
        while True:
            assert proc.poll() is None, "the robot server exited early"
            assert time.monotonic() < deadline, "the robot server never answered"
            try:
                if httpx.get(f"{base_url}/health", timeout=0.5).status_code == 200:
                    break
            except httpx.HTTPError:
                time.sleep(0.05)
        yield base_url
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


def _health(url):
    return httpx.get(f"{url}/health").json()


def _wd(url):
    return _health(url)["watchdog"]


def _start(url):
    """A mission's Start, as the brain sends it, with nobody holding the robot."""
    time.sleep(WATCHDOG_S + SLACK_S)
    assert httpx.post(f"{url}/autonomy/resume", headers=BRAIN).status_code == 200


def _move(url, headers=BRAIN):
    return httpx.post(f"{url}/action", json={"action": "LOOK_CENTER"}, headers=headers).json()


def _stop(url, headers=BRAIN):
    assert httpx.post(f"{url}/stop", headers=headers).status_code == 200


def test_a_brain_that_goes_silent_mid_mission_is_a_trip(url):
    _start(url)
    assert _move(url)["executed"] is True
    assert _wd(url)["mission_live"] is True
    time.sleep(TRIP_AFTER_S + SLACK_S)
    wd = _wd(url)
    assert wd["trip"]["how"] == "silence" and wd["trip"]["driver"] == "brain", wd
    assert wd["trip"]["quiet_s"] >= TRIP_AFTER_S
    assert wd["last_fired"]["during"] == "mission", wd
    assert wd["trip_after_s"] == pytest.approx(TRIP_AFTER_S)


def test_a_trip_outlives_the_quiet_record_after_it_and_goes_at_the_next_start(url):
    """B3.3 ends a hung tick with the brain's own /stop; the watchdog then
    fires again over a mission that is over. That must not erase the hang."""
    _start(url)
    _move(url)
    time.sleep(TRIP_AFTER_S + SLACK_S)
    _stop(url)
    time.sleep(WATCHDOG_S + SLACK_S)
    wd = _wd(url)
    assert wd["mission_live"] is False and wd["last_fired"]["during"] is None, wd
    assert wd["trip"] is not None
    _start(url)
    assert _wd(url)["trip"] is None


def test_a_mission_that_ends_reads_calm(url):
    _start(url)
    _move(url)
    _stop(url)
    time.sleep(TRIP_AFTER_S + SLACK_S)
    wd = _wd(url)
    assert wd["trip"] is None and wd["mission_live"] is False, wd
    assert wd["last_fired"]["during"] is None, wd


def test_a_healthy_wait_longer_than_the_watchdog_is_no_trip(url):
    """A cloud call: B3.1 fires in the wait, and the brain carries on."""
    _start(url)
    for _ in range(2):
        _move(url)
        time.sleep(WATCHDOG_S + 0.5)
        wd = _wd(url)
        assert wd["last_fired"]["during"] == "mission" and wd["trip"] is None, wd
    _stop(url)


def test_a_start_that_never_moves_is_not_a_mission(url):
    """`/autonomy/resume` comes before the build, and a Start refused with
    nothing to relatch sends nothing after it."""
    _start(url)
    time.sleep(TRIP_AFTER_S + SLACK_S)
    wd = _wd(url)
    assert wd["mission_live"] is False and wd["trip"] is None, wd


def test_the_brains_stop_on_a_blind_step_does_not_end_the_mission(url):
    """`_safe_stop()` runs on each blind step too: a move after it is the
    mission still going, and a hang after that move is a hang."""
    _start(url)
    _move(url)
    _stop(url)
    assert _wd(url)["mission_live"] is False
    _move(url)
    assert _wd(url)["mission_live"] is True
    time.sleep(TRIP_AFTER_S + SLACK_S)
    assert _wd(url)["trip"]["how"] == "silence"


def test_a_navigator_still_beating_is_a_living_brain(url):
    """nav2 may pause; the brain beats on its own thread."""
    _start(url)
    _move(url)
    beat = {**BRAIN, "x-lease-id": "n1"}
    end = time.monotonic() + TRIP_AFTER_S + SLACK_S
    while time.monotonic() < end:
        httpx.post(f"{url}/autonomy/heartbeat", headers=beat)
        time.sleep(WATCHDOG_S / 3)
    assert _wd(url)["trip"] is None
    httpx.delete(f"{url}/autonomy/heartbeat", headers=beat)
    _stop(url)


def test_a_persons_stop_is_kept_beside_the_watchdogs_record(url):
    _start(url)
    _move(url)
    _stop(url, PERSON)
    time.sleep(TRIP_AFTER_S + SLACK_S)
    health = _health(url)
    assert health["last_refusal"]["reason"] == "watchdog", health["last_refusal"]
    assert health["last_stop_reason"]["reason"] == "person_stop"
    assert health["last_stop_reason"]["driver"] == "twin-dpad"
    assert health["last_stop_reason"]["seconds_ago"] > 0
    assert health["watchdog"]["trip"] is None, "a person ended it: no hang"
    assert health["watchdog"]["mission_live"] is False


def test_a_refusal_is_kept_beside_the_watchdogs_record(url):
    _start(url)
    _move(url, PERSON)
    refused = _move(url)
    assert refused["reason"] == "preempted", refused
    time.sleep(WATCHDOG_S + SLACK_S)
    health = _health(url)
    assert health["last_refusal"]["reason"] == "watchdog", health["last_refusal"]
    assert health["last_stop_reason"]["reason"] == "preempted"
    assert health["last_stop_reason"]["driver"] == "brain"


def test_a_lapse_is_a_trip_at_once_and_its_reason_is_kept(live):  # noqa: F811
    """3.145's lease: over the fake bridge, the app's own watchdog loop. A
    silent brain's goal lapses in a second, and that is already a hang."""
    client, bridge = live
    assert client.post("/autonomy/resume", headers=BRAIN).status_code == 200
    _beat(client)
    _goal(client, bridge, BRAIN)
    assert client.get("/health").json()["watchdog"]["mission_live"] is True
    end = time.monotonic() + 3.0
    while time.monotonic() < end and client.get("/health").json()["watchdog"]["trip"] is None:
        time.sleep(0.05)
    deadline = time.monotonic() + 1.0
    while (time.monotonic() < deadline
           and client.get("/health").json()["last_refusal"]["reason"] != "watchdog"):
        time.sleep(0.05)
    time.sleep(float(os.environ.get("WATCHDOG_REASON_HOLD_S", "0")))
    health = client.get("/health").json()
    assert health["watchdog"]["trip"]["how"] == "lease", health["watchdog"]
    assert health["last_refusal"]["reason"] == "watchdog", health["last_refusal"]
    assert health["last_stop_reason"]["reason"] == "autonomy_lapsed", health["last_stop_reason"]


def test_health_describes_the_new_fields_and_never_judges_them(url):
    """M5: only what a release can be blamed for reaches a verdict."""
    _start(url)
    _move(url)
    time.sleep(TRIP_AFTER_S + SLACK_S)
    report = check_robot(url)
    assert report["description"]["watchdog"]["trip"]["how"] == "silence", report
    assert report["description"]["last_stop_reason"] is not None
    assert report["problems"] == [] and report["status"] == "ok", report
