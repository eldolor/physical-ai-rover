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

import json
import os
import subprocess
import sys
import threading
import time

import httpx
import pytest
import yaml

from control import remote_robot
from control.health import check_robot
from control.mission_runner import MissionRunner
from control.remote_robot import RemoteRobot, RobotTransportError
from control.remote_world import RemoteWorld
from robot import server
from robot.safety import SafetyController
from tests.conftest import REPO_ROOT, SERVER_START_TIMEOUT_S, asgi_client, free_port
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


def test_a_hang_is_caught_while_the_actuator_keeps_the_watchdog_fed(url):
    """Under `drive: ros` the plugin's posts refresh `last_command_at`, so
    B3.1 never fires; the brain's own silence is what counts."""
    _start(url)
    _move(url)
    time.sleep(WATCHDOG_S + 0.1)        # the brain's claim lapses, so ros may post
    end = time.monotonic() + TRIP_AFTER_S + SLACK_S
    while time.monotonic() < end:
        r = httpx.post(f"{url}/wheels", json={"left_rad_s": 0.01, "right_rad_s": 0.01},
                       headers={"x-driver": "ros"}).json()
        assert r.get("executed") is True, r
        time.sleep(WATCHDOG_S / 3)
    health = _health(url)
    assert health["seconds_since_last_command"] < WATCHDOG_S, "the watchdog was not kept fed"
    assert health["watchdog"]["trip"]["how"] == "silence", health["watchdog"]
    httpx.post(f"{url}/wheels", json={"left_rad_s": 0.0, "right_rad_s": 0.0},
               headers={"x-driver": "ros"})


def test_a_verbs_end_is_the_brains_last_word(tmp_path, monkeypatch):
    """A long verb then a long wait is no longer than the wait. In process,
    with a verb that takes a second: the simulator's take no time."""
    base = yaml.safe_load((REPO_ROOT / "config" / "robot.yaml").read_text())
    base["safety"]["watchdog_timeout_s"] = WATCHDOG_S
    base.setdefault("brain", {})["tick_timeout_s"] = TICK_TIMEOUT_S
    config = tmp_path / "robot.yaml"
    config.write_text(yaml.safe_dump(base))
    vet = SafetyController.check_and_execute

    def slow_verb(self, *args, **kwargs):
        time.sleep(1.0)
        return vet(self, *args, **kwargs)

    monkeypatch.setattr(SafetyController, "check_and_execute", slow_verb)
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    client = asgi_client(server.create_app(str(config)))
    with client:
        assert client.post("/autonomy/resume", headers=BRAIN).status_code == 200
        began = time.monotonic()
        r = client.post("/action", json={"action": "LEFT", "angle": 90}, headers=BRAIN).json()
        assert r["executed"] is True and time.monotonic() - began >= 1.0, r
        time.sleep(TRIP_AFTER_S - 0.5)
        assert time.monotonic() - began > TRIP_AFTER_S, "the verb and the wait were too short"
        assert client.get("/health").json()["watchdog"]["trip"] is None
        client.post("/stop", headers=BRAIN)


def test_the_watchdog_records_one_firing_per_silence(url):
    """A refusal inside a silence replaces `last_refusal`; it is no new firing."""
    _start(url)
    _move(url)
    _stop(url, PERSON)
    time.sleep(WATCHDOG_S + SLACK_S)
    first = _wd(url)["last_fired"]["seconds_ago"]
    assert _move(url)["reason"] == "person_stop"
    time.sleep(0.4)
    assert _wd(url)["last_fired"]["seconds_ago"] >= first + 0.3


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
    assert _wd(url)["mission_live"] is True, "a beat must not end or start a mission"
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


def test_the_recorded_runs_are_pinned():
    """Criteria 1-3, as measured (`evaluations/watchdog-reason-3164/`)."""
    summary = json.loads((REPO_ROOT / "evaluations" / "watchdog-reason-3164" / "summary.json").read_text())
    assert all(row["ok"] == row["n"] for row in summary.values()), summary
    for run in ("c1:tick_hang", "c1:sigstop", "c1:sigkill", "c3:person_stop", "c3:veto", "c3:lapse"):
        assert summary[run]["n"] == 10, run
    assert sum(summary[f"c2:{how}"]["n"] for how in
               ("found", "budget", "brain_stop", "person_stop")) == 20


def test_health_describes_the_new_fields_and_never_judges_them(url):
    """M5: only what a release can be blamed for reaches a verdict."""
    _start(url)
    _move(url)
    time.sleep(TRIP_AFTER_S + SLACK_S)
    report = check_robot(url)
    assert report["description"]["watchdog"]["trip"]["how"] == "silence", report
    assert report["description"]["last_stop_reason"] is not None
    assert report["problems"] == [] and report["status"] == "ok", report


# ---------- 3.171.B: a mission's explicit close ----------

# Criterion 1's 10 of 10 is one run with MISSION_CLOSE_REPEATS=10 (amendment,
# 3.171); the suite keeps two.
CLOSE_REPEATS = int(os.environ.get("MISSION_CLOSE_REPEATS", "2"))


def _begin(url, headers=BRAIN):
    time.sleep(WATCHDOG_S + SLACK_S)
    r = httpx.post(f"{url}/autonomy/resume", headers=headers)
    assert r.status_code == 200
    return r.json()["mission_id"]


def _close(url, mission_id, headers=BRAIN):
    r = httpx.post(f"{url}/autonomy/mission-closed", json={"mission_id": mission_id},
                   headers=headers)
    assert r.status_code == 200, r.text
    return r.json()


def test_a_closed_mission_with_no_final_stop_reads_calm(url):
    mission = _begin(url)
    assert _move(url)["executed"] is True
    assert _wd(url)["mission_live"] is True
    assert _close(url, mission)["closed"] is True
    time.sleep(TRIP_AFTER_S + SLACK_S)
    wd = _wd(url)
    assert wd["mission_live"] is False and wd["trip"] is None, wd


def test_a_close_for_an_earlier_mission_leaves_the_next_one_open(url):
    earlier = _begin(url)
    current = _begin(url)
    assert current != earlier
    assert _move(url)["executed"] is True
    assert _close(url, earlier)["closed"] is False
    assert _wd(url)["mission_live"] is True
    time.sleep(TRIP_AFTER_S + SLACK_S)
    assert _wd(url)["trip"]["how"] == "silence"


def test_another_drivers_close_does_nothing(url):
    mission = _begin(url)
    assert _move(url)["executed"] is True
    for headers in (PERSON, {"x-driver": "teleop"}):
        assert _close(url, mission, headers)["closed"] is False
    assert _wd(url)["mission_live"] is True
    _stop(url)


def test_a_close_keeps_a_trip_already_recorded(url):
    mission = _begin(url)
    assert _move(url)["executed"] is True
    time.sleep(TRIP_AFTER_S + SLACK_S)
    assert _wd(url)["trip"] is not None
    assert _close(url, mission)["closed"] is True
    assert _wd(url)["trip"] is not None, "a close erased a hang"


def test_a_close_only_reports(monkeypatch):
    """No wheel, no stop, no latch lifted, no authority taken."""
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    app = server.create_app()
    body = app.state.battery["robot"]
    moved = []
    monkeypatch.setattr(body, "set_wheel_velocity", lambda *a: moved.append(a))
    with asgi_client(app) as client:
        mission = client.post("/autonomy/resume", headers=BRAIN).json()["mission_id"]
        client.post("/stop", headers=PERSON)
        latched = client.post("/action", json={"action": "FORWARD"}, headers=BRAIN).json()
        assert latched["reason"] == "person_stop", latched
        stops = body.stop_count
        before = client.get("/health").json()
        client.post("/autonomy/mission-closed", json={"mission_id": mission}, headers=BRAIN)
        after = client.get("/health").json()
        assert client.post("/action", json={"action": "FORWARD"},
                           headers=BRAIN).json()["reason"] == "person_stop"
    assert body.stop_count == stops and moved == []
    assert (before["last_stop_reason"]["reason"], before["last_stop_reason"]["driver"]) == (
        after["last_stop_reason"]["reason"], after["last_stop_reason"]["driver"])


def _mission(url, robot):
    runner = MissionRunner(robot, target_object="red backpack", max_steps=400,
                           world=RemoteWorld(url))
    runner.start()
    for _ in range(40):
        assert runner.tick(), runner.status()
        if _wd(url)["mission_live"]:
            return runner
    raise AssertionError("the mission never moved")


class LosesItsFinalStop(RemoteRobot):
    lose = False

    def stop(self):
        if self.lose:
            raise RobotTransportError("POST /stop failed: connection reset")
        return super().stop()


def test_a_lost_final_stop_is_no_false_trip(url):
    """The defect, through `MissionRunner` over `RemoteRobot`."""
    for _ in range(CLOSE_REPEATS):
        time.sleep(WATCHDOG_S + SLACK_S)
        robot = LosesItsFinalStop(url)
        robot.resume_autonomy()
        runner = _mission(url, robot)
        robot.lose = True
        runner.stop("the test's end")
        assert runner.status()["step"] > 0, runner.status()
        time.sleep(TRIP_AFTER_S + SLACK_S)
        wd = _wd(url)
        assert wd["mission_live"] is False and wd["trip"] is None, wd


def _counting_robot(answers):
    """A RemoteRobot over a server that answers `/autonomy/mission-closed`
    from `answers` (a status per call; the last repeats)."""
    calls = []

    def handler(request):
        if request.url.path != "/autonomy/mission-closed":
            return httpx.Response(200, json={"resumed": False, "mission_id": 7, "result": {}})
        calls.append(json.loads(request.content))
        status = answers[min(len(calls), len(answers)) - 1]
        if status == "drop":
            raise httpx.ConnectError("connection reset")
        return httpx.Response(status, json={"closed": status == 200})
    return RemoteRobot("http://brain.test",
                       client=httpx.Client(transport=httpx.MockTransport(handler))), calls


@pytest.fixture
def quick_close(monkeypatch):
    monkeypatch.setattr(remote_robot, "MISSION_CLOSE_EVERY_S", 0.01)
    monkeypatch.setattr(remote_robot, "MISSION_CLOSE_RETRY_S", 0.2)


def test_a_lost_close_is_sent_again(quick_close):
    robot, calls = _counting_robot(["drop", 503, 200])
    robot.close_mission(7)
    assert calls == [{"mission_id": 7}] * 3


def test_a_close_stops_retrying_at_its_window(quick_close):
    robot, calls = _counting_robot(["drop"])
    began = time.monotonic()
    robot.close_mission(7)
    assert len(calls) > 1 and time.monotonic() - began < 2.0


@pytest.mark.parametrize("status", [404, 401])
def test_a_refused_close_is_one_attempt(quick_close, status):
    robot, calls = _counting_robot([status])
    robot.close_mission(7)
    assert len(calls) == 1


def test_a_close_on_a_closed_client_does_not_raise(quick_close):
    robot, _ = _counting_robot([200])
    robot._client.close()
    robot.close_mission(7)


def test_a_start_keeps_its_mission_id_and_forgets_the_last_one():
    robot, _ = _counting_robot([200])
    robot.resume_autonomy()
    assert robot.mission_id == 7

    def refused(_request):
        raise httpx.ConnectError("connection reset")
    robot._client = httpx.Client(transport=httpx.MockTransport(refused))
    with pytest.raises(RobotTransportError):
        robot.resume_autonomy()
    assert robot.mission_id is None


class CountsCloses(RemoteRobot):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.closes = []
        self.closed = threading.Event()

    def close_mission(self, mission_id):
        self.closes.append(mission_id)
        super().close_mission(mission_id)
        self.closed.set()


def test_a_normal_end_sends_exactly_one_close(url):
    time.sleep(WATCHDOG_S + SLACK_S)
    robot = CountsCloses(url)
    robot.resume_autonomy()
    mission = robot.mission_id
    runner = _mission(url, robot)
    runner.stop("the test's end")
    runner.stop("and again")
    assert robot.closed.wait(5.0)
    assert robot.closes == [mission]
    assert runner.status()["step"] > 0
    wd = _wd(url)
    assert wd["mission_live"] is False and wd["trip"] is None, wd


class SlowFinalStop(CountsCloses):
    """A link so bad the final stop runs out its timeout."""
    slow = False
    close_landed_first = None

    def stop(self):
        if self.slow:
            self.close_landed_first = self.closed.wait(3.0)
        return super().stop()


def test_the_close_does_not_wait_for_a_slow_final_stop(url):
    """Review of 3.171: the server's silence runs from the brain's last word,
    so a close sent after a 10 s stop could arrive past the trip."""
    time.sleep(WATCHDOG_S + SLACK_S)
    robot = SlowFinalStop(url)
    robot.resume_autonomy()
    runner = _mission(url, robot)
    robot.slow = True
    runner.stop("the test's end")
    assert robot.close_landed_first is True


def test_a_mission_whose_start_had_no_id_sends_no_close(url):
    time.sleep(WATCHDOG_S + SLACK_S)
    robot = CountsCloses(url)
    runner = _mission_unstarted(url, robot)
    runner.stop("the test's end")
    time.sleep(0.2)
    assert robot.closes == []


def _mission_unstarted(url, robot):
    runner = MissionRunner(robot, target_object="red backpack", max_steps=400,
                           world=RemoteWorld(url))
    runner.start()
    assert runner.tick(), runner.status()
    return runner
