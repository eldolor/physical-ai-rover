# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.170: `POST /power/shutdown`, in process, with harmless commands only.
Criteria 1, 2, 3 and 5, docs/plans/ros-alignment/3.170-twin-shutdown-button.md.
The live runs are evaluations/shutdown-3170/run.py; the phone is
tests/test_ui_shutdown.py.
"""

import json
import threading
import time

import pytest
from fastapi.testclient import TestClient

import robot.server as server
from robot.battery_ladder import OK, POWEROFF
from robot.interface import PERSON_STOP, SHUTTING_DOWN
from tests.conftest import REPO_ROOT
from tests.test_battery_ladder import (Pack, drive_to, served as _battery_served,  # noqa: F401
                                       served_goals)

PERSON = {"x-driver": "twin-dpad"}
WAIT_S = 10.0
SECRET = "s" * 20


@pytest.fixture
def served(_battery_served, monkeypatch):  # noqa: F811
    """3.137's server: its power-off command is set, so the route needs a secret."""
    monkeypatch.setenv("APP_SHARED_SECRET", SECRET)
    _battery_served[0].headers["x-app-secret"] = SECRET
    return _battery_served


def shut_down(client, headers=PERSON):
    return client.post("/power/shutdown", headers=headers)


def request_of(client):
    return client.get("/battery").json()["ladder"]["request"]


def untouched(bat, log):
    assert bat["batt"]["requested"] is None and bat["batt"]["poweroff_calls"] == 0
    assert not log.exists() or "poweroff" not in log.read_text()


def app_with(monkeypatch, poweroff_cmd, steps='[]', secret=None):
    if secret is None:
        monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    else:
        monkeypatch.setenv("APP_SHARED_SECRET", secret)
    monkeypatch.setenv("BATTERY_SHUTDOWN_CMDS", steps)
    monkeypatch.setenv("BATTERY_POWEROFF_CMD", poweroff_cmd)
    app = server.create_app()
    app.state.battery["robot"].battery = Pack(12.0)
    return TestClient(app), app.state.battery


def test_the_recorded_runs_are_pinned():
    """Criteria 3, 4 and 7, as measured (`evaluations/shutdown-3170/`)."""
    summary = json.loads((REPO_ROOT / "evaluations" / "shutdown-3170" / "summary.json").read_text())
    assert all(row["ok"] == row["n"] for row in summary.values()), summary
    for run in ("c3:burst_and_retry", "c3:ladder_during", "c4:mid_mission", "c4:brain_down", "c7:phases"):
        assert summary[run]["n"] == 10, run
    assert summary["c4:mid_mission"]["outcomes"] == {"cancelled": 10}


# ---------- criterion 1: the gate ----------

@pytest.mark.parametrize("headers", [{}, {"x-app-secret": "wrong"}])
def test_a_set_secret_must_be_sent(monkeypatch, tmp_path, headers):
    log = tmp_path / "off"
    client, bat = app_with(monkeypatch, f"echo poweroff >> {log}", secret="s" * 20)
    assert client.post("/power/shutdown", headers={**PERSON, **headers}).status_code == 401
    untouched(bat, log)


def test_the_right_secret_is_served(monkeypatch, tmp_path):
    log = tmp_path / "off"
    client, bat = app_with(monkeypatch, f"echo poweroff >> {log}", secret="s" * 20)
    r = client.post("/power/shutdown", headers={**PERSON, "x-app-secret": "s" * 20})
    assert r.status_code == 200 and r.json()["started"] is True
    assert bat["batt"]["shutdown_done"].wait(WAIT_S)


def test_no_secret_on_a_robot_that_can_power_off_is_refused(monkeypatch, tmp_path):
    """`require_secret()` would have let this through."""
    log = tmp_path / "off"
    client, bat = app_with(monkeypatch, f"echo poweroff >> {log}")
    r = shut_down(client)
    assert r.status_code == 403 and "shutdown_no_secret" in r.text
    untouched(bat, log)
    reply = client.post("/action", json={"action": "FORWARD"}, headers=PERSON).json()
    assert reply.get("reason") != SHUTTING_DOWN, reply


def test_no_secret_is_served_where_nothing_can_power_off(monkeypatch):
    client, bat = app_with(monkeypatch, "")
    assert shut_down(client).status_code == 200
    assert bat["batt"]["shutdown_done"].wait(WAIT_S)
    assert request_of(client)["phase"] == "stub"


@pytest.mark.parametrize("driver", ["brain", "ros", "teleop", "twin-local-brain"])
def test_only_a_person_may_ask(served, driver):  # noqa: F811
    client, bat, _, _, log = served
    r = shut_down(client, {"x-driver": driver})
    assert r.status_code == 403 and "shutdown_not_a_person" in r.text
    untouched(bat, log)


# ---------- criterion 2: 3.137's sequence ----------

def test_one_sequence_in_order_and_the_ladder_does_not_move(served):  # noqa: F811
    client, bat, _, _, log = served
    assert shut_down(client).json()["started"] is True
    assert bat["batt"]["shutdown_done"].wait(WAIT_S)
    assert log.read_text().split() == ["brain", "ros", "vlm", "poweroff"]
    assert [s["step"] for s in bat["batt"]["shutdown"]] == [
        "wheels", "brain", "ros", "vlm", "sync", "poweroff"]
    assert bat["batt"]["shutdown"][0]["rc"] == 0
    reply = client.get("/battery").json()
    assert bat["ladder"].level == OK and reply["level"] == OK


def test_a_step_that_fails_does_not_stop_the_sequence(monkeypatch, tmp_path):
    """The brain is down: its step fails, is recorded, and the rest runs."""
    log = tmp_path / "off"
    client, bat = app_with(monkeypatch, f"echo poweroff >> {log}",
                           steps='[["brain", "exit 7"]]', secret="s" * 20)
    client.post("/power/shutdown", headers={**PERSON, "x-app-secret": "s" * 20})
    assert bat["batt"]["shutdown_done"].wait(WAIT_S)
    assert [s["rc"] for s in bat["batt"]["shutdown"] if s["step"] == "brain"] == [7]
    assert log.read_text().split() == ["poweroff"]


# ---------- what the phone reads ----------

@pytest.mark.parametrize("cmd,phase", [("true", "powering_off"), ("exit 3", "failed"), ("", "stub")])
def test_the_phase_says_how_it_ended(monkeypatch, cmd, phase):
    client, bat = app_with(monkeypatch, cmd, secret="s" * 20)
    secret = {"x-app-secret": "s" * 20}
    client.post("/power/shutdown", headers={**PERSON, **secret})
    assert bat["batt"]["shutdown_done"].wait(WAIT_S)
    req = client.get("/battery", headers=secret).json()["ladder"]["request"]
    assert req["phase"] == phase and req["settle_s"] == server.SHUTDOWN_SETTLE_S


def test_powering_off_is_marked_before_the_command_returns(monkeypatch):
    """`systemctl poweroff` returns at once and the server dies right after."""
    client, bat = app_with(monkeypatch, "sleep 1.5", secret="s" * 20)
    secret = {"x-app-secret": "s" * 20}
    client.post("/power/shutdown", headers={**PERSON, **secret})
    deadline = threading.Event()
    for _ in range(100):
        if bat["batt"]["poweroff_begun"]:
            break
        deadline.wait(0.02)
    assert not any(s["step"] == "poweroff" for s in bat["batt"]["shutdown"])
    assert client.get("/battery", headers=secret).json()["ladder"]["request"]["phase"] == "powering_off"
    assert bat["batt"]["shutdown_done"].wait(WAIT_S)


def test_nothing_is_reported_before_anyone_asks(served):  # noqa: F811
    client, *_ = served
    assert request_of(client) is None


# ---------- criterion 3: exactly once ----------

def test_a_burst_and_a_retry_run_it_once(served):  # noqa: F811
    client, bat, _, _, log = served
    results = []
    barrier = threading.Barrier(20)

    def press():
        barrier.wait()
        results.append(shut_down(client).json())
    threads = [threading.Thread(target=press) for _ in range(20)]
    for th in threads:
        th.start()
    for th in threads:
        th.join()
    assert bat["batt"]["shutdown_done"].wait(WAIT_S)
    assert sum(r["started"] for r in results) == 1, results
    retry = shut_down(client).json()
    assert retry["started"] is False and retry["request"]["phase"] == "powering_off"
    assert log.read_text().split().count("poweroff") == 1
    assert bat["batt"]["poweroff_calls"] == 1


def test_the_ladder_firing_during_it_does_not_run_it_twice(served):  # noqa: F811
    client, bat, _, reading, log = served
    shut_down(client)
    drive_to(reading, 9.95)
    assert bat["ladder"].level == POWEROFF
    assert bat["batt"]["shutdown_done"].wait(WAIT_S)
    assert log.read_text().split().count("poweroff") == 1


def test_asking_during_the_ladders_power_off_does_not_run_it_twice(served):  # noqa: F811
    client, bat, _, reading, log = served
    drive_to(reading, 9.95)
    shut_down(client)
    assert bat["batt"]["shutdown_done"].wait(WAIT_S)
    assert log.read_text().split().count("poweroff") == 1


# ---------- criterion 5: nothing else moves ----------

def test_after_the_press_nothing_moves_and_a_stop_still_answers(served):  # noqa: F811
    client, bat, _, _, _ = served
    shut_down(client)
    r = client.post("/action", json={"action": "FORWARD"}, headers=PERSON).json()
    assert r["executed"] is False and r["reason"] == SHUTTING_DOWN
    r = client.post("/wheels", json={"left_rad_s": 2.0, "right_rad_s": 2.0}, headers=PERSON).json()
    assert r["reason"] == SHUTTING_DOWN
    r = client.post("/wheels", json={"left_rad_s": 2.0, "right_rad_s": 2.0},
                    headers={"x-driver": "twin-local-brain"}).json()
    assert r["reason"] == SHUTTING_DOWN, r
    r = client.post("/action", json={"action": "FORWARD"}, headers={"x-driver": "brain"}).json()
    assert r["executed"] is False and r["reason"] == PERSON_STOP
    assert client.post("/stop", headers=PERSON).status_code == 200
    assert bat["batt"]["shutdown_done"].wait(WAIT_S)


def test_a_goal_ends_and_no_new_one_is_taken(served_goals):  # noqa: F811
    client, bridge, bat = served_goals
    assert client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0}, headers=PERSON).json()["accepted"]
    assert shut_down(client).status_code == 200
    end = time.monotonic() + 3.0
    while bridge.goal is not None and time.monotonic() < end:
        time.sleep(0.02)
    assert bridge.goal is None, "the goal outlived the shutdown"
    r = client.post("/world/goal", json={"x_m": 1.0, "y_m": 2.0}, headers=PERSON).json()
    assert r["accepted"] is False and r["reason"] == SHUTTING_DOWN
    client.post("/wheels", json={"left_rad_s": 3.0, "right_rad_s": 3.0}, headers={"x-driver": "ros"})
    wheels = client.get("/wheels").json()
    assert wheels["left"]["velocity_rad_s"] == 0 and wheels["right"]["velocity_rad_s"] == 0, wheels
    assert bat["batt"]["shutdown_done"].wait(WAIT_S)


def test_a_start_cannot_lift_the_stop(served):  # noqa: F811
    client, bat, _, _, _ = served
    shut_down(client)
    assert client.post("/autonomy/resume", headers={"x-driver": "brain"}).json()["resumed"] is False
    r = client.post("/action", json={"action": "FORWARD"}, headers={"x-driver": "brain"}).json()
    assert r["executed"] is False and r["reason"] == PERSON_STOP
    assert bat["batt"]["shutdown_done"].wait(WAIT_S)


def test_a_person_stop_is_on_the_record(served):  # noqa: F811
    client, bat, _, _, _ = served
    shut_down(client)
    assert client.get("/health").json()["last_stop_reason"]["reason"] == PERSON_STOP
    assert bat["batt"]["shutdown_done"].wait(WAIT_S)
