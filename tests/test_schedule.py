# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_schedule.py

3.163.B: the scheduler's rules (control/scheduler.py) on a planted clock, then
the brain server's checks and start over the real robot server in process
(T2's reasons, T4 the person wins, T5 a restart, D4 an ordinary mission).
"""

import asyncio
import json
import time
from datetime import datetime, timedelta

import pytest
from fastapi.testclient import TestClient

from control import scheduler as sch
from control.remote_robot import RemoteRobot
from robot import server
from tests.conftest import ASGI_BASE_URL, asgi_client

MON_9 = datetime(2026, 11, 16, 9, 0)        # a Monday
DAILY = list(range(7))


def one(sid="morning", job="survey", target=None, at="09:00", days=DAILY, enabled=True):
    return {"id": sid, "job": job, "target": target, "days": days, "at": at, "enabled": enabled}


class Clock:
    def __init__(self, now):
        self.now = now

    def __call__(self):
        return self.now


def run(coro):
    return asyncio.run(coro)


# ---------- the rules, with planted checks ----------

def planted(tmp_path, schedules, clock, reasons=None, started=None, fail=None, saved=None):
    store = sch.ScheduleStore(tmp_path)
    store.save(sch.validate({"enabled": True, "schedules": schedules}),
               saved or clock() - timedelta(days=1))
    reasons = reasons if reasons is not None else {}
    started = started if started is not None else []

    async def check(state, s):
        if not state["enabled"] or not s["enabled"]:
            return sch.DISABLED
        return reasons.get(s["id"])

    async def start(s):
        if fail:
            raise sch.SkipStart(fail)
        started.append(s["id"])
        return f"m{len(started)}"
    return sch.Scheduler(store, check, start, clock=clock), store, started


def outcomes(store):
    return [(x["id"], x["outcome"], x.get("reason")) for x in store.lines() if x["outcome"] != sch.FIRING]


def test_validate_refuses_what_a_schedule_cannot_be():
    for bad in ({"schedules": [one(job="mop")]}, {"schedules": [one(job="find")]},
                {"schedules": [one(at="9:00")]}, {"schedules": [one(days=[])]},
                {"schedules": [one(days=[7])]}, {"schedules": [one(), one()]},
                {"schedules": [one(sid=f"s{i}") for i in range(11)]},
                {"schedules": [one(target="sock")]}, {"other": 1}):
        with pytest.raises(ValueError):
            sch.validate(bad)
    ok = sch.validate({"enabled": True, "schedules": [one(job="find", target=" keys ", days=[3, 1, 1])]})
    assert ok["schedules"][0]["target"] == "keys" and ok["schedules"][0]["days"] == [1, 3]


def test_a_due_time_fires_once_and_never_again(tmp_path):
    clock = Clock(MON_9 + timedelta(seconds=5))
    s, store, started = planted(tmp_path, [one()], clock)
    run(s.tick())
    clock.now += timedelta(seconds=10)
    run(s.tick())
    clock.now -= timedelta(minutes=5)          # a clock stepped back
    run(s.tick())
    assert started == ["morning"]
    assert outcomes(store) == [("morning", sch.FIRED, None)]


def test_a_firing_line_is_written_before_the_start(tmp_path):
    clock = Clock(MON_9 + timedelta(seconds=5))
    seen = []
    store = sch.ScheduleStore(tmp_path)
    store.save(sch.validate({"enabled": True, "schedules": [one()]}), MON_9 - timedelta(days=1))

    async def check(state, s):
        return None

    async def start(s):
        seen.append([x["outcome"] for x in store.lines()])
        raise RuntimeError("the brain fell over mid-start")
    run(sch.Scheduler(store, check, start, clock=clock).tick())
    assert seen == [[sch.FIRING]]
    # A start that failed after its firing line is skipped, never tried again.
    assert outcomes(store) == [("morning", sch.SKIPPED, sch.START_REFUSED)]
    run(sch.Scheduler(store, check, start, clock=clock).tick())
    assert len(seen) == 1


def test_a_reason_is_tried_again_until_the_grace_ends(tmp_path):
    clock = Clock(MON_9 + timedelta(seconds=5))
    reasons = {"morning": sch.NOT_DOCKED}
    s, store, started = planted(tmp_path, [one()], clock, reasons)
    run(s.tick())
    assert outcomes(store) == [] and s.pending
    del reasons["morning"]                      # docked again inside the grace
    clock.now += timedelta(minutes=3)
    run(s.tick())
    assert started == ["morning"]


@pytest.mark.parametrize("reason,recorded", [(sch.NOT_DOCKED, sch.NOT_DOCKED),
                                             (sch.MISSION_RUNNING, sch.BUSY),
                                             (sch.PERSON_STOP_LATCHED, sch.PERSON_STOP_LATCHED)])
def test_at_the_grace_the_last_reason_is_recorded(tmp_path, reason, recorded):
    clock = Clock(MON_9 + timedelta(seconds=5))
    s, store, started = planted(tmp_path, [one()], clock, {"morning": reason})
    run(s.tick())
    clock.now = MON_9 + timedelta(seconds=sch.GRACE_S + 1)
    run(s.tick())
    assert started == [] and outcomes(store) == [("morning", sch.SKIPPED, recorded)]


def test_a_brain_down_through_the_grace_records_missed_down(tmp_path):
    clock = Clock(MON_9 + timedelta(seconds=sch.GRACE_S + 30))
    s, store, started = planted(tmp_path, [one()], clock)
    run(s.tick())
    assert started == [] and outcomes(store) == [("morning", sch.MISSED_DOWN, None)]


def test_a_brain_back_inside_the_grace_runs_it(tmp_path):
    clock = Clock(MON_9 + timedelta(seconds=sch.GRACE_S - 30))
    s, store, started = planted(tmp_path, [one()], clock)
    run(s.tick())
    assert started == ["morning"]


def test_a_new_or_changed_schedule_owes_nothing_from_before(tmp_path):
    store = sch.ScheduleStore(tmp_path)
    store.save(sch.validate({"enabled": True, "schedules": [one(at="08:00")]}), MON_9)
    clock = Clock(MON_9 + timedelta(seconds=5))
    started = []

    async def check(state, s):
        return None

    async def start(s):
        started.append(s["id"])
    run(sch.Scheduler(store, check, start, clock=clock).tick())
    assert started == [] and store.lines() == []


def test_disabled_is_a_reason_like_any_other(tmp_path):
    clock = Clock(MON_9 + timedelta(seconds=5))
    s, store, started = planted(tmp_path, [one(enabled=False)], clock)
    run(s.tick())
    clock.now += timedelta(seconds=sch.GRACE_S)
    run(s.tick())
    assert started == [] and outcomes(store) == [("morning", sch.SKIPPED, sch.DISABLED)]


def test_the_view_names_the_next_run_and_the_last_ends(tmp_path):
    clock = Clock(MON_9 + timedelta(seconds=5))
    s, store, _ = planted(tmp_path, [one(), one(sid="keys", job="find", target="keys", at="22:00")], clock,
                          saved=MON_9 - timedelta(minutes=1))
    run(s.tick())
    view = s.view()
    assert view["next"]["id"] == "keys" and view["next"]["at"].startswith("2026-11-16T22:00")
    assert [r["id"] for r in view["runs"]] == ["morning"]
    off = sch.validate({"enabled": False, "schedules": [one()]})
    store.save(off, clock())
    assert s.view()["next"] is None


def test_an_unreadable_file_reads_as_off(tmp_path):
    (tmp_path / "schedules.json").write_text("{not json")
    store = sch.ScheduleStore(tmp_path)
    assert store.load()["enabled"] is False and "unreadable" in store.load()


def test_the_clock_file_moves_the_scheduler_only_when_set(tmp_path, monkeypatch):
    monkeypatch.delenv(sch.CLOCK_FILE_ENV, raising=False)
    assert abs((sch.default_clock() - datetime.now()).total_seconds()) < 2
    f = tmp_path / "offset"
    f.write_text("86400")
    monkeypatch.setenv(sch.CLOCK_FILE_ENV, str(f))
    assert abs((sch.default_clock() - datetime.now()).total_seconds() - 86400) < 2


# ---------- the robot server's side ----------

@pytest.fixture
def robot_client(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    client = asgi_client(server.create_app())
    yield client
    client.close()


def test_a_resume_that_may_not_lift_keeps_a_persons_stop(robot_client):
    robot_client.post("/stop", headers={"x-driver": "twin-dpad"})
    r = robot_client.post("/autonomy/resume", json={"lift": False}, headers={"x-driver": "brain"}).json()
    assert r["refused"] == "person_stop" and r["mission_id"] is None
    health = robot_client.get("/health").json()
    assert health["autonomy_latch"]["by"] == "twin-dpad"
    moved = robot_client.post("/action", json={"action": "LOOK_LEFT"}, headers={"x-driver": "brain"}).json()
    assert moved["reason"] == "person_stop", moved
    # With nothing latched it opens the mission, as a person's start does.
    assert robot_client.post("/autonomy/resume", headers={"x-driver": "brain"}).json()["resumed"]
    r = robot_client.post("/autonomy/resume", json={"lift": False}, headers={"x-driver": "brain"}).json()
    assert r["mission_id"] is not None and "refused" not in r
    assert robot_client.get("/health").json()["autonomy_latch"] is None


def test_the_sim_pack_is_set_only_in_the_simulator(robot_client, monkeypatch):
    assert robot_client.post("/sim/battery", json={"v": 10.4}).status_code == 404   # no pack
    monkeypatch.setenv("SIM_BATTERY_V", "12.0")
    client = asgi_client(server.create_app())
    assert client.post("/sim/battery", json={"v": 10.4}).json() == {"v": 10.4}
    assert abs(client.get("/battery").json()["v"] - 10.4) < 0.01


# ---------- the brain server's checks and start (T2, T4, T5, D4) ----------

def brain_config(tmp_path, **brain):
    cfg = {"tick_interval_s": 0.0, "schedule_dir": str(tmp_path / "sched"),
           "schedule_find_policy": "frontier", "schedule_survey_policy": "frontier",
           "intervention_dir": str(tmp_path / "iv"), **brain}
    path = tmp_path / "robot.yaml"
    path.write_text("brain:\n" + "".join(f"  {k}: {json.dumps(v)}\n" for k, v in cfg.items()))
    return str(path)


class Brain:
    """A brain over the in-process robot server, its scheduler on a planted
    clock, ticked by the test (never by the wall)."""

    def __init__(self, tmp_path, robot_client, **brain):
        from control.brain_server import create_app
        import os
        from control.remote_world import RemoteWorld
        saved = os.environ.get("SCHEDULE_DIR")
        os.environ["SCHEDULE_DIR"] = str(tmp_path / "sched")     # this brain's own, over the suite's
        try:
            self.app = create_app(config_path=brain_config(tmp_path, **brain),
                                  robot_factory=lambda: RemoteRobot(ASGI_BASE_URL, client=robot_client),
                                  world_factory=lambda r: RemoteWorld(ASGI_BASE_URL, client=robot_client))
        finally:
            if saved is None:
                os.environ.pop("SCHEDULE_DIR", None)
            else:
                os.environ["SCHEDULE_DIR"] = saved
        self.client = TestClient(self.app).__enter__()
        self.sched = self.app.state.scheduler
        self.clock = Clock(MON_9 - timedelta(minutes=1))
        self.sched.clock = self.clock

    def set(self, *schedules, enabled=True):
        r = self.client.put("/schedules", json={"enabled": enabled, "schedules": list(schedules)})
        assert r.status_code == 200, r.text
        # A schedule counts from when it was saved: a day before the test's due times.
        store = self.sched.store
        body = store.load()
        for s in body["schedules"]:
            s["since"] = (MON_9 - timedelta(days=1)).isoformat()
        store.path.write_text(json.dumps(body))

    def tick(self, at):
        self.clock.now = at
        return self.client.portal.call(self.sched.tick)

    def status(self):
        return self.client.get("/mission/status").json()

    def wait_end(self, seconds=60.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            s = self.status()
            if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
                return s
            time.sleep(0.05)
        raise AssertionError(f"never ended: {self.status()}")

    def close(self):
        self.client.__exit__(None, None, None)


@pytest.fixture
def brain(tmp_path, robot_client):
    b = Brain(tmp_path, robot_client)
    yield b
    b.close()


FIND = one(sid="keys", job="find", target="red backpack")


def last(brain):
    return [x for x in brain.sched.store.lines() if x["outcome"] != sch.FIRING][-1]


def skipped_with(brain, reason):
    """Due at 9:00; tried, then the grace runs out: skipped with `reason`,
    and nothing started."""
    lines = brain.tick(MON_9 + timedelta(seconds=5))
    assert lines == [], lines
    assert brain.status().get("running") is not True
    lines = brain.tick(MON_9 + timedelta(seconds=sch.GRACE_S + 1))
    assert [(x["outcome"], x.get("reason")) for x in lines] == [(sch.SKIPPED, reason)], lines
    assert brain.status().get("running") is not True


def test_a_scheduled_find_starts_as_an_ordinary_mission(brain):
    brain.set(FIND)
    lines = brain.tick(MON_9 + timedelta(seconds=5))
    assert [x["outcome"] for x in lines] == [sch.FIRED]
    s = brain.status()
    assert s["started_by"] == "schedule:keys" and s["policy"] == "frontier"
    end = brain.wait_end()
    assert end["started_by"] == "schedule:keys"
    assert lines[0]["mission_id"] == end.get("mission_id")


def test_disabled_skips(brain):
    brain.set(FIND, enabled=False)
    skipped_with(brain, sch.DISABLED)


def test_a_real_robot_without_the_users_word_skips(brain, monkeypatch):
    real = RemoteRobot.get_health

    def as_hardware(self):
        return {**real(self), "mode": "hardware"}
    monkeypatch.setattr(RemoteRobot, "get_health", as_hardware)
    brain.set(FIND)
    skipped_with(brain, sch.NOT_ALLOWED_HERE)


def test_a_mission_running_is_busy(brain):
    brain.set(FIND)
    assert brain.client.post("/mission/start", json={"target_object": "purple unicorn",
                                                      "max_steps": 100000}).status_code == 200
    try:
        skipped_with_running(brain)
    finally:
        brain.client.post("/mission/stop")


def skipped_with_running(brain):
    assert brain.tick(MON_9 + timedelta(seconds=5)) == []
    lines = brain.tick(MON_9 + timedelta(seconds=sch.GRACE_S + 1))
    assert [(x["outcome"], x.get("reason")) for x in lines] == [(sch.SKIPPED, sch.BUSY)]
    assert brain.status()["started_by"] is None      # still the person's mission


def test_a_latched_stop_skips_and_stays_latched(brain, robot_client):
    brain.set(FIND)
    robot_client.post("/stop", headers={"x-driver": "twin-dpad"})
    skipped_with(brain, sch.PERSON_STOP_LATCHED)
    assert robot_client.get("/health").json()["autonomy_latch"]["by"] == "twin-dpad"


def test_a_stop_landing_between_the_check_and_the_start_is_kept(brain, robot_client, monkeypatch):
    """The check saw nothing latched; a person's Stop then lands before the
    start opens the mission. The start opens nothing, and the latch stays."""
    brain.set(FIND)
    real = RemoteRobot.open_mission

    def stop_first(self):
        robot_client.post("/stop", headers={"x-driver": "twin-dpad"})
        return real(self)
    monkeypatch.setattr(RemoteRobot, "open_mission", stop_first)
    lines = brain.tick(MON_9 + timedelta(seconds=5))
    assert [(x["outcome"], x.get("reason")) for x in lines] == [(sch.SKIPPED, sch.PERSON_STOP_LATCHED)]
    assert robot_client.get("/health").json()["autonomy_latch"]["by"] == "twin-dpad"
    assert brain.status().get("running") is not True


def test_a_person_holding_the_robot_skips(brain, robot_client):
    brain.set(FIND)
    hold = robot_client.post("/action", json={"action": "LOOK_LEFT"}, headers={"x-driver": "twin-dpad"})
    assert hold.json()["executed"]
    assert brain.tick(MON_9 + timedelta(seconds=5)) == []
    assert brain.sched.pending and list(brain.sched.pending.values()) == [sch.PERSON_HOLDING]


def test_off_the_dock_skips(brain, robot_client):
    brain.set(FIND)
    brain.tick(MON_9 - timedelta(seconds=30))          # the first check takes the dock
    robot_client.post("/action", json={"action": "FORWARD", "duration": 1.0},
                      headers={"x-driver": "twin-dpad"})
    end = time.monotonic() + 10     # the person's authority lapses; the robot stays where they left it
    while robot_client.get("/health").json()["authority_holder"] and time.monotonic() < end:
        time.sleep(0.05)
    skipped_with(brain, sch.NOT_DOCKED)


def test_a_low_battery_skips(tmp_path, monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setenv("SIM_BATTERY_V", "10.3")
    client = asgi_client(server.create_app())
    b = Brain(tmp_path, client)
    try:
        b.set(FIND)
        skipped_with(b, sch.BATTERY_NOT_NORMAL)
    finally:
        b.close()
        client.close()


def test_a_survey_with_no_slam_pose_is_ros_not_ready(tmp_path, robot_client, monkeypatch):
    b = Brain(tmp_path, robot_client, schedule_survey_policy="explore")
    try:
        b.set(one())
        b.tick(MON_9 - timedelta(seconds=30))           # the dock, from a usable pose
        from control.remote_world import RemoteWorld
        monkeypatch.setattr(RemoteWorld, "get_pose", lambda self: {"usable": False})
        skipped_with(b, sch.ROS_NOT_READY)
    finally:
        b.close()


def test_a_persons_stop_ends_a_scheduled_run_cancelled_10_of_10(brain, robot_client):
    long = one(sid="keys", job="find", target="purple unicorn")
    brain.set(long)
    for i in range(10):
        due = MON_9 + timedelta(days=i)
        brain.tick(due + timedelta(seconds=1))
        assert brain.status()["started_by"] == "schedule:keys", i
        robot_client.post("/stop", headers={"x-driver": "twin-dpad"})
        end = brain.wait_end()
        assert end["outcome"] == "cancelled", (i, end)
        assert robot_client.get("/health").json()["autonomy_latch"]["by"] == "twin-dpad"
        robot_client.post("/autonomy/resume", headers={"x-driver": "brain"})   # a person's next start


def test_schedules_and_their_record_survive_a_restart(tmp_path, robot_client):
    a = Brain(tmp_path, robot_client)
    a.set(FIND, one(sid="evening", job="find", target="red backpack", at="21:00"))
    a.tick(MON_9 + timedelta(seconds=5))
    a.wait_end()
    a.close()
    b = Brain(tmp_path, robot_client)
    try:
        view = b.client.get("/schedules").json()
        assert [s["id"] for s in view["schedules"]] == ["keys", "evening"]
        assert ("keys", sch.FIRED) in [(r["id"], r["outcome"]) for r in view["runs"]]
        assert b.tick(MON_9 + timedelta(seconds=30)) == []          # never fired twice
        # Down through 21:00's grace: missed_down. Back inside it: it runs.
        lines = b.tick(MON_9.replace(hour=21) + timedelta(seconds=sch.GRACE_S + 5))
        assert [(x["id"], x["outcome"]) for x in lines] == [("evening", sch.MISSED_DOWN)]
        lines = b.tick(MON_9.replace(hour=21) + timedelta(days=1, seconds=sch.GRACE_S - 60))
        assert [(x["id"], x["outcome"]) for x in lines if x["id"] == "evening"] == [("evening", sch.FIRED)]
        b.wait_end()
    finally:
        b.close()


def test_a_scheduled_run_is_named_in_the_intervention_line_and_the_report(tmp_path, robot_client):
    b = Brain(tmp_path, robot_client)
    try:
        b.set({**FIND, "id": "report-check"})
        b.tick(MON_9 + timedelta(seconds=5))
        b.wait_end()
        import os
        log = os.path.join(os.environ.get("INTERVENTION_DIR") or str(tmp_path / "iv"), "interventions.jsonl")
        end = time.monotonic() + 10     # written on its own thread after the stop
        while time.monotonic() < end and not (os.path.exists(log) and "schedule:report-check" in open(log).read()):
            time.sleep(0.05)
        lines = [json.loads(x) for x in open(log).read().splitlines()]
        assert any(x.get("started_by") == "schedule:report-check" for x in lines)
        mid = b.status().get("mission_id")
        end = time.monotonic() + 10     # 3.162's report, written on its own thread
        report = None
        while time.monotonic() < end:
            r = b.client.get(f"/mission/reports/{mid}")
            if r.status_code == 200:
                report = r.json()
                break
            time.sleep(0.05)
        assert report is not None, mid
        assert (report.get("report") or report).get("started_by") == "schedule:report-check", report
    finally:
        b.close()


def test_the_routes_refuse_a_bad_schedule(brain):
    r = brain.client.put("/schedules", json={"enabled": True, "schedules": [one(job="find")]})
    assert r.status_code == 422 and "target" in r.text


def test_off_by_default(tmp_path, robot_client):
    b = Brain(tmp_path, robot_client)
    try:
        view = b.client.get("/schedules").json()
        assert view["enabled"] is False and view["schedules"] == [] and view["next"] is None
        assert b.tick(MON_9 + timedelta(seconds=5)) == []
        assert not (tmp_path / "sched").exists() or not (tmp_path / "sched" / "runs.jsonl").exists()
    finally:
        b.close()


# ---------- review round 1 (each seen red first) ----------

def test_a_torn_last_line_never_swallows_the_next_one(tmp_path):
    store = sch.ScheduleStore(tmp_path)
    store.append({"due": "x", "id": "a", "outcome": sch.FIRED})
    with open(store.runs_path, "a") as f:
        f.write('{"due": "y", "id": "b", "outco')          # a power cut mid-write
    store.append({"due": "z", "id": "c", "outcome": sch.FIRING})
    assert ("z", "c") in store.finals()


def test_a_file_that_is_not_schedules_reads_as_off(tmp_path):
    for bad in ("[1, 2]", '{"enabled": true, "schedules": [{"id": "x"}]}'):
        (tmp_path / "schedules.json").write_text(bad)
        body = sch.ScheduleStore(tmp_path).load()
        assert body["enabled"] is False and body["schedules"] == [] and body.get("unreadable"), body


def test_the_dock_is_never_guessed_on_a_real_robot(tmp_path, robot_client, monkeypatch):
    real = RemoteRobot.get_health
    monkeypatch.setattr(RemoteRobot, "get_health", lambda self: {**real(self), "mode": "hardware"})
    b = Brain(tmp_path, robot_client, schedules_on_robot=True)
    try:
        b.set(FIND)
        b.tick(MON_9 - timedelta(seconds=30))
        skipped_with(b, sch.NOT_DOCKED)
    finally:
        b.close()


def test_a_lapsed_lease_is_not_called_a_persons_stop(brain, robot_client, monkeypatch):
    real = RemoteRobot.get_health
    monkeypatch.setattr(RemoteRobot, "get_health",
                        lambda self: {**real(self), "autonomy_latch": {"by": "brain", "lapsed": True}})
    brain.set(FIND)
    skipped_with(brain, sch.AUTONOMY_LAPSED)


def test_a_failed_scheduled_start_closes_the_mission_it_opened(brain, monkeypatch):
    closed = []
    monkeypatch.setattr(RemoteRobot, "close_mission", lambda self, mid: closed.append(mid))
    brain.app.state.brain_state["building"] = None
    from control import brain_server

    def refuse(*a, **kw):
        raise ValueError("no such policy here")
    monkeypatch.setattr(brain_server.MissionRunner, "__init__", refuse)
    brain.set(FIND)
    lines = brain.tick(MON_9 + timedelta(seconds=5))
    assert [(x["outcome"], x.get("reason")) for x in lines] == [(sch.SKIPPED, sch.START_REFUSED)]
    assert len(closed) == 1 and closed[0] is not None


def test_a_schedule_with_no_since_is_repaired_by_a_save(tmp_path):
    (tmp_path / "schedules.json").write_text(json.dumps({"enabled": True, "schedules": [one()]}))
    store = sch.ScheduleStore(tmp_path)
    saved = store.save(store.load() | {"schedules": [one()]}, MON_9 - timedelta(hours=1))
    assert saved["schedules"][0]["since"] == (MON_9 - timedelta(hours=1)).isoformat()


# ---------- the orchestrator's conditions (2026-11-12) ----------

@pytest.mark.parametrize("mode, env", [
    ("hardware", {"SIM_MOTOR_BOARD": "fake", "WORLD_MODE": "none"}),
    ("teleop", {"WORLD_MODE": "none"}),
])
def test_the_sim_pack_route_is_absent_on_a_real_or_teleop_body(monkeypatch, sim_programs, mode, env):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setenv("SIM_MAP", "scaled_house")
    monkeypatch.setenv("SIM_BATTERY_V", "12.0")       # even with a sim pack asked for
    for k in ("SIM_MOTOR_BOARD", "WORLD_MODE", "ROBOT_DRIVE", "SIM_BODY_URL", "SIM_SENSORS_URL"):
        monkeypatch.delenv(k, raising=False)
    if env.get("SIM_MOTOR_BOARD") == "fake":
        sim_programs(SIM_MAP="scaled_house").apply(monkeypatch)
    monkeypatch.setenv("ROBOT_MODE", mode)
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    with TestClient(server.create_app()) as client:
        assert client.post("/sim/battery", json={"v": 10.3}).status_code == 404


def test_schedules_stay_off_on_a_real_robot_by_default():
    from pathlib import Path

    import yaml

    from control.brain_config import DEFAULTS, load_brain_config
    root = Path(__file__).resolve().parents[1]
    assert DEFAULTS["schedules_on_robot"] is False
    assert yaml.safe_load((root / "config" / "robot.yaml").read_text())["brain"]["schedules_on_robot"] is False
    assert load_brain_config(str(root / "config" / "robot.yaml"))["schedules_on_robot"] is False


def test_the_week_is_pinned():
    """T1, T3 and T5 as measured (`evaluations/schedule-3163/`, run 2)."""
    from pathlib import Path
    here = Path(__file__).resolve().parents[1] / "evaluations" / "schedule-3163"
    week = [json.loads(x) for x in (here / "week.jsonl").read_text().splitlines()]
    fired = [r for r in week if r["expect"] == "fired"]
    assert len(week) == 21 and len(fired) == 15
    assert all(r["ok_end"] for r in week), [r for r in week if not r["ok_end"]]
    assert sum(r["contacts"] for r in fired) == 0 and all(r["docked"] for r in fired)
    assert all(r["t1_s"] is None or r["t1_s"] <= 60 for r in fired)
    assert json.loads((here / "week-runs.json").read_text())["twice"] == []
    wall = [json.loads(x) for x in (here / "wall.jsonl").read_text().splitlines()]
    assert len(wall) == 3 and all(r["t1_s"] <= 60 for r in wall)
    hand = [json.loads(x) for x in (here / "hand.jsonl").read_text().splitlines()]
    finds = lambda rows: sum(bool(r.get("found_right")) for r in rows if r["job"] == "find")  # noqa: E731
    assert abs(finds(fired) - finds(hand)) <= 2
