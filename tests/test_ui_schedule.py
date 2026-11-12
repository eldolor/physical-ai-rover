# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_ui_schedule.py

3.163.B T6-T7: the twin's Schedules (Settings) and Home's line, at 390 px in
operator mode, over a REAL brain (its scheduler and store, on a planted clock)
and the real robot server in process. Taps are 3.132's M1, with the store's
file as the goal state; M2-M5 and M7 are 3.132's probes
(`evaluations/ux-3132/measure.py`). `UI_SHOTS=<dir>` saves the screenshots.
"""

import json
import os
from datetime import timedelta
from pathlib import Path

import pytest

pytest.importorskip("playwright")

from control import scheduler as sch  # noqa: E402
from robot import server  # noqa: E402
from tests.conftest import asgi_client  # noqa: E402
from tests.test_schedule import FIND, MON_9, Brain  # noqa: E402
from tests.test_ui import browser, open_twin, twin_server  # noqa: F401,E402
from tests.test_ui_room_names import _bars, _clean  # noqa: E402


@pytest.fixture
def stack(tmp_path, monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    robot_client = asgi_client(server.create_app())
    b = Brain(tmp_path, robot_client)
    yield b, robot_client
    b.close()
    robot_client.close()


def _forward(page, client):
    taps = []

    def handler(route):
        req = route.request
        path = req.url.split("/brain-stub", 1)[1]
        r = client.request(req.method, path, content=req.post_data,
                           headers={"content-type": "application/json"})
        route.fulfill(status=r.status_code, content_type="application/json", body=r.text)
    page.route("**/brain-stub/**", handler)
    return taps


def _open(browser, twin_server, brain, tab="home"):  # noqa: F811
    page, errors = open_twin(browser, twin_server, engineer=False, tab=tab)
    _forward(page, brain.client)
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_selector("#schedules-panel:not([hidden])", timeout=8000)
    return page, errors


def _home(page):
    page.click('.tab-btn[data-tab="home"]')


def _stored(brain):
    return json.loads(brain.sched.store.path.read_text())


def _shot(page, name):
    d = os.environ.get("UI_SHOTS")
    if d:
        Path(d).mkdir(parents=True, exist_ok=True)
        page.mouse.move(1, 1)          # no hover on the button just tapped
        page.wait_for_timeout(300)
        page.screenshot(path=str(Path(d) / f"twin-3.163-{name}.png"), full_page=False)


class Taps:
    def __init__(self, page):
        self.page, self.n = page, 0

    def tap(self, selector):
        self.page.click(selector)
        self.n += 1


def test_survey_every_day_at_nine_in_four_taps_and_the_home_line(browser, twin_server, stack):  # noqa: F811
    brain, _ = stack
    page, errors = _open(browser, twin_server, brain)
    _home(page)                            # a person starts on Home
    taps = Taps(page)
    taps.tap("#btn-settings")
    taps.tap("#sched-add")
    taps.tap("#sched-job-survey")
    _shot(page, "adding")
    taps.tap("#sched-save")               # 9:00 and every day are the form's own
    page.wait_for_function("() => document.querySelectorAll('#sched-list .sched-row').length === 1")
    stored = _stored(brain)
    assert stored["enabled"] is True
    assert [(s["job"], s["at"], s["days"], s["enabled"]) for s in stored["schedules"]] == \
        [("survey", "09:00", list(range(7)), True)]
    assert taps.n <= 6, taps.n
    _shot(page, "list")
    # The next run on Home, with no tap.
    _home(page)
    page.wait_for_function(
        "() => /check the house/i.test(document.getElementById('mission-schedule').textContent)",
        timeout=8000)
    assert not errors, errors


def test_all_schedules_off_is_one_tap(browser, twin_server, stack):  # noqa: F811
    brain, _ = stack
    brain.set(FIND)
    page, errors = _open(browser, twin_server, brain, tab="settings")
    page.wait_for_function("() => document.getElementById('sched-master').getAttribute('aria-pressed') === 'true'")
    page.click("#sched-master")
    page.wait_for_function("() => document.getElementById('sched-master').getAttribute('aria-pressed') === 'false'")
    assert _stored(brain)["enabled"] is False
    _shot(page, "off")
    _home(page)
    page.wait_for_function(
        "() => /schedules are off/i.test(document.getElementById('mission-schedule').textContent)",
        timeout=8000)
    assert not errors, errors


def test_a_find_needs_its_target_and_a_day(browser, twin_server, stack):  # noqa: F811
    brain, _ = stack
    page, _ = _open(browser, twin_server, brain, tab="settings")
    page.click("#sched-add")
    page.click("#sched-save")
    assert "find" in page.inner_text("#sched-error").lower()
    page.fill("#sched-target", "keys")
    page.click("#sched-days .sched-every")          # every day off: none chosen
    page.click("#sched-save")
    assert "day" in page.inner_text("#sched-error").lower()
    assert not brain.sched.store.path.exists()


def test_a_skipped_run_is_said_in_plain_words(browser, twin_server, stack):  # noqa: F811
    brain, robot_client = stack
    brain.set(FIND)
    robot_client.post("/stop", headers={"x-driver": "twin-dpad"})
    brain.tick(MON_9 + timedelta(seconds=5))
    brain.tick(MON_9 + timedelta(seconds=sch.GRACE_S + 1))
    page, errors = _open(browser, twin_server, brain, tab="settings")
    page.wait_for_function(
        "() => /someone had pressed stop/i.test(document.getElementById('sched-list').textContent)",
        timeout=8000)
    _shot(page, "skipped")
    _home(page)
    page.wait_for_function(
        "() => /skipped .*someone had pressed stop/i.test(document.getElementById('mission-schedule').textContent)",
        timeout=8000)
    assert not errors, errors


def test_a_robot_that_does_not_run_schedules_says_so(browser, twin_server, stack, monkeypatch):  # noqa: F811
    brain, _ = stack
    from control.remote_robot import RemoteRobot
    real = RemoteRobot.get_health
    monkeypatch.setattr(RemoteRobot, "get_health", lambda self: {**real(self), "mode": "hardware"})
    page, _ = _open(browser, twin_server, brain, tab="settings")
    page.wait_for_selector("#sched-not-here:not([hidden])", timeout=8000)
    assert "don't run on this robot" in page.inner_text("#sched-not-here")
    _shot(page, "not-here")


@pytest.mark.parametrize("state", ["off", "list", "adding", "skipped", "not-here"])
def test_each_state_meets_3132s_bars(browser, twin_server, stack, monkeypatch, state):  # noqa: F811
    brain, robot_client = stack
    if state in ("list", "off", "skipped"):
        brain.set(FIND, enabled=state != "off")
    if state == "skipped":
        robot_client.post("/stop", headers={"x-driver": "twin-dpad"})
        brain.tick(MON_9 + timedelta(seconds=5))
        brain.tick(MON_9 + timedelta(seconds=sch.GRACE_S + 1))
    if state == "not-here":
        from control.remote_robot import RemoteRobot
        real = RemoteRobot.get_health
        monkeypatch.setattr(RemoteRobot, "get_health", lambda self: {**real(self), "mode": "hardware"})
    page, errors = _open(browser, twin_server, brain, tab="settings")
    if state == "adding":
        page.click("#sched-add")
    page.locator("#schedules-panel").scroll_into_view_if_needed()
    page.emulate_media(reduced_motion="reduce")     # M7 is read under reduced motion (3.132)
    page.wait_for_timeout(500)
    bars = _bars(page)
    assert _clean(bars), (bars["M7"], bars)
    assert not errors, errors
