# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.176.C: the twin's Find box reads a sentence and shows what it understood
before Start (C1-C10 of docs/plans/ros-alignment/3.176-plain-language-request.md).

The parse route is answered by the REAL brain app (`control/brain_server.py`,
in process, rules only, no saved map); `/mission/start` is captured. Each test
was seen failing on `dev`'s page first.

`UI_SHOTS=<dir>` keeps C1's three phone-size screenshots.
"""

import json
import os
from pathlib import Path

import pytest

pytest.importorskip("playwright")

from fastapi.testclient import TestClient  # noqa: E402
from playwright import sync_api  # noqa: E402

from tests.test_ui import TIERED_BRAIN_HEALTH, browser, open_twin, twin_server  # noqa: F401,E402

PARSE = "/mission/request/parse"
IDLE = {"running": False, "outcome": None, "job": "find", "policy": "frontier",
        "step": 0, "log_tail": [], "tier": None, "perception": None}
RUNNING = dict(IDLE, running=True, outcome="running", step=1)


class _World:
    def get_pose(self):
        return {"usable": False}


@pytest.fixture(scope="module")
def brain():
    from control import brain_server
    app = brain_server.create_app(robot_factory=lambda: object(),
                                  world_factory=lambda robot: _World())
    return TestClient(app)


def _shot(page, name):
    out = os.environ.get("UI_SHOTS")
    if out:
        Path(out).mkdir(parents=True, exist_ok=True)
        page.locator("#brain-target-row").scroll_into_view_if_needed()
        page.screenshot(path=str(Path(out) / f"twin-3.176c-{name}.png"))


def _open(browser, twin_server, brain, *, parse="real", engineer=True):
    """The Home tab with a stubbed brain connected. `parse`: "real" answers
    the parse route from the real brain, "held" keeps each request for the
    test to answer, "abort" fails it with no answer, None leaves it to the
    stub's 404 (a brain built before 3.176.B)."""
    page, errors = open_twin(browser, twin_server, engineer=engineer, tab="home")
    calls = {"parse": [], "start": [], "other": [], "held": []}

    def stub(route):
        req = route.request
        path = req.url.split("/brain-stub", 1)[1].split("?")[0]
        if path == "/health":
            return route.fulfill(status=200, content_type="application/json",
                                 body=json.dumps(TIERED_BRAIN_HEALTH))
        if path == "/mission/status":
            return route.fulfill(status=200, content_type="application/json",
                                 body=json.dumps(RUNNING if calls["start"] else IDLE))
        if path == "/mission/start":
            calls["start"].append(req.post_data_json)
            return route.fulfill(status=200, content_type="application/json",
                                 body=json.dumps({"started": True, "status": RUNNING}))
        if path == PARSE:
            calls["parse"].append(req.post_data_json["text"])
            if parse == "real":
                r = brain.post(PARSE, json=req.post_data_json)
                return route.fulfill(status=r.status_code, content_type="application/json",
                                     body=r.text)
            if parse == "held":
                return calls["held"].append(route)
            if parse == "abort":
                return route.abort()
        calls["other"].append(path)
        route.fulfill(status=404, content_type="application/json",
                      body=json.dumps({"detail": "Not Found"}))

    page.route("**/brain-stub/**", stub)
    robot = []
    page.on("request", lambda r: robot.append(r.url) if "/brain-stub" not in r.url
            and any(p in r.url for p in ("/action", "/stop", "/wheels", "/world/goal")) else None)
    calls["robot"] = robot
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(600)
    page.click('.tab-btn[data-tab="home"]')
    return page, errors, calls


def _fulfil(route, brain):
    r = brain.post(PARSE, json=route.request.post_data_json)
    route.fulfill(status=r.status_code, content_type="application/json", body=r.text)


START = "#btn-brain-mission"
LINE = "#find-reading"


def test_a_hinted_sentence_is_shown_then_started_as_shown(browser, twin_server, brain):
    """C2 (the reading before Start; Start sends what was shown), C3 (nothing
    but the parse route while typing), C1's first image."""
    page, errors, calls = _open(browser, twin_server, brain, engineer=False)
    text = "find my small black wallet, probably in the kitchen"
    page.fill("#brain-target", text)
    line = page.locator(LINE)
    sync_api.expect(line).to_contain_text("small black wallet", timeout=5000)
    sync_api.expect(line).to_contain_text("the kitchen")
    sync_api.expect(line).to_contain_text("black, small")
    sync_api.expect(page.locator("#btn-find-drop-room")).to_be_visible()
    sync_api.expect(page.locator(START)).to_be_enabled()
    _shot(page, "hinted")
    assert calls["start"] == [] and calls["robot"] == [], calls
    # The page's own schedules read (3.163.B) is the only other brain call.
    assert set(calls["other"]) <= {"/schedules"}, calls
    page.click(START)
    sync_api.expect(page.locator(START)).to_have_text("Stop", timeout=5000)
    body = calls["start"][0]
    assert body == {"target_object": "small black wallet", "room_hint": {"type": "kitchen"},
                    "request": text, "fault": "none", "policy": body["policy"]}, body
    assert not errors, errors
    page.context.close()


def test_dropping_the_room_starts_without_a_hint(browser, twin_server, brain):
    """C2: "search everywhere" sends no room_hint; the line says so."""
    page, errors, calls = _open(browser, twin_server, brain)
    page.fill("#brain-target", "where are my glasses, maybe in the bedroom")
    sync_api.expect(page.locator(LINE)).to_contain_text("the bedroom", timeout=5000)
    page.click("#btn-find-drop-room")
    sync_api.expect(page.locator(LINE)).not_to_contain_text("bedroom")
    sync_api.expect(page.locator("#btn-find-drop-room")).to_be_hidden()
    page.click(START)
    sync_api.expect(page.locator(START)).to_have_text("Stop", timeout=5000)
    body = calls["start"][0]
    assert body["target_object"] == "glasses" and "room_hint" not in body, body
    assert body["request"] == "where are my glasses, maybe in the bedroom", body
    assert not errors, errors
    page.context.close()


def test_a_plain_target_sends_devs_body_exactly(browser, twin_server, brain):
    """C2 (a one-word target), C7 (no `request`, the typed case kept), C1's
    second image."""
    page, errors, calls = _open(browser, twin_server, brain, engineer=False)
    page.fill("#brain-target", "Backpack")
    sync_api.expect(page.locator(LINE)).to_contain_text("backpack", timeout=5000)
    sync_api.expect(page.locator("#btn-find-drop-room")).to_be_hidden()
    _shot(page, "plain")
    page.click(START)
    sync_api.expect(page.locator(START)).to_have_text("Stop", timeout=5000)
    body = calls["start"][0]
    assert set(body) == {"target_object", "fault", "policy"}, body
    assert body["target_object"] == "Backpack", body
    assert not errors, errors
    page.context.close()


def test_an_ambiguous_room_gives_no_hint_and_shows_the_words_read_past(browser, twin_server, brain):
    """C5 (the room line only when a room was read), C1's third image.
    C1's three images are the operator view: what a person sees."""
    page, errors, calls = _open(browser, twin_server, brain, engineer=False)
    page.fill("#brain-target", "find my keys in the kitchen or living room")
    line = page.locator(LINE)
    sync_api.expect(line).to_contain_text("keys", timeout=5000)
    sync_api.expect(line).to_contain_text("kitchen or living room")   # read past
    sync_api.expect(line).not_to_contain_text("starting in")
    sync_api.expect(page.locator("#btn-find-drop-room")).to_be_hidden()
    _shot(page, "ambiguous")
    page.click(START)
    sync_api.expect(page.locator(START)).to_have_text("Stop", timeout=5000)
    body = calls["start"][0]
    assert body["target_object"] == "keys" and "room_hint" not in body, body
    assert not errors, errors
    page.context.close()


def test_no_single_thing_keeps_start_off_and_still_shows_the_room(browser, twin_server, brain):
    """C5: "find it in the kitchen" names no thing."""
    page, errors, calls = _open(browser, twin_server, brain)
    page.fill("#brain-target", "find it in the kitchen")
    line = page.locator(LINE)
    sync_api.expect(line).to_contain_text("target: null", timeout=5000)
    sync_api.expect(line).to_contain_text("the kitchen")
    sync_api.expect(page.locator(START)).to_be_disabled()
    assert not errors, errors
    page.context.close()


def test_the_brain_not_answering_keeps_start_off_with_the_reason(browser, twin_server, brain):
    """C2: no answer to the parse, no Start; the line says why."""
    page, errors, calls = _open(browser, twin_server, brain, parse="abort")
    page.fill("#brain-target", "find my keys in the kitchen")
    sync_api.expect(page.locator(LINE)).to_contain_text("Parse failed", timeout=5000)
    sync_api.expect(page.locator(START)).to_be_disabled()
    assert calls["start"] == []
    page.context.close()


def test_an_older_brain_without_the_route_runs_devs_find(browser, twin_server, brain):
    """C6: a 404 on the parse is a brain built before 3.176.B; the words go
    as typed, as `dev` sent them."""
    page, errors, calls = _open(browser, twin_server, brain, parse=None)
    page.fill("#brain-target", "find my keys in the kitchen")
    sync_api.expect(page.locator(LINE)).to_contain_text("404", timeout=5000)
    page.click(START)
    sync_api.expect(page.locator(START)).to_have_text("Stop", timeout=5000)
    body = calls["start"][0]
    assert body == {"target_object": "find my keys in the kitchen", "fault": "none",
                    "policy": body["policy"]}, body
    assert not errors, errors
    page.context.close()


def test_an_edit_reads_again_and_an_older_reply_is_not_shown(browser, twin_server, brain):
    """C4: Start is off from the keystroke until the new text is read, and a
    reply for the older text arriving last changes nothing."""
    page, errors, calls = _open(browser, twin_server, brain, parse="held")
    # Connecting reads the box's text already there: answer it first.
    page.wait_for_function("() => true")
    for _ in range(50):
        if calls["held"]:
            break
        page.wait_for_timeout(100)
    while calls["held"]:
        _fulfil(calls["held"].pop(0), brain)
    page.fill("#brain-target", "find the red backpack")
    page.wait_for_timeout(500)
    assert len(calls["held"]) == 1, calls
    page.fill("#brain-target", "find the blue mug in the kitchen")
    sync_api.expect(page.locator(START)).to_be_disabled()
    page.wait_for_timeout(500)
    assert len(calls["held"]) == 2, calls
    old, new = calls["held"]
    _fulfil(new, brain)
    sync_api.expect(page.locator(LINE)).to_contain_text("blue mug", timeout=5000)
    _fulfil(old, brain)
    page.wait_for_timeout(300)
    sync_api.expect(page.locator(LINE)).to_contain_text("blue mug")
    sync_api.expect(page.locator(LINE)).not_to_contain_text("red backpack")
    page.click(START)
    sync_api.expect(page.locator(START)).to_have_text("Stop", timeout=5000)
    assert calls["start"][0]["target_object"] == "blue mug", calls["start"]
    assert not errors, errors
    page.context.close()


@pytest.mark.parametrize("engineer", [False, True])
def test_both_views_show_the_reading_in_their_words(browser, twin_server, brain, engineer):
    """C8: plain words for a person; field names and the names read for an
    engineer."""
    page, errors, calls = _open(browser, twin_server, brain, engineer=engineer)
    page.fill("#brain-target", "find my blue mug in the kitchen please")
    line = page.locator(LINE)
    sync_api.expect(line).to_contain_text("blue mug", timeout=5000)
    if engineer:
        sync_api.expect(line).to_contain_text("room_hint: the kitchen")
        sync_api.expect(line).to_contain_text("names_read: 0")
    else:
        sync_api.expect(line).to_contain_text("I'll look for blue mug, starting in the kitchen.")
        sync_api.expect(line).not_to_contain_text("_")
    assert not errors, errors
    page.context.close()


def test_an_empty_box_keeps_start_off(browser, twin_server, brain):
    """C9: `dev` searched for "red backpack" when the box was empty."""
    page, errors, calls = _open(browser, twin_server, brain)
    page.fill("#brain-target", "   ")
    sync_api.expect(page.locator(LINE)).to_contain_text("Empty target", timeout=5000)
    sync_api.expect(page.locator(START)).to_be_disabled()
    assert not errors, errors
    page.context.close()


def test_a_survey_shows_no_reading_and_reads_nothing(browser, twin_server, brain):
    """C10: a survey has no sentence; its start is `dev`'s."""
    page, errors, calls = _open(browser, twin_server, brain)
    page.select_option("#brain-job", "survey")
    n = len(calls["parse"])
    sync_api.expect(page.locator(LINE)).to_be_hidden()
    page.click(START)
    sync_api.expect(page.locator(START)).to_have_text("Stop", timeout=5000)
    assert len(calls["parse"]) == n, calls
    body = calls["start"][0]
    assert body.get("job") == "survey" and "target_object" not in body, body
    assert "request" not in body and "room_hint" not in body, body
    assert not errors, errors
    page.context.close()
