# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.201: the Home target field shows the running find's target when the find
was started elsewhere (the API), and never overwrites a person's words
(criteria 2-4 of docs/plans/ros-alignment/3.201-twin-nogo-toast-target.md).

The brain is a stub whose /mission/status the test controls.
`UI_SHOTS=<dir>` keeps the phone-size screenshot.
"""

import json
import os
from pathlib import Path

import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402

from tests.test_ui import TIERED_BRAIN_HEALTH, browser, open_twin, twin_server  # noqa: F401,E402

IDLE = {"running": False, "outcome": None, "job": "find", "policy": "frontier",
        "step": 0, "log_tail": [], "tier": None, "perception": None}
FIELD = "#brain-target"


def _open(browser, twin_server):
    page, errors = open_twin(browser, twin_server, engineer=False, tab="home")
    box = {"status": dict(IDLE), "start": []}

    def stub(route):
        path = route.request.url.split("/brain-stub", 1)[1].split("?")[0]
        if path == "/health":
            body = TIERED_BRAIN_HEALTH
        elif path == "/mission/status":
            body = box["status"]
        elif path == "/mission/start":
            box["start"].append(route.request.post_data_json)
            box["status"] = dict(IDLE, running=True, outcome="running", step=1,
                                 target_object=route.request.post_data_json["target_object"])
            body = {"started": True, "status": box["status"]}
        else:
            return route.fulfill(status=404, content_type="application/json",
                                 body=json.dumps({"detail": "Not Found"}))
        route.fulfill(status=200, content_type="application/json", body=json.dumps(body))

    page.route("**/brain-stub/**", stub)
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(600)
    page.click('.tab-btn[data-tab="home"]')
    return page, errors, box


def _api_find(box, target, **extra):
    box["status"] = dict(IDLE, running=True, outcome="running", step=3,
                         target_object=target, **extra)


def test_an_api_find_shows_its_target_in_the_field(browser, twin_server):
    page, errors, box = _open(browser, twin_server)
    sync_api.expect(page.locator(FIELD)).to_have_value("red backpack")
    _api_find(box, "blue mug")
    sync_api.expect(page.locator(FIELD)).to_have_value("blue mug", timeout=5000)
    out = os.environ.get("UI_SHOTS")
    if out:
        Path(out).mkdir(parents=True, exist_ok=True)
        page.locator("#brain-target-row").scroll_into_view_if_needed()
        page.screenshot(path=str(Path(out) / "twin-3.201-target.png"))
    assert page.evaluate("() => document.documentElement.scrollWidth") <= 390
    assert not errors, errors
    page.context.close()


def test_a_focused_field_is_not_overwritten(browser, twin_server):
    page, errors, box = _open(browser, twin_server)
    page.focus(FIELD)
    page.keyboard.press("End")
    page.keyboard.type("x")          # the field now holds "red backpackx"
    _api_find(box, "blue mug")
    page.wait_for_timeout(2500)      # several polls
    sync_api.expect(page.locator(FIELD)).to_have_value("red backpackx")
    assert not errors, errors
    page.context.close()


def test_words_typed_earlier_are_not_overwritten_after_blur(browser, twin_server):
    page, errors, box = _open(browser, twin_server)
    page.fill(FIELD, "garden hose")
    page.click("body", position={"x": 5, "y": 5})
    _api_find(box, "blue mug")
    page.wait_for_timeout(2500)
    sync_api.expect(page.locator(FIELD)).to_have_value("garden hose")
    assert not errors, errors
    page.context.close()


def test_a_find_this_page_started_keeps_the_typed_sentence(browser, twin_server):
    page, errors, box = _open(browser, twin_server)
    page.fill(FIELD, "red backpack")
    sync_api.expect(page.locator("#btn-brain-mission")).to_be_enabled(timeout=5000)
    page.click("#btn-brain-mission")
    sync_api.expect(page.locator("#btn-brain-mission")).to_have_text("Stop", timeout=5000)
    page.wait_for_timeout(2000)
    sync_api.expect(page.locator(FIELD)).to_have_value("red backpack")
    assert len(box["start"]) == 1
    assert not errors, errors
    page.context.close()


def test_a_survey_leaves_the_field_alone(browser, twin_server):
    page, errors, box = _open(browser, twin_server)
    _api_find(box, "blue mug", job="survey")
    page.wait_for_timeout(2500)
    sync_api.expect(page.locator(FIELD)).to_have_value("red backpack")
    assert not errors, errors
    page.context.close()
