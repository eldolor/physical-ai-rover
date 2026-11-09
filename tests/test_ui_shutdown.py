# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.170: the twin's Shut down, at a phone's width, in the operator's view.
Criteria 6 and 7, docs/plans/ros-alignment/3.170-twin-shutdown-button.md; each
seen to fail on the page before the change.

`GET /battery` is stubbed per state (the live phases are
evaluations/shutdown-3170/run.py); every test runs in operator mode, with no
brain connected unless it says so.
"""

import json

import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402

from tests.test_ui import (_battery_page, _battery_reply, _battery_shot, _json,  # noqa: F401,E402
                           _route_battery, browser, tiered_status, twin_server)
from tests.test_ui_operator_words import jargon_hits  # noqa: E402
from tests.test_ui_stage3 import _connect_brain  # noqa: E402

IDLE = _battery_reply("ok", 12.0, request=None)


def reply(phase, settle_s=30.0):
    return _battery_reply("ok", 12.0, request={"by": "twin-dpad", "seconds_ago": 1.0,
                                               "phase": phase, "settle_s": settle_s})


def _page(browser, twin_server, battery=IDLE, status=200, detail="", answer=None):  # noqa: F811
    """The twin, its robot connected, on Settings. Returns the POSTs seen."""
    page, errors = _battery_page(browser, twin_server, battery)
    posts = []

    def handle(route):
        posts.append(route.request.post_data)
        route.fulfill(status=status, content_type="application/json",
                      body=_json({"detail": detail} if status != 200
                                 else {"started": True, "request": answer}))
    page.route("**/power/shutdown", handle)
    page.click("#btn-settings")
    if battery is IDLE:     # under way, the button is rightly off
        sync_api.expect(page.locator("#btn-shutdown")).to_be_enabled(timeout=6000)
    return page, errors, posts


def _shut_down(page):
    page.click("#btn-shutdown")
    page.click("#btn-shutdown-yes")


def _banner(page, text, timeout=6000):
    banner = page.locator("#battery-banner")
    sync_api.expect(banner).to_be_visible(timeout=timeout)
    sync_api.expect(page.locator("#battery-title")).to_contain_text(text, timeout=timeout)
    return banner.inner_text()


def _not_safe(text):
    assert "safe to press" not in text.lower(), text


def test_an_operator_is_asked_first_and_cancel_sends_nothing(browser, twin_server):  # noqa: F811
    page, errors, posts = _page(browser, twin_server)
    confirm = page.locator("#shutdown-confirm")
    sync_api.expect(page.locator("#btn-shutdown")).to_be_visible()
    sync_api.expect(confirm).to_be_hidden()
    page.click("#btn-shutdown")
    sync_api.expect(confirm).to_be_visible()
    sync_api.expect(page.locator("#shutdown-confirm-text")).to_contain_text("Shut down the robot")
    assert not jargon_hits(confirm.inner_text()), confirm.inner_text()
    _battery_shot(page, "shutdown-confirm-390.png")
    assert posts == []
    page.click("#btn-shutdown-cancel")
    sync_api.expect(confirm).to_be_hidden()
    page.wait_for_timeout(300)
    assert posts == [] and not errors, (posts, errors)
    page.context.close()


def test_a_double_tap_sends_one_request(browser, twin_server):  # noqa: F811
    page, errors, posts = _page(browser, twin_server, answer=reply("stopping")["ladder"]["request"])
    page.click("#btn-shutdown")
    _route_battery(page, reply("stopping"))
    page.dblclick("#btn-shutdown-yes", force=True)
    page.wait_for_timeout(500)
    assert len(posts) == 1, posts
    sync_api.expect(page.locator("#btn-shutdown")).to_be_disabled()
    assert not errors, errors
    page.context.close()


def test_a_confirmation_names_the_live_mission(browser, twin_server):  # noqa: F811
    page, errors, posts = _page(browser, twin_server)
    status = {"body": tiered_status(running=False)}
    _connect_brain(page, twin_server, status)
    status["body"] = tiered_status(running=True)
    page.click('.tab-btn[data-tab="home"]')
    page.fill("#brain-target", "red backpack")
    page.click("#btn-brain-mission")
    page.wait_for_function(
        "document.getElementById('btn-brain-mission').textContent.trim() === 'Stop'", timeout=5000)
    page.click("#btn-settings")
    page.click("#btn-shutdown")
    text = page.locator("#shutdown-confirm-text")
    sync_api.expect(text).to_contain_text("red backpack")
    sync_api.expect(text).to_contain_text("end")
    _battery_shot(page, "shutdown-confirm-mission-390.png")
    assert posts == [] and not errors, (posts, errors)
    page.context.close()


def test_a_mission_only_the_robot_knows_of_is_named_too(browser, twin_server):  # noqa: F811
    """The brain is out of reach: `/health` still says a mission is live."""
    page, errors, posts = _page(browser, twin_server)
    health = page.request.get(twin_server + "/health").json()
    health["watchdog"] = {"mission_live": True, "trip": None, "trip_after_s": 24.0, "last_fired": None}
    page.route("**/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(health)))
    page.click("#btn-health-check")
    page.wait_for_timeout(2500)
    page.click("#btn-shutdown")
    sync_api.expect(page.locator("#shutdown-confirm-text")).to_contain_text("search", timeout=3000)
    assert not errors, errors
    page.context.close()


def test_while_it_answers_it_is_not_safe(browser, twin_server):  # noqa: F811
    page, errors, posts = _page(browser, twin_server, answer=reply("stopping")["ladder"]["request"])
    _shut_down(page)
    _route_battery(page, reply("stopping"))
    text = _banner(page, "don't press")
    _not_safe(text)
    assert not jargon_hits(text), text
    _battery_shot(page, "shutdown-stopping-390.png")
    _route_battery(page, reply("powering_off"))
    sync_api.expect(page.locator("#battery-title")).to_contain_text("Powering off", timeout=4000)
    _not_safe(page.locator("#battery-banner").inner_text())
    # Every tab shows it.
    page.click('.tab-btn[data-tab="drive"]')
    sync_api.expect(page.locator("#battery-banner")).to_be_visible()
    assert posts and not errors, (posts, errors)
    page.context.close()


def test_quiet_after_the_power_off_waits_then_names_the_light(browser, twin_server):  # noqa: F811
    page, errors, _ = _page(browser, twin_server, battery=reply("powering_off", settle_s=3.0))
    _banner(page, "Powering off")
    _route_battery(page, None)
    text = _banner(page, "wait")
    _not_safe(text)
    text = _banner(page, "Safe to press", timeout=10000)
    assert "light" in text and "can't see" in text, text
    assert not jargon_hits(text), text
    _battery_shot(page, "shutdown-safe-390.png")
    assert not errors, errors
    page.context.close()


def test_a_reload_keeps_what_it_knew(browser, twin_server):  # noqa: F811
    page, errors, _ = _page(browser, twin_server, battery=reply("powering_off", settle_s=1.0))
    _banner(page, "Powering off")
    _route_battery(page, None)
    _banner(page, "Safe to press", timeout=8000)
    page.reload(wait_until="networkidle")
    _banner(page, "Safe to press", timeout=8000)
    page.context.close()


def test_quiet_before_the_power_off_says_it_cannot_know(browser, twin_server):  # noqa: F811
    page, errors, _ = _page(browser, twin_server, battery=reply("stopping", settle_s=1.0))
    _banner(page, "don't press")
    _route_battery(page, None)
    text = _banner(page, "Can't tell")
    assert "light" in text, text
    page.wait_for_timeout(2500)      # well past settle_s: time proves nothing here
    text = page.locator("#battery-banner").inner_text()
    assert "Can't tell" in text, text
    _not_safe(text)
    _battery_shot(page, "shutdown-unknown-390.png")
    assert not errors, errors
    page.context.close()


def test_a_failed_power_off_says_it_is_still_on(browser, twin_server):  # noqa: F811
    page, errors, _ = _page(browser, twin_server, battery=reply("failed"))
    text = _banner(page, "couldn't power itself off")
    assert "still on" in text and "hard cut" in text, text
    _not_safe(text)
    assert not jargon_hits(text), text
    _battery_shot(page, "shutdown-failed-390.png")
    assert not errors, errors
    page.context.close()


def test_the_stub_says_nothing_was_powered_off(browser, twin_server):  # noqa: F811
    page, errors, _ = _page(browser, twin_server, battery=reply("stub"))
    text = _banner(page, "Nothing was powered off")
    _not_safe(text)
    assert not errors, errors
    page.context.close()


def test_a_fresh_boot_clears_it(browser, twin_server):  # noqa: F811
    page, errors, _ = _page(browser, twin_server, battery=reply("powering_off"))
    _banner(page, "Powering off")
    _route_battery(page, None)
    _banner(page, "wait")
    _route_battery(page, IDLE)
    sync_api.expect(page.locator("#battery-banner")).to_be_hidden(timeout=6000)
    sync_api.expect(page.locator("#btn-shutdown")).to_be_enabled(timeout=3000)
    assert not errors, errors
    page.context.close()


@pytest.mark.parametrize("status,detail,words", [
    (401, "Missing or invalid x-app-secret header.", "access code"),
    (403, "shutdown_no_secret: set APP_SHARED_SECRET on this robot", "access code"),
])
def test_a_refused_request_says_why_and_the_button_stays(browser, twin_server, status, detail, words):  # noqa: F811
    page, errors, posts = _page(browser, twin_server, status=status, detail=detail)
    _shut_down(page)
    err = page.locator("#shutdown-error")
    sync_api.expect(err).to_contain_text(words, timeout=4000)
    assert not jargon_hits(err.inner_text()), err.inner_text()
    sync_api.expect(page.locator("#battery-banner")).to_be_hidden()
    sync_api.expect(page.locator("#btn-shutdown")).to_be_enabled()
    assert len(posts) == 1 and json.loads(posts[0]) == {}, posts
    page.context.close()
