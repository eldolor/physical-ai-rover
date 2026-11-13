# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_ui_dock.py

3.156.E (D6, bar G6): the twin shows the trip home. The mission card gains a
Return line in operator mode (plain words) and a "Return" row in Engineer
mode (the raw state and reason, and where the dock came from: the saved
map's or the mission's start), for each of returning, docked, failed (with
its reason), skipped and charging. Driven at the 390 px phone viewport
(test_ui.py's `open_twin`), against a stubbed brain, as
tests/test_ui_local_vlm.py.
"""

import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402

from tests.test_ui import (TIERED_BRAIN_HEALTH, _json, browser,  # noqa: F401,E402 -- fixtures
                           open_twin, tiered_status, twin_server)
from tests.test_twin_strings import jargon_hits  # noqa: E402

SAVED = {"x_m": 0.45, "y_m": 0.45, "theta": 90.0, "source": "saved_map"}
START = {**SAVED, "source": "start"}


def status(ret=None, charge=None, dock=SAVED, running=True, outcome=None):
    body = tiered_status(running=running)
    body["outcome"] = outcome or ("running" if running else "exhausted")
    body["return"], body["charge"], body["dock"] = ret, charge, dock
    return body


CASES = {
    "returning": (status({"state": "returning", "after": "found"}), "returning",
                  "Going back to the dock.", "returning · dock: saved_map"),
    "docked": (status({"state": "docked", "reason": "0.032 m, 0.7 deg from the dock"},
                      running=False, outcome="found"), "docked",
               "Back on the dock.", "docked: 0.032 m, 0.7 deg from the dock · dock: saved_map"),
    "failed": (status({"state": "failed", "reason": "nav2_aborted"}, dock=START, running=False),
               "failed", "Couldn't get back to the dock — the way back was blocked.",
               "failed: nav2_aborted · dock: start"),
    "skipped": (status({"state": "skipped", "reason": "no_nav2"}, dock=START, running=False),
                "skipped", "Didn't go back to the dock — it can't plan a route on this robot.",
                "skipped: no_nav2 · dock: start"),
    "charging": (status({"state": "docked", "reason": "0.04 m"},
                        charge={"state": "charging", "reason": None, "cycles": 1,
                                "resume_v": 11.8, "v": 11.23, "history": []}),
                 "charging", "Charging on the dock (11.23 of 11.8 volts), then it carries on.",
                 "charging 11.23 → 11.8 V (charge 1) · dock: saved_map"),
}


def _connect(page, twin_server, current):
    page.route("**/brain-stub/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(TIERED_BRAIN_HEALTH)))
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(current["body"])))
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(600)
    page.click('.tab-btn[data-tab="home"]')


@pytest.mark.parametrize("case", sorted(CASES))
def test_operator_line_says_the_trip_home_in_plain_words(browser, twin_server, case):
    body, kind, words, _ = CASES[case]
    current = {"body": status(None)}
    page, errors = open_twin(browser, twin_server, engineer=False)
    _connect(page, twin_server, current)
    line = page.locator("#mission-return")
    sync_api.expect(page.locator("#mission-card")).to_be_visible(timeout=5000)
    sync_api.expect(line).to_be_hidden()           # no return yet: no line

    current["body"] = body
    sync_api.expect(line).to_have_text(words, timeout=5000)
    sync_api.expect(line).to_have_attribute("data-state", kind)
    assert not jargon_hits(line.inner_text()), jargon_hits(line.inner_text())
    sync_api.expect(page.locator("#brain-tel-return")).to_be_hidden()   # Engineer's
    assert not errors, errors
    page.close()


@pytest.mark.parametrize("case", sorted(CASES))
def test_engineer_row_names_the_state_the_reason_and_the_dock(browser, twin_server, case):
    body, _, _, eng = CASES[case]
    current = {"body": status(None)}
    page, errors = open_twin(browser, twin_server)
    _connect(page, twin_server, current)
    row = page.locator("#brain-tel-return")
    sync_api.expect(row).to_have_text("–", timeout=5000)
    current["body"] = body
    sync_api.expect(row).to_have_text(eng, timeout=5000)
    if case == "docked":
        sync_api.expect(row).to_have_class("val safe")
    if case == "failed":
        sync_api.expect(row).to_have_class("val warn")
    assert not errors, errors
    page.close()


def test_a_reason_the_twin_does_not_know_is_said_plainly(browser, twin_server):
    current = {"body": status({"state": "failed", "reason": "error: preempted by an "
                                                          "autonomous driver: ros"},
                              running=False)}
    page, errors = open_twin(browser, twin_server, engineer=False)
    _connect(page, twin_server, current)
    sync_api.expect(page.locator("#mission-return")).to_have_text(
        "Couldn't get back to the dock — something went wrong.", timeout=5000)
    assert not errors, errors
    page.close()


def test_a_resumed_mission_shows_no_return(browser, twin_server):
    """After a charge the runner clears `return`: the search is on again."""
    current = {"body": CASES["charging"][0]}
    page, errors = open_twin(browser, twin_server, engineer=False)
    _connect(page, twin_server, current)
    line = page.locator("#mission-return")
    sync_api.expect(line).to_be_visible(timeout=5000)
    current["body"] = status(None, charge={"state": "resumed", "reason": "11.8 V",
                                           "cycles": 1, "resume_v": 11.8, "v": 11.81,
                                           "history": []})
    sync_api.expect(line).to_be_hidden(timeout=5000)
    assert not errors, errors
    page.close()


def test_the_line_goes_when_the_brain_is_lost(browser, twin_server):
    """Review: a stale "going back to the dock" over a brain that no longer
    answers says something nobody knows."""
    current = {"body": CASES["returning"][0]}
    page, errors = open_twin(browser, twin_server, engineer=False)
    _connect(page, twin_server, current)
    line = page.locator("#mission-return")
    sync_api.expect(line).to_be_visible(timeout=5000)
    page.unroute("**/brain-stub/mission/status")
    page.unroute("**/brain-stub/health")
    page.route("**/brain-stub/**", lambda route: route.abort())
    sync_api.expect(line).to_be_hidden(timeout=15000)
    page.close()
