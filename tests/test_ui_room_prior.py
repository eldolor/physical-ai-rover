# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_ui_room_prior.py

3.190.D (criterion 15's page half): the twin's mission card gains one line
in operator mode saying where a find looks first -- the likeliest room types
for its target from `status.room_prior` (a cloud answer, its cache or the
shipped table), or, with no prior, why not, in plain words. Hidden with the
key off (no `room_prior` in the status), for a survey, for a cause the
person can do nothing about on this mission, for a prior that does not reach
the goal choice, and with the brain lost. Driven at the 390 px phone
viewport (test_ui.py's `open_twin`) against a stubbed brain, as
tests/test_ui_dock.py.
"""

import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402

from tests.test_ui import (TIERED_BRAIN_HEALTH, _json, browser,  # noqa: F401,E402 -- fixtures
                           open_twin, tiered_status, twin_server)
from tests.test_twin_strings import jargon_hits  # noqa: E402

MODEL = "us.anthropic.claude-opus-5-5"
FRIDGE = {"kitchen": 0.95, "living room": 0.05, "bedroom": 0.02, "bathroom": 0.01}


def status(prior, job="find", target="refrigerator", running=True):
    body = tiered_status(running=running)
    body["job"], body["target_object"] = job, target
    if prior is not None:
        body["room_prior"] = prior
    return body


def prior(source, rates=None, why=None, acts=None):
    return {"source": source, "why": why, "rates": rates,
            "model_id": None if source == "shipped" else MODEL,
            "acts": bool(rates) if acts is None else acts}


SHOWN = {
    "cloud": (prior("cloud", FRIDGE), "cloud",
              "Likeliest for the refrigerator: the kitchen (asked online), so it looks there first."),
    "cache": (prior("cache", FRIDGE), "cache",
              "Likeliest for the refrigerator: the kitchen "
              "(remembered from an earlier search), so it looks there first."),
    "shipped": (prior("shipped", {"kitchen": 0.2, "living room": 0.6, "bedroom": 0.1,
                                  "bathroom": 0.05}), "shipped",
                "Likeliest for the refrigerator: the living room, so it looks there first."),
    # Only a true tie names two, in the types' order (3.116.C's garbage can:
    # kitchen and bathroom at 0.85); a near second is not named (Thermos: the
    # goal choice promises no order past the top).
    "near tie": (prior("cloud", {"kitchen": 0.85, "living room": 0.4, "bedroom": 0.3,
                                 "bathroom": 0.80}), "cloud",
                 "Likeliest for the refrigerator: the kitchen (asked online), so it looks "
                 "there first."),
    "two rooms": (prior("cloud", {"kitchen": 0.85, "living room": 0.4, "bedroom": 0.3,
                                  "bathroom": 0.85}), "cloud",
                  "Likeliest for the refrigerator: the kitchen and the bathroom (asked "
                  "online), so it looks there first."),
    # 3.190.B's one flat reply (candle): orders nothing, so says so.
    "flat": (prior("cloud", {"kitchen": 0.30, "living room": 0.45, "bedroom": 0.30,
                             "bathroom": 0.35}), "flat",
             "No room stands out for the refrigerator, so every room is searched alike."),
    "no names": (prior("none", why="no room-label layer"), "none",
                 "No guess at which room the refrigerator is in (no rooms are named on the "
                 "map), so every room is searched alike."),
    "offline": (prior("none", why="unreachable"), "none",
                "No guess at which room the refrigerator is in (it couldn't ask online), so "
                "every room is searched alike."),
    "timeout": (prior("none", why="timeout"), "none",
                "No guess at which room the refrigerator is in (it couldn't ask online), so "
                "every room is searched alike."),
    "cooled": (prior("none", why="failed recently (http 502): not asked again yet"), "none",
               "No guess at which room the refrigerator is in (it couldn't ask online), so "
               "every room is searched alike."),
    # A cooled target is said by what cooled it (review): a cut reply was answered.
    "cooled cut": (prior("none", why="failed recently (reply cut (max_tokens)): not asked again yet"),
                   "none", "No guess at which room the refrigerator is in (the answer couldn't be "
                   "read), so every room is searched alike."),
    "bad reply": (prior("none", why="reply is not a prior"), "none",
                  "No guess at which room the refrigerator is in (the answer couldn't be "
                  "read), so every room is searched alike."),
    "unknown cause": (prior("none", why="target too long to ask"), "none",
                      "No guess at which room the refrigerator is in (it couldn't get one), "
                      "so every room is searched alike."),
}

HIDDEN = {
    "key off": status(None),
    "survey": status(prior("none", why="not a find"), job="survey"),
    "no explore": status(prior("none", why="no explore search")),
    "rule policy": status(prior("none", why="explore_goal_policy is not 'rooms'")),
    "acts false": status(prior("cloud", FRIDGE, acts=False)),
    "shipped, acts false": status(prior("shipped", FRIDGE, acts=False)),
    # A survey carrying a prior that would show: the job alone hides it.
    "survey with a prior": status(prior("cloud", FRIDGE), job="survey"),
}

# Review: an ended find keeps its line in the past tense, under the card's
# outcome, never "Looking ... first" under "Found the ...".
ENDED = {
    "cloud": (prior("cloud", FRIDGE), "cloud",
              "Likeliest for the refrigerator: the kitchen (asked online), so it looked there first."),
    "cache": (prior("cache", FRIDGE), "cache",
              "Likeliest for the refrigerator: the kitchen "
              "(remembered from an earlier search), so it looked there first."),
    "shipped": (prior("shipped", FRIDGE), "shipped",
                "Likeliest for the refrigerator: the kitchen, so it looked there first."),
    "flat": (SHOWN["flat"][0], "flat",
             "No room stood out for the refrigerator, so every room was searched alike."),
    "none": (prior("none", why="timeout"), "none",
             "There was no guess at which room the refrigerator was in (it couldn't ask "
             "online), so every room was searched alike."),
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


@pytest.mark.parametrize("case", list(SHOWN))
def test_operator_line_says_where_the_find_looks_first(browser, twin_server, case):
    body, source, words = SHOWN[case]
    current = {"body": status(None)}
    page, errors = open_twin(browser, twin_server, engineer=False)
    _connect(page, twin_server, current)
    line = page.locator("#mission-prior")
    sync_api.expect(page.locator("#mission-card")).to_be_visible(timeout=5000)
    sync_api.expect(line).to_be_hidden()            # the key off: no line

    current["body"] = status(body)
    sync_api.expect(line).to_have_text(words, timeout=5000)
    sync_api.expect(line).to_have_attribute("data-source", source)
    assert not jargon_hits(line.inner_text()), jargon_hits(line.inner_text())
    assert not errors, errors
    page.close()


@pytest.mark.parametrize("case", list(ENDED))
def test_an_ended_find_says_it_in_the_past(browser, twin_server, case):
    body, source, words = ENDED[case]
    current = {"body": status(body)}
    page, errors = open_twin(browser, twin_server, engineer=False)
    _connect(page, twin_server, current)
    line = page.locator("#mission-prior")
    sync_api.expect(line).to_be_visible(timeout=5000)
    sync_api.expect(line).not_to_have_text(words)    # running: the present tense
    current["body"] = status(body, running=False)
    sync_api.expect(line).to_have_text(words, timeout=5000)
    sync_api.expect(line).to_have_attribute("data-source", source)
    assert not jargon_hits(line.inner_text()), jargon_hits(line.inner_text())
    assert not errors, errors
    page.close()


@pytest.mark.parametrize("case", list(HIDDEN))
def test_no_line_where_the_prior_cannot_act(browser, twin_server, case):
    """Shown first with a cloud prior, then each body must take it away: a
    line that stayed would say a guess the mission is not using."""
    current = {"body": status(SHOWN["cloud"][0])}
    page, errors = open_twin(browser, twin_server, engineer=False)
    _connect(page, twin_server, current)
    line = page.locator("#mission-prior")
    sync_api.expect(line).to_be_visible(timeout=5000)
    current["body"] = HIDDEN[case]
    sync_api.expect(line).to_be_hidden(timeout=5000)
    assert not errors, errors
    page.close()


def test_the_line_is_operator_mode_only(browser, twin_server):
    current = {"body": status(SHOWN["cloud"][0])}
    page, errors = open_twin(browser, twin_server)        # Engineer mode
    _connect(page, twin_server, current)
    sync_api.expect(page.locator("#brain-tel-outcome")).not_to_have_text("–", timeout=5000)
    sync_api.expect(page.locator("#mission-prior")).to_be_hidden()
    assert not errors, errors
    page.close()


def test_the_line_goes_when_the_brain_is_lost(browser, twin_server):
    current = {"body": status(SHOWN["cloud"][0])}
    page, errors = open_twin(browser, twin_server, engineer=False)
    _connect(page, twin_server, current)
    line = page.locator("#mission-prior")
    sync_api.expect(line).to_be_visible(timeout=5000)
    page.unroute("**/brain-stub/mission/status")
    page.unroute("**/brain-stub/health")
    page.route("**/brain-stub/**", lambda route: route.abort())
    sync_api.expect(line).to_be_hidden(timeout=15000)
    page.close()
