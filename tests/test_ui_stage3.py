# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.132 Stage 3A: the twin's layout (criteria 1-4,
docs/plans/ros-alignment/3.132-ux-redesign.md).

* 1: four tabs -- Home, Drive, Record, Settings -- each owning its panels,
  and the names a device saved before opening where the scope says.
* 2: the STOP bar -- hidden while nothing moves, on every tab while a
  mission runs or anyone holds the robot, and one tap sends the robot's
  /stop and then the mission's stop (only the mission's with no robot).
* 3: the mission card's sentence passes the M5 deny-list in every state,
  and the header names the robot's real mode in both voices.
* 4: an idle robot's watchdog is drawn neutral; during a mission, amber.

Each was written before the change it pins and seen to fail on the
Stage 2B page.
"""

import pytest

pytest.importorskip("playwright")

from tests.test_ui import (TIERED_BRAIN_HEALTH, _json, browser,  # noqa: F401,E402
                           open_twin, private_twin_server, tiered_status, twin_server)
from tests.test_ui_operator_words import ENDINGS, ended, jargon_hits  # noqa: E402

TABS = ["home", "drive", "record", "settings"]


def _connect_brain(page, twin_server, status, health=None):
    """A stubbed brain at <twin>/brain-stub, connected through Settings."""
    page.route("**/brain-stub/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(health or TIERED_BRAIN_HEALTH)))
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(status["body"])))
    page.route("**/brain-stub/mission/start", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({"started": True})))
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_selector("#btn-brain-mission:not([disabled])", state="attached", timeout=5000)


def _start(page, status):
    """Start a (stubbed) mission from Home's Start, as a person does: an idle
    twin does not notice one started elsewhere (F1, 3B's)."""
    status["body"] = tiered_status(running=True)
    _tab(page, "home")
    page.click("#btn-brain-mission")
    page.wait_for_function(
        "() => /stop/i.test(document.getElementById('btn-brain-mission').textContent)", timeout=5000)


def _tab(page, name):
    page.click(f'.tab-btn[data-tab="{name}"]')
    page.wait_for_function("n => document.body.dataset.tab === n", arg=name, timeout=3000)


def _in_tab(page, selector):
    return page.evaluate(
        "s => { const e = document.querySelector(s); const p = e && e.closest('.tab-page');"
        " return p ? p.dataset.tab : null; }", selector)


# ---------- criterion 1: the tabs ----------

def test_four_tabs_in_order_each_owning_its_panels(browser, twin_server):  # noqa: F811
    page, errors = open_twin(browser, twin_server, engineer=False, tab=None)
    tabs = page.evaluate("() => Array.from(document.querySelectorAll('.tab-bar .tab-btn'))"
                         ".map(b => [b.dataset.tab, b.getAttribute('role'), b.textContent.trim()])")
    assert tabs == [["home", "tab", "Home"], ["drive", "tab", "Drive"],
                    ["record", "tab", "Record"], ["settings", "tab", "Settings"]]
    owners = {
        "home": ["#brain-job", "#brain-target", "#btn-brain-mission", "#mission-card",
                 "#brain-telemetry", "#brain-connect-hint"],
        "drive": ["#fpv-canvas", "#depth-strip", "#world-map", "#btn-forward", "#btn-stop",
                  "#btn-look-left", "#tel-safety", "#log"],
        "record": ["#btn-mode-guide", "#btn-mode-robot", "#cfg-record-walk",
                   "#cfg-drive-via-brain", "#btn-guidance"],
        "settings": ["#cfg-server-url", "#cfg-brain-url", "#cfg-url", "#cfg-debug-readouts"],
    }
    for tab, selectors in owners.items():
        for sel in selectors:
            assert _in_tab(page, sel) == tab, f"{sel} is on {_in_tab(page, sel)}, not {tab}"
    # The old gear is the tab bar's Settings button, id kept.
    assert page.locator("header #btn-settings").count() == 0
    assert page.evaluate("() => document.body.dataset.tab") == "home"
    assert not errors, errors
    page.close()


@pytest.mark.parametrize("saved, opens", [
    (None, "home"), ("guide", "record"), ("sim", "home"), ("autonomous", "home"),
    ("drive", "drive"), ("record", "record"), ("settings", "settings"), ("bogus", "home"),
])
def test_a_saved_tab_opens_where_the_scope_says(browser, twin_server, saved, opens):  # noqa: F811
    context = browser.new_context(viewport={"width": 390, "height": 844}, legacy_tab=None)
    if saved is not None:
        context.add_init_script(
            "(() => { try { localStorage.setItem('vp_active_tab', %s); } catch (e) {} })();" % _json(saved))
    page = context.new_page()
    page.goto(twin_server, wait_until="load")
    page.wait_for_function("() => !!document.body.dataset.tab")
    assert page.evaluate("() => document.body.dataset.tab") == opens
    active = page.evaluate("() => document.querySelector('.tab-page.active').dataset.tab")
    assert active == opens
    context.close()


# ---------- criterion 2: the STOP bar ----------

def _bar_visible(page):
    return page.locator("#stop-bar").is_visible()


def test_the_stop_bar_is_hidden_while_nothing_moves(browser, twin_server):  # noqa: F811
    page, errors = open_twin(browser, twin_server, engineer=False)
    status = {"body": tiered_status(running=False)}
    _connect_brain(page, twin_server, status)
    page.wait_for_timeout(1500)        # a few /health reads
    for tab in TABS:
        _tab(page, tab)
        assert not _bar_visible(page), f"STOP bar shown on {tab} with nothing moving"
    assert not errors, errors
    page.close()


def test_the_stop_bar_shows_on_every_tab_and_stops_robot_then_mission(browser, private_twin_server):  # noqa: F811
    page, errors = open_twin(browser, private_twin_server, engineer=False)
    status = {"body": tiered_status(running=True)}
    sent = []
    page.on("request", lambda r: sent.append((r.method, r.url.split("?")[0]))
            if r.method == "POST" and r.url.split("?")[0].rsplit("/", 1)[-1] in ("stop",) else None)

    def stop_mission(route):
        status["body"] = ended("cancelled-stop")
        route.fulfill(status=200, content_type="application/json",
                      body=_json({"stopped": True, "status": status["body"]}))
    page.route("**/brain-stub/mission/stop", stop_mission)
    _connect_brain(page, private_twin_server, status)
    _tab(page, "home")
    page.wait_for_function("() => !document.getElementById('stop-bar').hidden", timeout=5000)
    for tab in TABS:
        _tab(page, tab)
        assert _bar_visible(page), f"no STOP bar on {tab} while a mission runs"
    page.click("#btn-stop-bar")                       # one tap
    page.wait_for_function("() => document.getElementById('stop-bar').hidden", timeout=5000)
    assert [u for _, u in sent] == [private_twin_server + "/stop",
                                     private_twin_server + "/brain-stub/mission/stop"], sent
    assert not errors, errors
    page.close()


def test_with_no_robot_the_stop_bar_stops_the_mission(browser, twin_server):  # noqa: F811
    page, errors = open_twin(browser, twin_server, engineer=False)
    # No robot: every robot route refuses to connect.
    page.route(twin_server + "/health", lambda route: route.abort())
    page.route(twin_server + "/frame", lambda route: route.abort())
    status = {"body": tiered_status(running=True)}
    sent = []
    page.on("request", lambda r: sent.append(r.url.split("?")[0])
            if r.method == "POST" and r.url.split("?")[0].endswith("/stop") else None)
    page.route("**/brain-stub/mission/stop", lambda route: (
        status.update(body=ended("cancelled-stop")),
        route.fulfill(status=200, content_type="application/json",
                      body=_json({"stopped": True, "status": status["body"]}))))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", "http://127.0.0.1:9")      # nothing listens
    page.click("#btn-connect")
    page.wait_for_timeout(800)
    _connect_brain(page, twin_server, status)
    _tab(page, "drive")
    page.wait_for_function("() => !document.getElementById('stop-bar').hidden", timeout=5000)
    page.click("#btn-stop-bar")
    page.wait_for_function("() => document.getElementById('stop-bar').hidden", timeout=5000)
    assert sent == [twin_server + "/brain-stub/mission/stop"], sent
    assert not errors, errors
    page.close()


def test_the_stop_bar_shows_while_another_driver_holds_the_robot(browser, twin_server):  # noqa: F811
    page, errors = open_twin(browser, twin_server, engineer=False)
    holder = {"on": False}

    def health(route):
        resp = route.fetch()
        body = resp.json()
        if holder["on"]:
            body.update({"driver": "ros", "authority_holder": "ros"})
        route.fulfill(response=resp, body=_json(body))
    page.route(twin_server + "/health", health)
    page.wait_for_timeout(1200)
    _tab(page, "record")
    assert not _bar_visible(page)
    holder["on"] = True
    page.wait_for_function("() => !document.getElementById('stop-bar').hidden", timeout=5000)
    holder["on"] = False
    page.wait_for_function("() => document.getElementById('stop-bar').hidden", timeout=5000)
    assert not errors, errors
    page.close()


# ---------- criterion 3: the mission card and the header ----------

def test_the_mission_card_is_one_plain_sentence_in_every_state(browser, twin_server):  # noqa: F811
    page, errors = open_twin(browser, twin_server, engineer=False)
    status = {"body": tiered_status(running=False)}
    status["body"].update({"outcome": "idle", "end_state": None})
    _connect_brain(page, twin_server, status)
    _tab(page, "home")
    card = page.locator("#mission-card")
    page.wait_for_function("() => !document.getElementById('mission-card').hidden", timeout=5000)
    seen = {"idle": card.inner_text()}
    _start(page, status)
    page.wait_for_function("() => /searching/i.test(document.getElementById('mission-card').textContent)",
                           timeout=5000)
    seen["running"] = card.inner_text()
    assert "red backpack" in seen["running"]
    for case in ENDINGS:
        status["body"] = ended(case)
        page.wait_for_function("() => !/searching/i.test(document.getElementById('mission-card').textContent)",
                               timeout=5000)
        page.wait_for_timeout(300)
        seen[case] = card.inner_text()
        _start(page, status)
        page.wait_for_function("() => /searching/i.test(document.getElementById('mission-card').textContent)",
                               timeout=5000)
    for state, text in seen.items():
        assert text.strip(), f"{state}: empty card"
        assert text.count(". ") == 0, f"{state}: more than one sentence: {text!r}"
        assert not jargon_hits(text), f"{state}: jargon {jargon_hits(text)} in {text!r}"
    # The telemetry rows are Engineer mode's; with it off only the card says it.
    assert not page.locator("#brain-tel-outcome").is_visible()
    assert not errors, errors
    page.close()


@pytest.mark.parametrize("engineer", [False, True])
def test_the_header_names_the_robots_real_mode(browser, twin_server, engineer):  # noqa: F811
    page, errors = open_twin(browser, twin_server, engineer=engineer)
    mode = {"value": "sim"}

    def health(route):
        resp = route.fetch()
        body = resp.json()
        body["mode"] = mode["value"]
        route.fulfill(response=resp, body=_json(body))
    page.route(twin_server + "/health", health)
    eyebrow = page.locator("#header-mode")
    want = {"sim": ("Simulator", "sim mode"), "teleop": ("Phone camera", "teleop mode"),
            "hardware": ("Rover", "hardware mode")}
    for value, words in want.items():
        mode["value"] = value
        word = words[1] if engineer else words[0]
        page.wait_for_function("w => document.getElementById('header-mode').textContent.endsWith(w)",
                               arg=word, timeout=5000)
        for tab in TABS:
            _tab(page, tab)
            assert eyebrow.is_visible(), f"no mode on {tab}"
    page.unroute(twin_server + "/health")
    page.click("#btn-settings")
    page.fill("#cfg-server-url", "http://127.0.0.1:9")
    page.click("#btn-connect")
    none = "not connected" if engineer else "Not connected"
    page.wait_for_function("w => document.getElementById('header-mode').textContent.endsWith(w)",
                           arg=none, timeout=8000)
    if not engineer:
        assert not jargon_hits(eyebrow.inner_text())
    assert not errors, errors
    page.close()


# ---------- criterion 4: idle is neutral ----------

def test_an_idle_robot_is_neutral_and_a_fired_watchdog_in_a_mission_is_amber(browser, twin_server):  # noqa: F811
    page, errors = open_twin(browser, twin_server, engineer=True)
    fired = {"on": True}

    def health(route):
        resp = route.fetch()
        body = resp.json()
        if fired["on"]:
            body["seconds_since_last_command"] = body["watchdog_timeout_s"] + 4.0
        route.fulfill(response=resp, body=_json(body))
    page.route(twin_server + "/health", health)
    status = {"body": tiered_status(running=False)}
    status["body"].update({"outcome": "idle", "end_state": None})
    _connect_brain(page, twin_server, status)
    _tab(page, "home")
    wd = page.locator("#brain-tel-watchdog")
    page.wait_for_function("() => /stopped/.test(document.getElementById('brain-tel-watchdog').textContent)",
                           timeout=5000)
    cls = wd.get_attribute("class")
    assert "alert" not in cls and "warn" not in cls, f"idle drawn as {cls!r}: {wd.inner_text()!r}"
    _start(page, status)
    page.wait_for_function("() => / warn\\b/.test(document.getElementById('brain-tel-watchdog').className)",
                           timeout=5000)
    assert "alert" not in wd.get_attribute("class")
    assert not errors, errors
    page.close()


# ---------- review fixes ----------

def test_a_hung_robot_stop_does_not_hold_up_the_missions_stop(browser, twin_server):  # noqa: F811
    """Both reviewers: the mission's stop waited behind the robot's /stop
    (and the frame read after it), with no timeout. Now both go out at once
    and the button comes back."""
    page, errors = open_twin(browser, twin_server, engineer=False)
    status = {"body": tiered_status(running=False)}
    sent = []
    page.route(twin_server + "/stop", lambda route: sent.append("robot"))      # never answered
    page.route("**/brain-stub/mission/stop", lambda route: (
        sent.append("mission"),
        status.update(body=ended("cancelled-stop")),
        route.fulfill(status=200, content_type="application/json",
                      body=_json({"stopped": True, "status": status["body"]}))))
    _connect_brain(page, twin_server, status)
    _start(page, status)
    page.wait_for_function("() => !document.getElementById('stop-bar').hidden", timeout=5000)
    page.click("#btn-stop-bar")
    page.wait_for_function("() => document.getElementById('stop-bar').hidden", timeout=3000)
    assert sorted(sent) == ["mission", "robot"], sent
    page.wait_for_function("() => !document.getElementById('btn-stop-bar').disabled", timeout=6000)
    assert not errors, errors
    page.context.close()


def test_on_record_no_robot_is_not_said_beside_a_ready_vision_service(browser, twin_server):  # noqa: F811
    page, errors = open_twin(browser, twin_server, engineer=False)
    page.click("#btn-settings")
    page.fill("#cfg-server-url", "http://127.0.0.1:9")
    page.click("#btn-connect")
    page.wait_for_function(
        "() => document.getElementById('header-mode').textContent.endsWith('Not connected')", timeout=8000)
    _tab(page, "record")
    assert page.locator("#header-mode").inner_text().strip().lower() == "digital twin"
    assert not errors, errors
    page.context.close()
