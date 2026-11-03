# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.132 Stage 3B: the twin's behaviours (criteria 5-8,
docs/plans/ros-alignment/3.132-ux-redesign.md).

* 5: the first-run card -- on Home for a device with no robot saved and no
  setup link; Connect connects the robot and offers the autopilot at an
  address guessed from the robot's; Skip, or a connected autopilot, gives
  the ordinary Home; a saved robot or a setup link never shows it.
* 6 (F1): an idle twin notices a mission started through the brain's API.
* 7 (F2): a Start 0.3 s after this page's D-pad waits out the D-pad's hold
  and starts a mission that is still running 3 s on. Against the live
  robot server and brain, not stubs.
* 8: operator mode's fullscreen view shows no code line; Engineer mode
  does. (`connstatus.js`'s registry is pinned in tests/test_twin_modules.py.)

Each was written before the change it pins and seen to fail on the
Stage 3A page.
"""

import contextlib
import os
import re
import subprocess
import sys
import time

import httpx
import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402

from tests.conftest import REPO_ROOT  # noqa: E402
from tests.test_ui import (PHONE, TIERED_BRAIN_HEALTH, _json, browser,  # noqa: F401,E402
                           free_port, open_twin, tiered_status, twin_server)
from tests.test_ui_operator_words import jargon_hits  # noqa: E402

START_TIMEOUT_S = 60


def _stub_brain(page, running=False):
    page.route("**/brain-stub/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(TIERED_BRAIN_HEALTH)))
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(tiered_status(running=running))))


# ---------- criterion 5: the first run ----------

def _elsewhere(browser, twin_server, path=""):
    """A first visit to a page served away from the robot (as CloudFront
    serves it): the page is opened as `localhost`, whose own silent connect
    at load is refused, and the robot is `127.0.0.1` -- one server, two
    origins."""
    port = twin_server.rsplit(":", 1)[1]
    origin = f"http://localhost:{port}"
    context = browser.new_context(viewport=PHONE, legacy_tab=None, first_run=True)
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.route(f"{origin}/frame", lambda route: route.abort())
    page.goto(origin + path, wait_until="load")
    page.wait_for_function("() => !!document.body.dataset.tab")
    return context, page, errors


CONNECTED = "() => document.getElementById('connection-status').classList.contains('is-ok')"


def test_a_first_visit_opens_on_the_robot_step_in_homes_place(browser, twin_server):  # noqa: F811
    context, page, errors = _elsewhere(browser, twin_server)
    sync_api.expect(page.locator("#first-run-robot")).to_be_visible(timeout=5000)
    assert page.evaluate("() => document.body.dataset.tab") == "home"
    sync_api.expect(page.locator("#first-run-brain")).to_be_hidden()
    sync_api.expect(page.locator("#btn-brain-mission")).to_be_hidden()
    assert jargon_hits(page.locator("#first-run").inner_text()) == []
    assert not errors, errors
    context.close()


def test_connect_takes_one_field_and_one_tap_and_offers_the_guessed_autopilot(browser, twin_server):  # noqa: F811
    context, page, errors = _elsewhere(browser, twin_server)
    robot = twin_server.replace("localhost", "127.0.0.1")
    page.fill("#wiz-robot-url", robot)              # the one field
    page.click("#btn-wiz-connect")                  # the one tap
    page.wait_for_function(CONNECTED, timeout=10000)
    sync_api.expect(page.locator("#first-run-brain")).to_be_visible(timeout=5000)
    assert page.input_value("#wiz-brain-url") == "http://127.0.0.1:8001"
    # The same fields and the same saved settings as the Settings tab.
    assert page.input_value("#cfg-server-url") == robot
    assert page.evaluate("() => localStorage.getItem('vp_server_url')") == robot
    assert not errors, errors
    context.close()


def test_a_tunnelled_robots_autopilot_is_guessed_under_brain(browser, twin_server):  # noqa: F811
    context, page, errors = _elsewhere(browser, twin_server)
    tunnel = "https://rover.example.ngrok.app"
    # The tunnel's domain, answered by the test's robot server.
    page.route(tunnel + "/**", lambda route: route.fulfill(
        response=route.fetch(url=route.request.url.replace(tunnel, twin_server))))
    page.fill("#wiz-robot-url", tunnel)
    page.click("#btn-wiz-connect")
    sync_api.expect(page.locator("#first-run-brain")).to_be_visible(timeout=10000)
    assert page.input_value("#wiz-brain-url") == tunnel + "/brain"
    page.unroute_all(behavior="ignoreErrors")      # polls still in flight through the route
    context.close()


def test_a_wrong_robot_address_says_so_and_stays(browser, twin_server):  # noqa: F811
    context, page, errors = _elsewhere(browser, twin_server)
    page.fill("#wiz-robot-url", f"http://127.0.0.1:{free_port()}")
    page.click("#btn-wiz-connect")
    sync_api.expect(page.locator("#wiz-robot-status")).to_contain_text("Couldn't reach the robot",
                                                                      timeout=10000)
    sync_api.expect(page.locator("#first-run-robot")).to_be_visible()
    assert page.evaluate("() => localStorage.getItem('vp_server_url')") is None
    context.close()


def _first_visit(browser, url):
    """A first visit to a page the robot serves: it connects itself at load,
    so the card opens at the autopilot."""
    context = browser.new_context(viewport=PHONE, legacy_tab=None, first_run=True)
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(url, wait_until="load")
    return context, page, errors


def test_skip_gives_the_ordinary_home(browser, twin_server):  # noqa: F811
    context, page, errors = _first_visit(browser, twin_server)
    sync_api.expect(page.locator("#first-run-brain")).to_be_visible(timeout=10000)
    page.click("#btn-wiz-skip")
    sync_api.expect(page.locator("#first-run")).to_be_hidden()
    sync_api.expect(page.locator("#btn-brain-mission")).to_be_visible()
    assert not errors, errors
    context.close()


def test_a_connected_autopilot_gives_the_ordinary_home(browser, twin_server):  # noqa: F811
    context, page, errors = _first_visit(browser, twin_server)
    _stub_brain(page)
    sync_api.expect(page.locator("#first-run-brain")).to_be_visible(timeout=10000)
    page.fill("#wiz-brain-url", twin_server + "/brain-stub")
    page.click("#btn-wiz-brain")
    sync_api.expect(page.locator("#first-run")).to_be_hidden(timeout=5000)
    sync_api.expect(page.locator("#btn-brain-mission")).to_be_enabled()
    assert page.evaluate("() => localStorage.getItem('vp_brain_url')") == twin_server + "/brain-stub"
    assert not errors, errors
    context.close()


@pytest.mark.parametrize("how", ["saved robot", "setup link"])
def test_a_saved_robot_or_a_setup_link_never_shows_it(browser, twin_server, how):  # noqa: F811
    if how == "saved robot":
        context = browser.new_context(viewport=PHONE, legacy_tab=None)   # a robot saved
        page = context.new_page()
        page.goto(twin_server, wait_until="load")
    else:
        context = browser.new_context(viewport=PHONE, legacy_tab=None, first_run=True)
        page = context.new_page()
        page.goto(f"{twin_server}/?serverUrl={twin_server}", wait_until="load")
    page.wait_for_function(CONNECTED, timeout=10000)
    page.wait_for_timeout(500)
    assert not page.locator("#first-run").is_visible()
    sync_api.expect(page.locator("#btn-brain-mission")).to_be_visible()
    context.close()


# ---------- the live stack: criteria 6 and 7 ----------

@contextlib.contextmanager
def _live(module, port, env, cwd):
    proc = subprocess.Popen(
        [sys.executable, "-m", "uvicorn", module, "--host", "127.0.0.1", "--port", str(port),
         "--log-level", "warning"], cwd=cwd, env=env)
    url = f"http://127.0.0.1:{port}"
    try:
        deadline = time.monotonic() + START_TIMEOUT_S
        while True:
            if proc.poll() is not None:
                pytest.fail(f"{module} exited early with code {proc.returncode}")
            with contextlib.suppress(httpx.HTTPError):
                if httpx.get(url + "/health", timeout=1).status_code == 200:
                    break
            if time.monotonic() > deadline:
                pytest.fail(f"{module} was not healthy within {START_TIMEOUT_S}s")
            time.sleep(0.1)
        yield url
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


@pytest.fixture
def live_stack(tmp_path):
    """A robot server and a brain of this test's own (sim, free policy),
    with no paid service reachable."""
    robot_port, brain_port = free_port(), free_port()
    env = {**os.environ, "PYTHONPATH": str(REPO_ROOT),
           "ROBOT_URL": f"http://127.0.0.1:{robot_port}",
           "VISION_URL": "http://127.0.0.1:9",
           "INTERVENTION_DIR": str(tmp_path / "interventions")}
    env.pop("APP_SHARED_SECRET", None)
    with _live("robot.server:app", robot_port, env, tmp_path) as robot:
        with _live("control.brain_server:app", brain_port, env, tmp_path) as brain:
            yield robot, brain
            with contextlib.suppress(httpx.HTTPError):
                httpx.post(brain + "/mission/stop", timeout=10)


def _open_live(browser, robot, brain):  # noqa: F811
    page, errors = open_twin(browser, robot, engineer=False, tab="home")
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", brain)
    page.click("#btn-brain-connect")
    page.click('.tab-btn[data-tab="home"]')
    page.wait_for_selector("#btn-brain-mission:not([disabled])", timeout=10000)
    return page, errors


def test_f1_an_idle_twin_notices_a_mission_started_through_the_api(browser, live_stack):  # noqa: F811
    robot, brain = live_stack
    page, errors = _open_live(browser, robot, brain)
    btn = page.locator("#btn-brain-mission")
    sync_api.expect(btn).to_have_text("Start")
    r = httpx.post(brain + "/mission/start",
                   json={"target_object": "red backpack", "policy": "frontier", "fault": "none"},
                   timeout=30)
    assert r.status_code == 200, r.text
    started = time.monotonic()
    sync_api.expect(btn).to_have_text("Stop", timeout=6000)
    assert time.monotonic() - started <= 6.0
    sync_api.expect(page.locator("#mission-card")).to_contain_text("Searching for the red backpack")
    assert not errors, errors
    page.context.close()


def test_f2_a_start_right_after_the_dpad_waits_and_the_mission_runs(browser, live_stack):  # noqa: F811
    robot, brain = live_stack
    page, errors = _open_live(browser, robot, brain)
    page.fill("#brain-target", "red backpack")
    page.click('.tab-btn[data-tab="drive"]')
    page.wait_for_selector("#btn-forward:not([disabled])", timeout=5000)
    # FORWARD, then Home's Start 0.3 s after the press, in the page's own
    # time (two clicks from Playwright would take longer than that).
    said = page.evaluate("""async () => {
        document.getElementById('btn-forward').click();
        await new Promise(r => setTimeout(r, 300));
        document.querySelector('.tab-btn[data-tab="home"]').click();
        const btn = document.getElementById('btn-brain-mission');
        btn.click();
        await new Promise(r => setTimeout(r, 150));
        return btn.textContent;
    }""")
    assert "Waiting" in said, f"the button did not say it was waiting: {said!r}"
    sync_api.expect(page.locator("#btn-brain-mission")).to_have_text("Stop", timeout=10000)
    page.wait_for_timeout(3000)
    status = httpx.get(brain + "/mission/status", timeout=10).json()
    assert status["running"] is True, (status.get("outcome"), status.get("cancelled_by"),
                                       status.get("log_tail"))
    assert not errors, errors
    page.context.close()


def test_f2_a_stop_while_start_waits_cancels_the_start(browser, live_stack):  # noqa: F811
    robot, brain = live_stack
    page, errors = _open_live(browser, robot, brain)
    page.click('.tab-btn[data-tab="drive"]')
    page.wait_for_selector("#btn-forward:not([disabled])", timeout=5000)
    page.evaluate("""async () => {
        document.getElementById('btn-forward').click();
        await new Promise(r => setTimeout(r, 200));
        document.querySelector('.tab-btn[data-tab="home"]').click();
        document.getElementById('btn-brain-mission').click();
        await new Promise(r => setTimeout(r, 200));
        document.getElementById('btn-stop-bar').click();
    }""")
    page.wait_for_timeout(3000)
    status = httpx.get(brain + "/mission/status", timeout=10).json()
    assert not status["running"] and status.get("outcome") == "idle", status.get("outcome")
    sync_api.expect(page.locator("#btn-brain-mission")).to_have_text("Start")
    assert not errors, errors
    page.context.close()


# ---------- criterion 8: the Guide HUD's codes ----------

HUD_CODES = ["TGT", "ACT", "VIS", "OBS", "SEQ", "OUT", "VF", "CLD", "RMS", "FPS", "STA", "FORWARD",
             "pan:"]


@pytest.mark.parametrize("engineer", [False, True])
def test_the_fullscreen_view_shows_codes_only_in_engineer_mode(browser, twin_server, engineer):  # noqa: F811
    context = browser.new_context(viewport=PHONE)
    context.add_init_script(
        "(() => { try {"
        " localStorage.setItem('guidanceMode','robot');"
        " localStorage.setItem('vp_debug_readouts', %s);"
        " localStorage.setItem('vp_guide_onboarded','1');"
        " localStorage.setItem('vp_drive_via_brain','1');"
        " localStorage.setItem('vp_drive_policy','tiered');"
        " } catch (e) {} })();" % _json("1" if engineer else "0"))
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.route("**/brain-stub/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(TIERED_BRAIN_HEALTH)))
    page.route("**/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({
            "status": "ok", "mode": "teleop", "seconds_since_last_command": 0.1,
            "watchdog_timeout_s": 1.0})))
    page.route("**/teleop/frame", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({"ok": True})))
    page.route("**/brain-stub/mission/start", lambda route: route.fulfill(
        status=200, content_type="application/json",
        body=_json({"started": True, "status": {"running": True}})))
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(tiered_status(running=True))))
    page.goto(twin_server, wait_until="load")
    page.wait_for_timeout(500)
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(600)
    page.click('.tab-btn[data-tab="record"]')
    page.fill("#guidance-target", "red backpack")
    page.click("#btn-guidance")
    sync_api.expect(page.locator("#robot-action-badge")).to_be_visible(timeout=15000)
    page.wait_for_timeout(1500)
    shown = page.evaluate("() => document.getElementById('guide-fullscreen').innerText")
    if engineer:
        for code in ("TGT", "VF", "FORWARD"):
            assert code in shown, f"Engineer mode lost {code}: {shown!r}"
    else:
        found = [c for c in HUD_CODES if re.search(r"(?<![\w])" + re.escape(c) + r"(?![\w])", shown)]
        assert found == [], f"codes in operator mode's fullscreen view: {found}\n{shown!r}"
        assert jargon_hits(shown) == [], jargon_hits(shown)
        assert "Forward" in shown, shown
    assert not errors, errors
    context.close()


# ---------- the reviews' findings, each seen to fail first ----------

def _drive_via_brain_page(browser, twin_server, engineer=False):
    context = browser.new_context(viewport=PHONE)
    context.add_init_script(
        "(() => { try {"
        " localStorage.setItem('guidanceMode','robot');"
        " localStorage.setItem('vp_debug_readouts', %s);"
        " localStorage.setItem('vp_guide_onboarded','1');"
        " localStorage.setItem('vp_drive_via_brain','1');"
        " localStorage.setItem('vp_drive_policy','tiered');"
        " } catch (e) {} })();" % _json("1" if engineer else "0"))
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.route("**/brain-stub/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(TIERED_BRAIN_HEALTH)))
    page.route("**/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({
            "status": "ok", "mode": "teleop", "seconds_since_last_command": 0.1,
            "watchdog_timeout_s": 1.0})))
    page.route("**/teleop/frame", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({"ok": True})))
    # Idle until the walk starts it: a mission already running when the
    # brain connects is picked up at connect, an older rule than F1's.
    walking = {"on": False}

    def start(route):
        walking["on"] = True
        route.fulfill(status=200, content_type="application/json",
                      body=_json({"started": True, "status": {"running": True}}))
    page.route("**/brain-stub/mission/start", start)
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json",
        body=_json(tiered_status(running=walking["on"]))))
    page.goto(twin_server, wait_until="load")
    page.wait_for_timeout(500)
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(600)
    page.click('.tab-btn[data-tab="record"]')
    page.fill("#guidance-target", "red backpack")
    page.click("#btn-guidance")
    return context, page, errors


def test_f1_leaves_this_pages_own_drive_via_brain_walk_to_its_loop(browser, twin_server):  # noqa: F811
    """Both reviewers: the idle ping adopted Record's own walk as a Home
    mission -- Home's poll started beside the walk's loop, Home's Start read
    Stop, and the walk's ending would be told twice."""
    context, page, errors = _drive_via_brain_page(browser, twin_server)
    sync_api.expect(page.locator("#robot-action-badge")).to_be_visible(timeout=15000)
    page.wait_for_timeout(6500)                     # more than one liveness period
    assert page.evaluate("() => document.getElementById('btn-brain-mission').textContent") == "Start"
    assert not errors, errors
    context.close()


def _held_robot(page, holder):
    """The robot's /health, saying `holder["now"]` holds it."""
    def health(route):
        route.fulfill(status=200, content_type="application/json", body=_json({
            "status": "ok", "mode": "sim", "seconds_since_last_command": 0.1,
            "watchdog_timeout_s": 1.0, "authority_holder": holder["now"]}))
    page.route("**/health", health)


def test_f2_a_start_waits_out_a_persons_map_goal_and_then_starts(browser, twin_server):  # noqa: F811
    """Both reviewers: a person's map goal holds the robot while it drives,
    and no D-pad press here counts it; the server's holder does."""
    page, errors = open_twin(browser, twin_server, engineer=False, tab="home")
    holder = {"now": "twin-dpad"}
    _held_robot(page, holder)
    _stub_brain(page)
    starts = []
    page.route("**/brain-stub/mission/start", lambda route: (starts.append(1), route.fulfill(
        status=200, content_type="application/json", body=_json({"started": True})))[1])
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.click('.tab-btn[data-tab="home"]')
    page.wait_for_selector("#btn-brain-mission:not([disabled])", timeout=5000)
    page.click("#btn-brain-mission")
    sync_api.expect(page.locator("#btn-brain-mission")).to_contain_text("Waiting", timeout=3000)
    page.wait_for_timeout(1200)
    assert starts == [], "started while a person's goal held the robot"
    holder["now"] = None
    # The start itself: the stub then reports an ended mission, so Stop
    # would show only for a poll.
    for _ in range(50):
        if starts:
            break
        page.wait_for_timeout(100)
    assert starts == [1]
    assert not errors, errors
    page.close()


def test_f2_a_hold_that_outlasts_the_wait_starts_nothing(browser, twin_server):  # noqa: F811
    """Code review: past the wait, Start went out anyway, into the same
    'D-pad took over'. Now it is given up, with a reason."""
    page, errors = open_twin(browser, twin_server, engineer=False, tab="home")
    _held_robot(page, {"now": "twin-dpad"})
    _stub_brain(page)
    starts = []
    page.route("**/brain-stub/mission/start", lambda route: (starts.append(1), route.fulfill(
        status=200, content_type="application/json", body=_json({"started": True})))[1])
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.click('.tab-btn[data-tab="home"]')
    page.wait_for_selector("#btn-brain-mission:not([disabled])", timeout=5000)
    page.click("#btn-brain-mission")
    sync_api.expect(page.locator(".toast").last).to_contain_text("still driving the robot", timeout=14000)
    assert starts == []
    sync_api.expect(page.locator("#btn-brain-mission")).to_have_text("Start")
    assert not errors, errors
    page.close()


def test_a_brain_that_connects_first_leaves_the_robot_step(browser, twin_server):  # noqa: F811
    """Code review: a saved autopilot connecting before any robot closed the
    card, leaving Home over no robot at all."""
    port = twin_server.rsplit(":", 1)[1]
    origin = f"http://localhost:{port}"
    context = browser.new_context(viewport=PHONE, legacy_tab=None, first_run=True)
    context.add_init_script(
        "(() => { try { localStorage.setItem('vp_brain_url', %s); } catch (e) {} })();"
        % _json(origin + "/brain-stub"))
    page = context.new_page()
    page.route(f"{origin}/frame", lambda route: route.abort())
    _stub_brain(page)
    page.goto(origin, wait_until="load")
    page.wait_for_function("() => document.getElementById('brain-connection-status')"
                           ".classList.contains('is-ok')", timeout=5000)
    page.wait_for_timeout(300)
    sync_api.expect(page.locator("#first-run-robot")).to_be_visible()
    context.close()


def test_a_robot_that_wants_its_code_keeps_the_address_and_asks(browser, twin_server):  # noqa: F811
    """Code review: a robot that refused the load's silent connect for want
    of its code left the card's address empty; the address was right."""
    port = twin_server.rsplit(":", 1)[1]
    origin = f"http://localhost:{port}"
    context = browser.new_context(viewport=PHONE, legacy_tab=None, first_run=True)
    page = context.new_page()
    page.route(f"{origin}/frame", lambda route: route.fulfill(
        status=401, content_type="application/json",
        body=_json({"detail": "Missing or invalid x-app-secret header."})))
    page.goto(origin, wait_until="load")
    sync_api.expect(page.locator("#wiz-robot-status")).to_contain_text("access code", timeout=5000)
    assert page.input_value("#wiz-robot-url") == origin
    context.close()


def test_an_autopilot_that_refuses_the_code_keeps_the_card(browser, twin_server):  # noqa: F811
    """Both reviewers: the card closed on the open /health, and a wrong code
    then failed every mission call quietly."""
    context = browser.new_context(viewport=PHONE, legacy_tab=None, first_run=True)
    page = context.new_page()
    page.route("**/brain-stub/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(TIERED_BRAIN_HEALTH)))
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=401, content_type="application/json",
        body=_json({"detail": "Missing or invalid x-app-secret header."})))
    page.goto(twin_server, wait_until="load")
    sync_api.expect(page.locator("#first-run-brain")).to_be_visible(timeout=10000)
    page.fill("#wiz-brain-url", twin_server + "/brain-stub")
    page.click("#btn-wiz-brain")
    sync_api.expect(page.locator("#wiz-brain-status")).to_contain_text("refused the access code",
                                                                      timeout=5000)
    sync_api.expect(page.locator("#first-run-brain")).to_be_visible()
    context.close()
