# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.219: the twin's D-pad drives while a button is held, and stops when the
hold ends (docs/plans/ros-alignment/3.219-dpad-hold.md, bars 1-5).

A real Chromium at a 390 px phone viewport drives a live robot server (the
simulator). "Wheels zero" is the robot's own `GET /wheels`, polled every
20 ms from here, so each bound is measured where the motors are, not in the
page. Each stop test was seen to fail with its stop removed from app.js.

`UI_SHOTS=<dir>` keeps the phone-size screenshot of a hold.
"""

import json
import math
import os
import time

import httpx
import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402,F401

from tests.test_ui import PHONE, browser, open_twin, private_twin_server  # noqa: F401,E402

STOP_BOUND_S = 0.30          # bar 2: release, cancel, slide off, hidden, closed; bar 4
WATCHDOG_BOUND_S = 1.5       # bar 2: the network dropped
HOLD_S = 2.0


def _wheels(url):
    w = httpx.get(url + "/wheels", timeout=2).json()
    return w["left"]["velocity_rad_s"], w["right"]["velocity_rad_s"]


def _moving(url):
    left, right = _wheels(url)
    return left != 0 or right != 0


def _until_zero(url, t0, limit=3.0, page=None):
    """Seconds from `t0` until the robot reports its wheels at zero. With
    `page`, it waits inside Playwright, so the page's route handlers run (the
    sync API serves them only during its own calls)."""
    while time.monotonic() - t0 < limit:
        if not _moving(url):
            return time.monotonic() - t0
        if page is not None:
            page.wait_for_timeout(20)
        else:
            time.sleep(0.02)
    return math.inf


def _until_moving(url, limit=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < limit:
        if _moving(url):
            return True
        time.sleep(0.02)
    return False


def _odometry(url):
    return httpx.get(url + "/odometry", timeout=2).json()


# The sync API delivers page events to Python only between its own calls, so
# a request's time is taken in the page, where it is sent.
_RECORD = """(() => {
    window.__posts = [];
    const real = window.fetch;
    window.fetch = function (url, opts) {
        const path = String(url).split('?')[0];
        if (opts && opts.method === 'POST' && /\\/(wheels|action|stop)$/.test(path)) {
            const h = opts.headers || {};
            window.__posts.push({route: path.slice(path.lastIndexOf('/')), t: performance.now(),
                                 body: JSON.parse(opts.body || '{}'), driver: h['x-driver'],
                                 keepalive: !!opts.keepalive});
        }
        return real.apply(this, arguments);
    };
})();"""


def _record(page):
    """Starts recording every POST the page sends to the motion routes;
    `_posts(page)` reads them (times in the page's ms)."""
    page.evaluate(_RECORD)


def _posts(page):
    return page.evaluate("() => window.__posts")



def _connect(page, url):
    page.click("#btn-settings")
    page.fill("#cfg-server-url", url)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="drive"]')
    page.wait_for_function("() => !document.getElementById('btn-forward').disabled",
                           timeout=8000)


def _open(browser, url, touch=False, scale=1):
    if not touch and scale == 1:
        page, errors = open_twin(browser, url)
    else:
        ctx = browser.new_context(viewport=PHONE, has_touch=touch, is_mobile=touch,
                                  device_scale_factor=scale, legacy_tab=None)
        ctx.add_init_script(
            "(() => { try { localStorage.setItem('vp_debug_readouts', '1');"
            " localStorage.setItem('vp_active_tab', 'drive'); } catch (e) {} })();")
        page = ctx.new_page()
        errors = []
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.goto(url, wait_until="load")
        page.wait_for_function("() => !!document.body.dataset.tab")
    _connect(page, url)
    return page, errors


def _centre(page, sel):
    box = page.locator(sel).bounding_box()
    return box["x"] + box["width"] / 2, box["y"] + box["height"] / 2


def _held(browser, url, sel="#btn-forward"):
    """A page connected to `url`, with `sel` held by the mouse and driving."""
    page, errors = _open(browser, url)
    _record(page)
    x, y = _centre(page, sel)
    page.mouse.move(x, y)
    page.mouse.down()
    assert _until_moving(url), "the hold never drove the wheels"
    return page, errors


def _drive_posts(seen, after=0.0):
    return [s for s in seen if s["route"] == "/wheels" and s["t"] > after
            and (s["body"].get("left_rad_s") or s["body"].get("right_rad_s"))]


# ---- bar 1: holding drives, through arbitration and safety; progress ----

def test_a_held_forward_drives_continuously_and_covers_ground(browser, private_twin_server):
    url = private_twin_server
    page, errors = _open(browser, url)
    _record(page)
    odo0 = _odometry(url)
    x, y = _centre(page, "#btn-forward")
    page.mouse.move(x, y)
    page.mouse.down()
    time.sleep(HOLD_S)
    page.mouse.up()
    _until_zero(url, time.monotonic())
    odo1 = _odometry(url)
    seen = _posts(page)
    posts = _drive_posts(seen)
    assert len(posts) >= 5, posts
    assert all(p["driver"] == "twin-dpad" for p in posts)
    gaps = [b["t"] - a["t"] for a, b in zip(posts, posts[1:])]
    dist = odo1["distance_m"] - odo0["distance_m"]
    print(f"\n[3.219 bar 1] posts={len(posts)} max_gap_ms={max(gaps):.0f} dist_m={dist:.3f}")
    assert max(gaps) <= 400, gaps
    assert dist >= 0.40, (odo0, odo1)
    assert not [s for s in seen if s["route"] == "/action"], "a hold sent a verb (bar 3)"
    log = page.locator("#log").inner_text()
    assert "FORWARD held" in log, log
    assert not errors, errors
    page.close()


def test_a_held_left_turns_the_robot(browser, private_twin_server):
    url = private_twin_server
    page, errors = _open(browser, url)
    th0 = _odometry(url)["heading_deg"]
    x, y = _centre(page, "#btn-left")
    page.mouse.move(x, y)
    page.mouse.down()
    time.sleep(HOLD_S)
    page.mouse.up()
    _until_zero(url, time.monotonic())
    turned = abs((_odometry(url)["heading_deg"] - th0 + 540) % 360 - 180)
    print(f"\n[3.219 bar 1] turned_deg={turned:.1f}")
    assert turned >= 60, turned
    assert not errors, errors
    page.close()


def test_every_hold_post_is_answered_by_the_vetting_route(browser, private_twin_server):
    url = private_twin_server
    page, errors = _open(browser, url)
    answers = []
    page.on("response", lambda r: answers.append(r.json()) if r.url.endswith("/wheels")
            and r.request.method == "POST" else None)
    x, y = _centre(page, "#btn-forward")
    page.mouse.move(x, y)
    page.mouse.down()
    time.sleep(1.0)
    page.mouse.up()
    page.wait_for_timeout(300)
    assert answers and all(a.get("executed") is True for a in answers), answers
    assert all(a.get("driver") == "twin-dpad" for a in answers), answers
    assert not errors, errors
    page.close()


def test_a_hold_into_a_wall_is_stopped_by_safety_not_the_page(browser, private_twin_server):
    url = private_twin_server
    page, errors = _open(browser, url)
    x, y = _centre(page, "#btn-forward")
    page.mouse.move(x, y)
    page.mouse.down()
    stopped_while_held = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < 15:
        time.sleep(0.1)
        if time.monotonic() - t0 > 0.6 and not _moving(url):
            stopped_while_held = True
            break
    page.mouse.up()
    refusal = (httpx.get(url + "/health", timeout=2).json().get("last_refusal") or {})
    print(f"\n[3.219 bar 1c] stopped_while_held={stopped_while_held} "
          f"after_s={time.monotonic() - t0:.1f} refusal={refusal.get('reason')}")
    assert stopped_while_held, "a 15 s hold never met a wall"
    assert refusal.get("reason") == "safety_distance", refusal
    assert not errors, errors
    page.close()


# ---- bar 2: every way a hold can end stops the wheels ----

def _bound(name, elapsed, limit, url=None):
    print(f"\n[3.219 bar 2/4] {name}: {elapsed * 1000:.0f} ms (bar {limit * 1000:.0f} ms)")
    assert elapsed <= limit, f"{name}: wheels still turning after {elapsed:.2f}s"
    if url:
        # Stopped, and stays stopped: a zero between two drive posts is not a stop.
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.6:
            assert not _moving(url), f"{name}: the wheels turned again after stopping"
            time.sleep(0.02)


def test_release_stops_the_wheels(browser, private_twin_server):
    page, errors = _held(browser, private_twin_server)
    time.sleep(0.5)
    t0 = time.monotonic()
    page.mouse.up()
    _bound("release", _until_zero(private_twin_server, t0), STOP_BOUND_S, private_twin_server)
    assert not errors, errors
    page.close()


def test_pointer_cancel_stops_the_wheels(browser, private_twin_server):
    page, errors = _held(browser, private_twin_server)
    time.sleep(0.5)
    t0 = time.monotonic()
    page.evaluate("""() => {
        const b = document.getElementById('btn-forward');
        b.dispatchEvent(new PointerEvent('pointercancel', {pointerId: 1, bubbles: true}));
    }""")
    _bound("pointercancel", _until_zero(private_twin_server, t0), STOP_BOUND_S, private_twin_server)
    page.mouse.up()
    page.close()


def test_sliding_off_the_button_stops_the_wheels(browser, private_twin_server):
    page, errors = _held(browser, private_twin_server)
    time.sleep(0.5)
    x, y = _centre(page, "#btn-forward")
    t0 = time.monotonic()
    page.mouse.move(x, y - 120)
    _bound("slide off (mouse)", _until_zero(private_twin_server, t0), STOP_BOUND_S, private_twin_server)
    page.mouse.up()
    assert not errors, errors
    page.close()


def test_hiding_the_page_stops_the_wheels(browser, private_twin_server):
    page, errors = _held(browser, private_twin_server)
    time.sleep(0.5)
    t0 = time.monotonic()
    page.evaluate("""() => {
        Object.defineProperty(document, 'visibilityState', {configurable: true, get: () => 'hidden'});
        document.dispatchEvent(new Event('visibilitychange'));
    }""")
    _bound("page hidden", _until_zero(private_twin_server, t0), STOP_BOUND_S, private_twin_server)
    # A phone locked or a tab closing: `visibilitychange` comes before
    # `pagehide`, so the zero that outlives the page must go here (review).
    zeros = [p for p in _posts(page) if p["route"] == "/wheels" and p["keepalive"]
             and not (p["body"].get("left_rad_s") or p["body"].get("right_rad_s"))]
    assert zeros, _posts(page)
    page.close()


def test_closing_the_tab_stops_the_wheels(browser, private_twin_server):
    page, errors = _held(browser, private_twin_server)
    time.sleep(0.5)
    t0 = time.monotonic()
    page.close(run_before_unload=True)
    _bound("tab closed", _until_zero(private_twin_server, t0), STOP_BOUND_S, private_twin_server)


def test_a_dropped_network_is_stopped_by_the_watchdog(browser, private_twin_server):
    page, errors = _held(browser, private_twin_server)
    time.sleep(0.5)
    t0 = time.monotonic()
    page.context.set_offline(True)
    _bound("network dropped", _until_zero(private_twin_server, t0), WATCHDOG_BOUND_S, private_twin_server)
    page.mouse.up()
    page.context.set_offline(False)
    page.close()


# ---- bar 3: a tap keeps today's behaviour ----

def test_a_tap_still_sends_one_verb_and_no_hold(browser, private_twin_server):
    page, errors = _open(browser, private_twin_server)
    _record(page)
    page.click("#btn-left")
    page.wait_for_timeout(1200)
    seen = _posts(page)
    verbs = [s for s in seen if s["route"] == "/action"]
    assert len(verbs) == 1, seen
    assert verbs[0]["body"]["action"] == "LEFT" and verbs[0]["body"].get("angle"), verbs
    assert verbs[0]["driver"] == "twin-dpad"
    assert not [s for s in seen if s["route"] == "/wheels"], seen
    assert not errors, errors
    page.close()


def test_the_click_after_a_hold_sends_no_verb(browser, private_twin_server):
    page, errors = _held(browser, private_twin_server)
    time.sleep(0.5)
    page.mouse.up()
    page.wait_for_timeout(800)
    seen = _posts(page)
    assert not [s for s in seen if s["route"] == "/action"], seen
    # ...and the next tap is a tap again.
    page.click("#btn-forward")
    page.wait_for_timeout(800)
    seen = _posts(page)
    assert len([s for s in seen if s["route"] == "/action"]) == 1, seen
    assert not errors, errors
    page.close()


# ---- bar 4: STOP wins at once ----

def test_stop_during_a_hold_stops_at_once_and_nothing_drives_after(browser, private_twin_server):
    page, errors = _held(browser, private_twin_server)
    time.sleep(0.6)
    t0 = time.monotonic()
    # A second finger on STOP: the hold's own pointer never lifts.
    t_stop = page.evaluate(
        "() => { const t = performance.now(); document.getElementById('btn-stop').click(); return t; }")
    elapsed = _until_zero(private_twin_server, t0)
    time.sleep(0.6)                # two refresh periods: none may follow
    _bound("STOP", elapsed, STOP_BOUND_S)
    seen = _posts(page)
    assert _drive_posts(seen), seen
    assert not _drive_posts(seen, after=t_stop), _drive_posts(seen, after=t_stop)
    assert not _moving(private_twin_server)
    page.mouse.up()
    assert not errors, errors
    page.close()


# ---- bar 5: a phone ----

def _touch(cdp, kind, points):
    cdp.send("Input.dispatchTouchEvent", {"type": kind, "touchPoints": points})


def test_a_touch_hold_drives_and_lifting_the_finger_stops(browser, private_twin_server):
    url = private_twin_server
    page, errors = _open(browser, url, touch=True)
    cdp = page.context.new_cdp_session(page)
    x, y = _centre(page, "#btn-forward")
    _touch(cdp, "touchStart", [{"x": x, "y": y, "id": 1}])
    assert _until_moving(url), "a touch hold did not drive"
    time.sleep(0.4)
    t0 = time.monotonic()
    _touch(cdp, "touchEnd", [])
    _bound("touch end", _until_zero(url, t0), STOP_BOUND_S, url)
    assert not errors, errors
    page.close()


def test_a_finger_sliding_off_stops_despite_implicit_capture(browser, private_twin_server):
    url = private_twin_server
    page, errors = _open(browser, url, touch=True)
    cdp = page.context.new_cdp_session(page)
    x, y = _centre(page, "#btn-forward")
    _touch(cdp, "touchStart", [{"x": x, "y": y, "id": 1}])
    assert _until_moving(url), "a touch hold did not drive"
    time.sleep(0.4)
    t0 = time.monotonic()
    _touch(cdp, "touchMove", [{"x": x, "y": y - 150, "id": 1}])
    _bound("slide off (touch)", _until_zero(url, t0), STOP_BOUND_S, url)
    _touch(cdp, "touchEnd", [])
    assert not errors, errors
    page.close()


def test_the_dpad_is_built_for_a_long_press_on_a_phone(browser, private_twin_server):
    page, errors = _open(browser, private_twin_server, touch=True)
    for sel in ("#btn-forward", "#btn-reverse", "#btn-left", "#btn-right"):
        css = page.eval_on_selector(sel, """b => { const s = getComputedStyle(b);
            return [s.touchAction, s.userSelect || s.webkitUserSelect,
                    s.webkitTouchCallout || ''] }""")
        assert css[0] == "none" and css[1] == "none", (sel, css)
        prevented = page.eval_on_selector(sel, """b => !b.dispatchEvent(
            new MouseEvent('contextmenu', {bubbles: true, cancelable: true}))""")
        assert prevented, f"{sel}: a long press would open a context menu"
    box = page.locator("#btn-forward").bounding_box()
    assert box["x"] >= 0 and box["x"] + box["width"] <= PHONE["width"], box
    assert not errors, errors
    page.close()


@pytest.mark.skipif(not os.environ.get("UI_SHOTS"), reason="UI_SHOTS=<dir> keeps the screenshot")
def test_screenshot_of_a_hold_on_a_phone(browser, private_twin_server):
    url = private_twin_server
    page, errors = _open(browser, url, touch=True, scale=2)
    cdp = page.context.new_cdp_session(page)
    page.eval_on_selector(".dpad", "d => d.scrollIntoView({block: 'center'})")
    page.wait_for_timeout(4000)        # the connect toast fades
    x, y = _centre(page, "#btn-forward")
    _touch(cdp, "touchStart", [{"x": x, "y": y, "id": 1}])
    assert _until_moving(url)
    time.sleep(1.0)
    assert "holding" in page.get_attribute("#btn-forward", "class")
    os.makedirs(os.environ["UI_SHOTS"], exist_ok=True)
    page.screenshot(path=os.path.join(os.environ["UI_SHOTS"], "dpad-hold-390.png"))
    _touch(cdp, "touchEnd", [])
    assert _until_zero(url, time.monotonic()) <= STOP_BOUND_S
    page.close()


# ---- review (code-review, 2026-11-26) ----

def test_lifting_long_after_stop_sends_no_verb(browser, private_twin_server):
    """The finger left on FORWARD after STOP ended its hold lifts 2 s later:
    no step may follow the STOP (a 1 s click window used to let one)."""
    page, errors = _held(browser, private_twin_server)
    time.sleep(0.5)
    page.evaluate("() => document.getElementById('btn-stop').click()")
    time.sleep(2.0)
    page.mouse.up()
    page.wait_for_timeout(800)
    seen = _posts(page)
    assert not [s for s in seen if s["route"] == "/action"], seen
    assert not _moving(private_twin_server)
    assert not errors, errors
    page.close()


def test_a_hold_refused_by_drive_ros_never_steps_and_later_presses_are_taps(
        browser, private_twin_server):
    """Under `drive: ros` the server refuses a person's /wheels
    (`not_the_actuator`): the hold ends, its release sends no verb, and the
    next long press is a tap (one verb) rather than another refused hold."""
    page, errors = _open(browser, private_twin_server)
    page.route("**/wheels", lambda r: r.fulfill(
        status=200, content_type="application/json",
        body=json.dumps({"executed": False, "reason": "not_the_actuator",
                         "detail": "drive: ros -- only 'ros' writes the wheels"}))
        if r.request.method == "POST" else r.continue_())
    _record(page)
    x, y = _centre(page, "#btn-forward")
    page.mouse.move(x, y)
    page.mouse.down()
    page.wait_for_timeout(1500)
    page.mouse.up()
    page.wait_for_timeout(500)
    seen = _posts(page)
    assert [s for s in seen if s["route"] == "/wheels"], seen
    assert not [s for s in seen if s["route"] == "/action"], seen
    page.mouse.down()
    page.wait_for_timeout(800)
    page.mouse.up()
    page.wait_for_timeout(800)
    before = len([s for s in seen if s["route"] == "/wheels"])
    seen = _posts(page)
    assert len([s for s in seen if s["route"] == "/action"]) == 1, seen
    assert len([s for s in seen if s["route"] == "/wheels"]) == before, seen
    assert not errors, errors
    page.close()


def test_a_rejected_hold_post_still_zeroes_the_wheels(browser, private_twin_server):
    """A post answered with an HTTP error: the server is there, so the page
    sends the zero rather than leaving the last command to the watchdog."""
    url = private_twin_server
    page, errors = _held(browser, url)
    time.sleep(0.4)
    state = {"n": 0}

    def fail_drive(route):
        body = json.loads(route.request.post_data or "{}")
        if body.get("left_rad_s") or body.get("right_rad_s"):
            route.fulfill(status=500, content_type="application/json",
                          body=json.dumps({"detail": "boom"}))
            state["n"] += 1
        else:
            route.continue_()
    t0 = time.monotonic()
    page.route("**/wheels", lambda r: fail_drive(r) if r.request.method == "POST"
               else r.continue_())
    elapsed = _until_zero(url, t0, page=page)
    print(f"\n[3.219 review] rejected post -> zero: {elapsed * 1000:.0f} ms after the route")
    assert state["n"] >= 1
    assert elapsed <= 0.25 + STOP_BOUND_S, elapsed  # the next refresh, then its zero
    page.mouse.up()
    page.close()
