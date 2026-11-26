# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.220.B (amendment 1): the twin's camera-panned badge and its look
buttons' press-and-hold (docs/plans/ros-alignment/3.220-pan-tilt.md, bars
7-8).

A real Chromium at a 390 px phone viewport drives a live robot server (the
simulator). The camera's pan is the robot's own `GET /frame` `pan_deg`,
read from here, so each bound is measured at the robot, not in the page.

`UI_SHOTS=<dir>` keeps the phone-size screenshot of the badge.
"""

import os
import time

import httpx
import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402,F401

from tests.test_ui import browser, private_twin_server  # noqa: F401,E402
from tests.test_ui_dpad_hold import _centre, _open, _posts, _record  # noqa: E402

STOP_BOUND_S = 0.30          # bar 8: every way of ending the hold
STEP_MS = 150                # app.js LOOK_HOLD_STEP_MS


def _pan(url):
    return httpx.get(url + "/frame", timeout=5).json()["pan_deg"]


def _settle_after(url, t0, quiet_s=0.6, limit=4.0):
    """Seconds from `t0` to the camera's last pan change, judged once the pan
    has held still for `quiet_s`."""
    last, last_change = _pan(url), time.monotonic()
    while time.monotonic() - t0 < limit:
        time.sleep(0.03)
        now = _pan(url)
        if now != last:
            last, last_change = now, time.monotonic()
        elif time.monotonic() - last_change >= quiet_s:
            break
    return max(0.0, last_change - t0), last


# Every /frame the page receives, with the badge as it stands just after the
# page has rendered that frame (frames come >= 150 ms apart, so 40 ms later
# is still "this frame").
_FRAMES = """(() => {
    window.__frames = [];
    const real = window.fetch;
    window.fetch = function (url, opts) {
        const p = real.apply(this, arguments);
        const path = String(url).split('?')[0];
        if ((!opts || !opts.method || opts.method === 'GET') && /\\/frame$/.test(path)) {
            p.then(r => r.clone().json()).then(j => setTimeout(() => {
                const b = document.getElementById('pan-badge');
                window.__frames.push({pan: j.pan_deg, hidden: b.hidden, text: b.textContent});
            }, 40)).catch(() => {});
        }
        return p;
    };
})();"""


def _press(page, sel):
    x, y = _centre(page, sel)
    page.mouse.move(x, y)
    page.mouse.down()
    return x, y


def _until_turning(url, limit=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < limit:
        if abs(_pan(url)) >= 10:
            return True
        time.sleep(0.03)
    return False


# ---------- bar 7: the badge ----------

def test_the_badge_follows_the_measured_pan_within_one_frame(browser, private_twin_server):
    url = private_twin_server
    page, errors = _open(browser, url)
    page.evaluate(_FRAMES)
    assert page.locator("#pan-badge").is_hidden()
    page.click("#btn-look-left")
    page.wait_for_function("() => window.__frames.some(f => f.pan <= -89)", timeout=5000)
    page.wait_for_timeout(100)
    shots = os.environ.get("UI_SHOTS")
    if shots:
        page.screenshot(path=os.path.join(shots, "3220-pan-badge.png"), full_page=False)
    page.click("#btn-look-center")
    page.wait_for_function("() => window.__frames.some(f => f.pan === 0 && "
                           "window.__frames.some(g => g.pan <= -89))", timeout=5000)
    page.wait_for_timeout(100)
    frames = page.evaluate("() => window.__frames")
    seen = [(f["pan"], f["hidden"]) for f in frames]
    # Each frame's own render: shown exactly when |pan| >= 3, never a frame late.
    assert all(f["hidden"] == (f["pan"] is None or abs(f["pan"]) < 3) for f in frames), seen
    assert any(not f["hidden"] and "90" in f["text"] and "left" in f["text"] for f in frames)
    assert any(f["hidden"] for f in frames if f["pan"] == 0)
    # The D-pad never centres the camera: a drive leaves the pan as it was.
    page.click("#btn-look-right")
    page.wait_for_timeout(300)
    page.click("#btn-forward")
    page.wait_for_timeout(400)
    assert _pan(url) == pytest.approx(90)
    assert not errors
    page.close()


# ---------- bar 8: the hold ----------

def test_a_tap_stays_one_verb(browser, private_twin_server):
    url = private_twin_server
    page, _ = _open(browser, url)
    _record(page)
    page.click("#btn-look-right")
    page.wait_for_timeout(500)
    actions = [p["body"].get("action") for p in _posts(page) if p["route"] == "/action"]
    assert actions == ["LOOK_RIGHT"]
    page.close()


def test_a_held_look_pans_in_paced_steps_and_stops_at_the_limit(browser, private_twin_server):
    url = private_twin_server
    page, _ = _open(browser, url)
    _record(page)
    _press(page, "#btn-look-left")
    assert _until_turning(url)
    pans = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.5:
        pans.append(_pan(url))
        time.sleep(0.05)
    assert min(pans) >= -90 and pans[-1] == pytest.approx(-90)
    # Ended at the limit, though the button is still held.
    assert "holding" not in (page.get_attribute("#btn-look-left", "class") or "")
    page.mouse.up()
    page.wait_for_timeout(300)
    steps = [p for p in _posts(page) if p["body"].get("action") == "LOOK_STEP"]
    assert all(p["body"]["angle"] == -10 for p in steps)
    assert len(steps) == 9                      # 0 -> -90 in ten-degree steps
    gaps = [b["t"] - a["t"] for a, b in zip(steps, steps[1:])]
    assert min(gaps) >= STEP_MS - 5, gaps       # at most ~6.7 commands a second
    # The click after a hold sends no verb.
    assert not [p for p in _posts(page) if p["body"].get("action") == "LOOK_LEFT"]
    page.close()


def _held_then(browser, url, end):
    page, _ = _open(browser, url)
    x, y = _press(page, "#btn-look-right")
    assert _until_turning(url)
    t0 = time.monotonic()
    end(page, x, y)
    took, where = _settle_after(url, t0)
    assert 0 < where < 90, where                # stopped mid-way, not at the limit
    return page, took


def _check(why, took):
    print(f"3.220.B stop: {why} {took:.3f} s")
    assert took <= STOP_BOUND_S, f"{why}: the pan kept changing {took:.3f} s after the end"


def test_release_stops_the_pan(browser, private_twin_server):
    page, took = _held_then(browser, private_twin_server, lambda p, x, y: p.mouse.up())
    _check("release", took)
    page.close()


def test_pointer_cancel_stops_the_pan(browser, private_twin_server):
    def cancel(page, x, y):
        page.evaluate("""() => {
            const b = document.getElementById('btn-look-right');
            b.dispatchEvent(new PointerEvent('pointercancel', {pointerId: 1, bubbles: true}));
        }""")
    page, took = _held_then(browser, private_twin_server, cancel)
    _check("pointercancel", took)
    page.close()


def test_sliding_off_stops_the_pan(browser, private_twin_server):
    page, took = _held_then(browser, private_twin_server,
                            lambda p, x, y: p.mouse.move(x, y - 120))
    _check("slide off", took)
    page.close()


def test_hiding_the_page_stops_the_pan(browser, private_twin_server):
    def hide(page, x, y):
        page.evaluate("""() => {
            Object.defineProperty(document, 'visibilityState', {configurable: true, get: () => 'hidden'});
            document.dispatchEvent(new Event('visibilitychange'));
        }""")
    page, took = _held_then(browser, private_twin_server, hide)
    _check("hidden", took)
    page.close()


def test_closing_the_tab_stops_the_pan(browser, private_twin_server):
    def close(page, x, y):
        page.close(run_before_unload=False)
    _, took = _held_then(browser, private_twin_server, close)
    _check("closed", took)


# ---------- from the reviews ----------

def test_a_body_that_cannot_step_still_looks_on_release(browser, private_twin_server):
    """A held look whose step comes back `pan: null` says so, and the
    release's click sends the plain look verb (it was swallowed)."""
    url = private_twin_server
    page, _ = _open(browser, url)
    _record(page)

    def cannot(route):
        body = route.request.post_data_json or {}
        if body.get("action") == "LOOK_STEP":
            route.fulfill(status=200, content_type="application/json",
                          body='{"executed": true, "result": {"action": "look_step", '
                               '"pan": null, "reason": "this body cannot pan the camera by steps"}}')
        else:
            route.continue_()
    page.route("**/action", cannot)
    _press(page, "#btn-look-left")
    page.wait_for_timeout(700)
    page.mouse.up()
    page.wait_for_timeout(500)
    actions = [p["body"].get("action") for p in _posts(page) if p["route"] == "/action"]
    assert actions == ["LOOK_STEP", "LOOK_LEFT"], actions
    assert "LOOK_LEFT (held)" in page.inner_text("#log")
    assert "cannot pan" in (page.get_attribute("#log .entry", "title") or "")
    page.close()


def test_the_badge_does_not_outlive_a_refused_frame(browser, private_twin_server):
    url = private_twin_server
    page, _ = _open(browser, url)
    page.click("#btn-look-right")
    page.wait_for_function("() => !document.getElementById('pan-badge').hidden", timeout=5000)
    page.route("**/frame", lambda route: route.fulfill(
        status=503, content_type="application/json",
        body='{"detail": "the camera\'s pan is unknown"}'))
    page.click("#btn-look-left")
    page.wait_for_function("() => document.getElementById('pan-badge').hidden", timeout=5000)
    page.close()
