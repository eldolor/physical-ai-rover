# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.203: the twin polls no robot before one is connected.

A static host (the deployed twin on CloudFront) is not a robot: a first-run
page there must request no robot route. A page the robot server serves is
its own robot and still polls and connects; a saved robot is still polled.
"""

import functools
import http.server
import threading

import pytest

pytest.importorskip("playwright")

from tests.conftest import REPO_ROOT  # noqa: E402
from tests.test_ui import (PHONE, browser, open_twin, twin_server)  # noqa: F401,E402

# Robot-only routes. /health is not one: the env banner reads the page's own
# /health, which the deployed page answers from the vision service.
ROBOT_ROUTES = ("/frame", "/battery", "/depth", "/action", "/wheels")
CONNECTED = "() => document.getElementById('connection-status').classList.contains('is-ok')"


@pytest.fixture
def static_origin():
    """web-twin/ on a plain static server: the page without a robot behind it."""
    seen = []

    class Handler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, fmt, *args):
            seen.append(self.path.split("?")[0])

    handler = functools.partial(Handler, directory=str(REPO_ROOT / "web-twin"))
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        yield f"http://127.0.0.1:{srv.server_address[1]}", seen
    finally:
        srv.shutdown()
        srv.server_close()


def _page(browser, url, *, first_run, saved_robot=None):
    context = browser.new_context(viewport=PHONE, legacy_tab=None, first_run=first_run)
    if saved_robot:
        context.add_init_script(
            "(() => { try { localStorage.setItem('vp_server_url', %r); } catch (e) {} })();" % saved_robot)
    page = context.new_page()
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(url, wait_until="load")
    page.wait_for_function("() => !!document.body.dataset.tab")
    return context, page, errors


def test_a_first_run_page_on_a_static_host_requests_no_robot_route(browser, static_origin):  # noqa: F811
    url, seen = static_origin
    context, page, errors = _page(browser, url, first_run=True)
    page.wait_for_timeout(10000)
    assert page.input_value("#cfg-server-url") == ""
    robot = [p for p in seen if p.startswith(ROBOT_ROUTES)]
    assert robot == [], f"a page with no robot asked: {robot}"
    assert errors == []
    page.screenshot(path="/tmp/first-run-390.png")
    context.close()


def test_a_page_the_robot_serves_still_connects_to_itself(browser, twin_server):  # noqa: F811
    context, page, errors = _page(browser, twin_server, first_run=True)
    assert page.input_value("#cfg-server-url") == twin_server
    page.wait_for_function(CONNECTED, timeout=10000)
    assert errors == []
    context.close()


def test_a_saved_robot_is_still_polled_from_a_static_host(browser, static_origin, twin_server):  # noqa: F811
    url, seen = static_origin
    battery = []
    context = browser.new_context(viewport=PHONE, legacy_tab=None, first_run=True)
    context.add_init_script(
        "(() => { try { localStorage.setItem('vp_server_url', %r); } catch (e) {} })();" % twin_server)
    page = context.new_page()
    page.on("request", lambda r: battery.append(r.url) if r.url.startswith(twin_server + "/battery") else None)
    page.goto(url, wait_until="load")
    page.wait_for_function(CONNECTED, timeout=10000)
    page.wait_for_timeout(1500)
    assert battery, "a saved robot was not polled for its battery"
    assert [p for p in seen if p.startswith(ROBOT_ROUTES)] == []
    context.close()
