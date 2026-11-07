# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.162.B: the twin's "What it saw" at 390 px (W2, W8 of
docs/plans/ros-alignment/3.162-what-i-saw.md).

A report is written by the real store (`brain/what_i_saw.py`) from a frame
the simulator rendered (the robot server's own `/frame`), and the brain's
report routes are answered from that store -- their wiring into the brain
server is 3.162.C. Taps are counted as 3.132's M1 counts them; M2-M5 and M7
are 3.132's probes, in operator mode.

`UI_SHOTS=<dir>` keeps the phone-size screenshots (the simulator's frames only).
"""

import base64
import io
import json
import os
from pathlib import Path

import httpx
import pytest

pytest.importorskip("playwright")

from PIL import Image  # noqa: E402
from playwright import sync_api  # noqa: E402

from brain import what_i_saw as wis  # noqa: E402
from tests.test_ui import browser, open_twin, twin_server  # noqa: F401,E402
from tests.test_ui_room_names import _bars, _clean  # noqa: E402

BOX = [0.35, 0.30, 0.65, 0.85]


def _sim_jpeg(twin_server) -> bytes:
    b64 = httpx.get(twin_server + "/frame", timeout=10).json()["image_base64"]
    img = Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB")
    buf = io.BytesIO()
    img.save(buf, "JPEG", quality=85)
    return buf.getvalue()


def _status(outcome="found", job="find"):
    return {"running": False, "outcome": outcome, "end_state": outcome, "job": job,
            "target_object": "red backpack", "step": 42, "end_reason": "Found.",
            "rooms_searched": ["kitchen"]}


def _serve(page, store, status, seen):
    def handler(route):
        req = route.request
        path = req.url.split("/brain-stub", 1)[1].split("?")[0]
        seen.append((req.method, path))
        if path == "/health":
            body = {"status": "ok", "mission_running": False, "identity": {"git_revision": "t"}}
        elif path == "/mission/status":
            body = status
        elif path == "/mission/reports" and req.method == "GET":
            body = {"reports": store.list()}
        elif path.startswith("/mission/reports/"):
            rest = path[len("/mission/reports/"):]
            mid, _, name = rest.partition("/")
            try:
                if req.method == "DELETE":
                    body = {"deleted": store.delete(mid)}
                elif name:
                    return route.fulfill(status=200, content_type="image/jpeg",
                                         body=store.image(mid, name))
                else:
                    body = store.get(mid)
            except wis.ReportRefused:
                return route.fulfill(status=404, content_type="application/json",
                                     body='{"detail":"no report"}')
        else:
            return route.fulfill(status=404, content_type="application/json", body='{"detail":"x"}')
        route.fulfill(status=200, content_type="application/json", body=json.dumps(body))
    page.route("**/brain-stub/**", handler)


def _open(browser, twin_server, store, status):
    page, errors = open_twin(browser, twin_server, engineer=False, tab="home")
    seen = []
    _serve(page, store, status, seen)
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.click('.tab-btn[data-tab="home"]')
    return page, errors, seen


def _shot(page, name):
    d = os.environ.get("UI_SHOTS")
    if d:
        page.wait_for_function("() => document.getAnimations().every(a => a.playState !== 'running')",
                               timeout=8000)
        Path(d).mkdir(parents=True, exist_ok=True)
        page.screenshot(path=str(Path(d) / f"twin-3.162-{name}.png"))


@pytest.fixture
def report(tmp_path, twin_server):
    store = wis.ReportStore(tmp_path)
    frame = _sim_jpeg(twin_server)
    looks = [{"label": name, "x_m": 1.0 + i, "y_m": 2.0, "room": room, "belief": 0.9 - 0.1 * i,
              "looks": 3, "jpeg": frame, "box": BOX}
             for i, (name, room) in enumerate([("red backpack", "kitchen"), ("chair", "kitchen"),
                                               ("sofa", "living room")])]
    store.save("m-162", {"outcome": "found", "job": "find", "target": "red backpack"},
               found={"jpeg": frame, "box": BOX, "confirmed": True}, looks=looks)
    return store


def test_the_found_frame_is_one_tap_from_the_card_with_its_box(browser, twin_server, report):
    page, errors, _ = _open(browser, twin_server, report, _status())
    found = page.locator("#saw-found")
    sync_api.expect(found).to_be_visible(timeout=8000)
    _shot(page, "card")
    found.click()                                                    # tap 1
    sync_api.expect(page.locator("#saw-viewer")).to_be_visible()
    sync_api.expect(page.locator("#saw-box")).to_be_visible()
    sync_api.expect(page.locator("#saw-caption")).to_contain_text("red backpack")
    _shot(page, "found")
    assert not errors, errors
    page.context.close()


def test_a_listed_things_picture_is_two_taps(browser, twin_server, report):
    page, errors, _ = _open(browser, twin_server, report, _status())
    sync_api.expect(page.locator("#saw-open")).to_have_text("What it saw (3)", timeout=8000)
    page.click("#saw-open")                                          # tap 1
    items = page.locator("#saw-items .saw-item")
    sync_api.expect(items).to_have_count(3, timeout=5000)
    _shot(page, "list")
    items.nth(2).click()                                             # tap 2
    sync_api.expect(page.locator("#saw-viewer")).to_be_visible()
    sync_api.expect(page.locator("#saw-caption")).to_contain_text("sofa · living room")
    assert not errors, errors
    page.context.close()


def test_delete_takes_two_taps_and_removes_every_file(browser, twin_server, report, tmp_path):
    page, errors, seen = _open(browser, twin_server, report, _status())
    sync_api.expect(page.locator("#saw-delete")).to_be_visible(timeout=8000)
    page.click("#saw-delete")                                        # armed only
    assert ("DELETE", "/mission/reports/m-162") not in seen
    page.wait_for_timeout(500)
    page.click("#saw-delete")                                        # deleted
    sync_api.expect(page.locator("#saw")).to_be_hidden(timeout=5000)
    assert ("DELETE", "/mission/reports/m-162") in seen
    assert [p for p in tmp_path.iterdir() if not p.name.startswith(".")] == []
    assert not errors, errors
    page.context.close()


def test_an_unconfirmed_arrival_says_so_and_a_brain_without_reports_shows_nothing(
        browser, twin_server, report, tmp_path):
    rep = report.get("m-162")
    frame = report.image("m-162", "found.jpg")
    report.save("m-163", {k: rep[k] for k in ("job", "target")} | {"outcome": "arrived_unconfirmed"},
                found={"jpeg": frame, "box": BOX, "confirmed": False})
    page, errors, _ = _open(browser, twin_server, report, _status("arrived_unconfirmed"))
    sync_api.expect(page.locator("#saw-found")).to_be_visible(timeout=8000)
    page.click("#saw-found")
    sync_api.expect(page.locator("#saw-caption")).to_contain_text("couldn't confirm")
    _shot(page, "unconfirmed")
    assert not errors, errors
    page.context.close()
    empty = wis.ReportStore(tmp_path / "none")
    page, errors, _ = _open(browser, twin_server, empty, _status())
    page.wait_for_timeout(1500)
    sync_api.expect(page.locator("#saw")).to_be_hidden()
    assert not errors, errors
    page.context.close()


def test_whats_shown_meets_3132s_bars(browser, twin_server, report):
    page, errors, _ = _open(browser, twin_server, report, _status())
    sync_api.expect(page.locator("#saw-found")).to_be_visible(timeout=8000)
    # The connection toasts fade in as the card appears; axe reads a fading
    # toast's contrast mid-animation. Measured once they have settled.
    page.wait_for_function("() => document.getAnimations().every(a => a.playState !== 'running')",
                           timeout=8000)
    results = {"card": _bars(page)}
    page.click("#saw-open")
    sync_api.expect(page.locator("#saw-items .saw-item")).to_have_count(3, timeout=5000)
    results["list"] = _bars(page)
    page.locator("#saw-items .saw-item").first.click()
    sync_api.expect(page.locator("#saw-viewer")).to_be_visible()
    results["viewer"] = _bars(page)
    bad = {k: v for k, v in results.items() if not _clean(v)}
    assert not bad, bad
    assert not errors, errors
    page.context.close()
