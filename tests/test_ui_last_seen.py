# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.175.C: "Where did you last see it?" on the twin's Home (C6-C7 of
docs/plans/ros-alignment/3.175-memory-when-things-move.md).

The ask, the thumbnail and the parse are answered by the REAL brain app
(`control/brain_server.py`, in process) over a chain and a 3.162 report saved
in a temporary directory; `/mission/start` is captured. Each test was seen
failing on `dev`'s page first.

`UI_SHOTS=<dir>` keeps the phone-size screenshots.
"""

import io
import json
import os
from pathlib import Path

import pytest

pytest.importorskip("playwright")

from fastapi.testclient import TestClient  # noqa: E402
from playwright import sync_api  # noqa: E402

from tests.test_ui import TIERED_BRAIN_HEALTH, browser, open_twin, twin_server  # noqa: F401,E402

ASK = "/mission/last-seen"
IDLE = {"running": False, "outcome": None, "job": "find", "policy": "frontier",
        "step": 0, "log_tail": [], "tier": None, "perception": None}
RUNNING = dict(IDLE, running=True, outcome="running", step=1)
FRAME = "map-ui"


class _Untouchable:
    def __getattr__(self, name):
        raise AttributeError(name)


def _jpeg():
    from PIL import Image
    buf = io.BytesIO()
    Image.new("RGB", (64, 48), (200, 30, 30)).save(buf, "JPEG")
    return buf.getvalue()


@pytest.fixture(scope="module")
def brain(tmp_path_factory):
    import time

    from brain.what_i_saw import ReportStore
    from control.inventory_store import InventoryStore
    d = tmp_path_factory.mktemp("last-seen-ui")
    row = {"label": "red backpack", "x_m": 2.0, "y_m": 1.0, "belief": 0.95, "seen_seq": 1,
           "room": "living_room", "votes": {"red backpack": 3}}
    doubt = {"label": "mug", "x_m": 4.0, "y_m": 1.0, "belief": 0.7, "seen_seq": 0,
             "room": "kitchen", "votes": {"mug": 2}}
    InventoryStore(str(d / "inv")).save("m-1", {"frame_id": FRAME, "carry_seq": 1,
                                                "reported": [row], "candidates": [doubt]})
    ReportStore(d / "saw").save("m-1", {"job": "find", "target": "keys", "frame_id": FRAME,
                                        "ended": time.time() - 2 * 3600},
                                looks=[{"label": "red backpack", "x_m": 2.0, "y_m": 1.0,
                                        "room": "the hall", "belief": 0.95, "looks": 3,
                                        "jpeg": _jpeg(), "box": [0.2, 0.2, 0.6, 0.8]}])
    old = {k: os.environ.get(k) for k in ("INVENTORY_DIR", "WHAT_I_SAW_DIR")}
    os.environ["INVENTORY_DIR"], os.environ["WHAT_I_SAW_DIR"] = str(d / "inv"), str(d / "saw")
    from control import brain_server
    app = brain_server.create_app(robot_factory=lambda: _Untouchable(),
                                  world_factory=lambda robot: _Untouchable())
    # Kept for the module: the report store's directory is read at first use.
    yield TestClient(app)
    for k, v in old.items():
        if v is None:
            os.environ.pop(k, None)
        else:
            os.environ[k] = v


def _shot(page, name):
    out = os.environ.get("UI_SHOTS")
    if out:
        Path(out).mkdir(parents=True, exist_ok=True)
        page.locator("#brain-target-row").scroll_into_view_if_needed()
        page.screenshot(path=str(Path(out) / f"twin-3.175c-{name}.png"))


def _open(browser, twin_server, brain, *, ask="real", engineer=False):
    """Home with a stubbed brain connected. `ask`: "real" answers from the
    real brain; "old" a brain without the route (FastAPI's 404); "abort" no
    answer."""
    page, errors = open_twin(browser, twin_server, engineer=engineer, tab="home")
    calls = {"ask": [], "start": [], "other": [], "thumb": []}

    def fwd(route, path):
        r = brain.get(path)
        route.fulfill(status=r.status_code, headers={"content-type": r.headers["content-type"]},
                      body=r.content)

    def stub(route):
        req = route.request
        full = req.url.split("/brain-stub", 1)[1]
        path = full.split("?")[0]
        if path == "/health":
            return route.fulfill(status=200, content_type="application/json",
                                 body=json.dumps(TIERED_BRAIN_HEALTH))
        if path == "/mission/status":
            return route.fulfill(status=200, content_type="application/json",
                                 body=json.dumps(RUNNING if calls["start"] else IDLE))
        if path == "/mission/start":
            calls["start"].append(req.post_data_json)
            return route.fulfill(status=200, content_type="application/json",
                                 body=json.dumps({"started": True, "status": RUNNING}))
        if path == "/mission/request/parse":
            r = brain.post(path, json=req.post_data_json)
            return route.fulfill(status=r.status_code, content_type="application/json", body=r.text)
        if path == ASK:
            calls["ask"].append(full)
            if ask == "real":
                return fwd(route, full)
            if ask == "abort":
                return route.abort()
        if path.startswith("/mission/reports/") and "/thumbs/" in path:
            calls["thumb"].append(path)
            return fwd(route, path)
        calls["other"].append(path)
        route.fulfill(status=404, content_type="application/json",
                      body=json.dumps({"detail": "Not Found"}))

    page.route("**/brain-stub/**", stub)
    robot = []
    page.on("request", lambda r: robot.append(r.url) if "/brain-stub" not in r.url
            and any(p in r.url for p in ("/action", "/stop", "/wheels", "/world/goal")) else None)
    calls["robot"] = robot
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(600)
    page.click('.tab-btn[data-tab="home"]')
    return page, errors, calls


ASK_BTN = "#btn-last-seen"
TEXT = "#last-seen-text"
GO = "#btn-last-seen-go"
START = "#btn-brain-mission"


def test_the_answer_shows_where_when_and_its_picture_and_go_check_is_starts_find(browser, twin_server, brain):
    """C6: the reader's target is asked; place, room, when and thumbnail
    are shown; Go check sends exactly Start's body for that target; the row
    hides while the mission runs; nothing reaches the robot; no sideways
    scroll at 390 px."""
    page, errors, calls = _open(browser, twin_server, brain)
    page.fill("#brain-target", "where is my red backpack")
    sync_api.expect(page.locator("#find-reading")).to_contain_text("red backpack", timeout=5000)
    sync_api.expect(page.locator(ASK_BTN)).to_be_visible()
    page.click(ASK_BTN)
    text = page.locator(TEXT)
    sync_api.expect(text).to_contain_text("Last seen: red backpack, in the hall, 2 h ago", timeout=5000)
    assert calls["ask"] == ["/mission/last-seen?name=red%20backpack"], calls
    img = page.locator("#last-seen-img")
    sync_api.expect(img).to_be_visible(timeout=5000)
    assert page.evaluate("() => document.getElementById('last-seen-img').naturalWidth") > 0
    assert page.evaluate("() => document.documentElement.scrollWidth") <= 390
    sync_api.expect(page.locator(GO)).to_be_visible()
    _shot(page, "seen")
    assert calls["start"] == [] and calls["robot"] == [], calls
    page.click(GO)
    sync_api.expect(page.locator(START)).to_have_text("Stop", timeout=5000)
    assert len(calls["start"]) == 1, calls
    body = calls["start"][0]
    assert body == {"target_object": "red backpack", "fault": "none", "policy": body["policy"]}, body
    sync_api.expect(page.locator("#last-seen")).to_be_hidden()
    sync_api.expect(page.locator(ASK_BTN)).to_be_hidden()
    assert calls["robot"] == [], calls
    assert not errors, errors
    page.context.close()


def test_go_checks_body_is_starts_own_for_the_same_words(browser, twin_server, brain):
    """C6: Start with "red backpack" typed sends the same body Go check does."""
    page, errors, calls = _open(browser, twin_server, brain)
    page.fill("#brain-target", "red backpack")
    sync_api.expect(page.locator(START)).to_be_enabled(timeout=5000)
    page.click(START)
    sync_api.expect(page.locator(START)).to_have_text("Stop", timeout=5000)
    assert calls["start"] == [{"target_object": "red backpack", "fault": "none",
                               "policy": calls["start"][0]["policy"]}], calls
    assert not errors, errors
    page.context.close()


def test_doubtful_and_not_seen_are_said_plainly_and_typing_clears_the_answer(browser, twin_server, brain):
    page, errors, calls = _open(browser, twin_server, brain)
    page.fill("#brain-target", "mug")
    page.click(ASK_BTN)
    sync_api.expect(page.locator(TEXT)).to_contain_text("wasn't there when the robot looked again",
                                                        timeout=5000)
    # The mission's own label, as words; a person's room name is shown as given.
    sync_api.expect(page.locator(TEXT)).to_contain_text("in the kitchen")
    sync_api.expect(page.locator("#last-seen-img")).to_be_hidden()
    _shot(page, "doubtful")
    page.fill("#brain-target", "zeppelin")
    sync_api.expect(page.locator("#last-seen")).to_be_hidden()
    page.click(ASK_BTN)
    sync_api.expect(page.locator(TEXT)).to_contain_text("hasn't seen zeppelin", timeout=5000)
    assert calls["start"] == [] and calls["robot"] == [], calls
    assert not errors, errors
    page.context.close()


def test_an_old_brain_says_it_needs_an_update_without_a_toast(browser, twin_server, brain):
    """C7, the first half: FastAPI's own 404."""
    page, errors, calls = _open(browser, twin_server, brain, ask="old")
    page.fill("#brain-target", "red backpack")
    page.click(ASK_BTN)
    sync_api.expect(page.locator(TEXT)).to_contain_text("needs an update", timeout=5000)
    assert page.locator(".toast.err").count() == 0
    assert not errors, errors
    page.context.close()


def test_no_answer_says_so_and_can_be_asked_again(browser, twin_server, brain):
    """C7, the second half."""
    page, errors, calls = _open(browser, twin_server, brain, ask="abort")
    page.fill("#brain-target", "red backpack")
    page.click(ASK_BTN)
    sync_api.expect(page.locator(TEXT)).to_contain_text("Couldn't ask the robot", timeout=5000)
    sync_api.expect(page.locator(ASK_BTN)).to_be_enabled()
    page.click(ASK_BTN)
    sync_api.expect(page.locator(TEXT)).to_contain_text("Couldn't ask the robot", timeout=5000)
    assert len(calls["ask"]) == 2, calls
    assert not errors, errors
    page.context.close()


def test_engineer_mode_names_the_fields(browser, twin_server, brain):
    page, errors, calls = _open(browser, twin_server, brain, engineer=True)
    page.fill("#brain-target", "red backpack")
    page.click(ASK_BTN)
    sync_api.expect(page.locator(TEXT)).to_contain_text("frame_id map-ui", timeout=5000)
    sync_api.expect(page.locator(TEXT)).to_contain_text("report m-1")
    assert not errors, errors
    page.context.close()
