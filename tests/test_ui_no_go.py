# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.161.B: the twin's no-go zones at 390 px (D6; Z6's wording, Z7's taps and
Z8's bars of docs/plans/ros-alignment/3.161-no-go-zones-on-twin.md).

End to end as tests/test_ui_room_names.py: the robot server's map and pose
are stubbed with `home_first_floor` fully mapped at 0.10 m as a SLAM map,
and every request to the brain is answered by the REAL brain app
(`control/brain_server.py`, in process) over a world reporting that map with
a saved map's `frame_id`, its zones stored in a temporary `NOGO_DIR`. Taps
are counted as M1 counts them, the store's file as the goal state (opening
the panel is not counted, as 3.160's N1 did not count opening its editor);
M2-M5 and M7 are 3.132's probes (`evaluations/ux-3132/measure.py`), in
operator mode.

`UI_SHOTS=<dir>` keeps the phone-size screenshots (the simulator's house only).
"""

import json
import os
from pathlib import Path

import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402

from control import nogo_store  # noqa: E402
from tests.test_ui import browser, twin_server  # noqa: F401,E402
from tests.test_ui_room_names import (  # noqa: E402
    _bars, _clean, _forward, _house, _World)
from tests.test_ui import _stub_world, open_twin  # noqa: E402

FRAME = "map-5161beef"
# 3.161.B part 2: a saved map whose dock is at (1.0, 1.0), facing north.
DOCK = {"x_m": 1.0, "y_m": 1.0, "heading_deg": 0.0}


class _DockWorld(_World):
    def __init__(self, m, frame_id, dock):
        super().__init__(m, frame_id)
        self.dock = dock

    def get_pose(self):
        return dict(super().get_pose(), dock=dict(self.dock))


@pytest.fixture
def brain(tmp_path, monkeypatch):
    monkeypatch.setenv("NOGO_DIR", str(tmp_path))
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setattr(nogo_store, "_DEFAULT", None)
    from fastapi.testclient import TestClient
    from control import brain_server

    def make(m, frame_id=FRAME, dock=None):
        w = _World(m, frame_id) if dock is None else _DockWorld(m, frame_id, dock)
        app = brain_server.create_app(robot_factory=lambda: object(),
                                      world_factory=lambda robot: w)
        return TestClient(app)
    return make


def _stored():
    p = Path(os.environ["NOGO_DIR"]) / f"{FRAME}.json"
    return json.loads(p.read_text())["zones"] if p.exists() else []


def _seed(zones):
    store = nogo_store.NoGoStore(os.environ["NOGO_DIR"])
    for z in zones:
        store.edit(FRAME, "add", zone=z)
    return store.zones(FRAME)


def _open(browser, twin_server, client, m, *, suspended=None, goal_reply=None):
    page, errors = open_twin(browser, twin_server, engineer=False)
    _stub_world(page, m, {"usable": True, "map_id": m["map_id"], "x_m": 1.0, "y_m": 1.0,
                          "heading_deg": 0.0}, {"usable": False})
    taps = []

    def goal(route):
        if route.request.method == "POST":
            taps.append(json.loads(route.request.post_data))
            body = goal_reply or {"accepted": True}
        else:
            body = {"goal": None, "plan": []}
        route.fulfill(status=200, content_type="application/json", body=json.dumps(body))
    page.route("**/world/goal", goal)
    if suspended is not None:
        page.route("**/world/nogo", lambda r: r.fulfill(
            status=200, content_type="application/json",
            body=json.dumps({"frame_id": FRAME, "zones": [], "bridge": {"suspended": suspended}})))
    seen = []
    _forward(page, client, seen)
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.click('.tab-btn[data-tab="drive"]')
    page.wait_for_function(
        "() => (document.getElementById('map-readout') || {}).textContent.includes('cells seen')",
        timeout=8000)
    return page, errors, seen, taps


def _shot(page, name):
    d = os.environ.get("UI_SHOTS")
    if d:
        Path(d).mkdir(parents=True, exist_ok=True)
        # Evidence of the map and the panel: the connect toasts taken off,
        # the panel's foot brought to the bottom of the screen.
        page.evaluate("document.querySelectorAll('.toast').forEach(t => t.remove());"
                      "document.querySelector('.nogo-row').scrollIntoView({block: 'end'});"
                      "window.scrollBy(0, 70)")
        page.screenshot(path=str(Path(d) / f"twin-3.161-{name}.png"), full_page=False)


def _to_px(page, m, x_m, y_m):
    box = page.locator("#world-map").bounding_box()
    s = box["width"] / (m["width"] * m["resolution_m"])
    return box["x"] + (x_m - m["origin_x_m"]) * s, box["y"] + (y_m - m["origin_y_m"]) * s


def _drag(page, m, a, b):
    ax, ay = _to_px(page, m, *a)
    bx, by = _to_px(page, m, *b)
    page.mouse.move(ax, ay)
    page.mouse.down()
    page.mouse.move((ax + bx) / 2, (ay + by) / 2, steps=3)
    page.mouse.move(bx, by, steps=3)
    page.mouse.up()


def _wait_stored(page, n):
    for _ in range(60):
        if len(_stored()) == n:
            return
        page.wait_for_timeout(100)
    assert len(_stored()) == n, _stored()


# ---------- Z7: drawing and deleting ----------

def test_a_zone_is_drawn_in_three_taps_and_stored_with_the_saved_map(browser, twin_server, brain):
    m = _house()
    page, errors, seen, _ = _open(browser, twin_server, brain(m), m)
    sync_api.expect(page.locator("#btn-nogo")).to_be_visible(timeout=5000)
    page.click("#btn-nogo")                                  # open the panel (not counted)
    sync_api.expect(page.locator("#nogo-draw")).to_be_visible(timeout=5000)
    page.click("#nogo-draw")                                 # tap 1
    _shot(page, "drawing")
    _drag(page, m, (1.5, 1.5), (2.5, 2.2))                   # tap 2: the drag
    _wait_stored(page, 1)
    page.click("#nogo-done")                                 # tap 3
    z = _stored()[0]
    assert z["kind"] == "area" and z["by"] == "person"
    assert 0.9 <= z["x1"] - z["x0"] <= 1.1 and 0.6 <= z["y1"] - z["y0"] <= 0.8, z
    assert ("POST", "/mission/nogo") in seen
    sync_api.expect(page.locator("#nogo-panel")).to_be_hidden()
    assert not errors, errors
    page.context.close()


def test_a_wall_is_a_line_and_is_three_taps(browser, twin_server, brain):
    m = _house()
    page, errors, _, _ = _open(browser, twin_server, brain(m), m)
    page.click("#btn-nogo")
    page.click("#nogo-wall")                                 # tap 1
    _drag(page, m, (1.0, 3.0), (3.0, 3.1))                   # tap 2
    _wait_stored(page, 1)
    _shot(page, "wall")
    page.click("#nogo-done")                                 # tap 3
    z = _stored()[0]
    assert z["kind"] == "wall"
    assert abs((z["y1"] - z["y0"]) - 0.10) < 1e-6 and z["x1"] - z["x0"] > 1.8, z
    assert not errors, errors
    page.context.close()


def test_a_zone_is_deleted_in_two_taps(browser, twin_server, brain):
    m = _house()
    _seed([{"x0": 1.5, "y0": 1.5, "x1": 2.5, "y1": 2.5}])
    page, errors, _, _ = _open(browser, twin_server, brain(m), m)
    page.click("#btn-nogo")                                  # open the panel (not counted)
    sync_api.expect(page.locator("#nogo-hint")).to_contain_text("1 zone", timeout=5000)
    page.evaluate("document.getElementById('world-map').scrollIntoView({block: 'center'})")
    x, y = _to_px(page, m, 2.0, 2.0)
    page.mouse.click(x, y)                                   # tap 1: the zone
    sync_api.expect(page.locator("#nogo-delete")).to_be_visible()
    page.click("#nogo-delete")                               # tap 2
    _wait_stored(page, 0)
    assert not errors, errors
    page.context.close()


def test_a_too_small_drag_is_said_and_stores_nothing(browser, twin_server, brain):
    m = _house()
    page, errors, _, _ = _open(browser, twin_server, brain(m), m)
    page.click("#btn-nogo")
    page.click("#nogo-draw")
    _drag(page, m, (1.5, 1.5), (1.6, 2.5))                   # 0.10 m wide
    sync_api.expect(page.locator(".toast").last).to_contain_text("at least 20 cm", timeout=3000)
    assert _stored() == []
    assert not errors, errors
    page.context.close()


# ---------- Z6: a person's goal tap inside a zone ----------

def test_a_goal_tap_refused_for_a_zone_is_worded(browser, twin_server, brain):
    m = _house()
    _seed([{"x0": 1.5, "y0": 1.5, "x1": 2.5, "y1": 2.5}])
    refusal = {"accepted": False, "reason": "no_go_zone", "zone": "z1",
               "detail": "that point is inside a no-go zone a person drew"}
    page, errors, _, taps = _open(browser, twin_server, brain(m), m, goal_reply=refusal)
    page.wait_for_timeout(300)
    page.evaluate("document.getElementById('world-map').scrollIntoView({block: 'center'})")
    x, y = _to_px(page, m, 2.0, 2.0)
    page.mouse.click(x, y)
    # The operator's words for D5's reason, not the server's detail.
    sync_api.expect(page.locator(".toast").last).to_contain_text(
        "That's in a no-go zone", timeout=3000)
    assert len(taps) == 1 and 1.9 < taps[0]["x_m"] < 2.1
    assert not errors, errors
    page.context.close()


# ---------- closed, and where it is offered ----------

def test_no_saved_map_says_survey_first(browser, twin_server, brain):
    m = _house()
    page, errors, _, _ = _open(browser, twin_server, brain(m, frame_id=None), m)
    page.click("#btn-nogo")
    sync_api.expect(page.locator("#nogo-closed")).to_contain_text("Survey first", timeout=5000)
    sync_api.expect(page.locator("#nogo-draw")).to_be_hidden()
    assert not errors, errors
    page.context.close()


# ---------- Z8: the bars at 390 px, operator mode ----------

def test_the_states_meet_3132s_bars(browser, twin_server, brain):
    m = _house()
    results = {}
    page, errors, _, _ = _open(browser, twin_server, brain(m), m)
    page.click("#btn-nogo")
    sync_api.expect(page.locator("#nogo-hint")).to_contain_text("No zones yet", timeout=5000)
    results["none"] = _bars(page)
    _shot(page, "none")
    page.click("#nogo-draw")
    results["drawing"] = _bars(page)
    page.click("#nogo-cancel")
    page.click("#nogo-wall")
    results["wall"] = _bars(page)
    page.context.close()
    zones = _seed([{"x0": 0.5 + i, "y0": 1.0, "x1": 0.9 + i, "y1": 1.6} for i in range(4)]
                  + [{"x0": 1.0, "y0": 3.0, "x1": 3.0, "y1": 3.1, "kind": "wall"}])
    page, errors2, _, _ = _open(browser, twin_server, brain(m), m)
    page.click("#btn-nogo")
    sync_api.expect(page.locator("#nogo-hint")).to_contain_text("5 zones", timeout=5000)
    results["five"] = _bars(page)
    _shot(page, "five")
    page.context.close()
    page, errors3, _, _ = _open(browser, twin_server, brain(m), m, suspended=[zones[0]["id"]])
    page.click("#btn-nogo")
    sync_api.expect(page.locator("#nogo-paused")).to_be_visible(timeout=5000)
    results["suspended"] = _bars(page)
    _shot(page, "suspended")
    bad = {k: v for k, v in results.items() if not _clean(v)}
    assert not bad, bad
    assert not errors and not errors2 and not errors3, (errors, errors2, errors3)
    page.context.close()


# ---------- 3.161.B part 2: a zone over the dock (E-e) ----------

def test_a_zone_drawn_over_the_dock_is_refused_in_plain_words(browser, twin_server, brain):
    m = _house()
    page, errors, _, _ = _open(browser, twin_server, brain(m, dock=DOCK), m)
    page.click("#btn-nogo")
    page.click("#nogo-draw")
    _drag(page, m, (0.6, 0.6), (1.4, 1.5))                   # over the dock
    sync_api.expect(page.locator(".toast").last).to_contain_text(
        "covers the robot's dock, so it couldn't get home", timeout=3000)
    assert _stored() == []
    assert not errors, errors
    page.context.close()


def test_a_zone_clear_of_the_dock_is_drawn_as_before(browser, twin_server, brain):
    m = _house()
    page, errors, _, _ = _open(browser, twin_server, brain(m, dock=DOCK), m)
    page.click("#btn-nogo")
    page.click("#nogo-draw")
    _drag(page, m, (2.0, 2.0), (2.8, 2.6))
    _wait_stored(page, 1)
    sync_api.expect(page.locator("#nogo-dock")).to_be_hidden()
    assert not errors, errors
    page.context.close()


def test_a_stored_zone_over_the_dock_is_said_and_meets_3132s_bars(browser, twin_server, brain):
    m = _house()
    _seed([{"x0": 0.7, "y0": 0.7, "x1": 1.3, "y1": 1.3}])   # saved before: no dock known then
    page, errors, _, _ = _open(browser, twin_server, brain(m, dock=DOCK), m)
    page.click("#btn-nogo")
    sync_api.expect(page.locator("#nogo-dock")).to_be_visible(timeout=5000)
    sync_api.expect(page.locator("#nogo-dock")).to_contain_text(
        "covers the robot's dock, so the robot ignores it")
    bars = _bars(page)
    _shot(page, "dock")
    assert _clean(bars), bars
    assert len(_stored()) == 1                                # never deleted for the person
    assert not errors, errors
    page.context.close()
