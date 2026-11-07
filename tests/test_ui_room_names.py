# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.160.B: the twin's room names editor at 390 px (N1, N2, N7 of
docs/plans/ros-alignment/3.160-rooms-named-on-twin.md).

End to end: the robot server's map and pose are stubbed with
`home_first_floor` fully mapped at 0.10 m as a SLAM map, and every request to
the brain is answered by the REAL brain app (`control/brain_server.py`, in
process) over a world reporting that map with a saved map's `frame_id`, its
names stored in a temporary `ROOM_NAMES_DIR`. Taps are counted as M1 counts
them (the store's file as the goal state); M2-M5 and M7 are 3.132's probes
(`evaluations/ux-3132/measure.py`), in operator mode.

`UI_SHOTS=<dir>` keeps the phone-size screenshots (the simulator's house only).
"""

import importlib.util
import json
import os
from pathlib import Path

import pytest

pytest.importorskip("playwright")

from fastapi.testclient import TestClient  # noqa: E402
from playwright import sync_api  # noqa: E402

from brain import room_names as rn  # noqa: E402
from tests import room_cut_compare as rc  # noqa: E402
from tests.test_ui import _stub_world, browser, open_twin, twin_server  # noqa: F401,E402

FRAME = "map-5160beef"
ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("ux3132_measure", ROOT / "evaluations/ux-3132/measure.py")
ux = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ux)


def _house():
    m = rc.house_map("home_first_floor", k=3)
    m.update({"usable": True, "map_id": "slam-h160", "map_version": 1,
              "origin_x_m": 0.0, "origin_y_m": 0.0})
    return m


class _World:
    def __init__(self, m, frame_id):
        self.m, self.frame_id = m, frame_id

    def get_map(self):
        return self.m

    def get_pose(self):
        return {"usable": True, "x_m": 1.0, "y_m": 1.0, "heading_deg": 0.0,
                "map_id": self.m["map_id"], "frame_id": self.frame_id}


class _Runner:
    """A finished survey's status: where its labels were asked (the suggestions)."""

    def __init__(self, seeds):
        self.seeds = seeds

    def is_running(self):
        return False

    def status(self):
        return {"running": False, "outcome": "exhausted", "job": "survey", "step": 0,
                "survey": {"seeds": self.seeds}}


@pytest.fixture
def brain(tmp_path, monkeypatch):
    monkeypatch.setenv("ROOM_NAMES_DIR", str(tmp_path))
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setattr(rn, "_DEFAULT", None)
    from control import brain_server

    def make(m, frame_id=FRAME, seeds=None):
        app = brain_server.create_app(robot_factory=lambda: object(),
                                      world_factory=lambda robot: _World(m, frame_id))
        if seeds:
            app.state.brain_state["runner"] = _Runner(seeds)
        return TestClient(app)
    return make


def _forward(page, client, seen):
    def handler(route):
        req = route.request
        path = req.url.split("/brain-stub", 1)[1]
        seen.append((req.method, path.split("?")[0]))
        r = client.request(req.method, path, content=req.post_data,
                           headers={"content-type": "application/json"})
        route.fulfill(status=r.status_code, content_type="application/json", body=r.text)
    page.route("**/brain-stub/**", handler)


def _open(browser, twin_server, client, m, engineer=False):
    page, errors = open_twin(browser, twin_server, engineer=engineer)
    _stub_world(page, m, {"usable": True, "map_id": m["map_id"], "x_m": 1.0, "y_m": 1.0,
                          "heading_deg": 0.0}, {"usable": False})
    page.route("**/world/goal", lambda r: r.fulfill(
        status=200, content_type="application/json", body=json.dumps({"goal": None, "plan": []})))
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
    return page, errors, seen


def _centre(brain, m):
    """A seed where a room of the cut really is: its centre, as the brain says."""
    r = brain(m).get("/mission/rooms").json()["rooms"][0]
    return [[r["x_m"], r["y_m"], "garage"]]


def _stored(tmp):
    p = Path(os.environ["ROOM_NAMES_DIR"]) / f"{FRAME}.json"
    return json.loads(p.read_text())["rooms"] if p.exists() else []


def _shot(page, name):
    d = os.environ.get("UI_SHOTS")
    if d:
        Path(d).mkdir(parents=True, exist_ok=True)
        page.screenshot(path=str(Path(d) / f"twin-3.160-{name}.png"), full_page=False)


def _bars(page):
    """M2-M5 and M7 of 3.132's harness, on the page as it is now."""
    page.add_script_tag(content=ux.axe_source())
    found = page.evaluate("async (tags) => (await axe.run(document, {runOnly: {type: 'tag', values: tags}}))"
                          ".violations.filter(v => ['serious','critical'].includes(v.impact))"
                          ".map(v => v.id + ': ' + v.nodes.length)", ux.AXE_TAGS)
    small = page.evaluate(ux.JS_SMALL_TARGETS)
    sideways = page.evaluate(ux.JS_SCROLL)
    text = page.evaluate(ux.JS_VISIBLE_TEXT)
    jargon = [(k, t) for t in text for k, rx in ux.JARGON if rx.search(t)]
    moving = page.evaluate(ux.JS_MOVING)
    return {"M2": found, "M3": small, "M4": sideways, "M5": jargon, "M7": moving}


def _clean(bars):
    sideways = bars["M4"]
    wide = sideways.get("scrollWidth", 0) > sideways.get("clientWidth", 0) + 1 \
        if isinstance(sideways, dict) else bool(sideways)
    return not bars["M2"] and not bars["M3"] and not wide and not bars["M5"] and not bars["M7"]


# ---------- N1: naming ----------

def test_a_room_is_named_in_two_taps_and_stored_with_the_saved_map(browser, twin_server, brain):
    m = _house()
    page, errors, seen = _open(browser, twin_server, brain(m), m)
    sync_api.expect(page.locator("#btn-rooms")).to_be_visible(timeout=5000)
    page.click("#btn-rooms")                                         # open the editor: 1
    sync_api.expect(page.locator("#btn-rooms")).to_be_hidden()      # Done closes it
    rows = page.locator("#rooms-list li")
    sync_api.expect(rows.first).to_be_visible(timeout=5000)
    assert rows.count() >= 5
    first = rows.first.locator("input")
    first.click()                                                    # tap 1
    first.type("Kitchen")
    first.press("Enter")                                             # tap 2
    page.wait_for_function("() => [...document.querySelectorAll('#rooms-list input')]"
                           ".some(i => i.value === 'Kitchen')", timeout=5000)
    assert [r["name"] for r in _stored(None)] == ["Kitchen"]
    assert ("POST", "/mission/rooms/names") in seen
    _shot(page, "naming")
    assert not errors, errors
    page.context.close()


def test_a_suggestion_is_one_tap(browser, twin_server, brain):
    m = _house()
    page, errors, _ = _open(browser, twin_server, brain(m, seeds=_centre(brain, m)), m)
    page.click("#btn-rooms")
    sug = page.locator("#rooms-list .rooms-suggest").first
    sync_api.expect(sug).to_be_visible(timeout=5000)
    sug.click()                                                      # tap 1
    page.wait_for_function("() => [...document.querySelectorAll('#rooms-list input')]"
                           ".some(i => i.value === 'garage')", timeout=5000)
    assert [r["name"] for r in _stored(None)] == ["garage"]
    assert not errors, errors
    page.context.close()


# ---------- N2: merge and split ----------

def test_join_is_two_taps_and_split_is_two_taps_with_a_drag(browser, twin_server, brain):
    m = _house()
    page, errors, seen = _open(browser, twin_server, brain(m), m)
    page.click("#btn-rooms")
    rows = page.locator("#rooms-list li")
    sync_api.expect(rows.first).to_be_visible(timeout=5000)
    n = rows.count()
    rows.nth(0).get_by_role("button", name="Join").first.click()     # tap 1
    _shot(page, "join")
    rows.nth(1).get_by_role("button", name="Join with").click()      # tap 2 (the row; the map works too)
    page.wait_for_function(f"() => document.querySelectorAll('#rooms-list li').length === {n - 1}",
                           timeout=10000)
    assert len(_stored(None)) == 1
    # Split the joined room back: Split..., then one drag across it on the map.
    rows.nth(0).get_by_role("button", name="Split").click()          # tap 1
    _shot(page, "split")
    view = brain(m).get("/mission/rooms").json()
    room = view["rooms"][0]
    box = page.locator("#world-map").bounding_box()                  # in view: the mode scrolled to it
    assert box["y"] >= 0 and box["y"] + box["height"] <= 844, box
    sx = box["width"] / m["width"] / m["resolution_m"]
    cx = box["x"] + room["x_m"] * sx
    cy = box["y"] + room["y_m"] * sx
    page.mouse.move(cx, box["y"] + 3)
    page.mouse.down()                                                # tap 2: the drag, top to bottom
    page.mouse.move(cx, cy, steps=4)
    page.mouse.move(cx, box["y"] + box["height"] - 3, steps=4)
    page.mouse.up()
    for _ in range(50):                                              # the edit lands (a loaded desktop)
        if len(_stored(None)) == 2:
            break
        page.wait_for_timeout(100)
    assert len(_stored(None)) == 2, seen[-3:]
    assert not errors, errors
    page.context.close()


# ---------- where the editor is offered, and why it is closed ----------

def test_no_saved_map_says_survey_first_and_a_plain_map_offers_nothing(browser, twin_server, brain):
    m = _house()
    page, errors, _ = _open(browser, twin_server, brain(m, frame_id=None), m)
    page.click("#btn-rooms")
    sync_api.expect(page.locator("#rooms-closed")).to_contain_text("Survey first", timeout=5000)
    _shot(page, "closed")
    assert _clean(_bars(page)), _bars(page)
    page.context.close()
    plain = dict(m, map_id="sim-grid")
    page, errors, _ = _open(browser, twin_server, brain(plain), plain)
    page.wait_for_timeout(500)
    sync_api.expect(page.locator("#btn-rooms")).to_be_hidden()
    assert not errors, errors
    page.context.close()


# ---------- N7: the bars at 390 px, operator mode ----------

def test_the_editors_states_meet_3132s_bars(browser, twin_server, brain, tmp_path):
    m = _house()
    store = rn.RoomNames(os.environ["ROOM_NAMES_DIR"])
    attic = {"id": "attic", "by": "person", "name": "Attic", "runs": [[900, 900, 905]]}
    store._save(FRAME, {"version": 1, "frame_id": FRAME, "rooms": [attic], "history": []})
    page, errors, _ = _open(browser, twin_server, brain(m, seeds=_centre(brain, m)), m)
    page.click("#btn-rooms")
    sync_api.expect(page.locator("#rooms-unplaced li")).to_contain_text("Attic", timeout=5000)
    results = {"naming": _bars(page)}
    _shot(page, "unplaced")
    page.locator("#rooms-list li").nth(0).get_by_role("button", name="Join").first.click()
    results["join"] = _bars(page)
    page.get_by_role("button", name="Cancel").click()
    page.locator("#rooms-list li").nth(0).get_by_role("button", name="Split").click()
    results["split"] = _bars(page)
    bad = {k: v for k, v in results.items() if not _clean(v)}
    assert not bad, bad
    assert not errors, errors
    page.context.close()
