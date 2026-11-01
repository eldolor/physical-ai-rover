"""3.127 criterion 3: the twin names a nav2 still starting and a stopped
room labeller.

(a) A map tap the robot server refuses `nav2_starting` / `slam_not_ready`
(the bridge holding goals at a start, 3.105 / 3.50.D3) is a start-up wait,
told as information -- it was an error toast quoting the bridge's 503.
(b) A survey whose label source refused for good shows why
(`source_error`), not "nothing can name rooms here", which a brain with no
source at all also says. Written before the change and seen to fail
(docs/plans/ros-alignment/3.127-twin-survey-error-nav2-hold.md).

`UI_SHOTS=<dir>` keeps the phone-size screenshots (criterion 4).
"""

import json
import os

import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402

from tests.test_ui import (_SLAM_ERROR, _SLAM_MAP, _SLAM_POSE, browser,  # noqa: F401,E402
                           open_twin, open_with_brain, twin_server, _stub_world)
from tests.test_ui_survey import HEALTH as SURVEY_HEALTH, survey_status  # noqa: E402


def _json(obj):
    return json.dumps(obj)


def _shot(page, name):
    where = os.environ.get("UI_SHOTS")
    if where:
        os.makedirs(where, exist_ok=True)
        page.screenshot(path=os.path.join(where, name), full_page=False)


def _tap_refused(browser, twin_server, reply):
    page, errors = open_twin(browser, twin_server)
    _stub_world(page, _SLAM_MAP, _SLAM_POSE, _SLAM_ERROR)
    page.route("**/world/goal", lambda r: r.fulfill(
        status=200, content_type="application/json",
        body=_json(reply if r.request.method == "POST" else {"goal": None, "plan": []})))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_function(
        "() => (document.getElementById('map-readout') || {}).textContent.includes('tap the map')",
        timeout=8000)
    if os.environ.get("UI_SHOTS"):
        page.wait_for_timeout(3600)        # the "Connected" toast leaves first
    box = page.locator("#world-map").bounding_box()
    page.locator("#world-map").click(position={"x": box["width"] * 0.25, "y": box["height"] * 0.5})
    return page, errors


NAV2_REPLY = {"accepted": False, "reason": "nav2_starting", "retryable": True,
              "bridge_status": 503,
              "detail": "the ROS bridge did not take the goal: the ROS bridge answered 503 "
                        "to POST /goal: nav2 is not active: busy starting nav2, or hung"}
SLAM_REPLY = {"accepted": False, "reason": "slam_not_ready", "retryable": True,
              "bridge_status": 503,
              "detail": "the ROS bridge did not take the goal: the ROS bridge answered 503 "
                        "to POST /goal: SLAM is not ready (starting)"}


# ---------- (a) a map tap during the hold ----------

@pytest.mark.parametrize("reply, words", [
    (NAV2_REPLY, "nav2 is not active yet"),
    (SLAM_REPLY, "SLAM is not ready yet"),
])
def test_a_map_tap_during_a_start_up_hold_is_information_not_an_error(
        browser, twin_server, reply, words):
    """Red on dev 989ba9f8: an error toast, "Could not send the goal: the ROS
    bridge did not take the goal: ... 503 ..."."""
    page, errors = _tap_refused(browser, twin_server, reply)
    toast = page.locator(".toast.info", has_text=words)
    sync_api.expect(toast).to_have_count(1, timeout=3000)
    assert page.locator(".toast.err").count() == 0, page.locator(".toast.err").all_text_contents()
    assert "503" not in toast.text_content()
    # 3.134: only a start is this hold since 3.133 (red on dev d46c5173).
    assert "restart" not in toast.text_content().lower(), toast.text_content()
    if reply is NAV2_REPLY and os.environ.get("UI_SHOTS"):
        page.wait_for_timeout(400)         # past the toast's entrance
        _shot(page, "nav2-hold-toast.png")   # 3.134: one name for every section
    assert not errors, errors
    page.context.close()


def test_a_map_tap_refused_as_ros_down_is_still_an_error(browser, twin_server):
    """A guard: an outage keeps its error toast and the bridge's words."""
    page, errors = _tap_refused(browser, twin_server, {
        "accepted": False, "reason": "ros_unavailable", "retryable": True,
        "bridge_status": None, "detail": "the ROS bridge did not take the goal: bridge gone"})
    sync_api.expect(page.locator(".toast.err", has_text="bridge gone")).to_have_count(1,
                                                                                  timeout=3000)
    assert not errors, errors
    page.context.close()


# ---------- (b) the survey row ----------

def _with_survey(**fields):
    status = survey_status(rooms=("kitchen",), calls=3)
    status["survey"].update(fields)
    return status


def test_a_survey_whose_source_refused_says_why(browser, twin_server):
    """Red on dev: "nothing can name rooms here", the words for a brain with
    no source at all."""
    err = "/room: the vision service answered 401: unauthorised (check APP_SHARED_SECRET)"
    page, errors = open_with_brain(browser, twin_server, health=SURVEY_HEALTH,
                                   status=_with_survey(no_source=True, source_error=err))
    row = page.locator("#brain-tel-survey")
    sync_api.expect(row).to_contain_text("room naming stopped", timeout=5000)
    sync_api.expect(row).to_contain_text("401")
    sync_api.expect(row).not_to_contain_text("nothing can name rooms here")
    assert "alert" in (row.get_attribute("class") or "").split()
    if os.environ.get("UI_SHOTS"):
        page.wait_for_timeout(3600)        # the "Connected" toast leaves first
        row.scroll_into_view_if_needed()
        _shot(page, "3127-survey-source-error.png")
    assert not errors, errors
    page.context.close()


def test_a_long_source_error_is_cut_and_shown_as_text(browser, twin_server):
    """The reason is server text: shown as text, never markup, and cut so a
    200-character reason does not push the panel around."""
    err = "/room: <b>bold</b> " + "x" * 190
    page, errors = open_with_brain(browser, twin_server, health=SURVEY_HEALTH,
                                   status=_with_survey(no_source=True, source_error=err))
    row = page.locator("#brain-tel-survey")
    sync_api.expect(row).to_contain_text("<b>bold</b>", timeout=5000)
    assert row.locator("b").count() == 0
    assert len(row.text_content()) < len(err) + 40, row.text_content()


def test_a_cut_never_splits_a_character(browser, twin_server):
    """Review round 1: the 100th character an emoji (two UTF-16 units) --
    a unit cut left half of it before the ellipsis."""
    err = "x" * 99 + "\U0001F600" + "y" * 50
    page, errors = open_with_brain(browser, twin_server, health=SURVEY_HEALTH,
                                   status=_with_survey(no_source=True, source_error=err))
    row = page.locator("#brain-tel-survey")
    sync_api.expect(row).to_contain_text("\u2026", timeout=5000)
    assert "x" * 99 + "\U0001F600\u2026" in row.text_content()
    assert not errors, errors
    page.context.close()


def test_a_survey_with_no_source_at_all_reads_as_before(browser, twin_server):
    """A guard: `no_source` without a `source_error` keeps its words."""
    page, errors = open_with_brain(browser, twin_server, health=SURVEY_HEALTH,
                                   status=_with_survey(no_source=True, source_error=None))
    sync_api.expect(page.locator("#brain-tel-survey")).to_contain_text(
        "nothing can name rooms here", timeout=5000)
    assert not errors, errors
    page.context.close()
