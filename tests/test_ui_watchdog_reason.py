# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.164: the twin tells a hung brain from a mission that ended, and still
shows why the robot last stopped, at a phone's width. Criterion 4,
docs/plans/ros-alignment/3.164-watchdog-keeps-stop-reason.md; each seen to
fail on the page before the change.
"""

import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402

from tests.test_ui import _json, browser, open_twin, twin_server  # noqa: F401,E402

VETO = {"reason": "safety_distance", "detail": "Blocked FORWARD", "driver": "brain",
        "at": 1.0, "seconds_ago": 14.0}
STOP = {"reason": "person_stop", "detail": "twin-dpad pressed Stop", "driver": "twin-dpad",
        "at": 1.0, "seconds_ago": 12.0}
WATCHDOG = {"reason": "watchdog", "detail": "silence", "driver": "brain",
            "at": 2.0, "seconds_ago": 11.0}
TRIP = {"how": "silence", "driver": "brain", "quiet_s": 24.1, "at": 3.0, "seconds_ago": 8.0}


def _open(browser, twin_server, health):  # noqa: F811
    page, errors = open_twin(browser, twin_server)
    base = {"status": "ok", "seconds_since_last_command": 30.0, "watchdog_timeout_s": 1.0,
            "min_distance_cm": 20, "mode": "sim", "env_label": "", "driver": "brain",
            "authority_holder": None}
    page.route("**/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json({**base, **health})))
    page.click("#btn-settings")
    page.fill("#cfg-server-url", twin_server)
    page.click("#btn-connect")
    page.click('.tab-btn[data-tab="home"]')
    return page, errors


def _classes(locator):
    return (locator.get_attribute("class") or "").split()


def test_a_hung_brain_is_a_warning_and_the_last_reason_stays(browser, twin_server):  # noqa: F811
    page, errors = _open(browser, twin_server, {
        "last_refusal": WATCHDOG, "last_stop_reason": VETO,
        "watchdog": {"mission_live": True, "trip": TRIP, "trip_after_s": 24.0,
                     "last_fired": {"during": "mission", "driver": "brain", "at": 2.0,
                                    "seconds_ago": 11.0}}})
    row = page.locator("#brain-tel-watchdog")
    sync_api.expect(row).to_contain_text("went quiet mid-mission", timeout=5000)
    assert "warn" in _classes(row) and "alert" not in _classes(row), _classes(row)
    reason = page.locator("#brain-tel-refusal")
    sync_api.expect(reason).to_contain_text("SAFETY VETO")
    assert "alert" in _classes(reason), _classes(reason)
    page.locator("#brain-tel-watchdog").scroll_into_view_if_needed()
    assert not errors, errors
    page.close()


def test_a_quiet_end_is_calm_and_the_last_reason_stays(browser, twin_server):  # noqa: F811
    page, errors = _open(browser, twin_server, {
        "last_refusal": WATCHDOG, "last_stop_reason": STOP,
        "watchdog": {"mission_live": False, "trip": None, "trip_after_s": 24.0,
                     "last_fired": {"during": None, "driver": "brain", "at": 2.0,
                                    "seconds_ago": 11.0}}})
    row = page.locator("#brain-tel-watchdog")
    sync_api.expect(row).to_contain_text("(at rest)", timeout=5000)
    for colour in ("warn", "alert"):
        assert colour not in _classes(row), _classes(row)
    reason = page.locator("#brain-tel-refusal")
    sync_api.expect(reason).to_contain_text("STOPPED BY A PERSON")
    for colour in ("warn", "alert"):
        assert colour not in _classes(reason), _classes(reason)
    assert "WATCHDOG" not in reason.inner_text()
    assert not errors, errors
    page.close()


def test_a_quiet_end_with_nothing_refused_says_none(browser, twin_server):  # noqa: F811
    page, errors = _open(browser, twin_server, {
        "last_refusal": WATCHDOG, "last_stop_reason": None,
        "watchdog": {"mission_live": False, "trip": None, "trip_after_s": 24.0, "last_fired": None}})
    sync_api.expect(page.locator("#brain-tel-refusal")).to_have_text("none", timeout=5000)
    assert not errors, errors
    page.close()


def test_a_server_older_than_this_reads_as_it_did(browser, twin_server):  # noqa: F811
    page, errors = _open(browser, twin_server, {"last_refusal": WATCHDOG})
    reason = page.locator("#brain-tel-refusal")
    sync_api.expect(reason).to_contain_text("WATCHDOG", timeout=5000)
    assert "alert" in _classes(reason), _classes(reason)
    assert not errors, errors
    page.close()
