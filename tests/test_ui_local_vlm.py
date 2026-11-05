# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_ui_local_vlm.py

3.152 criterion 8: the twin shows whether a tiered mission holds its local
VLM down (`status.tier.stats.local_vlm`). Engineer mode has a "Local VLM"
row; operator mode, while a mission runs with it down, a line in plain words
under the mission card. Driven at the 390 px phone viewport (test_ui.py's
`open_twin`), against a stubbed brain.
"""

import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402

from tests.test_ui import (TIERED_BRAIN_HEALTH, _json, browser,  # noqa: F401,E402 -- fixtures
                           open_twin, tiered_status, twin_server)


def _status(local_vlm, running=True):
    body = tiered_status(running=running)
    if local_vlm is not None:
        body["tier"]["stats"]["local_vlm"] = local_vlm
    return body


def _connect(page, twin_server, current):
    page.route("**/brain-stub/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(TIERED_BRAIN_HEALTH)))
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(current["body"])))
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(600)
    page.click('.tab-btn[data-tab="home"]')


DOWN = {"state": "down", "since": 1760000000.0, "checked_at": 1760000010.0, "probes": 2}


def test_engineer_row_names_the_local_vlm_state(browser, twin_server):
    current = {"body": _status({"state": "up", "since": None, "probes": 0})}
    page, errors = open_twin(browser, twin_server)
    _connect(page, twin_server, current)
    row = page.locator("#brain-tel-localvlm")
    sync_api.expect(row).to_have_text("up", timeout=5000)
    sync_api.expect(row).to_have_class("val safe")

    current["body"] = _status(DOWN)
    sync_api.expect(row).to_contain_text("down since", timeout=5000)
    sync_api.expect(row).to_contain_text("probed 2 times")
    sync_api.expect(row).to_have_class("val warn")

    current["body"] = _status({"state": "off"})
    sync_api.expect(row).to_contain_text("off", timeout=5000)

    # A brain before 3.152 sends no `local_vlm`: a dash, never "up".
    current["body"] = _status(None)
    sync_api.expect(row).not_to_contain_text("up", timeout=5000)
    assert not errors, errors
    page.close()


def test_operator_sees_plain_words_only_while_down(browser, twin_server):
    current = {"body": _status({"state": "up", "since": None, "probes": 0})}
    page, errors = open_twin(browser, twin_server, engineer=False)
    _connect(page, twin_server, current)
    note = page.locator("#mission-local-down")
    sync_api.expect(page.locator("#mission-card")).to_be_visible(timeout=5000)
    sync_api.expect(note).to_be_hidden()

    current["body"] = _status(DOWN)
    sync_api.expect(note).to_be_visible(timeout=5000)
    text = note.inner_text()
    assert "restarting" in text and "cloud" in text
    for jargon in ("VLM", "tier", "/health", "local_vlm", "probe"):
        assert jargon not in text, jargon
    # The Engineer row stays hidden in operator mode.
    sync_api.expect(page.locator("#brain-tel-localvlm")).to_be_hidden()

    # Once VE is back, or the mission has ended, the line goes.
    current["body"] = _status({"state": "up", "since": 1760000020.0, "probes": 3})
    sync_api.expect(note).to_be_hidden(timeout=5000)
    current["body"] = _status(DOWN, running=False)
    page.wait_for_timeout(1500)
    sync_api.expect(note).to_be_hidden()
    assert not errors, errors
    page.close()
