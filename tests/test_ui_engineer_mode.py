# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.132 Stage 2A criteria 4-5: Engineer mode loses nothing, and the switch
applies live.

* Operator mode (the default) hides every `.eng-only` element and shows the
  `.op-only` wording beside it.
* Settings' switch reads "Engineer mode"; flipped, with no reload, every
  `.eng-only` element is shown again unless the page's own script hides it
  (its inline display, which still decides inside Engineer mode), with the
  wording the page had before Stage 2A; the voiced texts the script writes
  switch too. Flipped back, the plain view returns. The choice is saved
  under the old key, so nobody's setting changed.

`UI_SHOTS=<dir>` keeps the phone-size screenshots.
"""

import os

import pytest

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402

from tests.test_ui import (TIERED_BRAIN_HEALTH, _json, browser, open_twin,  # noqa: F401,E402
                           tiered_status, twin_server)

# Elements the operator view hides, and whether the page's script may also
# hide them (an inline display:none it toggles) -- those are judged by their
# script, not by Engineer mode.
STATE_JS = r"""() => {
  const out = [];
  document.querySelectorAll('.eng-only, .op-only').forEach(el => {
    const parentShown = !el.parentElement || el.parentElement.checkVisibility();
    out.push({cls: el.classList.contains('eng-only') ? 'eng' : 'op',
              id: el.id || el.className, inlineHidden: el.style.display === 'none',
              display: getComputedStyle(el).display, parentShown});
  });
  return out;
}"""

# Today's (pre-2A) wording that Engineer mode must still show.
ENGINEER_WORDS = ["Remote brain", "control/brain_server.py", "What the drills prove",   # Sim
                  "Brain service connection", "Robot server connection"]                  # Settings
OPERATOR_WORDS = ["Autopilot", "Autopilot connection", "The robot searches on its own"]


def _connect_brain(page, twin_server, running=True):
    page.route("**/brain-stub/health", lambda r: r.fulfill(
        status=200, content_type="application/json", body=_json(TIERED_BRAIN_HEALTH)))
    page.route("**/brain-stub/mission/status", lambda r: r.fulfill(
        status=200, content_type="application/json", body=_json(tiered_status(running=running))))
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(600)


def _flip(page):
    page.click("#btn-settings")
    page.locator("label[for=cfg-debug-readouts]").click()
    page.click("#btn-settings")


def test_engineer_mode_loses_nothing_and_applies_live(browser, twin_server):  # noqa: F811
    page, errors = open_twin(browser, twin_server, engineer=False)
    _connect_brain(page, twin_server)
    page.click("#btn-settings")
    sync_api.expect(page.locator("label[for=cfg-debug-readouts] .switch-title")).to_have_text("Engineer mode")
    page.click("#btn-settings")
    page.click('.tab-btn[data-tab="sim"]')

    # Operator mode: every eng-only element hidden, the op-only wording shown.
    states = page.evaluate(STATE_JS)
    assert len([s for s in states if s["cls"] == "eng"]) >= 25, "fewer eng-only elements than marked"
    shown_eng = [s["id"] for s in states if s["cls"] == "eng" and s["display"] != "none"]
    assert not shown_eng, f"operator mode shows engineer-only elements: {shown_eng}"
    # Rendered text: section labels are upper-cased by CSS, so compare folded.
    sim_text = page.locator('.tab-page[data-tab="sim"]').inner_text().lower()
    assert "autopilot" in sim_text and "remote brain" not in sim_text, sim_text[:300]
    assert page.locator("#brain-tel-driver").inner_text() != "brain"   # a person's words, never the id

    # Engineer mode, live: everything back, in today's words.
    _flip(page)
    page.click('.tab-btn[data-tab="sim"]')
    states = page.evaluate(STATE_JS)
    lost = [s["id"] for s in states if s["cls"] == "eng" and s["display"] == "none"
            and not s["inlineHidden"]]
    assert not lost, f"Engineer mode still hides: {lost}"
    shown_op = [s["id"] for s in states if s["cls"] == "op" and s["display"] != "none"]
    assert not shown_op, f"Engineer mode still shows operator-only wording: {shown_op}"
    text = page.locator("body").inner_text().lower()
    for words in ENGINEER_WORDS[:3]:
        assert words.lower() in text, words
    assert page.locator("#brain-tel-outcome").inner_text() == "running"     # the raw id, as before
    page.click("#btn-settings")
    settings = page.locator('.tab-page[data-tab="settings"]').inner_text().lower()
    for words in ENGINEER_WORDS[3:]:
        assert words.lower() in settings, words
    assert page.evaluate("localStorage.getItem('vp_debug_readouts')") == "1"
    if os.environ.get("UI_SHOTS"):
        os.makedirs(os.environ["UI_SHOTS"], exist_ok=True)
        page.screenshot(path=os.path.join(os.environ["UI_SHOTS"], "engineer-mode-settings-390.png"))
    page.click("#btn-settings")

    # And back: the plain view returns, with no reload.
    _flip(page)
    page.click('.tab-btn[data-tab="sim"]')
    assert page.locator("#brain-tel-outcome").inner_text() == "Searching"
    sim_text = page.locator('.tab-page[data-tab="sim"]').inner_text().lower()
    for words in OPERATOR_WORDS[::2]:
        assert words.lower() in sim_text, words
    assert page.evaluate("localStorage.getItem('vp_debug_readouts')") == "0"
    assert not errors, errors
    page.close()


def test_a_saved_paid_policy_is_said_and_a_drill_never_starts_in_operator_mode(browser, twin_server):  # noqa: F811
    """Both reviewers (3.132 Stage 2A): the policy and drill pickers are
    Engineer mode's, so a vision policy saved there ran paid calls with
    nothing on screen, and a drill picked there started from a person's
    Start. Operator mode says what a paid policy costs beside Start, and
    always starts a plain search."""
    page, errors = open_twin(browser, twin_server, engineer=False)
    page.evaluate("localStorage.setItem('vp_brain_policy', 'vision')")
    page.reload(wait_until="networkidle")
    starts = []
    page.route("**/brain-stub/mission/start", lambda r: (starts.append(r.request.post_data_json),
                                                         r.fulfill(status=200, content_type="application/json",
                                                                   body=_json({"started": True}))))
    _connect_brain(page, twin_server, running=False)
    page.click('.tab-btn[data-tab="sim"]')
    cost = page.locator("#brain-cost-hint")
    sync_api.expect(cost).to_be_visible()
    sync_api.expect(cost).to_contain_text("paid vision service at every step")
    # A drill left selected in Engineer mode (hidden here) is not sent.
    page.evaluate("document.getElementById('brain-fault').value = 'vision_error'")
    page.click("#btn-brain-mission")
    page.wait_for_timeout(500)
    assert starts and starts[0].get("fault") == "none", starts
    assert starts[0].get("policy") == "vision", starts      # the saved policy still runs -- and is said
    assert not errors, errors
    page.close()


def test_a_connection_detail_follows_every_switch(browser, twin_server):  # noqa: F811
    """Thermos on the review fixes: revoice() replayed each voiced status
    through setConnStatus(), which dropped it from the registry, so only the
    FIRST switch changed its voice -- switched back, Engineer mode's wording
    stayed in operator mode."""
    page, errors = open_twin(browser, twin_server, engineer=False)
    page.route("**/brain-dead/**", lambda r: r.abort())
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-dead")
    page.click("#btn-brain-connect")
    detail = page.locator("#brain-connection-status .conn-status-detail")
    sync_api.expect(detail).to_contain_text("Check that the autopilot is running", timeout=5000)
    toggle = page.locator("label[for=cfg-debug-readouts]")
    for want in ("control/brain_server.py", "Check that the autopilot is running",
                 "control/brain_server.py", "Check that the autopilot is running"):
        toggle.click()
        sync_api.expect(detail).to_contain_text(want, timeout=2000)
    assert not errors, errors
    page.close()
