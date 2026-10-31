"""3.132 Stage 2A criterion 3: every end state in plain words, in operator
mode (Engineer mode off, the default).

For each ending -- found, exhausted, out of budget, cancelled by Stop and by
the D-pad, arrived but unconfirmed, failed -- the toast the twin raises and
the panel's Mission row must carry no hit from the harness's M5 deny-list
(`evaluations/ux-3132/measure.py` JARGON, imported, not copied): no raw
outcome id, no plan code, no file name. A failure's error text comes from
the brain and may be jargon ("(B3.2)"), so operator mode does not echo it;
Engineer mode does. Written before the change and seen to fail
(`arrived_unconfirmed` reached the toast raw, and the failure echoed its
error).

`UI_SHOTS=<dir>` keeps the phone-size screenshots.
"""

import importlib.util
import os

import pytest

from tests.conftest import REPO_ROOT

pytest.importorskip("playwright")

from playwright import sync_api  # noqa: E402

from tests.test_ui import (END_REPORT, TIERED_BRAIN_HEALTH, _json, browser,  # noqa: F401,E402
                           open_twin, tiered_status, twin_server)


def _jargon():
    spec = importlib.util.spec_from_file_location(
        "ux_measure", REPO_ROOT / "evaluations" / "ux-3132" / "measure.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.JARGON


JARGON = _jargon()


def jargon_hits(text):
    hits = []
    for label, rx in JARGON:
        for m in rx.finditer(text):
            hits.append((label, (m.group(1) if m.groups() else m.group(0)).strip()))
    return hits


ENDINGS = {
    "found": {"end_reason": "Found red backpack in kitchen."},
    "exhausted": {"end_reason": "no reachable frontier left (3 on the map)"},
    "out_of_budget": {"end_reason": "time limit of 120 s reached (120.4 s)"},
    "cancelled-stop": {"outcome": "cancelled", "end_reason": "stopped by operator",
                       "cancelled_by": "stop"},
    "cancelled-dpad": {"outcome": "cancelled", "end_reason": "a person took over",
                       "cancelled_by": "dpad"},
    "arrived_unconfirmed": {"arrival": {"range_m": 0.38}, "late_confirmation": None},
    "failed": {"error": "vision service hung past its deadline (B3.2)"},
}


def ended(case):
    fields = dict(ENDINGS[case])
    outcome = fields.pop("outcome", case)
    s = dict(tiered_status(running=False))
    s.update({"outcome": outcome, "end_state": outcome, "error": None, "cancelled_by": None,
              "report": END_REPORT, "log_tail": [f"mission ended ({outcome})"]})
    s.update(fields)
    return s


@pytest.mark.parametrize("case", list(ENDINGS))
def test_each_ending_is_told_in_plain_words(browser, twin_server, case):  # noqa: F811
    current = {"body": tiered_status()}     # running, so the panel polls
    page, errors = open_twin(browser, twin_server)
    page.route("**/brain-stub/health", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(TIERED_BRAIN_HEALTH)))
    page.route("**/brain-stub/mission/status", lambda route: route.fulfill(
        status=200, content_type="application/json", body=_json(current["body"])))
    page.click("#btn-settings")
    page.fill("#cfg-brain-url", twin_server + "/brain-stub")
    page.click("#btn-brain-connect")
    page.wait_for_timeout(600)
    page.click('.tab-btn[data-tab="sim"]')

    page.wait_for_timeout(300)
    before = page.locator(".toast").count()      # the connect toast, if still up
    current["body"] = ended(case)
    page.wait_for_function(
        "n => document.querySelectorAll('.toast:not(.leaving)').length > n || "
        "document.querySelectorAll('.toast').length > n", arg=before, timeout=5000)
    page.wait_for_function(
        "() => !/running/i.test(document.getElementById('brain-tel-outcome').textContent)", timeout=5000)
    toast = page.locator(".toast").last          # the ending's own toast
    texts = {"toast": toast.inner_text(),
             "mission row": page.locator("#brain-tel-outcome").inner_text()}
    if os.environ.get("UI_SHOTS"):
        os.makedirs(os.environ["UI_SHOTS"], exist_ok=True)
        page.screenshot(path=os.path.join(os.environ["UI_SHOTS"], f"ending-{case}-390.png"))
    bad = {where: jargon_hits(t) for where, t in texts.items() if jargon_hits(t)}
    assert not bad, f"{case}: jargon in operator mode: {bad} (texts: {texts})"
    assert toast.inner_text().strip()
    assert not errors, errors
    page.close()
