"""
tests/test_ui_lab.py

3.132 Stage 4: the Lab console -- the walks console and the metrics console
as one page (control/lab.html), in a real browser at a 390 px phone width.
Each test pins one of the stage's criteria
(`docs/plans/ros-alignment/3.132-ux-redesign.md`) and was seen to fail on
`dev` before the page was built. Routes are stubbed as in
`tests/test_ui_admin.py`, so nothing here makes a paid call.
"""

import json

import pytest

from tests.test_ui_admin import (  # noqa: F401  (fixtures)
    MODELS, PHONE, WALKS, admin_server, browser, confirm, open_metrics)

sync_api = pytest.importorskip("playwright.sync_api")

PAID = ("/evaluation", "/evaluate", "/replay")
UNSCORED = "red-backpack-qwen3-vl-235b-a22b-20260830-104120"
ESTIMATE = {"walk": "w", "action": "score", "calls": 12, "navigate_calls": 0,
            "judge_calls": 12, "judge_enabled": True, "upper_bound": False,
            "model_id": None, "judge_model_id": "us.anthropic.claude-opus-4-5-20251101-v1:0",
            "tokens_per_call": {"input": 1700, "output": 100, "recorded": False},
            "usd": 0.1452}


def _json(route, body, status=200):
    route.fulfill(status=status, content_type="application/json", body=json.dumps(body))


def _walks(**unscored):
    """WALKS with its second walk finished and not scored -- the walk that
    opening the console used to score by itself."""
    walks = json.loads(json.dumps(WALKS))
    walks["walks"][1].update(eval=None, finished=True, **unscored)
    return walks


def _lab(browser, admin_server, *, walks=None, path="/admin", secret="test", estimate=ESTIMATE,
         viewport=PHONE, routes=()):
    context = browser.new_context(viewport=viewport)
    if secret is not None:
        context.add_init_script(
            '(() => { try { localStorage.setItem("vp_admin_secret", %s); } catch (e) {} })();'
            % json.dumps(secret))
    page = context.new_page()
    requests = []
    page.on("request", lambda r: requests.append(r.method + " " + r.url))
    walks = walks or _walks()
    page.route("**/recording/models", lambda r: _json(r, MODELS))
    page.route("**/recording/walks", lambda r: _json(r, walks))
    page.route("**/recording/summary", lambda r: _json(r, {"rows": []}))
    page.route("**/stats", lambda r: _json(r, {"walks": 2, "frames": 53, "bytes": 1}))
    page.route("**/estimate*", lambda r: (
        _json(r, estimate) if isinstance(estimate, dict) else _json(r, {"detail": "x"}, 500)))
    page.route("**/metrics/summary*", lambda r: _json(r, {"runs": [], "days": 14}))
    for pattern, handler in routes:
        page.route(pattern, handler)
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(admin_server + path, wait_until="load")
    return page, requests, errors


def _paid(requests):
    return [r for r in requests if any(r.split("?")[0].endswith(p) for p in PAID)]


# ---------- criteria 4 and 5: nothing paid without a click and a confirm ----------


def test_opening_the_console_spends_nothing(browser, admin_server):
    """Criterion 4. Red on dev: the console scored every finished, unscored
    walk on opening (a paid judge run each, Stage 0's M6 = 1)."""
    page, requests, errors = _lab(browser, admin_server)
    page.wait_for_selector(".walk", timeout=10_000)
    page.wait_for_timeout(2500)
    assert _paid(requests) == []
    sync_api.expect(page.locator("#pending-bar")).to_contain_text("1 finished walk")
    sync_api.expect(page.locator(".walk", has_text=UNSCORED).locator(".score-badge")).to_have_text(
        "not scored")
    assert errors == []
    page.context.close()


def test_score_them_asks_with_the_cost_and_cancel_sends_nothing(browser, admin_server):
    page, requests, _ = _lab(browser, admin_server, routes=[
        ("**/evaluation", lambda r: _json(r, {"score": 44, "verdict": "mixed", "flags": []}))])
    page.wait_for_selector(".walk", timeout=10_000)
    page.click("#btn-score-pending")
    body = page.locator("#confirm-body")
    sync_api.expect(body).to_contain_text("12 paid calls")
    sync_api.expect(body).to_contain_text("$0.15")
    page.click("#confirm-cancel")
    page.wait_for_timeout(500)
    assert _paid(requests) == []
    page.click("#btn-score-pending")
    confirm(page)
    sync_api.expect(page.locator(".walk", has_text=UNSCORED).locator(".score-badge")).to_contain_text("44")
    assert len([r for r in _paid(requests) if UNSCORED in r]) == 1
    page.context.close()


@pytest.mark.parametrize("walk,label", [(UNSCORED, "Evaluate"), (WALKS["walks"][0]["walk"], "Re-score")])
def test_score_and_rescore_ask_first(browser, admin_server, walk, label):
    posted = []

    def evaluate(route):
        posted.append(route.request.url)
        _json(route, {"score": 51, "verdict": "mixed", "flags": []})

    page, requests, _ = _lab(browser, admin_server, routes=[("**/evaluate", evaluate)])
    row = page.locator(".walk", has_text=walk)
    row.locator("button", has_text=label).click()
    sync_api.expect(page.locator("#confirm-title")).to_contain_text("Score this walk")
    sync_api.expect(page.locator("#confirm-body")).to_contain_text(walk)
    page.click("#confirm-cancel")
    page.wait_for_timeout(400)
    assert posted == []
    row.locator("button", has_text=label).click()
    confirm(page)
    sync_api.expect(row.locator(".score-badge")).to_contain_text("51")
    assert len(posted) == 1
    page.context.close()


def test_a_replay_pick_asks_first(browser, admin_server):
    """Red on dev: the replay started the moment the pick changed."""
    sent = []
    est = dict(ESTIMATE, action="replay", calls=26, navigate_calls=14, judge_calls=12,
               upper_bound=True, model_id=MODELS["models"][2]["id"], usd=None)
    page, requests, _ = _lab(browser, admin_server, estimate=est, routes=[
        ("**/replays", lambda r: _json(r, {"replays": []})),
        ("**/replay", lambda r: (sent.append(r.request.url), _json(r, {
            "model_id": MODELS["models"][2]["id"], "prompt_variant": "default", "score": 30,
            "verdict": "poor", "agreement": 0.4, "errors": 0, "flags": []}))),
    ])
    sel = page.locator(".replay-select").first
    sync_api.expect(sel.locator("option")).to_have_count(len(MODELS["models"]) + 1)
    sel.select_option(MODELS["models"][2]["id"] + "|default")
    body = page.locator("#confirm-body")
    sync_api.expect(body).to_contain_text("Up to 26 paid calls")
    sync_api.expect(body).to_contain_text("Price unknown")
    page.click("#confirm-cancel")
    page.wait_for_timeout(400)
    assert sent == []
    sel.select_option(MODELS["models"][2]["id"] + "|default")
    confirm(page)
    sync_api.expect(page.locator(".walk").first).to_contain_text("score 30")
    assert len(sent) == 1
    page.context.close()


def test_an_estimate_that_fails_still_asks(browser, admin_server):
    """The estimate is advice; when it cannot be had the confirm says so and
    still waits for a person."""
    posted = []
    page, _, _ = _lab(browser, admin_server, estimate=None, routes=[
        ("**/evaluate", lambda r: (posted.append(1), _json(r, {"score": 1})))])
    page.locator(".walk", has_text=UNSCORED).locator("button", has_text="Evaluate").click()
    sync_api.expect(page.locator("#confirm-body")).to_contain_text("could not be worked out")
    page.wait_for_timeout(300)
    assert posted == []
    page.context.close()


def test_a_newer_question_replaces_an_older_one_and_stays(browser, admin_server):
    """Both reviews of Stage 4: replacing an open question closed the dialog,
    and the close event, firing later, cancelled the newer question too."""
    page, _, _ = _lab(browser, admin_server)
    page.wait_for_selector(".walk", timeout=10_000)
    page.evaluate("""() => {
      window.__answers = [];
      Lab.confirm("first", ["a"]).then(a => window.__answers.push(["first", a]));
      Lab.confirm("second", ["b"]).then(a => window.__answers.push(["second", a]));
    }""")
    page.wait_for_timeout(400)
    sync_api.expect(page.locator("#confirm")).to_be_visible()
    sync_api.expect(page.locator("#confirm-title")).to_have_text("second")
    assert page.evaluate("window.__answers") == [["first", False]]
    page.click("#confirm-ok")
    assert page.evaluate("window.__answers") == [["first", False], ["second", True]]
    page.context.close()


def test_a_second_click_while_the_estimate_loads_asks_once(browser, admin_server):
    """Review of Stage 4: nothing marked Evaluate busy while its estimate
    was out, so a second click raced a second question for the dialog."""
    held, posted = [], []
    page, _, _ = _lab(browser, admin_server, routes=[
        ("**/estimate*", lambda r: held.append(r)),
        ("**/evaluate", lambda r: (posted.append(1), _json(r, {"score": 5, "verdict": "poor", "flags": []}))),
    ])
    btn = page.locator(".walk", has_text=UNSCORED).locator("button", has_text="Evaluate")
    btn.click()
    sync_api.expect(btn).to_be_disabled()
    btn.click(force=True)
    page.wait_for_timeout(300)
    assert len(held) == 1
    _json(held[0], ESTIMATE)
    sync_api.expect(page.locator("#confirm")).to_be_visible()
    page.wait_for_timeout(400)
    sync_api.expect(page.locator("#confirm")).to_be_visible()
    confirm(page)
    sync_api.expect(page.locator(".walk", has_text=UNSCORED).locator(".score-badge")).to_contain_text("5")
    assert posted == [1]
    page.context.close()


def test_a_free_score_says_why(browser, admin_server):
    """Review of Stage 4: 0 calls read "the judge is off" even with it on."""
    est = dict(ESTIMATE, calls=0, judge_calls=0, usd=0.0)
    page, _, _ = _lab(browser, admin_server, estimate=est)
    page.locator(".walk", has_text=UNSCORED).locator("button", has_text="Evaluate").click()
    sync_api.expect(page.locator("#confirm-body")).to_contain_text("no frame here for the judge")
    page.context.close()


# ---------- criterion 3: one secret ----------


def test_a_secret_saved_by_the_metrics_console_connects_both_tabs(browser, admin_server):
    context = browser.new_context(viewport=PHONE)
    context.add_init_script(
        '(() => { try { localStorage.setItem("vp_metrics_secret", "m-secret"); } catch (e) {} })();')
    page = context.new_page()
    seen = []

    def walks(route):
        if route.request.headers.get("x-app-secret") == "m-secret":
            _json(route, WALKS)
        else:
            _json(route, {"detail": "Missing or invalid x-app-secret header."}, 401)

    def summary(route):
        seen.append(route.request.headers.get("x-app-secret"))
        _json(route, {"runs": [], "days": 14})

    page.route("**/recording/walks", walks)
    page.route("**/recording/models", lambda r: _json(r, MODELS))
    page.route("**/recording/summary", lambda r: _json(r, {"rows": []}))
    page.route("**/stats", lambda r: _json(r, {"walks": 2}))
    page.route("**/metrics/summary*", summary)
    page.goto(admin_server + "/admin")
    sync_api.expect(page.locator(".walk")).to_have_count(len(WALKS["walks"]))
    page.click("#tab-metrics")
    sync_api.expect(page.locator("#runs")).to_contain_text("No runs")
    assert seen == ["m-secret"]
    assert page.evaluate("localStorage.getItem('vp_admin_secret')") == "m-secret"
    context.close()


def test_a_pasted_secret_line_is_cleaned(browser, admin_server):
    page, _, _ = _lab(browser, admin_server, secret=None)
    page.fill("#secret", 'WALKS_SECRET="abc123"\n')
    page.click("#btn-connect")
    sync_api.expect(page.locator("#secret")).to_have_value("abc123")
    page.context.close()


def test_each_old_address_opens_its_own_tab(browser, admin_server):
    page, _, _ = _lab(browser, admin_server, path="/metrics")
    sync_api.expect(page.locator("#tab-metrics")).to_have_attribute("aria-selected", "true")
    sync_api.expect(page.locator("#panel-walks")).to_be_hidden()
    page.goto(admin_server + "/admin")
    sync_api.expect(page.locator("#tab-walks")).to_have_attribute("aria-selected", "true")
    sync_api.expect(page.locator("#panel-metrics")).to_be_hidden()
    page.context.close()


# ---------- criterion 9: the Stage 0 bugs ----------


def _frames_route(name, n=2):
    return (f"**/recording/walks/{name}", lambda r: _json(r, {
        "walk": name, "frames": [{"file": f"frame-{i:04d}.jpg", "bytes": 1000} for i in range(n)],
        "entries": [{"seq": i, "file": f"frame-{i:04d}.jpg", "navigate": {"action": "FORWARD"}}
                    for i in range(n)]}))


def test_typing_in_the_search_keeps_an_open_walk_open(browser, admin_server):
    """Red on dev: every keystroke rebuilt the rows, and an open walk closed."""
    name = WALKS["walks"][0]["walk"]
    page, _, _ = _lab(browser, admin_server, routes=[
        _frames_route(name),
        ("**/frames/*", lambda r: r.fulfill(status=200, content_type="image/jpeg", body=b"\xff\xd8\xff\xd9")),
    ])
    page.locator(".walk-name", has_text=name).click()
    sync_api.expect(page.locator(".frame")).to_have_count(2)
    page.fill("#search", "red-back")
    sync_api.expect(page.locator(".frame")).to_have_count(2)
    sync_api.expect(page.locator(".walk", has_text=name).locator(".walk-head-main")).to_have_attribute(
        "aria-expanded", "true")
    page.context.close()


def test_a_walks_name_is_a_button_the_keyboard_reaches(browser, admin_server):
    """Red on dev: the name was a div only a mouse could open."""
    name = WALKS["walks"][0]["walk"]
    page, _, _ = _lab(browser, admin_server, routes=[_frames_route(name)])
    head = page.locator(".walk", has_text=name).locator(".walk-head-main")
    sync_api.expect(head).to_have_attribute("aria-expanded", "false")
    assert head.evaluate("el => el.tagName") == "BUTTON"
    head.focus()
    page.keyboard.press("Enter")
    sync_api.expect(page.locator(".frame")).to_have_count(2)
    sync_api.expect(head).to_have_attribute("aria-expanded", "true")
    page.context.close()


def test_metrics_with_no_secret_asks_to_connect_and_shows_no_error(browser, admin_server):
    """Red on dev: a first open showed an error naming WALKS_SECRET."""
    def refuse(route):
        if route.request.headers.get("x-app-secret"):
            _json(route, WALKS)
        else:
            _json(route, {"detail": "Missing or invalid x-app-secret header."}, 401)

    page, _, _ = _lab(browser, admin_server, path="/metrics", secret=None,
                      routes=[("**/recording/walks", refuse),
                              ("**/metrics/summary*", lambda r: _json(r, {"detail": "no"}, 401))])
    sync_api.expect(page.locator("#metrics-prompt")).to_be_visible()
    page.wait_for_timeout(500)
    sync_api.expect(page.locator("#error")).to_be_hidden()
    assert "WALKS_SECRET" not in page.locator("body").inner_text()
    assert "err" not in (page.locator("#conn-status").get_attribute("class") or "")
    page.context.close()


def test_every_walk_control_has_a_name(browser, admin_server):
    page, _, _ = _lab(browser, admin_server)
    page.wait_for_selector(".walk", timeout=10_000)
    for name in (w["walk"] for w in WALKS["walks"]):
        sync_api.expect(page.get_by_role("checkbox", name="Select " + name)).to_have_count(1)
        sync_api.expect(page.get_by_role("combobox", name="Label for " + name)).to_have_count(1)
        sync_api.expect(page.get_by_role("combobox", name="Replay " + name, exact=False)).to_have_count(1)
    for sel in ("#search", "#sort", "#filter-label", "#filter-score", "#secret"):
        assert page.locator(sel).evaluate(
            "el => !!(el.labels && el.labels.length) || !!el.getAttribute('aria-label')"), sel
    page.context.close()


# ---------- criterion 10: the slideshow is a dialog ----------


def test_the_slideshow_keeps_focus_and_gives_it_back(browser, admin_server):
    name = WALKS["walks"][0]["walk"]
    page, _, _ = _lab(browser, admin_server, routes=[
        _frames_route(name),
        ("**/frames/*", lambda r: r.fulfill(status=200, content_type="image/jpeg", body=b"\xff\xd8\xff\xd9")),
    ])
    page.locator(".walk-name", has_text=name).click()
    opener = page.locator(".frame-open").nth(1)
    opener.click()
    show = page.locator("#slideshow")
    sync_api.expect(show).to_have_attribute("role", "dialog")
    sync_api.expect(show).to_have_attribute("aria-modal", "true")
    inside = "() => document.getElementById('slideshow').contains(document.activeElement)"
    assert page.evaluate(inside)
    for _ in range(8):
        page.keyboard.press("Tab")
        assert page.evaluate(inside)
    page.keyboard.press("Shift+Tab")
    assert page.evaluate(inside)
    page.keyboard.press("Escape")
    sync_api.expect(show).not_to_have_class("slideshow show")
    assert opener.evaluate("el => el === document.activeElement")
    page.context.close()


# ---------- criterion 11: filters ----------


def test_the_walks_filter_by_label_and_by_score(browser, admin_server):
    walks = _walks(label="bad")
    page, _, _ = _lab(browser, admin_server, walks=walks)
    sync_api.expect(page.locator(".walk")).to_have_count(2)
    page.select_option("#filter-score", "unscored")
    sync_api.expect(page.locator(".walk")).to_have_count(1)
    sync_api.expect(page.locator(".walk")).to_contain_text(UNSCORED)
    page.select_option("#filter-score", "")
    page.select_option("#filter-label", "none")
    sync_api.expect(page.locator(".walk")).to_have_count(1)
    sync_api.expect(page.locator(".walk")).to_contain_text(WALKS["walks"][0]["walk"])
    page.select_option("#filter-label", "bad")
    sync_api.expect(page.locator(".walk")).to_contain_text(UNSCORED)
    page.select_option("#filter-label", "good")
    sync_api.expect(page.locator(".walk")).to_have_count(0)
    sync_api.expect(page.locator("#walks")).to_contain_text("No walk matches")
    page.context.close()


# ---------- criterion 8: metrics at phone width ----------


def test_the_runs_are_cards_on_a_phone(browser, admin_server):
    """Red on dev: the table scrolled sideways inside its own region."""
    runs = [{"run_id": f"r{i}", "finished_at": 1760000000 + i, "policy": "tiered",
             "outcome": "found", "steps": 30, "target_object": "red backpack",
             "git_revision": "0123456789", "config": {"tier_cold_search_after": 6,
                                                      "tier_local_vlm_url": "http://127.0.0.1:8005"},
             "stats": {"cloud_ms": {"n": 4, "p50": 2100, "p90": 3300, "p99": 3900},
                       "perception_ms": {"p50": 61}, "cloud_calls": 4, "frames": 30}}
            for i in range(3)]
    page = open_metrics(browser, admin_server, lambda r: _json(r, {"runs": runs, "days": 14}))
    sync_api.expect(page.locator("td .pill")).to_have_count(3, timeout=5000)
    wrap = page.locator("#runs-wrap")
    assert wrap.evaluate("el => el.scrollWidth - el.clientWidth") <= 1
    overflow = page.evaluate(
        "() => document.documentElement.scrollWidth - document.documentElement.clientWidth")
    assert overflow <= 1
    cell = page.locator("#runs td[data-label='Config']").first
    box = cell.bounding_box()
    assert box["x"] >= 0 and box["x"] + box["width"] <= PHONE["width"] + 1, box
    label = cell.evaluate("el => getComputedStyle(el, '::before').content")
    assert "Config" in label, label
    page.context.close()
