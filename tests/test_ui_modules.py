"""3.132 Stage 2B criteria 4 and 6, in a real browser.

* **No new global (criterion 4).** app.js is an ES module, so its top level
  and every module's are module scope, not `window`. The twin's own
  `window` properties, against a blank page of the same origin, were
  exactly `twinStrings` before the split (measured on `dev` d65256b1, in
  both voices), and stay so.
* **Every module only declares (criterion 6).** Each module app.js reaches
  is imported in a module Web Worker: no `document`, no `window`, no
  `localStorage`. A module whose top level touches the page, or reads
  another module's binding before it is initialised, fails to import there.
  That is the mechanical half of the move rule (its static half, no module
  importing app.js, is tests/test_twin_modules.py).
"""

import pytest

pytest.importorskip("playwright")

from tests.test_ui import PHONE, browser, open_twin, twin_server  # noqa: F401,E402
from tests.twin_sources import TWIN, modules  # noqa: E402

OWN = "Object.getOwnPropertyNames(window)"


def own_globals(browser, url, *, engineer):
    """The page's own window properties beyond a blank same-origin page."""
    context = browser.new_context(viewport=PHONE)
    blank = context.new_page()
    blank.goto(url + "/manifest.json")
    base = set(blank.evaluate(OWN))
    context.close()
    page, errors = open_twin(browser, url, engineer=engineer)
    page.wait_for_timeout(500)
    extra = set(page.evaluate(OWN)) - base
    return page, errors, extra, base


@pytest.mark.parametrize("engineer", [True, False], ids=["engineer", "operator"])
def test_the_page_adds_no_global(browser, twin_server, engineer):
    page, errors, extra, base = own_globals(browser, twin_server, engineer=engineer)
    assert errors == [], errors
    assert extra == {"twinStrings"}, f"the twin's own globals changed: {sorted(extra)}"
    # Seen to fail: a planted global is what this comparison reports.
    page.evaluate("window.plantedGlobal = 1")
    assert set(page.evaluate(OWN)) - base == {"twinStrings", "plantedGlobal"}
    page.close()


IMPORT_IN_WORKER = """async (url) => {
  const src = `import(${JSON.stringify(url)}).then(
    () => postMessage("ok"),
    (e) => postMessage("error: " + (e && (e.stack || e.message) || e)));`;
  const blob = URL.createObjectURL(new Blob([src], {type: "text/javascript"}));
  return await new Promise((resolve) => {
    const w = new Worker(blob, {type: "module"});
    const done = (v) => { w.terminate(); resolve(v); };
    w.onmessage = (e) => done(e.data);
    w.onerror = (e) => { e.preventDefault(); done("error: " + (e.message || "worker failed")); };
    setTimeout(() => done("error: no answer in 10 s"), 10000);
  });
}"""


def import_in_worker(page, url):
    return page.evaluate(IMPORT_IN_WORKER, url)


def test_every_module_imports_without_a_page(browser, twin_server):
    page, errors = open_twin(browser, twin_server)
    assert errors == [], errors
    failed = {}
    for path in modules():
        url = twin_server + "/" + path.relative_to(TWIN).as_posix()
        answer = import_in_worker(page, url)
        if answer != "ok":
            failed[path.name] = answer
    assert not failed, f"modules whose top level does more than declare: {failed}"
    # Seen to fail: a module that touches the page, one that reads storage,
    # and one that reads a binding before it is initialised.
    for bad in ("document.title;", "localStorage.length;",
                "const a = b; const b = 1;"):
        url = page.evaluate("(s) => URL.createObjectURL(new Blob([s + '\\nexport {};'],"
                            " {type: 'text/javascript'}))", bad)
        assert import_in_worker(page, url).startswith("error"), bad
    assert import_in_worker(page, page.evaluate(
        "URL.createObjectURL(new Blob(['export function f() { return document; }'],"
        " {type: 'text/javascript'}))")) == "ok"
    page.close()
