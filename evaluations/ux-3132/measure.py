"""
evaluations/ux-3132/measure.py -- 3.132's UX harness (Stage 0 and after).

Measures the project's three pages -- the twin, the walks console and the
metrics console -- at a 390 x 844 phone viewport, on the metrics 3.132 wrote
down before measuring (docs/plans/ros-alignment/3.132-ux-redesign.md):

  M1 taps per task (goal state verified), M2 axe-core serious/critical,
  M3 targets under 44 px, M4 sideways scroll, M5 jargon in visible text,
  M6 paid calls with no click.

It starts its own robot server and brain on this section's ports
(`plan_section.py env`), and serves `web-twin/` and `control/` from a plain
static server on another port -- the deployed case (CloudFront), so the
twin's first run has nothing filled in. The consoles' data and every paid
route are Playwright stubs: no recordings volume, no vision service, no
model call, no spend.

    python evaluations/ux-3132/measure.py --out evaluations/ux-3132/baseline

Writes <out>/results.json and <out>/shots/*.png.
"""

from __future__ import annotations

import argparse
import base64
import contextlib
import hashlib
import http.server
import io
import json
import os
import re
import socketserver
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
from functools import partial
from pathlib import Path

import httpx
from playwright.sync_api import sync_playwright

REPO = Path(__file__).resolve().parents[2]
PY = sys.executable
PHONE = {"width": 390, "height": 844}

# ---------- axe-core, pinned by version and npm's sha512 ----------

AXE_VERSION = "4.10.3"
AXE_URL = f"https://registry.npmjs.org/axe-core/-/axe-core-{AXE_VERSION}.tgz"
AXE_SHA512 = "Xm7bpRXnDSX2YE2YFfBk2FnF0ep6tmG7xPh8iHee8MIcrgq762Nkce856dYtJYLkuIoYZvGfTs/PbZhideTcEg=="
AXE_TAGS = ["wcag2a", "wcag2aa", "wcag21a", "wcag21aa", "wcag22aa"]


def axe_source() -> str:
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "vision-picar"
    path = cache / f"axe-{AXE_VERSION}.min.js"
    if path.exists():
        return path.read_text()
    blob = httpx.get(AXE_URL, timeout=60, follow_redirects=True).content
    got = base64.b64encode(hashlib.sha512(blob).digest()).decode()
    if got != AXE_SHA512:
        raise SystemExit(f"axe-core tarball sha512 mismatch: {got}")
    with tarfile.open(fileobj=io.BytesIO(blob)) as tf:
        src = tf.extractfile("package/axe.min.js").read().decode()
    cache.mkdir(parents=True, exist_ok=True)
    path.write_text(src)
    return src


# ---------- M5: the jargon deny-list (frozen with 3.132's criteria) ----------

JARGON = [
    ("file name", re.compile(r"\b[\w./-]+\.(?:py|yaml|yml|js|json|md|html)\b")),
    # Amendment 3: a section number is 3.NN not inside a larger number and
    # not followed by a unit, so a readout (3.42 m, 3.70 V) is not jargon.
    ("plan code", re.compile(r"\b[BPRSMNT]\d+(?:\.\d+)*[a-z]?\b"
                             r"|(?<![\d.])3\.\d{2,3}(?:\.[A-Z]\d*)?(?![\d.]|\s*[A-Za-z°%])")),
    ("snake_case", re.compile(r"\b[a-z][a-z0-9]*_[a-z0-9_]+\b")),
    ("ENV_VAR", re.compile(r"\b[A-Z][A-Z0-9]*_[A-Z0-9_]+\b")),
    ("API path", re.compile(r"(?:^|[\s(`'\"])(/[a-z][a-z_-]*(?:/[a-z_{}-]+)*)\b")),
    ("term", re.compile(
        r"\b(?:SLAM|nav2|ROS|frontier|CLIP|YOLOE?|watchdog|tier(?:ed)?|x-app-secret|uvicorn|"
        r"CORS|odometry|deliberation|corroboration|p50|p90|p99|drills?|preempt(?:ed)?|"
        r"brain service)\b", re.I)),
    ("HUD code", re.compile(r"\b(?:RMS|CLD|VF|STA|ERR|OUT|LATE|SEQ|OBS|VIS|ACT|TGT|VOTE)\b")),
]

# M6, amendment 3: every route that spends -- the judge (GET .../evaluation
# scores on a miss, POST .../evaluate), a replay, and the cloud's model
# routes (app.js VISION_SERVICE_ROUTES, plus /room) -- by method.
PAID = re.compile(r"^(?:GET .*/recording/walks/[^/]+/evaluation"
                  r"|POST .*/recording/walks/[^/]+/(?:evaluate|replay)"
                  r"|POST .*/(?:navigate|room|describe|analyze|guidance))(?:\?.*)?$")


def paid(requests):
    return [r for r in requests if PAID.search(r)]

VISION = "http://vision.stub"   # never resolved: every request to it is a route stub

# ---------- page-side probes ----------

JS_VISIBLE_TEXT = r"""() => {
  const out = [];
  const vis = el => el && el.checkVisibility({checkOpacity: true, checkVisibilityCSS: true});
  const w = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT);
  let n;
  while ((n = w.nextNode())) {
    const t = n.textContent.replace(/\s+/g, " ").trim();
    const p = n.parentElement;
    if (!t || !p || ["SCRIPT", "STYLE", "OPTION", "NOSCRIPT"].includes(p.tagName)) continue;
    if (vis(p)) out.push(t);
  }
  document.querySelectorAll("input[placeholder], textarea[placeholder]").forEach(el => {
    if (vis(el)) out.push(el.placeholder);
  });
  document.querySelectorAll("select").forEach(s => {
    if (vis(s)) s.querySelectorAll("option").forEach(o => out.push(o.textContent.trim()));
  });
  return out;
}"""

JS_SMALL_TARGETS = r"""() => {
  const sel = 'button, a[href], input:not([type=hidden]), select, textarea, summary, [role=button], [tabindex]';
  const out = [];
  document.querySelectorAll(sel).forEach(el => {
    if (el.matches('[tabindex="-1"]') && !el.matches('button, a[href], input, select, textarea, summary')) return;
    if (el.disabled) return;
    if (!el.checkVisibility({checkOpacity: true, checkVisibilityCSS: true})) return;
    const cs = getComputedStyle(el);
    if (cs.display === "inline") return;            // WCAG 2.5.8's inline-link exception
    const r = el.getBoundingClientRect();
    if (r.width < 0.5 || r.height < 0.5) return;
    if (r.width < 44 || r.height < 44) {
      const label = (el.getAttribute("aria-label") || el.textContent || el.value || "").replace(/\s+/g, " ").trim().slice(0, 40);
      out.push({id: el.id || null, tag: el.tagName.toLowerCase(), label,
                w: Math.round(r.width), h: Math.round(r.height)});
    }
  });
  return out;
}"""

# M7 (amendment 6): what still moves under prefers-reduced-motion: reduce.
JS_MOVING = r"""() => document.getAnimations().filter(a => {
    if (a.playState !== "running") return false;
    const t = a.effect && a.effect.getComputedTiming ? a.effect.getComputedTiming() : {};
    const d = typeof t.duration === "number" ? t.duration : 0;
    return d > 10 || t.iterations === Infinity;
  }).map(a => {
    const el = a.effect && a.effect.target;
    return {name: a.animationName || a.transitionProperty || a.constructor.name,
            target: el ? (el.id ? "#" + el.id : el.tagName.toLowerCase() + "." + [...el.classList].join(".")) : null};
  })"""

JS_SCROLL = r"""() => { const s = document.scrollingElement;
  return {scrollWidth: s.scrollWidth, clientWidth: s.clientWidth}; }"""


# ---------- servers ----------

def _port(name: str, default: int) -> int:
    return int(os.environ.get(name, default))


ROBOT_PORT = _port("PICAR_ROBOT_PORT", 10320)
BRAIN_PORT = _port("PICAR_BRAIN_PORT", 10321)
STATIC_PORT = _port("PICAR_UX_STATIC_PORT", ROBOT_PORT + 6)
ROBOT = f"http://127.0.0.1:{ROBOT_PORT}"
BRAIN = f"http://127.0.0.1:{BRAIN_PORT}"
STATIC = f"http://127.0.0.1:{STATIC_PORT}"


class _Deployed(http.server.BaseHTTPRequestHandler):
    """The static bucket as CloudFront serves it (amendment 7): exactly the
    keys, sources and content types in service/static/assets.json, read per
    request, so the pages load their files by the same relative paths as
    deployed (index.html at the root, the consoles at /admin and /metrics)."""

    def log_message(self, *a):
        pass

    def do_GET(self):
        key = self.path.split("?")[0].lstrip("/") or "index.html"
        assets = json.loads((REPO / "service/static/assets.json").read_text())["assets"]
        hit = next((a for a in assets if a["key"] == key), None)
        if hit is None:
            self.send_error(404)
            return
        body = (REPO / hit["src"]).read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", hit["type"])
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


def _port_busy(port: int) -> bool:
    import socket
    with socket.socket() as sk:
        sk.settimeout(0.5)
        return sk.connect_ex(("127.0.0.1", port)) == 0


@contextlib.contextmanager
def servers(logdir: Path):
    """The robot server and brain (sim, defaults), plus the static host.

    Refuses ports something already answers on: measuring another session's
    stack -- and stopping its mission between tasks -- would be worse than
    not running."""
    busy = [p for p in (ROBOT_PORT, BRAIN_PORT, STATIC_PORT) if _port_busy(p)]
    if busy:
        raise SystemExit(f"ports already in use: {busy} (another stack? see plan_section.py env)")
    run = Path(tempfile.mkdtemp(prefix="ux3132-"))
    env = {**os.environ, "PYTHONPATH": str(REPO), "ROBOT_URL": ROBOT,
           # A dead vision URL: nothing here may reach a paid service.
           "VISION_URL": "http://127.0.0.1:9", "INTERVENTION_DIR": str(run / "interventions")}
    env.pop("APP_SHARED_SECRET", None)
    logdir.mkdir(parents=True, exist_ok=True)
    procs, httpd = [], None
    try:
        socketserver.TCPServer.allow_reuse_address = True
        httpd = socketserver.ThreadingTCPServer(("127.0.0.1", STATIC_PORT), _Deployed)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        for mod, port in (("robot.server:app", ROBOT_PORT), ("control.brain_server:app", BRAIN_PORT)):
            log = open(logdir / f"{mod.split('.')[0]}.log", "w")
            procs.append(subprocess.Popen(
                [PY, "-m", "uvicorn", mod, "--host", "127.0.0.1", "--port", str(port),
                 "--log-level", "warning"], cwd=run, env=env, stdout=log, stderr=subprocess.STDOUT))
        for proc, url in zip(procs, (ROBOT, BRAIN)):
            deadline = time.monotonic() + 60
            while True:
                if proc.poll() is not None:
                    raise SystemExit(f"{url} exited with {proc.returncode}; logs in {logdir}")
                with contextlib.suppress(httpx.HTTPError):
                    if httpx.get(url + "/health", timeout=1).status_code == 200:
                        break
                if time.monotonic() > deadline:
                    raise SystemExit(f"{url} not healthy; logs in {logdir}")
                time.sleep(0.2)
        yield
    finally:
        if httpd is not None:
            httpd.shutdown()
            httpd.server_close()
        for proc in procs:
            proc.terminate()
        for proc in procs:
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()


def brain_status() -> dict:
    """The brain's own view, kept beside a measurement as evidence."""
    with contextlib.suppress(httpx.HTTPError, ValueError):
        st = httpx.get(BRAIN + "/mission/status", timeout=5).json()
        return {k: st.get(k) for k in ("running", "outcome", "cancelled_by", "step", "last_action")}
    return {}


def reset_robot_and_brain():
    """Between tasks: no mission running, the person's Stop latch lifted."""
    with contextlib.suppress(httpx.HTTPError):
        httpx.post(BRAIN + "/mission/stop", timeout=10)
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        with contextlib.suppress(httpx.HTTPError, ValueError):
            st = httpx.get(BRAIN + "/mission/status", timeout=5).json()
            if not st.get("running"):
                break
        time.sleep(0.3)
    with contextlib.suppress(httpx.HTTPError):
        httpx.post(ROBOT + "/autonomy/resume", timeout=5)
    # A person's D-pad claim lapses watchdog_timeout_s (1 s) after its last
    # command or verb; a mission started inside it is refused and ends
    # "D-pad took over" (3.132 finding F2). No task may inherit another's.
    time.sleep(3.0)


# ---------- console stubs ----------

MODELS = {"default": "us.anthropic.claude-opus-4-5-20251101-v1:0",
          "models": [{"id": "us.anthropic.claude-opus-4-5-20251101-v1:0", "label": "Claude Opus 4.5"},
                     {"id": "amazon.nova-lite-v1:0", "label": "Nova Lite"}]}
WALK_A = "red-backpack-opus-4-5-20260830-104705"
WALK_B = "red-backpack-nova-lite-20260830-104120"
WALKS = {"recording_dir": "/tmp/recordings", "walks": [
    {"walk": WALK_A, "frames": 3, "bytes": 210_000, "label": None, "finished": True,
     "model_id": MODELS["default"], "recorded_at": 1788000000.0,
     "eval": {"score": 72, "verdict": "good", "flags": ["oscillating"], "basis": "judge+metrics",
              "model_id": MODELS["default"]}, "replays": []},
    # Finished and unscored: what the console scores on open (M6).
    {"walk": WALK_B, "frames": 3, "bytes": 190_000, "label": None, "finished": True,
     "model_id": "amazon.nova-lite-v1:0", "recorded_at": 1787999000.0, "replays": []},
]}
DETAIL = {"frames": [{"file": f"frame_{i:04d}.jpg", "bytes": 70_000} for i in range(3)],
          "entries": [{"file": f"frame_{i:04d}.jpg", "action": "FORWARD", "model_id": MODELS["default"],
                       "reasoning": "The backpack is ahead, slightly left.",
                       "usage": {"input_tokens": 1500, "output_tokens": 80}} for i in range(3)]}
JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8UHRofHh0aHBwgJC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/"
    "wAALCAABAAEBAREA/8QAFAABAAAAAAAAAAAAAAAAAAAACf/EABQQAQAAAAAAAAAAAAAAAAAAAAD/2gAIAQEAAD8AKp//2Q==")
NOW = time.time()
RUNS = {"days": 14, "count": 3, "runs": [
    {"run_id": f"run-{i}", "started_at": NOW - 3600 * (i + 1), "finished_at": NOW - 3600 * i - 600,
     "policy": "tiered", "outcome": ["found", "exhausted", "found"][i], "steps": 40 + i,
     "target_object": "red backpack", "config": {"tier_cold_search_after": 6},
     "stats": {"cloud_ms": {"p50": 2100 + 50 * i, "p90": 3200, "p99": 4100},
               "perception_ms": {"p50": 61, "p90": 110}, "cloud_calls": 5 + i, "frames": 120,
               "perception": {"detected": 30, "missed": 90}}} for i in range(3)]}


def stub_consoles(page):
    def denied(route):
        if route.request.headers.get("x-app-secret"):
            return False
        route.fulfill(status=401, content_type="application/json",
                      body=json.dumps({"detail": "missing or wrong x-app-secret"}))
        return True

    def j(body, status=200):
        def h(route):
            if not denied(route):
                route.fulfill(status=status, content_type="application/json", body=json.dumps(body))
        return h


    page.route("**/health", lambda r: r.fulfill(status=200, content_type="application/json",
                                                body=json.dumps({"status": "ok"})))
    page.route("**/stats", j({"walks": 2, "frames": 6, "bytes": 400_000}))
    page.route("**/recording/models", j(MODELS))
    page.route("**/recording/summary", j({"rows": []}))
    page.route(re.compile(r".*/recording/walks(\?.*)?$"), j(WALKS))
    page.route(re.compile(r".*/recording/walks/[^/]+$"), lambda r: (
        j({"deleted": True})(r) if r.request.method == "DELETE" else j(DETAIL)(r)))
    page.route("**/frames/*", lambda r: r.fulfill(status=200, content_type="image/jpeg", body=JPEG))
    page.route("**/replays", j({"replays": []}))
    page.route("**/evaluation", j({"score": 61, "verdict": "mixed", "basis": "judge+metrics"}))
    page.route("**/evaluate", j({"score": 61, "verdict": "mixed", "basis": "judge+metrics"}))
    page.route("**/replay", j({"started": True}))
    page.route(re.compile(r".*/metrics/summary.*"), j(RUNS))


CORS = {"access-control-allow-origin": "*", "access-control-allow-headers": "*",
        "access-control-allow-methods": "GET, POST, OPTIONS"}


def stub_vision(page):
    """The cloud, as the real one answers a browser: CORS and preflights."""
    def h(route):
        if route.request.method == "OPTIONS":
            route.fulfill(status=204, headers=CORS)
            return
        u = route.request.url
        if u.endswith("/health"):
            body = {"status": "ok"}
        elif "/models" in u:
            body = MODELS
        else:
            body = {"action": "FORWARD", "reasoning": "stub", "target_visible": False,
                    "direction": "forward", "model_id": MODELS["default"]}
        route.fulfill(status=200, content_type="application/json", body=json.dumps(body), headers=CORS)
    page.route(VISION + "/**", h)


# ---------- measuring one state ----------

# Amendment 5: a state is measured settled -- no toast on screen (they live
# 3.2-5.4 s) and no finite animation running -- so a fade caught half way
# (attempt 1's Settings contrast hit) cannot make two runs differ.
JS_SETTLED = r"""() => {
  const stack = document.getElementById("toast-stack");
  if (stack && stack.children.length) return false;
  return document.getAnimations().every(a => {
    const t = a.effect && a.effect.getTiming ? a.effect.getTiming() : {};
    return t.iterations === Infinity || a.playState !== "running";
  });
}"""


def settle(page):
    """Amendment 8: after a scheme change, wait out the colour transitions it
    starts -- axe reading a button half way through its fade measures a
    colour neither theme has."""
    with contextlib.suppress(Exception):
        page.wait_for_function(JS_SETTLED, timeout=5_000)


def measure_state(page, name, axe, shots: Path, light_too=True) -> dict:
    page.wait_for_timeout(600)
    settled = True
    try:
        page.wait_for_function(JS_SETTLED, timeout=10_000)
    except Exception:  # noqa: BLE001 -- measured anyway, and marked so
        settled = False
    page.screenshot(path=str(shots / f"{name}.png"), full_page=True)
    if light_too:
        page.emulate_media(color_scheme="light")
        page.wait_for_timeout(200)
        settle(page)
        page.screenshot(path=str(shots / f"{name}.light.png"), full_page=True)
        page.emulate_media(color_scheme="dark")
        page.wait_for_timeout(200)
        settle(page)
    if not page.evaluate("() => typeof window.axe !== 'undefined'"):
        page.add_script_tag(content=axe)
    res = page.evaluate("""async (tags) => {
      const r = await axe.run(document, {runOnly: {type: 'tag', values: tags}, resultTypes: ['violations']});
      return r.violations.filter(v => v.impact === 'serious' || v.impact === 'critical')
        .map(v => ({id: v.id, impact: v.impact, nodes: v.nodes.map(n => n.target.join(' '))}));
    }""", AXE_TAGS)
    # M2L (amendment 6): axe's contrast rule under the light scheme.
    page.emulate_media(color_scheme="light")
    page.wait_for_timeout(300)
    settle(page)
    light = page.evaluate("""async () => {
      const r = await axe.run(document, {runOnly: {type: 'rule', values: ['color-contrast']},
                                         resultTypes: ['violations']});
      return r.violations.filter(v => v.impact === 'serious' || v.impact === 'critical')
        .flatMap(v => v.nodes.map(n => n.target.join(' ')));
    }""")
    page.emulate_media(color_scheme="dark")
    page.wait_for_timeout(200)
    settle(page)
    # M7: under reduced motion, after the page has had time to react.
    page.emulate_media(reduced_motion="reduce")
    page.wait_for_timeout(400)
    moving = page.evaluate(JS_MOVING)
    page.emulate_media(reduced_motion="no-preference")
    texts = page.evaluate(JS_VISIBLE_TEXT)
    hits = set()
    for t in texts:
        for label, rx in JARGON:
            for m in rx.finditer(t):
                hits.add((label, (m.group(1) if m.groups() else m.group(0)).strip()))
    small = page.evaluate(JS_SMALL_TARGETS)
    sc = page.evaluate(JS_SCROLL)
    return {
        "settled": settled,
        "M2_axe": {"rules": len(res), "nodes": sum(len(v["nodes"]) for v in res), "violations": res},
        "M2L_light_contrast": {"nodes": len(light), "targets": light},
        "M7_reduced_motion": {"count": len(moving), "moving": moving},
        "M3_small_targets": {"count": len(small), "elements": small},
        "M4_sideways_scroll": sc["scrollWidth"] > sc["clientWidth"] + 1, "scroll": sc,
        "M5_jargon": {"count": len(hits), "hits": sorted([list(h) for h in hits])},
    }


# ---------- the twin ----------

TWIN = STATIC + "/"          # the deployed keys (amendment 7)
ADMIN = STATIC + "/admin"
METRICS = STATIC + "/metrics"


class Taps:
    def __init__(self, page):
        self.page, self.taps, self.fields, self.path = page, 0, 0, []

    def tap(self, selector, what=None):
        self.page.locator(selector).first.click()
        self.taps += 1
        self.path.append(what or selector)

    def select(self, selector, value, what=None):
        self.page.locator(selector).first.select_option(value)
        self.taps += 1
        self.path.append(what or f"{selector}={value}")

    def fill(self, selector, text, what=None):
        self.page.locator(selector).first.fill(text)
        self.fields += 1
        self.path.append(what or f"type into {selector}")

    def result(self, ok, **extra):
        return {"ok": ok, "taps": self.taps, "fields": self.fields, "path": self.path, **extra}


CONNECTED = "() => document.getElementById('connection-status').classList.contains('is-ok')"
BRAIN_READY = "() => !document.getElementById('btn-brain-mission').disabled"


def twin_context(browser):
    return browser.new_context(viewport=PHONE, color_scheme="dark",
                               permissions=["camera"], service_workers="block")


def configure_twin(page, *, policy="frontier"):
    """What a person has done once: the setup link (robot, brain, cloud), the
    Sim panel's free policy. Not counted in any task."""
    q = f"?serverUrl={ROBOT}&brainUrl={BRAIN}&visionUrl={VISION}"
    page.goto(TWIN + q)
    page.wait_for_function(CONNECTED, timeout=20_000)
    page.click('.tab-btn[data-tab="sim"]')
    page.wait_for_selector("#btn-brain-mission:not([disabled])", timeout=20_000)
    # Amendment 9: the policy is saved as the page saves it (vp_brain_policy).
    # Since Stage 2A its picker is Engineer mode's, and choosing it there is a
    # one-time setup, not a task.
    page.evaluate("p => localStorage.setItem('vp_brain_policy', p)", policy)
    page.click('.tab-btn[data-tab="guide"]')
    page.goto(TWIN)
    wait_twin_ready(page)


def wait_twin_ready(page):
    page.wait_for_function(CONNECTED, timeout=20_000)
    page.wait_for_function(BRAIN_READY, timeout=20_000)


def new_twin_page(browser, *, configured=True, engineer=False):
    """A fresh phone context. Returns (context, page, requests): every request
    the page makes, as "METHOD url", from its first load -- M6 reads these."""
    ctx = twin_context(browser)
    page = ctx.new_page()
    reqs = []
    page.on("request", lambda r: reqs.append(f"{r.method} {r.url}"))
    stub_vision(page)
    if configured:
        configure_twin(page)
        if engineer:
            page.click("#btn-settings")
            page.locator("label[for=cfg-debug-readouts]").first.click()
            page.click("#btn-settings")
            page.goto(TWIN)
            wait_twin_ready(page)
    else:
        page.goto(TWIN)
        page.wait_for_timeout(1500)
    return ctx, page, reqs


def watch(page, pred):
    seen = []
    page.on("request", lambda r: seen.append(r) if pred(r) else None)
    return seen


def wait_for(page, seen, timeout_s=10.0):
    """Polls through Playwright: a time.sleep here would block the sync API's
    event delivery, and `seen` would never fill."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if seen:
            return True
        page.wait_for_timeout(100)
    return bool(seen)


def _is(path, method="POST"):
    return lambda r: r.method == method and r.url.split("?")[0].endswith(path)


# Not in any house: the mission runs on (explores) instead of finding it in
# seconds, so the Stop and status tasks meet a mission that is still running.
ABSENT = "purple unicorn"


def start_mission_quietly(page):
    """A mission the person started on this page (uncounted), then back to
    the default tab (amendment 2). Not through the API: a connected, idle
    twin does not notice a mission started elsewhere (finding F1)."""
    page.click('.tab-btn[data-tab="sim"]')
    page.fill("#brain-target", ABSENT)
    page.click("#btn-brain-mission")
    page.wait_for_function(
        "() => /stop/i.test(document.getElementById('btn-brain-mission').textContent)", timeout=20_000)
    page.wait_for_timeout(1500)
    page.click('.tab-btn[data-tab="guide"]')
    page.evaluate("() => window.scrollTo(0, 0)")


def run_task(out, key, opener, fn):
    """One task, isolated: its own context; a failure anywhere -- setup
    included -- is that task's recorded failure, never the run's."""
    ctx, t = None, Taps(None)
    try:
        ctx, page, reqs = opener()
        t = Taps(page)
        out[key] = fn(page, reqs, t)
    except Exception as e:  # noqa: BLE001 -- reported, the run goes on
        out[key] = t.result(False, reason=repr(e)[:300])
    finally:
        if ctx is not None:
            ctx.close()


# ---------- the twin's tasks ----------

def t1_first_connect(page, reqs, t):
    if not page.locator("#cfg-server-url").first.is_visible():
        t.tap("#btn-settings", "Settings (gear)")
    t.fill("#cfg-server-url", ROBOT, "type the robot server's URL")
    t.tap("#btn-connect", "Connect")
    page.wait_for_function(CONNECTED, timeout=15_000)
    return t.result(True)


def start_task(job):
    def fn(page, reqs, t):
        seen = watch(page, _is("/mission/start"))
        t.tap('.tab-btn[data-tab="sim"]', "Sim tab")
        if job == "survey":
            t.select("#brain-job", "survey", "Job: Survey the house")
        t.tap("#btn-brain-mission", "Start")
        sent = wait_for(page, seen)
        body, status = {}, None
        if sent:
            body = json.loads(seen[0].post_data or "{}")
            resp = seen[0].response()
            status = resp.status if resp else None
        # T2: answered 200. T3 (amendment 1): the twin's request with job
        # survey; the plain sim's brain refuses a survey (no nav2).
        good = (status == 200) if job == "find" else (body.get("job") == "survey")
        return t.result(sent and good, request=body, status=status)
    return fn


def t4_stop(page, reqs, t):
    start_mission_quietly(page)
    seen = watch(page, lambda r: r.method == "POST"
                 and (r.url.endswith("/mission/stop") or r.url.endswith("/stop")))
    t.tap('.tab-btn[data-tab="sim"]', "Sim tab")
    t.tap("#btn-brain-mission", "Stop")
    ok = wait_for(page, seen)
    return t.result(ok, request=seen[0].url if seen else None)


def t5_forward(page, reqs, t):
    seen = watch(page, lambda r: r.method == "POST" and r.url.endswith("/action")
                 and "FORWARD" in (r.post_data or ""))
    t.tap('.tab-btn[data-tab="sim"]', "Sim tab")
    t.tap("#btn-forward", "D-pad up")
    return t.result(wait_for(page, seen))


def t6_status(page, reqs, t):
    start_mission_quietly(page)
    t.tap('.tab-btn[data-tab="sim"]', "Sim tab")
    page.wait_for_timeout(1000)
    page.evaluate("() => window.scrollTo(0, 0)")
    el = page.locator("#brain-tel-outcome")
    box = el.bounding_box()
    brain = brain_status()
    if box is None:      # not rendered at all: no scroll reaches it
        return t.result(False, scroll_px=None, reason="status element not rendered", brain=brain)
    bar = page.evaluate("() => { const b = document.querySelector('.tab-bar'); "
                        "return b ? b.getBoundingClientRect().height : 0; }")
    # The fixed tab bar covers the bottom of the viewport.
    scroll = max(0, round(box["y"] + box["height"] - (PHONE["height"] - bar)))
    text = el.inner_text().strip()
    # The state shown must be the running mission's, not the idle default --
    # in either voice (amendment 9: operator mode says "Searching").
    # Exactly a running word at the start -- "Not running" (operator idle)
    # must not pass (review).
    ok = bool(brain.get("running")) and re.match(r"(running|searching)\b", text.strip().lower()) is not None
    return t.result(ok, scroll_px=scroll, text=text[:60], brain=brain)


def t7_record(page, reqs, t):
    page.route(BRAIN + "/recording/**", lambda r: r.fulfill(
        status=200, content_type="application/json", body=json.dumps({"saved": True, "walk": "stub"})))
    seen = watch(page, lambda r: "/recording/frame" in r.url)
    t.tap("#btn-mode-robot", "Robot view")
    t.tap("label[for=cfg-record-walk]", "Record this walk")
    t.tap("#btn-guidance", "Start")
    with contextlib.suppress(Exception):
        page.wait_for_selector("#btn-onboarding-start", state="visible", timeout=3000)
        t.tap("#btn-onboarding-start", "Got it, start")
    return t.result(wait_for(page, seen, 15))


def twin_tasks(browser, out):
    configured = partial(new_twin_page, browser)
    first_run = partial(new_twin_page, browser, configured=False)
    for key, opener, fn in (("T1", first_run, t1_first_connect),
                            ("T2", configured, start_task("find")),
                            ("T3", configured, start_task("survey")),
                            ("T4", configured, t4_stop),
                            ("T5", configured, t5_forward),
                            ("T6", configured, t6_status),
                            ("T7", configured, t7_record)):
        reset_robot_and_brain()
        run_task(out, key, opener, fn)
    reset_robot_and_brain()


# ---------- states ----------

def measure_into(states, page, name, axe, shots):
    try:
        states[name] = measure_state(page, name, axe, shots)
        states[name]["brain"] = brain_status()
    except Exception as e:  # noqa: BLE001 -- reported, the run goes on
        states[name] = {"error": repr(e)[:300]}


def twin_states(browser, axe, shots):
    states, m6 = {}, {}

    def guarded(names, fn):
        ctx = None
        try:
            ctx = fn()
        except Exception as e:  # noqa: BLE001
            for n in names:
                states.setdefault(n, {"error": repr(e)[:300]})
        finally:
            if ctx is not None:
                ctx.close()

    def first_run():
        ctx, page, _ = new_twin_page(browser, configured=False)
        measure_into(states, page, "twin.first_run", axe, shots)
        return ctx

    def main_states():
        ctx, page, reqs = new_twin_page(browser)
        reqs.clear()
        page.goto(TWIN)                    # M6: open with everything saved, no input
        wait_twin_ready(page)
        page.wait_for_timeout(8000)
        m6["twin"] = paid(reqs)
        measure_into(states, page, "twin.guide", axe, shots)
        page.click("#btn-mode-robot")
        measure_into(states, page, "twin.guide_robot_view", axe, shots)
        page.click("#btn-mode-guide")
        page.click('.tab-btn[data-tab="sim"]')
        page.wait_for_timeout(1500)
        measure_into(states, page, "twin.sim_idle", axe, shots)
        start_mission_quietly(page)
        page.click('.tab-btn[data-tab="sim"]')
        page.wait_for_timeout(1500)
        measure_into(states, page, "twin.sim_mission", axe, shots)
        reset_robot_and_brain()
        page.click("#btn-settings")
        measure_into(states, page, "twin.settings", axe, shots)
        return ctx

    def engineer():
        ctx, page, _ = new_twin_page(browser, engineer=True)
        page.click('.tab-btn[data-tab="sim"]')
        page.wait_for_timeout(1500)
        measure_into(states, page, "twin.engineer_sim_idle", axe, shots)
        return ctx

    guarded(["twin.first_run"], first_run)
    reset_robot_and_brain()
    guarded(["twin.guide", "twin.guide_robot_view", "twin.sim_idle", "twin.sim_mission",
             "twin.settings"], main_states)
    reset_robot_and_brain()
    guarded(["twin.engineer_sim_idle"], engineer)
    reset_robot_and_brain()
    return states, m6


# ---------- the consoles ----------

def console_page(browser, *, secret: bool, url=None):
    ctx = browser.new_context(viewport=PHONE, color_scheme="dark")
    if secret:
        ctx.add_init_script('(() => { try { localStorage.setItem("vp_admin_secret", "test"); } catch (e) {} })();')
    page = ctx.new_page()
    reqs = []
    page.on("request", lambda r: reqs.append(f"{r.method} {r.url}"))
    stub_consoles(page)
    if url:
        page.goto(url)
        if url == ADMIN and secret:
            page.wait_for_selector(".walk", timeout=10_000)
    return ctx, page, reqs


def console_states(browser, axe, shots):
    states, m6 = {}, {}

    def one(name, url, secret, then=None, m6_key=None):
        ctx = None
        try:
            ctx, page, reqs = console_page(browser, secret=secret, url=url)
            if m6_key:
                page.wait_for_timeout(8000)
                m6[m6_key] = paid(reqs)
            else:
                page.wait_for_timeout(1500)
            if then:
                then(page)
            measure_into(states, page, name, axe, shots)
        except Exception as e:  # noqa: BLE001
            states[name] = {"error": repr(e)[:300]}
        finally:
            if ctx is not None:
                ctx.close()

    def expand(page):
        page.locator(".walk-name").first.click()
        page.wait_for_selector(".frame", timeout=10_000)

    one("admin.not_connected", ADMIN, False)
    one("admin.walks", ADMIN, True, m6_key="admin")
    one("admin.walk_expanded", ADMIN, True, then=expand)
    one("metrics.first_open", METRICS, False)
    one("metrics.loaded", METRICS, True, m6_key="metrics")
    return states, m6


def a1_frames(page, reqs, t):
    t.tap(".walk-name", "the walk's name")
    page.wait_for_selector(".frame", timeout=10_000)
    return t.result(True)


def accept_dialogs(page, t):
    """A browser confirm, should a later UI add one, is a tap a person makes."""
    def on(d):
        t.taps += 1
        t.path.append(f"confirm: {d.message[:40]}")
        d.accept()
    page.on("dialog", on)


def a2_replay(page, reqs, t):
    accept_dialogs(page, t)
    value = page.locator(".replay-select").first.evaluate(
        "s => [...s.options].map(o => o.value).find(v => v.startsWith('amazon.nova-lite'))")
    t.select(".replay-select", value, "Replay with… nova-lite")
    page.wait_for_timeout(1500)
    return t.result(any(PAID.search(r) and "/replay" in r for r in reqs))


def a3_delete(page, reqs, t):
    accept_dialogs(page, t)
    walk = page.locator(".walk").first
    walk.get_by_role("button", name=re.compile(r"delete walk", re.I)).click()
    t.taps += 1
    t.path.append("Delete walk")
    deleted = lambda: any(r.startswith("DELETE ") for r in reqs)  # noqa: E731
    page.wait_for_timeout(300)
    if not deleted():
        walk.get_by_role("button", name=re.compile(r"delete|really|again|confirm", re.I)).first.click()
        t.taps += 1
        t.path.append("confirm the delete")
    page.wait_for_timeout(1000)
    return t.result(deleted())


def a4_metrics(page, reqs, t):
    link = page.locator("a[href*=metrics]")
    linked = link.count() > 0
    if linked:
        t.tap("a[href*=metrics]", "Metrics link")
    else:
        t.path.append("no link from the walks console: URL typed")
        page.goto(METRICS)
    page.wait_for_function("() => document.querySelectorAll('svg polyline, svg circle').length > 0",
                           timeout=10_000)
    return t.result(True, linked=linked)


def console_tasks(browser, out):
    admin = partial(console_page, browser, secret=True, url=ADMIN)
    for key, fn in (("A1", a1_frames), ("A2", a2_replay), ("A3", a3_delete), ("A4", a4_metrics)):
        run_task(out, key, admin, fn)


# ---------- main ----------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", required=True, type=Path)
    args = ap.parse_args()
    out = args.out.resolve()
    shots = out / "shots"
    shots.mkdir(parents=True, exist_ok=True)
    axe = axe_source()
    # Timed windows (the 8 s M6 waits, goal timeouts): the desktop's
    # exclusive lock, as any run whose numbers depend on timing (rule 7).
    sys.path.insert(0, str(REPO / "tools"))
    import desktop_lock
    desktop_lock.hold_for_life("exclusive", "3.132 UX harness (timed page windows)")
    commit = subprocess.run(["git", "rev-parse", "--short=9", "HEAD"], cwd=REPO,
                            capture_output=True, text=True).stdout.strip()
    result = {"commit": commit, "viewport": PHONE, "axe": AXE_VERSION, "states": {}, "tasks": {}, "M6": {}}
    with servers(out / "logs"):
        with sync_playwright() as p:
            browser = p.chromium.launch(args=["--use-fake-ui-for-media-stream",
                                              "--use-fake-device-for-media-stream"])
            try:
                for fn in (twin_states, console_states):
                    s, m6 = fn(browser, axe, shots)
                    result["states"].update(s)
                    result["M6"].update(m6)
                twin_tasks(browser, result["tasks"])
                console_tasks(browser, result["tasks"])
            finally:
                browser.close()
    result["M6_count"] = {k: len(v) for k, v in result["M6"].items()}
    (out / "results.json").write_text(json.dumps(result, indent=1, sort_keys=True))
    print(json.dumps(summary(result), indent=1))


def summary(r):
    def st(v):
        if "error" in v:
            return {"error": v["error"]}
        return {"settled": v.get("settled"), "axe": v["M2_axe"]["rules"], "axe_nodes": v["M2_axe"]["nodes"],
                "small": v["M3_small_targets"]["count"],
                "light_contrast": v.get("M2L_light_contrast", {}).get("nodes"),
                "reduced_motion": v.get("M7_reduced_motion", {}).get("count"), "scroll": v["M4_sideways_scroll"],
                "jargon": v["M5_jargon"]["count"]}
    return {
        "commit": r["commit"],
        "states": {k: st(v) for k, v in sorted(r["states"].items())},
        "tasks": {k: {"ok": v["ok"], "taps": v["taps"], "fields": v["fields"],
                      **({"scroll_px": v["scroll_px"]} if "scroll_px" in v else {}),
                      **({"reason": v["reason"]} if "reason" in v else {})}
                  for k, v in r["tasks"].items()},
        "M6": r["M6_count"],
    }


if __name__ == "__main__":
    main()
