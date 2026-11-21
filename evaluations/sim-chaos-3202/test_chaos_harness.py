# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.202's harness tests: the fault proxy against a local upstream, and the
scorer against made-up runs (each bar fails when its fault is planted in the
log). Not in the fast suite (tests/ only); run by hand:

    pytest evaluations/sim-chaos-3202/test_chaos_harness.py -q
"""

import http.server
import json
import sys
import threading
import time
from pathlib import Path

import httpx
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import fault_proxy  # noqa: E402
import score  # noqa: E402


# ------------------------------------------------------------------ proxy

class _Upstream(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    seen: list = []

    def log_message(self, *a):
        pass

    def _go(self):
        n = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(n) if n else b""
        _Upstream.seen.append((self.command, self.path, body))
        code = 503 if self.path == "/boom" else 200
        out = json.dumps({"path": self.path, "body": body.decode()}).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(out)))
        self.end_headers()
        self.wfile.write(out)

    do_GET = do_POST = _go


@pytest.fixture()
def link(tmp_path):
    _Upstream.seen = []
    up = http.server.ThreadingHTTPServer(("127.0.0.1", 0), _Upstream)
    threading.Thread(target=up.serve_forever, daemon=True).start()
    px = fault_proxy.serve(0, f"http://127.0.0.1:{up.server_address[1]}", str(tmp_path / "p.jsonl"))
    threading.Thread(target=px.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{px.server_address[1]}"
    with httpx.Client(base_url=url, timeout=5) as c:
        yield c, tmp_path / "p.jsonl"
    px.shutdown()
    up.shutdown()


def test_proxy_passes_through_and_keeps_alive(link):
    c, _ = link
    for i in range(3):
        r = c.post("/action", json={"action": "FORWARD", "i": i})
        assert r.status_code == 200 and json.loads(r.json()["body"])["i"] == i
    assert c.get("/boom").status_code == 503
    assert [s[1] for s in _Upstream.seen] == ["/action"] * 3 + ["/boom"]


def test_dead_link_resets_and_never_reaches_upstream(link):
    c, log = link
    c.post("/__fault", json={"dead": True})
    with pytest.raises(httpx.TransportError):
        c.post("/action", json={})
    assert _Upstream.seen == []
    c.post("/__fault", json={"dead": False})
    assert c.post("/action", json={}).status_code == 200
    snap = c.get("/__fault").json()
    assert snap["counts"]["dead"] == 1 and snap["counts"]["forwarded"] == 1
    assert any(json.loads(l).get("fate") == "dead" for l in log.read_text().splitlines())


def test_drop_share_is_seeded_and_about_right(link):
    c, _ = link
    c.post("/__fault", json={"drop": 0.3, "seed": 7})
    lost = 0
    for _ in range(200):
        try:
            c.get("/x")
        except httpx.TransportError:
            lost += 1
    assert 35 <= lost <= 85                     # 30% of 200 = 60
    assert c.get("/__fault").json()["counts"]["dropped"] == lost


def test_latency_is_added_each_way(link):
    c, _ = link
    c.post("/__fault", json={"latency_ms": 100, "jitter_ms": 0})
    t = time.monotonic()
    c.get("/x")
    assert time.monotonic() - t >= 0.2


def test_duplicate_action_reaches_upstream_twice(link):
    c, _ = link
    c.post("/__fault", json={"dup_action": True})
    assert c.post("/action", json={"action": "LEFT"}).status_code == 200
    c.get("/health")                            # not an /action: never duplicated
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline and len(_Upstream.seen) < 3:
        time.sleep(0.02)
    assert [s[1] for s in _Upstream.seen].count("/action") == 2
    assert [s[1] for s in _Upstream.seen].count("/health") == 1
    assert c.get("/__fault").json()["dup_status"] == {"200": 1}


# ------------------------------------------------------------------ scorer

CFG = {"vision_timeout_s": 20.0, "max_vision_failures": 3, "drill_tick_timeout_s": 3.0}


def _truth(t0, t1, moving_until, speed_m_s=0.2, gap_cm=50.0):
    """A robot driving +x at `speed_m_s` until `moving_until`, then still."""
    rows, x, t = [], 0.0, t0
    while t <= t1:
        mv = t < moving_until
        rows.append({"t": round(t, 3), "x": round(x, 5), "y": 0.0, "th": 0.0,
                     "sp": [speed_m_s, speed_m_s] if mv else [0.0, 0.0], "gap_cm": gap_cm})
        if mv:
            x += speed_m_s * 0.02
        t += 0.02
    return score.Truth(rows)


def test_k1_passes_a_prompt_stop_and_fails_a_late_one():
    f = {"kind": "K1", "fault_at": 10.0, "cleared_at": 10.0, "mission": "m1"}
    good = score.fault_measures(f, _truth(0, 30, 10.4), {"m1": {"outcome": "failed"}})
    assert score.judge_fault(f, good, CFG)["pass"], good
    late = score.fault_measures(f, _truth(0, 30, 13.0), {"m1": {"outcome": "failed"}})
    v = score.judge_fault(f, late, CFG)
    assert not v["pass"] and any("wheels zero" in r for r in v["reasons"])
    assert any("moved" in r for r in v["reasons"])


def test_a_fault_fired_while_still_is_invalid():
    f = {"kind": "K6", "fault_at": 10.0, "cleared_at": 20.0, "mission": "m1"}
    ms = score.fault_measures(f, _truth(0, 40, 5.0), {})
    v = score.judge_fault(f, ms, CFG)
    assert v["valid"] is False and not v["pass"]


def test_k2_stale_mission_driving_after_sigcont_fails():
    f = {"kind": "K2", "fault_at": 10.0, "cont_at": 40.0, "cleared_at": 40.0, "mission": "m1"}
    tr = _truth(0, 60, 10.3)
    tr.rows = [dict(r, sp=[0.1, 0.1]) if 42.0 <= r["t"] <= 43.0 else r for r in tr.rows]
    ms = score.fault_measures(f, tr, {})
    v = score.judge_fault(f, ms, CFG)
    assert ms["drive_after_cont"] > 0 and not v["pass"]


def test_k4_travel_while_stale_fails_over_2cm():
    f = {"kind": "K4", "fault_at": 10.0, "cont_at": 20.0, "cleared_at": 20.0, "mission": "m1"}
    assert score.judge_fault(f, score.fault_measures(f, _truth(0, 30, 10.2), {}), CFG)["pass"]
    bad = score.fault_measures(f, _truth(0, 30, 11.0), {})
    assert bad["travel_stale_cm"] > 2.0 and not score.judge_fault(f, bad, CFG)["pass"]


def test_k10_too_slow_or_found_fails():
    f = {"kind": "K10", "fault_at": 10.0, "cleared_at": 100.0, "mission": "m1"}
    ok = score.fault_measures(f, _truth(0, 120, 0), {"m1": {"t1": 95.0, "outcome": "failed"}})
    assert score.judge_fault(f, ok, CFG)["pass"], ok
    slow = score.fault_measures(f, _truth(0, 200, 0), {"m1": {"t1": 131.0, "outcome": "failed"}})
    assert not score.judge_fault(f, slow, CFG)["pass"]
    found = score.fault_measures(f, _truth(0, 120, 0), {"m1": {"t1": 50.0, "outcome": "found"}})
    assert not score.judge_fault(f, found, CFG)["pass"]


def test_k8_five_xx_fails():
    f = {"kind": "K8", "fault_at": 10.0, "cleared_at": 50.0, "mission": "m1"}
    ms = score.fault_measures(f, _truth(0, 60, 0), {"m1": {"t1": 50.0, "outcome": "found"}})
    assert score.judge_fault(f, ms, CFG, [{"t": 20.0, "dup_copy": 200}])["pass"]
    assert not score.judge_fault(f, dict(ms), CFG, [{"t": 20.0, "dup_copy": 500}])["pass"]
    assert score.judge_fault(f, dict(ms), CFG, [{"t": 20.0, "p": "/world/goal", "fate": 501}])["pass"]


def test_slope_and_memory_growth():
    assert score.slope_per_h([(0, 100), (3600, 110), (7200, 120)]) == 10.0
    rss = []
    for i in range(0, 3 * 3600, 5):
        leak = 100 + 40 * i / 3600                  # 40 MB/h on the brain
        rss.append({"t": float(i), "mission": f"m{i // 300}", "oom_kill": 0,
                    "rss": {"brain": [11, int(leak * 1024)], "body": [12, 80 * 1024],
                            "sensors": [13, 60 * 1024], "sensors2": [14, 60 * 1024]}})
    mem = score.memory(rss, 0.0, 3 * 3600.0)
    assert any("brain" in f and "slope" in f for f in mem["fails"])
    assert mem["growth_mb"]["body"] == 0.0


def _run_dir(tmp_path, missions, faults=(), house="starter_house"):
    (tmp_path / "run.json").write_text(json.dumps({
        "house": house, "target": "red backpack", "config": CFG, "t_start": 0.0, "plant": None}))
    (tmp_path / "end.json").write_text(json.dumps({"t_end": 100.0, "alive": {"robot": True},
                                                   "health": {"robot": True, "brain": True},
                                                   "fired": len(faults), "unfired": 0}))
    (tmp_path / "missions.jsonl").write_text("".join(json.dumps(x) + "\n" for x in missions))
    (tmp_path / "faults.jsonl").write_text("".join(json.dumps(x) + "\n" for x in faults))
    rows = _truth(0, 100, 0).rows
    (tmp_path / "truth.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    return tmp_path


def test_planted_wrong_found_is_caught(tmp_path):
    """Criterion 7's check of 4(b): a `found` far from the target, and a
    `found` under a fault that forbids it, are both wrong."""
    tx, ty = score.target_m("starter_house", "red backpack")
    near = {"id": "a", "outcome": "found", "tags": [], "t0": 0, "t1": 50,
            "end_truth": {"x": tx + 0.3, "y": ty}}
    far = {"id": "b", "outcome": "found", "tags": [], "t0": 50, "t1": 90,
           "end_truth": {"x": tx + 3.0, "y": ty}}
    forbidden = {"id": "c", "outcome": "found", "tags": ["K11"], "t0": 90, "t1": 95,
                 "end_truth": {"x": tx, "y": ty}}
    res = score.score(_run_dir(tmp_path, [near, far, forbidden]))
    wrong = " ".join(res["c4"]["wrong_found"])
    assert "mission b" in wrong and "mission c" in wrong and "mission a" not in wrong
    assert not res["c4"]["met"]
    res_ok = score.score(_run_dir(tmp_path, [near]))
    assert res_ok["c4"]["wrong_found"] == [] and res_ok["c4"]["rate"] == 1.0


def test_k7_failed_by_a_designed_guard_passes_and_by_anything_else_fails():
    f = {"kind": "K7", "fault_at": 10.0, "cleared_at": 50.0, "mission": "m1"}
    budget = {"m1": {"t1": 50.0, "outcome": "failed",
                     "end_reason": "camera unavailable 3 times in a row: GET /frame failed"}}
    assert score.judge_fault(f, score.fault_measures(f, _truth(0, 60, 0), budget), CFG)["pass"]
    crash = {"m1": {"t1": 50.0, "outcome": "failed", "end_reason": "KeyError: 'scan'"}}
    assert not score.judge_fault(f, score.fault_measures(f, _truth(0, 60, 0), crash), CFG)["pass"]


def test_k11_that_ended_before_its_first_step_is_invalid():
    f = {"kind": "K11", "fault_at": 10.0, "cleared_at": 12.0, "mission": "m1",
         "ended_before_step1": True}
    ms = score.fault_measures(f, _truth(0, 30, 0), {"m1": {"t1": 12.0, "outcome": "failed"}})
    v = score.judge_fault(f, ms, CFG)
    assert v["valid"] is False and not v["pass"]


def test_an_errored_fault_fails():
    v = score.judge_fault({"kind": "K1", "errored": "URLError: timed out"}, {}, CFG)
    assert not v["pass"] and "errored" in v["reasons"][0]


def test_found_is_judged_against_where_the_target_was_then(tmp_path):
    """Amendment 3: after a `target_moved` event a Found is measured from the
    new cell; one at the old cell after the move is wrong."""
    run = _run_dir(tmp_path, [], house="scaled_house")
    tx, ty = score.target_m("scaled_house", "red backpack")
    nx, ny = (24 + 0.5) * score.CELL_M, (11 + 0.5) * score.CELL_M
    first = {"id": "a", "outcome": "found", "tags": [], "t0": 0, "t1": 40,
             "end_truth": {"t": 40, "x": tx, "y": ty}}
    second = {"id": "b", "outcome": "found", "tags": [], "t0": 41, "t1": 80,
              "end_truth": {"t": 80, "x": nx + 0.2, "y": ny}}
    stale = {"id": "c", "outcome": "found", "tags": [], "t0": 81, "t1": 90,
             "end_truth": {"t": 90, "x": tx, "y": ty}}
    (run / "missions.jsonl").write_text("".join(json.dumps(x) + "\n" for x in (first, second, stale)))
    (run / "events.jsonl").write_text(json.dumps(
        {"t": 40.5, "what": "target_moved", "src": [23, 5], "dst": [24, 11]}) + "\n")
    res = score.score(run)
    wrong = " ".join(res["c4"]["wrong_found"])
    assert "mission c" in wrong and "mission a" not in wrong and "mission b" not in wrong
    assert res["c4"]["target_moves"] == 1


def test_move_target_cycles_and_skips_a_refused_cell(tmp_path, monkeypatch):
    import chaos
    from types import SimpleNamespace
    calls = []

    def fake_http(method, url, body=None, secret="", timeout=5.0):
        calls.append(tuple(body["dst"]))
        return (409, "inside the turning circle") if tuple(body["dst"]) == (24, 11) else (200, {})
    monkeypatch.setattr(chaos, "http", fake_http)
    c = chaos.Chaos(tmp_path, faults=1, seed=1, house="scaled_house", kinds=("K1",), gap=(1, 2))
    c.st = SimpleNamespace(urls={"body": "http://x"}, secret="s")
    c.move_target()
    assert calls == [(24, 11), (9, 11)] and c.cycle[c.target_i] == (9, 11)
    c.move_target()                       # on round to the kitchen
    assert c.cycle[c.target_i] == (23, 5)
    ev = [json.loads(x) for x in (tmp_path / "events.jsonl").read_text().splitlines()]
    assert [e["what"] for e in ev] == ["target_move_refused", "target_moved", "target_moved"]
