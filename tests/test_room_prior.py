# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.190.C: the question, reply check and cache of a find's cloud room prior
(`brain/room_prior.py`), moved from 3.116.C's harness unchanged; the vision
service's copy of the question kept identical; the start's bounded call."""
import json
import sys
import threading
import time
from pathlib import Path

import httpx
import pytest

from brain import room_prior as rp
from tests.conftest import REPO_ROOT

SEED = REPO_ROOT / "evaluations" / "room-prior-3116" / "llm_prior.json"
GOOD = '{"kitchen": 0.9, "living room": 0.05, "bedroom": 0.01, "bathroom": 0.02}'


def _vision_core():
    d = str(REPO_ROOT / "service" / "vision_analyze")
    if d not in sys.path:
        sys.path.insert(0, d)
    import vision_core
    return vision_core


# ---------- the question and the reply check ----------

@pytest.mark.parametrize("text, ok", [
    (GOOD, True),
    ("```json\n" + GOOD + "\n```", True),
    ("Here it is: " + GOOD, True),
    ('{"kitchen": 0.9, "living room": 0.05, "bedroom": 0.01}', False),          # a type missing
    ('{"kitchen": 1.2, "living room": 0.05, "bedroom": 0.01, "bathroom": 0}', False),
    ('{"kitchen": true, "living room": 0.05, "bedroom": 0.01, "bathroom": 0}', False),
    ('{"kitchen": "0.9", "living room": 0.05, "bedroom": 0.01, "bathroom": 0}', False),
    ('{"kitchen": 0.9, "living room": 0.05, "bedroom": 0.01, "bathroom": 0, "hall": 0}', False),
    ("I cannot say.", False),
    ("", False),
])
def test_a_reply_is_a_prior_only_in_the_shape_asked(text, ok):
    assert (rp.parse(text) is not None) is ok
    # The vision service's copy reads it the same way (CLAUDE.md section 6).
    assert _vision_core().parse_room_prior(text) == rp.parse(text)


def test_the_service_asks_the_same_question_on_rooms_model():
    vc = _vision_core()
    assert vc.ROOM_PRIOR_PROMPT == rp.PROMPT
    assert vc.ROOM_PRIOR_TYPES == rp.TYPES
    assert vc.ROOM_PRIOR_MAX_TOKENS == rp.MAX_TOKENS
    # The cache is read under /room's model: the code default, the template's
    # parameter and the brain's id are one string.
    src = (REPO_ROOT / "service" / "vision_analyze" / "vision_core.py").read_text()
    assert f'or "{rp.MODEL_ID}"' in src
    tpl = (REPO_ROOT / "cloudformation" / "serverless.yaml").read_text()
    assert f"RoomModelId:\n    Type: String\n    Default: {rp.MODEL_ID}\n" in tpl


def test_the_harness_reads_the_brains_question():
    """3.116.C's harness imports the moved question and check, so the two can
    never drift."""
    sys.path.insert(0, str(REPO_ROOT / "evaluations" / "room-prior-3116"))
    import llm_prior
    assert llm_prior.parse is rp.parse and llm_prior.PROMPT is rp.PROMPT
    assert llm_prior.TYPES is rp.TYPES and llm_prior.MAX_TOKENS == rp.MAX_TOKENS == 512


def test_the_29_cached_replies_still_parse_to_the_same_priors():
    got = json.loads(SEED.read_text())[rp.MODEL_ID]
    assert len(got) == 29
    for target, entry in got.items():
        assert rp.parse(entry["reply"]) == entry["prior"], target
    assert sum(e["prior"] is not None for e in got.values()) == 29


# ---------- the cache ----------

def test_the_cache_keeps_every_answer_under_its_model(tmp_path):
    path = tmp_path / "c.json"
    rp.remember("Oven  Mitt", {"reply": GOOD, "stop": "end_turn", "model_id": rp.MODEL_ID}, path)
    rp.remember("cat", {"reply": "no idea", "model_id": rp.MODEL_ID}, path)
    rp.remember("dog", {"reply": GOOD, "model_id": "other.model"}, path)
    assert rp.cached("oven mitt", path=path)["prior"]["kitchen"] == 0.9
    assert rp.cached("cat", path=path)["prior"] is None           # cached, not a prior
    assert rp.cached("dog", path=path) is None                    # another model's
    assert rp.cached("dog", "other.model", path=path)["prior"] is not None


def test_3116cs_file_seeds_the_cache(tmp_path):
    path = tmp_path / "c.json"
    path.write_text(SEED.read_text())
    st = rp.prior_at_start("house plant", "http://unused", path=path,
                           ask_fn=lambda *a, **k: pytest.fail("a cache hit pays nothing"))
    assert st == rp.status("cache", None, json.loads(SEED.read_text())[rp.MODEL_ID]
                           ["house plant"]["prior"], rp.MODEL_ID)


def test_a_damaged_cache_reads_as_empty(tmp_path):
    path = tmp_path / "c.json"
    path.write_text("{not json")
    assert rp.load_cache(path) == {}
    path.write_text("[1, 2]")
    assert rp.load_cache(path) == {}
    rp.remember("cup", {"reply": GOOD}, path)                     # and is rewritten whole
    assert rp.cached("cup", path=path)["prior"] is not None


def test_concurrent_writes_lose_nothing(tmp_path):
    path = tmp_path / "c.json"
    ts = [threading.Thread(target=rp.remember, args=(f"t{i}", {"reply": GOOD}, path))
          for i in range(16)]
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    assert len(rp.load_cache(path)[rp.MODEL_ID]) == 16


# ---------- the start's bounded call ----------

def _answer(reply=GOOD):
    return lambda target, url, secret=None, timeout_s=None: {
        "reply": reply, "stop": "end_turn", "model_id": rp.MODEL_ID}


def test_one_call_then_the_cache(tmp_path):
    path, calls = tmp_path / "c.json", []

    def ask(*a, **k):
        calls.append(a[0])
        return _answer()(*a, **k)

    first = rp.prior_at_start("spatula", "http://v", path=path, ask_fn=ask)
    second = rp.prior_at_start("Spatula", "http://v", path=path, ask_fn=ask)
    assert calls == ["spatula"]
    assert first["source"] == "cloud" and second["source"] == "cache"
    assert first["rates"] == second["rates"] == rp.parse(GOOD)


def test_a_reply_that_is_not_a_prior_is_none_and_cached(tmp_path):
    path = tmp_path / "c.json"
    st = rp.prior_at_start("spatula", "http://v", path=path, ask_fn=_answer("Kitchen, mostly."))
    assert st["source"] == "none" and st["why"] == "reply is not a prior"
    again = rp.prior_at_start("spatula", "http://v", path=path,
                              ask_fn=lambda *a, **k: pytest.fail("not paid twice"))
    assert again["source"] == "none" and again["why"] == "cached reply is not a prior"


def test_a_refused_connection_is_none_and_not_cached(tmp_path):
    path = tmp_path / "c.json"
    st = rp.prior_at_start("spatula", "http://127.0.0.1:1", path=path, deadline_s=2.0)
    assert st["source"] == "none" and st["why"] == "unreachable"
    assert rp.cached("spatula", path=path) is None


def test_a_5xx_is_named(tmp_path, monkeypatch):
    monkeypatch.setattr(rp, "_cooling", {})
    def ask(*a, **k):
        req = httpx.Request("POST", "http://v/room-prior")
        raise httpx.HTTPStatusError("bad", request=req, response=httpx.Response(502, request=req))

    st = rp.prior_at_start("spatula", "http://v", path=tmp_path / "c.json", ask_fn=ask)
    assert st == rp.status("none", "http 502", None, rp.MODEL_ID)


def test_no_vision_service_is_none(tmp_path):
    assert rp.prior_at_start("spatula", None, path=tmp_path / "c.json")["why"] == "no vision service"


def test_a_late_reply_misses_the_start_and_is_cached_for_the_next(tmp_path):
    path, release = tmp_path / "c.json", threading.Event()

    def slow(*a, **k):
        release.wait(5)
        return _answer()(*a, **k)

    t0 = time.monotonic()
    st = rp.prior_at_start("spatula", "http://v", path=path, deadline_s=0.3, ask_fn=slow)
    took = time.monotonic() - t0
    assert st == rp.status("none", "timeout", None, rp.MODEL_ID)
    assert took < 0.3 + 0.5
    release.set()
    for _ in range(100):
        if rp.cached("spatula", path=path):
            break
        time.sleep(0.02)
    assert rp.cached("spatula", path=path)["prior"] == rp.parse(GOOD)


def test_the_real_client_posts_the_target_with_the_secret():
    seen = {}

    def handler(request):
        seen["url"], seen["secret"] = str(request.url), request.headers.get("x-app-secret")
        seen["body"] = json.loads(request.content)
        return httpx.Response(200, json={"reply": GOOD, "model_id": rp.MODEL_ID})

    client = httpx.Client(transport=httpx.MockTransport(handler))
    out = rp.ask("oven mitt", "https://v.example/", secret="s", client=client)
    assert seen == {"url": "https://v.example/room-prior", "secret": "s",
                    "body": {"target_object": "oven mitt"}}
    assert out["reply"] == GOOD


def test_the_default_cache_lives_under_recordings():
    """Kept out of git (recordings/ is ignored) and overridable."""
    assert rp.DEFAULT_CACHE == Path(rp.__file__).resolve().parents[1] / "recordings" / "room-prior.json"


# ---------- review round 1 ----------

def test_a_deployment_on_another_model_still_hits_its_own_replies(tmp_path):
    path, calls = tmp_path / "c.json", []

    def ask(*a, **k):
        calls.append(1)
        return {"reply": GOOD, "stop": "end_turn", "model_id": "us.other.model"}

    assert rp.prior_at_start("spatula", "http://v", path=path, ask_fn=ask)["source"] == "cloud"
    again = rp.prior_at_start("spatula", "http://v", path=path, ask_fn=ask)
    assert calls == [1] and again["source"] == "cache" and again["model_id"] == "us.other.model"
    assert rp.cached("spatula", rp.MODEL_ID, path=path) is None      # never filed as another's


def test_a_cut_reply_is_not_cached_and_cools_the_target(tmp_path, monkeypatch):
    monkeypatch.setattr(rp, "_cooling", {})
    path, stops = tmp_path / "c.json", ["max_tokens", "end_turn"]

    def ask(*a, **k):
        return {"reply": '{"kitchen": 0.9, "living', "stop": stops.pop(0), "model_id": rp.MODEL_ID}

    st = rp.prior_at_start("spatula", "http://v", path=path, ask_fn=ask)
    assert (st["source"], st["why"]) == ("none", "reply cut (max_tokens)")
    assert rp.cached("spatula", path=path) is None
    st = rp.prior_at_start("spatula", "http://v", path=path, ask_fn=ask)
    assert st["why"] == "failed recently: not asked again yet" and stops == ["end_turn"]
    monkeypatch.setattr(rp, "COOL_S", 0.0)
    st = rp.prior_at_start("spatula", "http://v", path=path, ask_fn=ask)
    assert st["why"] == "reply is not a prior" and stops == []


def test_a_call_still_running_is_waited_on_not_paid_again(tmp_path):
    path, release, calls = tmp_path / "c.json", threading.Event(), []

    def slow(*a, **k):
        calls.append(1)
        release.wait(5)
        return _answer()(*a, **k)

    first = rp.prior_at_start("spatula", "http://v", path=path, deadline_s=0.2, ask_fn=slow)
    assert first["why"] == "timeout"
    threading.Timer(0.2, release.set).start()
    second = rp.prior_at_start("spatula", "http://v", path=path, deadline_s=3.0, ask_fn=slow)
    assert calls == [1]
    assert second["source"] == "cloud" and second["rates"] == rp.parse(GOOD)
    assert rp._in_flight == {}


# ---------- review round 1 (Thermos) ----------

def test_a_call_that_runs_out_its_http_deadline_cools_the_target(tmp_path, monkeypatch):
    path, calls = tmp_path / "c.json", []
    monkeypatch.setattr(rp, "_cooling", {})

    def hung(*a, **k):
        calls.append(1)
        raise httpx.ReadTimeout("slow")

    first = rp.prior_at_start("spatula", "http://v", path=path, ask_fn=hung)
    second = rp.prior_at_start("spatula", "http://v", path=path, ask_fn=hung)
    assert first["why"] == "timeout"
    assert second["why"] == "failed recently: not asked again yet" and calls == [1]
    other = rp.prior_at_start("kettle", "http://v", path=path, ask_fn=_answer())
    assert other["source"] == "cloud"                    # only that target waits
    monkeypatch.setattr(rp, "COOL_S", 0.0)
    rp.prior_at_start("spatula", "http://v", path=path, ask_fn=hung)
    assert calls == [1, 1]


def test_a_cache_that_cannot_be_written_still_gives_this_mission_the_reply(tmp_path, monkeypatch):
    def boom(*a, **k):
        raise OSError("read-only")

    monkeypatch.setattr(rp, "save_cache", boom)
    st = rp.prior_at_start("spatula", "http://v", path=tmp_path / "c.json", ask_fn=_answer())
    assert st["source"] == "cloud" and st["rates"] == rp.parse(GOOD)


# ---------- review round 2 ----------

def test_a_5xx_cools_the_target_and_a_refused_connection_does_not(tmp_path, monkeypatch):
    monkeypatch.setattr(rp, "_cooling", {})
    path = tmp_path / "c.json"

    def gw(*a, **k):
        req = httpx.Request("POST", "http://v/room-prior")
        raise httpx.HTTPStatusError("x", request=req, response=httpx.Response(504, request=req))

    assert rp.prior_at_start("spatula", "http://v", path=path, ask_fn=gw)["why"] == "http 504"
    assert rp.prior_at_start("spatula", "http://v", path=path, ask_fn=gw)["why"].startswith("failed recently")
    assert rp.prior_at_start("kettle", "http://127.0.0.1:1", path=path, deadline_s=2)["why"] == "unreachable"
    assert rp.prior_at_start("kettle", "http://127.0.0.1:1", path=path, deadline_s=2)["why"] == "unreachable"


def test_a_call_finishing_between_the_two_reads_is_not_paid_again(tmp_path, monkeypatch):
    """The cache is read again under the lock before a call is claimed."""
    path, calls = tmp_path / "c.json", []
    real = rp.cached
    seen = []

    def cached_then_written(target, model=rp.MODEL_ID, path=None):
        if not seen:                     # the first read misses ...
            seen.append(1)
            out = real(target, model, path)
            rp.remember(target, {"reply": GOOD, "model_id": rp.MODEL_ID}, path)  # ... then it lands
            return out
        return real(target, model, path)

    monkeypatch.setattr(rp, "cached", cached_then_written)
    st = rp.prior_at_start("spatula", "http://v", path=path,
                           ask_fn=lambda *a, **k: calls.append(1))
    assert st["source"] == "cache" and calls == []


def test_a_failed_cache_write_leaves_no_temp_file_and_cools(tmp_path, monkeypatch):
    monkeypatch.setattr(rp, "_cooling", {})
    real_replace = rp.os.replace
    monkeypatch.setattr(rp.os, "replace", lambda a, b: (_ for _ in ()).throw(OSError("ro")))
    st = rp.prior_at_start("spatula", "http://v", path=tmp_path / "c.json", ask_fn=_answer())
    monkeypatch.setattr(rp.os, "replace", real_replace)
    assert st["source"] == "cloud"
    assert list(tmp_path.iterdir()) == []
    again = rp.prior_at_start("spatula", "http://v", path=tmp_path / "c.json", ask_fn=_answer())
    assert again["why"].startswith("failed recently")
