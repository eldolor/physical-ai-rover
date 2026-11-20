# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
brain/room_prior.py -- 3.190.C: a find's room prior asked of the cloud once,
at its start, for a target the shipped prior (`brain/explore_goal_rooms.json`)
does not know.

The question, the reply check and the cache are 3.116.C's, moved out of
`evaluations/room-prior-3116/llm_prior.py` unchanged (that harness imports
them from here): per room type, the share of such rooms holding at least one
such object -- a rate per type, not a distribution. A reply is one JSON object
with exactly the four types, each a number in [0, 1]; anything else is no
prior, never a guess.

The call goes through the vision service's `POST /room-prior` (its own copy
of the prompt, `service/vision_analyze/vision_core.py`; CLAUDE.md section 6),
on /room's model. The cache is a JSON file keyed by model id then target,
holding the raw reply (`{"reply", "stop", "usage", "prior"}`), the shape of
3.116.C's `llm_prior.json`, so that file can seed it. A reply the model gave
is cached whether or not it parsed (3.116.C's rule: a bad reply is no prior
and is not paid for twice); a failed call (no connection, a 5xx, a timeout)
is not.

`prior_at_start()` is the start's bounded call: the call runs on a daemon
thread and the start waits at most `deadline_s` for it. A reply that comes
later is still cached -- for the next find -- and never reaches the mission
that gave up on it (the prior does not arrive mid-mission).
"""

from __future__ import annotations

import json
import logging
import os
import threading
from pathlib import Path
from typing import Callable, Optional

logger = logging.getLogger(__name__)

TYPES = ("kitchen", "living room", "bedroom", "bathroom")
MAX_TOKENS = 512

# /room's model, the one 3.116.C and 3.190.A measured. The vision service
# decides which model answers (vision_core.ROOM_MODEL_ID); this is the id the
# cache is READ under before a call. A reply from another model is cached
# under its own id, so it is never read as this one's
# (tests/test_room_prior.py pins the two equal).
MODEL_ID = "us.anthropic.claude-opus-5-5"

# The target as sent: a few words (the service refuses over 200 characters,
# app.py's MAX_TARGET_CHARS).
MAX_TARGET_CHARS = 200

DEFAULT_CACHE = Path(__file__).resolve().parents[1] / "recordings" / "room-prior.json"

PROMPT = """A small robot searches a home for an object. It needs a prior over room types.

Object: {target}

For each of these room types -- kitchen, living room, bedroom, bathroom -- estimate the share of such rooms in typical homes that contain at least one {target} somewhere a person could see it (on the floor, furniture or a counter). Each value is a probability between 0 and 1, judged for that room type on its own: the four values need not add up to 1.

Reply with one JSON object and nothing else, with exactly these keys: "kitchen", "living room", "bedroom", "bathroom"."""


def parse(text: str):
    """The reply's four rates, or None if it is not exactly the shape asked."""
    t = (text or "").strip()
    if t.startswith("```"):
        t = t.strip("`")
        t = t[t.find("{"):] if "{" in t else t
    try:
        obj = json.loads(t[t.find("{"): t.rfind("}") + 1])
    except ValueError:
        return None
    if not isinstance(obj, dict) or set(obj) != set(TYPES):
        return None
    out = {}
    for k in TYPES:
        v = obj[k]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not 0.0 <= v <= 1.0:
            return None
        out[k] = float(v)
    return out


def valid_rates(rates) -> bool:
    """Whether `rates` is a prior `parse()` could have given (the runner's
    check on what it is handed)."""
    if not isinstance(rates, dict) or set(rates) != set(TYPES):
        return False
    return all(not isinstance(v, bool) and isinstance(v, (int, float)) and 0.0 <= v <= 1.0
               for v in rates.values())


def target_key(target) -> str:
    """The cache's key for a target: lower case, single spaces."""
    return " ".join(str(target or "").split()).lower()


def cache_path() -> Path:
    return Path(os.environ.get("ROOM_PRIOR_CACHE") or DEFAULT_CACHE)


_lock = threading.Lock()


def load_cache(path: Optional[Path] = None) -> dict:
    """The cache, or {} when it is missing or cannot be read (a damaged file
    costs a paid call, never a start)."""
    p = Path(path or cache_path())
    try:
        data = json.loads(p.read_text())
    except FileNotFoundError:
        return {}
    except (OSError, ValueError) as e:
        logger.warning("room prior cache %s unreadable (%s): read as empty", p, e)
        return {}
    return data if isinstance(data, dict) else {}


def save_cache(cache: dict, path: Optional[Path] = None) -> None:
    p = Path(path or cache_path())
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_name(p.name + f".{os.getpid()}.{threading.get_ident()}.tmp")
    with open(tmp, "w") as f:
        json.dump(cache, f, indent=1, sort_keys=True)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, p)


def cached(target, model_id: str = MODEL_ID, path: Optional[Path] = None) -> Optional[dict]:
    """The cached entry for this target under `model_id`, or None."""
    got = load_cache(path).get(model_id)
    entry = got.get(target_key(target)) if isinstance(got, dict) else None
    return entry if isinstance(entry, dict) and "reply" in entry else None


def remember(target, answer: dict, path: Optional[Path] = None) -> dict:
    """Cache one answer from the service under the model that gave it; the
    entry as cached. Read-modify-write under one lock (a late reply's thread
    and a new start's call can both write)."""
    model = answer.get("model_id") or MODEL_ID
    reply = answer.get("reply")
    reply = reply if isinstance(reply, str) else ""
    entry = {"reply": reply, "stop": answer.get("stop"), "usage": answer.get("usage"),
             "prior": parse(reply)}
    with _lock:
        cache = load_cache(path)
        got = cache.get(model) if isinstance(cache.get(model), dict) else {}
        got[target_key(target)] = entry
        cache[model] = got
        save_cache(cache, path)
    return entry


def ask(target: str, vision_url: str, secret: Optional[str] = None,
        timeout_s: float = 10.0, client=None) -> dict:
    """One `POST /room-prior`: the service's JSON (`reply`, `stop`,
    `model_id`, `usage`). Raises on a failed call or a reply that is not an
    object."""
    import httpx
    headers = {"x-app-secret": secret} if secret else {}
    url = vision_url.rstrip("/") + "/room-prior"
    body = {"target_object": target}
    if client is not None:
        r = client.post(url, json=body, headers=headers, timeout=timeout_s)
    else:
        r = httpx.post(url, json=body, headers=headers, timeout=timeout_s)
    r.raise_for_status()
    out = r.json()
    if not isinstance(out, dict):
        raise ValueError(f"/room-prior answered {type(out).__name__}, not an object")
    return out


def _why_failed(e: BaseException) -> str:
    status = getattr(getattr(e, "response", None), "status_code", None)
    if status is not None:
        return f"http {status}"
    name = type(e).__name__
    if "Timeout" in name:
        return "timeout"
    if "Connect" in name or isinstance(e, ConnectionError):
        return "unreachable"
    return f"error: {name}"


def status(source: str, why: Optional[str] = None, rates: Optional[dict] = None,
           model_id: Optional[str] = None) -> dict:
    """The `status.room_prior` shape."""
    return {"source": source, "why": why, "rates": dict(rates) if rates else None,
            "model_id": model_id}


def prior_at_start(target: str, vision_url: Optional[str], secret: Optional[str] = None,
                   deadline_s: float = 4.0, path: Optional[Path] = None,
                   ask_fn: Optional[Callable[..., dict]] = None) -> dict:
    """The start's one bounded look-up for a target the shipped prior lacks:
    the cache under /room's model, else one call that the start waits at
    most `deadline_s` for. Never raises; the status says what happened.

    source `cache` or `cloud` carries rates; `none` carries the cause:
    `cached reply is not a prior`, `reply is not a prior`, `timeout`
    (past the deadline: the reply, if it comes, is cached for the next find),
    `unreachable`, `http <code>`, `error: <type>`, `no vision service`."""
    model = MODEL_ID
    hit = cached(target, model, path)
    if hit is not None:
        if hit.get("prior") is not None and valid_rates(hit["prior"]):
            return status("cache", None, hit["prior"], model)
        return status("none", "cached reply is not a prior", None, model)
    if not vision_url:
        return status("none", "no vision service", None, None)
    call = ask_fn or ask
    box: dict = {}
    done = threading.Event()
    gave_up = threading.Event()

    def run():
        try:
            # The HTTP deadline is the start's plus a margin: a reply that
            # misses the start may still land in the cache, but a hung
            # connection never outlives the thread by much.
            answer = call(target, vision_url, secret=secret, timeout_s=max(deadline_s, 1.0) * 3)
            box["entry"] = remember(target, answer, path)
            box["model"] = answer.get("model_id") or model
        except BaseException as e:  # noqa: BLE001 -- the start's status names it
            box["error"] = e
        finally:
            done.set()
            if gave_up.is_set():
                if "entry" in box:
                    logger.info("room prior for %r arrived after the start: cached for the "
                                "next find", target)
                elif "error" in box:
                    logger.info("room prior for %r failed after the start: %s", target,
                                _why_failed(box["error"]))

    threading.Thread(target=run, name="room-prior", daemon=True).start()
    if not done.wait(deadline_s):
        gave_up.set()
        if not done.is_set():
            return status("none", "timeout", None, model)
    if "error" in box:
        logger.warning("room prior for %r: %s", target, box["error"])
        return status("none", _why_failed(box["error"]), None, model)
    entry = box["entry"]
    if entry["prior"] is None:
        return status("none", "reply is not a prior", None, box["model"])
    return status("cloud", None, entry["prior"], box["model"])
