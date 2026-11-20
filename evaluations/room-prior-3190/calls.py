# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/room-prior-3190/calls.py -- 3.190.B: the 40 fresh targets asked of
/room's model, one call each, cached; criteria 4-6 scored.

    python evaluations/room-prior-3190/calls.py ask [N]   # paid: the next N targets not cached
    python evaluations/room-prior-3190/calls.py spend     # dollars so far, from the replies' usage
    python evaluations/room-prior-3190/calls.py score     # criteria 4-6 (free) -> b/score.json

The question, the reply check and the cache's shape are the brain's
(`brain/room_prior.py`: keyed by model id then `target_key`, each entry
{reply, stop, usage, prior}, plus `_served_model`), so `b/replies.json` can
seed a brain cache as it stands. A reply that does not parse is cached with
prior None: no prior, never a guess, and never asked again. The call is the
3.116.C harness's (Bedrock `converse`, effort low, `MAX_TOKENS`), straight to
Bedrock: `POST /room-prior`'s Lambda is not deployed (the user's), and the
route asks the same question of the same model.

Spend is counted from each reply's `usage` at this account's billed rates
(`control/admin_prices.py`, Opus 5.5 $4.40/M in, $22/M out). `ask` refuses a
call that could take the total past CAP_USD, judged on the dearest call so
far (MAX_TOKENS out before the first).
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "service", "vision_analyze"))
from brain.goal_policy import prior_is_informative  # noqa: E402
from brain.room_prior import (MAX_TOKENS, PROMPT, SERVED, TYPES, parse,  # noqa: E402
                              target_key)
from control.admin_prices import price_of  # noqa: E402
from vision_core import MODEL_REGIONS  # noqa: E402

sys.path.insert(0, HERE)
from targets import EXPECTED, TARGETS  # noqa: E402

# The model B asked (/room's default, 3.116.C's), pinned so the recorded
# replies are scored under it whatever BEDROCK_ROOM_MODEL_ID holds (review).
MODEL_ID = "us.anthropic.claude-opus-5-5"
REGION = MODEL_REGIONS.get(MODEL_ID) or "us-east-2"
CACHE = os.path.join(HERE, "b", "replies.json")
SCORE = os.path.join(HERE, "b", "score.json")
CAP_USD = 2.00


def load() -> dict:
    return json.load(open(CACHE)) if os.path.exists(CACHE) else {}


def save(cache: dict) -> None:
    os.makedirs(os.path.dirname(CACHE), exist_ok=True)
    tmp = CACHE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(cache, f, indent=1, sort_keys=True)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, CACHE)


def cost(usage) -> float:
    price = price_of(MODEL_ID)
    if price is None:                      # an unpriced model: no cap can be kept
        raise SystemExit(f"no billed price for {MODEL_ID} in control/admin_prices.py")
    pin, pout = price
    u = usage or {}
    return (u.get("inputTokens", 0) * pin + u.get("outputTokens", 0) * pout) / 1e6


def spend(cache: dict) -> dict:
    got = cache.get(MODEL_ID, {})
    costs = [cost(e.get("usage")) for e in got.values()]
    return {"calls": len(costs), "usd": sum(costs), "dearest": max(costs, default=0.0),
            "in": sum((e.get("usage") or {}).get("inputTokens", 0) for e in got.values()),
            "out": sum((e.get("usage") or {}).get("outputTokens", 0) for e in got.values())}


def ask_one(client, target: str) -> dict:
    r = client.converse(
        modelId=MODEL_ID,
        messages=[{"role": "user", "content": [{"text": PROMPT.format(target=target)}]}],
        inferenceConfig={"maxTokens": MAX_TOKENS},
        additionalModelRequestFields={"output_config": {"effort": "low"}},
    )
    text = "".join(c.get("text", "") for c in r["output"]["message"]["content"])
    return {"reply": text, "stop": r.get("stopReason"), "usage": r.get("usage"),
            "prior": parse(text)}


def cmd_ask(limit: int) -> None:
    import boto3
    client = boto3.client("bedrock-runtime", region_name=REGION)
    cache = load()
    got = cache.setdefault(MODEL_ID, {})
    todo = [t for t in TARGETS if target_key(t) not in got][:limit]
    for t in todo:
        s = spend(cache)
        worst = s["dearest"] or (2000 * price_of(MODEL_ID)[0] + MAX_TOKENS * price_of(MODEL_ID)[1]) / 1e6
        if s["usd"] + worst > CAP_USD:
            print(f"STOP: ${s['usd']:.4f} spent, the next call could pass ${CAP_USD}", flush=True)
            return
        got[target_key(t)] = ask_one(client, t)
        cache[SERVED] = MODEL_ID
        save(cache)                        # each paid answer kept at once
        e = got[target_key(t)]
        print(t, json.dumps(e["prior"]), e["stop"], e["usage"], f"${cost(e['usage']):.5f}",
              flush=True)
    print(json.dumps(spend(cache)), flush=True)


def score(cache: dict) -> dict:
    got = cache.get(MODEL_ID, {})
    priors = {t: (got.get(target_key(t)) or {}).get("prior") for t in TARGETS}
    asked = sum(target_key(t) in got for t in TARGETS)
    valid = sum(p is not None for p in priors.values())
    tops, matched = {}, 0
    for t, want in EXPECTED.items():
        p = priors[t]
        # 3.116's tie rule: the first maximum in TYPES' order.
        top = max(TYPES, key=lambda k: p[k]) if p else None
        tops[t] = {"expected": want, "top": top, "rates": p}
        matched += top == want
    informative = [t for t, p in priors.items() if prior_is_informative(p)]
    s = spend(cache)
    return {
        "model_id": MODEL_ID, "asked": asked, "of": len(TARGETS),
        "criterion_4": {"valid": valid, "bar": 38, "met": valid >= 38},
        "criterion_5": {"matched": matched, "of": len(EXPECTED), "bar": 18,
                        "met": matched >= 18, "rows": tops},
        "criterion_6": {"informative": len(informative), "bar": 30,
                        "met": len(informative) >= 30,
                        "flat": sorted(t for t in TARGETS if t not in informative)},
        "spend": {"calls": s["calls"], "usd": round(s["usd"], 4),
                  "input_tokens": s["in"], "output_tokens": s["out"]},
        "priors": priors,
    }


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "ask":
        cmd_ask(int(sys.argv[2]) if len(sys.argv) > 2 else len(TARGETS))
    elif cmd == "spend":
        print(json.dumps(spend(load())))
    elif cmd == "score":
        rep = score(load())
        with open(SCORE, "w") as f:
            json.dump(rep, f, indent=1)
        print(json.dumps({k: v for k, v in rep.items() if k != "priors"}, indent=1))
    else:
        raise SystemExit(__doc__)
