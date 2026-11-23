# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/room-prior-3190/b7.py -- 3.190.B criterion 7 (amendment 5): the
same question asked of another model (Sonnet 5.5 in amendment 5, stopped in
its pilot; Claude Fable 5.2 in amendment 6, `ACTIVE`), then compared with
Opus 5.5 on criterion 1's arm.

    python evaluations/room-prior-3190/b7.py ask [N]     # paid: the next N of the 40, then the nine
    python evaluations/room-prior-3190/b7.py spend
    python evaluations/room-prior-3190/b7.py score       # criteria 4-6 for ACTIVE (free) -> b7/score.json
    python evaluations/room-prior-3190/b7.py run OUT     # free: opus / challenger / none arms, one commit
    python evaluations/room-prior-3190/b7.py arms OUT    # free: (a) and (b) -> OUT/score.json

Only the model differs from B's Opus run: the question, the reply check, the
token cap and effort are `calls.py`'s (the brain's `room_prior`). Replies are
kept in the brain's cache shape (`b7/replies.json`, keyed by model id).
Spend is priced at each model's list plus this account's 10% Bedrock
mark-up (`MODELS`), and `ask` refuses a call that could take every model's
spend here past CAP_USD.
"""
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import arms                                                   # noqa: E402
import calls                                                  # noqa: E402
from arms import A, T, PROCTHOR, TYPES                        # noqa: E402
from calls import (MAX_TOKENS, PROMPT, SERVED, parse,         # noqa: E402
                   prior_is_informative, target_key)
from targets import EXPECTED, TARGETS                         # noqa: E402

# model key -> (Bedrock id, region, $/M (in, out)). Prices are list + this
# account's 10% Bedrock mark-up (amendments 5-6), derived, not billed figures.
MODELS = {
    # Amendment 5: stopped in its pilot (AWS Marketplace AccessDenied).
    "sonnet": ("us.anthropic.claude-sonnet-5-5", "us-east-2", (2.20, 11.00)),
    # Amendment 6 (the user's choice, 2026-11-23): pinned to us-east-1 as
    # vision_core.MODEL_REGIONS pins Fable 5.1, the only region Fable answered from.
    "fable": ("us.anthropic.claude-fable-5-2", "us-east-1", (8.80, 44.00)),
}
ACTIVE = "fable"
MODEL_ID, REGION, PRICE = MODELS[ACTIVE]
CAP_USD = 1.00                  # this run's cap, over every model asked here
CACHE = os.path.join(HERE, "b7", "replies.json")
SCORE = os.path.join(HERE, "b7", "score.json")
ASK_ORDER = tuple(TARGETS) + PROCTHOR      # the 40 (pilot first), then the nine
ARMS = ("opus", "challenger", "none")   # challenger = MODELS[ACTIVE]


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


def cost(usage, price=PRICE) -> float:
    u = usage or {}
    return (u.get("inputTokens", 0) * price[0] + u.get("outputTokens", 0) * price[1]) / 1e6


def spend(cache: dict, model_id: str = MODEL_ID) -> dict:
    """One model's calls; `usd_all` is every model's, which CAP_USD bounds."""
    prices = {mid: pr for mid, _, pr in MODELS.values()}
    got = cache.get(model_id, {})
    costs = [cost(e.get("usage"), prices[model_id]) for e in got.values()]
    every = sum(cost(e.get("usage"), prices[mid])
                for mid in prices for e in cache.get(mid, {}).values())
    return {"calls": len(costs), "usd": sum(costs), "dearest": max(costs, default=0.0),
            "usd_all": every,
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
    todo = [t for t in ASK_ORDER if target_key(t) not in got][:limit]
    for t in todo:
        s = spend(cache)
        worst = s["dearest"] or (2000 * PRICE[0] + MAX_TOKENS * PRICE[1]) / 1e6
        if s["usd_all"] + worst > CAP_USD:
            print(f"STOP: ${s['usd_all']:.4f} spent, the next call could pass ${CAP_USD}", flush=True)
            return
        got[target_key(t)] = ask_one(client, t)
        cache[SERVED] = MODEL_ID
        save(cache)                        # each paid answer kept at once
        e = got[target_key(t)]
        print(t, json.dumps(e["prior"]), e["stop"], e["usage"], f"${cost(e['usage']):.5f}",
              flush=True)
    print(json.dumps(spend(cache)), flush=True)


def score(cache: dict) -> dict:
    """Criteria 4-6 on the 40 only, by calls.score's own rules."""
    got = cache.get(MODEL_ID, {})
    priors = {t: (got.get(target_key(t)) or {}).get("prior") for t in TARGETS}
    valid = sum(p is not None for p in priors.values())
    tops, matched = {}, 0
    for t, want in EXPECTED.items():
        p = priors[t]
        top = max(TYPES, key=lambda k: p[k]) if p else None
        tops[t] = {"expected": want, "top": top, "rates": p}
        matched += top == want
    informative = [t for t, p in priors.items() if prior_is_informative(p)]
    s = spend(cache)
    return {
        "model_id": MODEL_ID, "asked": sum(target_key(t) in got for t in TARGETS),
        "of": len(TARGETS),
        "criterion_4": {"valid": valid, "bar": 38, "met": valid >= 38},
        "criterion_5": {"matched": matched, "of": len(EXPECTED), "bar": 18,
                        "met": matched >= 18, "rows": tops},
        "criterion_6": {"informative": len(informative), "bar": 30,
                        "met": len(informative) >= 30,
                        "flat": sorted(t for t in TARGETS if t not in informative)},
        "nine": {t: (got.get(target_key(t)) or {}).get("prior") for t in PROCTHOR},
        "spend": {"calls": s["calls"], "usd": round(s["usd"], 4),
                  "input_tokens": s["in"], "output_tokens": s["out"]},
        "priors": priors,
    }


# ---- criterion 1's arm (free) ----

def challenger_prior() -> dict:
    got = load().get(MODEL_ID, {})
    out = {}
    for t in PROCTHOR:
        p = (got.get(target_key(t)) or {}).get("prior")
        if p is None:
            raise SystemExit(f"no parsed {ACTIVE} prior for {t!r}")
        out[t] = dict(p)
    return out


def arm_prior(arm: str) -> dict:
    if arm == "opus":
        return arms.arm_prior("cloud")
    if arm == "challenger":
        return challenger_prior()
    if arm == "none":
        return {}
    raise ValueError(f"unknown arm {arm!r}")


def _use(arm: str) -> None:
    for t in PROCTHOR:
        T.PRIOR.pop(t, None)
    T.PRIOR.update(arm_prior(arm))


def cmd_run(out: str) -> None:
    """arms.cmd_run's rules: one commit for every arm, resumable per arm."""
    os.makedirs(out, exist_ok=True)
    meta, head = os.path.join(out, "run.json"), arms._head()
    if os.path.exists(meta):
        began = json.load(open(meta))["commit"]
        if began != head:
            raise SystemExit(f"{out} began at {began}, HEAD is {head}: start a new OUT_DIR")
    else:
        T.save_atomic(meta, lambda p: json.dump({"commit": head}, open(p, "w")))
    hs = A.houses("test")
    for arm in ARMS:
        path = os.path.join(out, f"{arm}-R.json")
        if os.path.exists(path):
            continue
        with ProcessPoolExecutor(T.WORKERS, initializer=_use, initargs=(arm,)) as ex:
            rows = list(ex.map(arms._job, [(s, h, arm) for s, h in hs], chunksize=2))
        T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))
        print(arm, T.summary(rows), flush=True)


def check(rows: dict) -> None:
    for arm, rs in rows.items():
        want = arm_prior(arm)
        for r in rs:
            t = r.get("target")
            if t is None:
                continue
            if r.get("arm") != arm or r.get("prior") != want.get(t):
                raise SystemExit(f"{arm} row {r['index']} read another prior -- not scoring")


def score_arms(rows: dict) -> dict:
    check(rows)
    s = {a: T.summary(rs) for a, rs in rows.items()}
    p_sn = arms.paired(rows["challenger"], rows["none"])
    p_on = arms.paired(rows["opus"], rows["none"])
    p_so = arms.paired(rows["challenger"], rows["opus"])
    a = {"paired_challenger_minus_none": p_sn, "goals": s["challenger"]["median_goals_found"],
         "goals_bar": s["none"]["median_goals_found"]}
    a["met"] = p_sn["ci95"][0] > 0 and a["goals"] <= a["goals_bar"]
    b = {"paired_challenger_minus_opus": p_so, "met": p_so["ci95"][0] > 0}
    return {"model_id": MODEL_ID, "summary": s, "a": a, "b": b,
            "beats_opus": a["met"] and b["met"],
            "reported": {"paired_opus_minus_none": p_on,
                         "by_category_challenger_minus_opus":
                             arms.by_category(rows["challenger"], rows["opus"]),
                         "by_category_challenger_minus_none":
                             arms.by_category(rows["challenger"], rows["none"])}}


def cmd_arms(out: str) -> dict:
    rows = {a: json.load(open(os.path.join(out, f"{a}-R.json"))) for a in ARMS}
    report = score_arms(rows)
    T.save_atomic(os.path.join(out, "score.json"),
                  lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps(report, indent=1))
    return report


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "ask":
        cmd_ask(int(sys.argv[2]) if len(sys.argv) > 2 else len(ASK_ORDER))
    elif cmd == "spend":
        print(json.dumps(spend(load())))
    elif cmd == "score":
        rep = score(load())
        with open(SCORE, "w") as f:
            json.dump(rep, f, indent=1)
        print(json.dumps({k: v for k, v in rep.items() if k != "priors"}, indent=1))
    elif cmd in ("run", "arms"):
        if len(sys.argv) != 3:
            raise SystemExit(__doc__)
        {"run": cmd_run, "arms": cmd_arms}[cmd](sys.argv[2])
    else:
        raise SystemExit(__doc__)
