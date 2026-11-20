# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/room-prior-3116/llm_prior.py -- 3.116.C: a room prior asked of
an LLM, one call per target, cached.

    python evaluations/room-prior-3116/llm_prior.py ask      # paid: only targets not cached
    python evaluations/room-prior-3116/llm_prior.py score    # criteria 8 and 10 (free)

The learned prior (`evaluations/goal-learn-396/prior.json`, 3.96) is, per
target and room type, the share of the training houses' rooms of that type
holding at least one such object on the floor -- a rate per type, not a
distribution over types. The question asks for exactly that, over the same
four types. A reply must be one JSON object with those four keys, each a
number in [0, 1]; anything else is recorded as a failed reply and gives the
target no prior (never a guess).

The model is /room's (`service/vision_analyze/vision_core.py`
`ROOM_MODEL_ID`, Opus 5.5 on Bedrock, effort low: confirmed by real calls in
3.70 S2b). Cache: `llm_prior.json`, keyed by model id then target, with the
raw reply, so a re-run pays nothing for what it has.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
CACHE = os.path.join(HERE, "llm_prior.json")
LEARNED = os.path.join(ROOT, "evaluations", "goal-learn-396", "prior.json")

# /room's own model and any region pin, read from the service so the two
# never drift apart (review).
sys.path.insert(0, os.path.join(ROOT, "service", "vision_analyze"))
from vision_core import MODEL_REGIONS, ROOM_MODEL_ID  # noqa: E402
# 3.190.C: the question, the reply check and the four types live in the
# brain now (`brain/room_prior.py`), moved unchanged; this harness reads them.
sys.path.insert(0, ROOT)
from brain.room_prior import MAX_TOKENS, PROMPT, TYPES, parse  # noqa: E402,F401

MODEL_ID = ROOM_MODEL_ID
REGION = MODEL_REGIONS.get(MODEL_ID) or os.environ.get("AWS_REGION", "us-east-2")
# 3.96's nine real-object categories (criterion 8), then the everyday
# targets ProcTHOR's floor objects lack (criterion 10), fixed before the
# first call.
PROCTHOR = ("basketball", "box", "cart", "dog bed", "garbage bag", "garbage can",
            "house plant", "laundry hamper", "plunger")
EVERYDAY = ("knife", "keys", "phone charger", "toothbrush", "remote control",
            "red backpack", "shoes", "reading glasses", "wallet", "laptop",
            "cat toy", "baby bottle", "water bottle", "umbrella", "towel",
            "frying pan", "book", "headphones", "hair dryer", "pillow")


def _load() -> dict:
    return json.load(open(CACHE)) if os.path.exists(CACHE) else {}


def _save(cache: dict) -> None:
    tmp = CACHE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(cache, f, indent=1, sort_keys=True)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, CACHE)


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


def cmd_ask(limit: int = 0) -> None:
    import boto3
    client = boto3.client("bedrock-runtime", region_name=REGION)
    cache = _load()
    got = cache.setdefault(MODEL_ID, {})
    todo = [t for t in PROCTHOR + EVERYDAY if t not in got]
    if limit:
        todo = todo[:limit]
    for t in todo:
        got[t] = ask_one(client, t)
        _save(cache)                       # each paid answer kept at once
        print(t, json.dumps(got[t]["prior"]), got[t]["stop"], flush=True)


def cmd_score() -> dict:
    learned = json.load(open(LEARNED))
    got = _load().get(MODEL_ID, {})
    rows, diffs, sure = [], [], {}
    for t in PROCTHOR:
        llm, lp = (got.get(t) or {}).get("prior"), learned[t]
        if llm is None:
            rows.append({"target": t, "llm": None})
            continue
        top_l, top_m = max(TYPES, key=lambda k: lp[k]), max(TYPES, key=lambda k: llm[k])
        diffs += [abs(llm[k] - lp[k]) for k in TYPES]
        if lp[top_l] >= 0.6:
            sure[t] = top_l == top_m
        rows.append({"target": t, "learned_top": top_l, "learned_share": lp[top_l],
                     "llm_top": top_m, "llm": llm, "learned": {k: lp[k] for k in TYPES}})
    missing = [r["target"] for r in rows if r["llm"] is None]
    mad = round(sum(diffs) / len(diffs), 4) if diffs else None
    c8 = {"sure_categories": sure, "mean_abs_diff": mad,
          "met": not missing and all(sure.values()) and mad is not None and mad <= 0.20,
          "missing": missing}
    every = {t: (got.get(t) or {}).get("prior") for t in EVERYDAY}
    c10 = {"parsed": sum(v is not None for v in every.values()), "of": len(EVERYDAY),
           "met": all(v is not None for v in every.values())}
    report = {"model_id": MODEL_ID, "criterion_8": c8, "criterion_10": c10,
              "procthor": rows, "everyday": every}
    print(json.dumps({"criterion_8": c8, "criterion_10": c10}, indent=1))
    with open(os.path.join(HERE, "llm_prior_score.json"), "w") as f:
        json.dump(report, f, indent=1)
    return report


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ("ask", "score"):
        raise SystemExit(__doc__)
    if sys.argv[1] == "ask":
        cmd_ask(int(sys.argv[2]) if len(sys.argv) > 2 else 0)
    else:
        cmd_score()
