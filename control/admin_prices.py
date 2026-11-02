# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""control/admin_prices.py -- what a paid console action is expected to cost
(3.132 Stage 4).

The Lab console asks before it spends: Score, Re-score and Replay each open
a confirm naming the number of paid calls and an estimated cost. This module
is the arithmetic; the call counts come from admin_server.py, which knows
its own rules (how many frames the judge samples, how many collision checks
run).

The prices are THIS ACCOUNT'S billed Bedrock rates, read from Cost Explorer
(docs/evaluations/EVAL-navigate-models-2026-09-22.md section 9), not a
published list: Bedrock bills Claude separately from Anthropic's own API,
and on this account it came to exactly 10% over the first-party list price
on three models. Opus 5.5 is 3.70 S2b's basis, by the same rule. A model not
here gets no dollar figure -- the console says "price unknown" and still
asks -- because a guessed price on a confirm button is worse than none.
"""

from statistics import median
from typing import Optional

# Dollars per million tokens, (input, output), keyed by the Bedrock model id
# the vision service's allow-list and the judge use.
PRICES_PER_MTOK = {
    "us.anthropic.claude-opus-4-5-20251101-v1:0": (5.50, 27.50),
    "us.anthropic.claude-opus-5": (5.50, 27.50),
    "us.anthropic.claude-opus-5-5": (4.40, 22.00),
    "us.anthropic.claude-fable-5-1": (11.00, 55.00),
    "global.anthropic.claude-fable-5-1": (11.00, 55.00),
    "us.openai.gpt-6-astra": (11.00, 55.00),
}

# Tokens per call when a walk recorded no usage: one 1280x720 frame plus the
# /navigate prompt on Opus 4.5 (1,691 in, the same evaluation's section 7)
# and its typical reply (82-96 out, 3.80's fresh calls).
DEFAULT_TOKENS_IN = 1700
DEFAULT_TOKENS_OUT = 100

# What a judge call and a collision check can reply with at most: the caps
# control/walk_eval.py sends (judge_frame's maxTokens 200, check_collisions'
# 150). Priced at the cap, since a judge reply is not a /navigate reply.
JUDGE_OUT_TOKENS = 200
COLLISION_OUT_TOKENS = 150


def price_of(model_id: Optional[str]) -> Optional[tuple]:
    return PRICES_PER_MTOK.get(model_id or "")


def tokens_per_call(entries: list) -> dict:
    """The walk's median recorded /navigate usage, or the defaults. The same
    pictures go to the judge and to a replay, so the recording's own counts
    are the best basis there is before the calls are made.

    Input never below the default: GPT-6 Astra's Bedrock usage leaves out
    its image tokens (3.80.Q), so a walk recorded under it reports a
    text-only count that would price every other model's call at a fraction
    of what it bills."""
    ins, outs = [], []
    for e in entries:
        usage = (e.get("navigate") or {}).get("usage") or {}
        if isinstance(usage.get("input_tokens"), (int, float)):
            ins.append(usage["input_tokens"])
        if isinstance(usage.get("output_tokens"), (int, float)):
            outs.append(usage["output_tokens"])
    recorded = bool(ins and outs)
    return {
        "input": max(round(median(ins)), DEFAULT_TOKENS_IN) if recorded else DEFAULT_TOKENS_IN,
        "output": round(median(outs)) if recorded else DEFAULT_TOKENS_OUT,
        "recorded": recorded,
    }


def cost_usd(calls: int, model_id: Optional[str], tokens: dict) -> Optional[float]:
    """Dollars for `calls` calls of `tokens` each, or None when the model has
    no price here. Zero calls cost nothing whatever the model."""
    if calls <= 0:
        return 0.0
    price = price_of(model_id)
    if price is None:
        return None
    per_call = (tokens["input"] * price[0] + tokens["output"] * price[1]) / 1e6
    return round(calls * per_call, 6)
