# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/steer-distill-3206/b2_bedrock.py

3.206.B2 (amendment 4): Qwen3-VL-235B's identity check. B1's question,
model, token cap and reply reading, unchanged, on the manifest's 2,268
frames (I-2261 and the 7 taupe-bag frames), unmirrored. B1's 100 unmirrored
S1 and S2 rows are the same question on the same frames and are read from
`qwen3vl-rows.jsonl`, not bought again. A yes is a parsed box; `[]` is a no.
Frames are read from the recordings bucket and sent to Bedrock only; none is
written anywhere. A re-run skips the ids already answered.

    .venv/bin/python evaluations/steer-distill-3206/b2_bedrock.py --pilot   # 2 calls
    .venv/bin/python evaluations/steer-distill-3206/b2_bedrock.py           # 2,166 more, stop at $3
    .venv/bin/python evaluations/steer-distill-3206/b2_bedrock.py --score   # b2-scores.json
    .venv/bin/python evaluations/steer-distill-3206/b2_bedrock.py --score --final
        # only when re-asking leaves frames unanswered: records B2-4 as measured, the rest unjudged
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Optional

HERE = Path(__file__).parent


def _by_path(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


b1 = _by_path("b1_3206", HERE / "b1_bedrock.py")
s3214 = b1.b0.s3214
mf = b1.mf

STOP_USD = 3.00
AT_ONCE = 4
FAIL_FAST = 12               # non-answers in a row, counted across the workers
# Refusals that never ran the model: not billed, so not counted as spend.
NOT_BILLED = {"ThrottlingException", "AccessDeniedException", "ValidationException",
              "ResourceNotFoundException", "ModelNotReadyException"}
ROWS = HERE / "qwen3vl-b2-rows.jsonl"
PILOT = HERE / "qwen3vl-b2-pilot.jsonl"
# The bars, written in amendment 4 before any call.
MAX_TEAL, MAX_TAUPE, MAX_MISSED, MIN_ANSWERED = 1, 0, 26, 0.99


def tasks() -> list:
    """Every manifest frame once, unmirrored, in the manifest's order."""
    return [{"id": key, "key": key, "target": row["target"], "mirror": False}
            for key, row in mf.build()["frames"].items()]


def reused() -> dict:
    """B1's answered unmirrored rows, by id: the same question on the same
    frame, so they are not asked again."""
    want = {t["id"] for t in tasks()}
    return {i: r for i, r in b1.latest(b1._load(b1.ROWS)).items()
            if i in want and not r["mirror"]}


def to_ask() -> list:
    have = reused()
    return [t for t in tasks() if t["id"] not in have]


def pilot_tasks() -> list:
    """The first frame people labelled in view and the first labelled not in
    view, among the frames still to ask: a box and an empty answer."""
    frames = mf.build()["frames"]
    ask = to_ask()
    seen = next(t for t in ask if frames[t["key"]]["visible"])
    unseen = next(t for t in ask if not frames[t["key"]]["visible"])
    return [seen, unseen]


def pilot_rows() -> dict:
    """The pilot's answered rows, by id: part of the run, never asked twice."""
    return b1.latest(b1._load(PILOT))


def pilot_passed() -> bool:
    """Both pilot replies are answers (parsed, in range, not cut off), and
    the in-view frame's reply holds a box: a reply format `parse_box` could
    not read would otherwise pass as two "no"s and buy the whole run."""
    rows = pilot_rows()
    seen, unseen = pilot_ids()
    return (seen["id"] in rows and unseen["id"] in rows
            and rows[seen["id"]]["frac_x"] is not None)


def pilot_ids() -> list:
    """The pilot's two tasks; `to_ask()` reads only B1's rows, so they do
    not change once the pilot itself has been answered."""
    return pilot_tasks()


def spent() -> float:
    return b1.cost(b1._load(PILOT)) + b1.cost(b1._load(ROWS))


def over_stop(spent_usd: float) -> bool:
    """True when another call may not start: the total so far, plus the
    worst case of every call that can be in flight at once, would reach the
    stop. Nothing is reserved per call; this fixed margin is what holds."""
    return spent_usd + AT_ONCE * b1.cost([{"billed_unknown": True}]) >= STOP_USD


def billed(e: Exception, asked: bool) -> bool:
    """Whether a failed call may have been billed: only one that reached
    Bedrock and was not refused before the model ran."""
    code = getattr(e, "response", {}).get("Error", {}).get("Code") if hasattr(e, "response") else None
    return asked and code not in NOT_BILLED


def run(pilot: bool) -> None:  # pragma: no cover -- paid calls
    import boto3
    from botocore.config import Config
    s3 = boto3.client("s3", region_name=mf.REGION)
    br = boto3.client("bedrock-runtime", region_name=b1.REGION,
                      config=Config(read_timeout=60, max_pool_connections=AT_ONCE * 2,
                                    retries={"mode": "standard", "max_attempts": b1.ATTEMPTS}))
    if not pilot and not pilot_passed():
        sys.exit("refused: the pilot has not passed (run --pilot first)")
    out = PILOT if pilot else ROWS
    answered = {r["id"] for r in b1._load(out) if r["status"] == "answered"} | set(pilot_rows())
    todo = [t for t in (pilot_tasks() if pilot else to_ask()) if t["id"] not in answered]
    lock = threading.Lock()
    state = {"spent": spent(), "errors_in_row": 0, "stop": None, "asked": 0}

    def one(t: dict) -> None:
        with lock:
            if state["stop"]:
                return
            if over_stop(state["spent"]):
                state["stop"] = f"${state['spent']:.4f} spent, at the ${STOP_USD} stop"
                return
        asked = False
        try:
            jpeg = s3.get_object(Bucket=mf.RECORDINGS_BUCKET,
                                 Key=f"{mf.RECORDINGS_PREFIX}/{t['key']}")["Body"].read()
            t0, asked = time.monotonic(), True
            got = br.converse(
                modelId=b1.MODEL,
                messages=[{"role": "user", "content": [
                    {"image": {"format": "jpeg", "source": {"bytes": jpeg}}},
                    {"text": b1.PROMPT.format(target=t["target"])}]}],
                inferenceConfig={"maxTokens": b1.MAX_TOKENS, "temperature": 0.0})
            row = {**b1.reply_row(t, got), "ms": round((time.monotonic() - t0) * 1000)}
        except Exception as e:                 # recorded, never read as a "no"
            row = {**t, "status": "error", "error": f"{type(e).__name__}: {e}"[:300],
                   "billed_unknown": billed(e, asked)}
        with lock:
            state["asked"] += 1
            # A kill mid-append leaves a line with no newline; start a fresh
            # line so the torn one stays the only unreadable one.
            torn = out.exists() and out.stat().st_size and not out.read_bytes().endswith(b"\n")
            with out.open("a") as f:           # one write per row, flushed
                f.write(("\n" if torn else "") + json.dumps(row) + "\n")
                f.flush()
            state["spent"] += b1.cost([row])
            state["errors_in_row"] = 0 if row["status"] == "answered" else state["errors_in_row"] + 1
            if state["errors_in_row"] >= FAIL_FAST and not state["stop"]:
                state["stop"] = f"{FAIL_FAST} non-answers in a row"
            n = len(b1._load(out)) if pilot else None
        if pilot or row["status"] != "answered":
            print(f"{row['id'][-46:]:46s} {row['status']:8s} {row.get('frac_x')} "
                  f"{(row.get('text') or row.get('error', ''))[:70]!r} n={n}")

    pool = ThreadPoolExecutor(max_workers=1 if pilot else AT_ONCE)
    try:
        list(pool.map(one, todo))
    except BaseException:                      # Ctrl-C, or a row that could not be written
        with lock:
            state["stop"] = state["stop"] or "interrupted"
        pool.shutdown(wait=True, cancel_futures=True)   # calls in flight finish and are recorded
        print(f"INTERRUPTED after {state['asked']} calls; ${state['spent']:.4f} spent")
        raise
    pool.shutdown(wait=True)
    print(f"asked {state['asked']} of {len(todo)}; ${state['spent']:.4f} spent"
          + (f"; STOP: {state['stop']}" if state["stop"] else ""))


def answers(rows: list) -> dict:
    """id -> `{"status", "yes", "frac_x"}`: this run's last answered row for
    each id, the pilot's, and B1's for the frames it already asked."""
    got = {**reused(), **pilot_rows(), **b1.latest(rows)}
    return {i: {"status": "answered", "yes": r["frac_x"] is not None, "frac_x": r["frac_x"]}
            for i, r in got.items()}


NOT_AN_ANSWER = {"truncated", "no_text", "out_of_range"}     # the model replied, unusably


def first_replies(rows: list) -> dict:
    """id -> the status of the model's first reply to it (the pilot, then
    the run). A call that errored is not a reply: it is a gap to re-ask."""
    first = {}
    for r in b1._load(PILOT) + rows:
        if r["status"] != "error":
            first.setdefault(r["id"], r["status"])
    return first


def first_try_share(rows: list) -> float:
    """B2-4: the share of frames whose first reply from the model was a
    parseable answer. B1's reused rows were; a frame the model has not yet
    replied to is not counted against it (the run is then not whole)."""
    first = first_replies(rows)
    want = [t["id"] for t in tasks()]
    return 1 - sum(first.get(i) in NOT_AN_ANSWER for i in want) / len(want)


def never_replied(rows: list) -> list:
    """Frames with no reply from the model at all: never asked, or only
    errors. `--final` is refused while there are any."""
    first, have = first_replies(rows), reused()
    return sorted(t["id"] for t in tasks() if t["id"] not in have and t["id"] not in first)


def scores(rows: list) -> dict:
    """Bars B2-1 to B2-4 (amendment 4), and what is reported beside them.
    B2-1 to B2-3 are judged only whole: a frame with no answer would read
    as a "no". B2-4 is the first-try share, so it can be recorded as failed."""
    man = mf.build()
    frames = man["frames"]
    ans = answers(rows)
    want = set(frames)
    missing = sorted(want - set(ans))
    unresolved = sorted({r["id"] for r in rows if r["status"] != "answered"} - set(ans))
    whole = not missing
    ident = [{"key": k, "visible": frames[k]["visible"], "yes": ans[k]["yes"]}
             for k in frames if "I2261" in frames[k]["sets"] and k in ans]
    pooled, objects = s3214.identity(ident), s3214.per_object(ident)
    ve = s3214.ve_answers()
    ve_objects = s3214.per_object(ve)
    look = {tag: sum(ans[k]["yes"] for k in frames if tag in frames[k]["sets"] and k in ans)
            for tag in ("teal", "taupe")}
    more_false_yes = sorted(o for o in objects
                            if objects[o]["false_yes"] > ve_objects[o]["false_yes"])
    # Reported: frame-by-frame agreement with the 72B's recorded answers.
    q72 = {r["id"]: r["yes"] for r in mf._jsonl(s3214.HERE / "qwen.jsonl") if not r["mirror"]}
    both = [k for k in ans if k in q72]
    share = first_try_share(rows)
    bars = {
        "B2-1 no object with more false yeses than VE": not more_false_yes,
        "B2-2 teal <= 1 and taupe 0": look["teal"] <= MAX_TEAL and look["taupe"] <= MAX_TAUPE,
        "B2-3 in-view frames without a box <= 26": pooled["missed"] <= MAX_MISSED,
        "B2-4 answered on >= 99%": share >= MIN_ANSWERED,
    }
    return {
        "coverage": {"expected": len(want), "answered": len(want & set(ans)),
                     "reused_from_b1": len(reused()), "missing": len(missing), "whole": whole,
                     "first_missing": missing[:3], "unresolved": unresolved[:5]},
        "cost_usd": round(b1.cost(rows) + b1.cost(b1._load(PILOT)), 4),
        "pooled": pooled, "per_object": objects,
        "ve_pooled": s3214.identity(ve), "ve_per_object": ve_objects,
        "lookalike_yes": look, "objects_with_more_false_yeses": more_false_yes,
        "mistakes_against_3214_criterion_1": {"mistakes": pooled["mistakes"],
                                              "bar": s3214.MAX_MISTAKES},
        "agreement_with_72b": {"frames": len(both),
                               "same": sum(ans[k]["yes"] == q72[k] for k in both)},
        "first_try_share": round(share, 4),
        "never_replied": len(never_replied(rows)),
        # Reported: a reply with no box that is not a bare empty list reads
        # as a "no"; a count above 0 means the replies need reading.
        "boxless_replies_not_an_empty_list": sum(
            r["frac_x"] is None and "".join((r.get("text") or "").split())
            not in ("[]", "```json[]```")
            for r in {**reused(), **pilot_rows(), **b1.latest(rows)}.values()),
        "bars": bars if whole else {k: (v if k.startswith("B2-4") else False)
                                    for k, v in bars.items()},
        "judged_whole": whole,
        "all_met": whole and all(bars.values()),
    }


def main(argv: Optional[list] = None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pilot", action="store_true")
    ap.add_argument("--score", action="store_true")
    ap.add_argument("--final", action="store_true",
                    help="with --score: record a run that re-asking could not make whole")
    args = ap.parse_args(argv)
    if args.score:
        out = scores(b1._load(ROWS))
        if args.final and out["never_replied"]:
            sys.exit(f"refused: {out['never_replied']} frames have no reply from the model yet "
                     "-- re-run; --final is for replies that stay unusable, not for gaps")
        if not out["coverage"]["whole"] and not args.final:
            sys.exit(f"not whole: {out['coverage']} -- re-run to fill the gaps before judging")
        (HERE / "b2-scores.json").write_text(json.dumps(out, indent=1) + "\n")
        print(json.dumps(out, indent=1))
        return
    run(args.pilot)


if __name__ == "__main__":
    main()
