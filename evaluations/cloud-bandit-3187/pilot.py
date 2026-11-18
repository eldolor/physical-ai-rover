# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/cloud-bandit-3187/pilot.py

3.187.B: `pilot.json`'s 20 frames, one at a time, through
`control/walk_replay.replay_walk` and `control/admin_server.post_navigate`
to the DEPLOYED /navigate (the shipped prompt and model). PAID: about 20
calls, sent only with `--pay`; without it a dry run checks every
precondition and sends nothing. Run on the desktop (it holds the frames and
the vision secret); see the section file's recipe.

    set -a; source ~/.vision-picar-local-secrets; set +a
    export VISION_SHARED_SECRET="$VISION_SECRET"
    export VISION_URL=https://d114x92g7i4syl.cloudfront.net
    PICAR_RECORDINGS=<recordings dir> .venv/bin/python evaluations/cloud-bandit-3187/pilot.py [--pay]

`pilot-answers.jsonl` is append-only: a `sent` row before each call, then
its result row. A frame with an answer is never sent again; a frame that
failed, or whose `sent` row has no result (the run died mid-call), is sent
again on the next run. `pilot-summary.json` holds the measured cost per
frame and C's estimate.

**Spend.** `usd_measured` prices each answer's own token counts. A call
that failed or never reported may still have been billed, for each of
`post_navigate`'s attempts, so the stop rule counts `usd_upper`: measured
plus `NAVIGATE_ATTEMPTS` default-token calls for every such frame.

**The stop rule** is read from the whole file before every frame, so it
holds across runs (`STOP_*`): `usd_upper` passes $1.00; an answer cost over
$0.10, came from a model other than the shipped one, or had no token counts;
or the last 3 results are failures. Clearing a stop is a person's decision:
read why, then move the file aside.
"""

import fcntl
import hashlib
import json
import os
import statistics
import sys
import time
from datetime import datetime, timezone

from common import HERE, RECORDINGS, SHIPPED_MODEL, jsonl, load_set, rows_by_file

ANSWERS, SUMMARY, LOCK = HERE / "pilot-answers.jsonl", HERE / "pilot-summary.json", HERE / ".pilot.lock"
STOP_TOTAL_USD = 1.00
STOP_CALL_USD = 0.10
STOP_ERRORS_IN_A_ROW = 3


def frame_entry(walk: str, frame: str) -> dict:
    """The walk's row for the frame, reduced to what replay_walk reads (a
    stream walk's `perceived: false` would make it skip a labelled frame)."""
    e = rows_by_file(walk)[0].get(frame) or {}
    return {"seq": e.get("seq"), "file": frame, "navigate": e.get("navigate")}


def preconditions(pilot: dict, pinned: dict, pay: bool) -> list:
    """Every reason not to send, or []. Each frame's bytes must hash to the
    pinned set's sha256 for that key."""
    sha = {r["key"]: r["sha256"] for r in pinned["frames"]}
    bad = [] if pilot.get("set_hash") == pinned["hash"] else ["pilot.json is not from the pinned set"]
    for p in pilot["frames"]:
        path = RECORDINGS / p["key"]
        if p["key"] not in sha:
            bad.append(f"{p['key']} is not in the pinned set")
        elif not path.exists():
            bad.append(f"missing {path}")
        elif hashlib.sha256(path.read_bytes()).hexdigest() != sha[p["key"]]:
            bad.append(f"{p['key']} is not the pinned frame")
    if pay:
        if not os.environ.get("VISION_URL", "").startswith("https://"):
            bad.append("VISION_URL is not an https URL")
        if len(os.environ.get("VISION_SHARED_SECRET", "")) < 16:
            bad.append("VISION_SHARED_SECRET is unset or short (VISION_SECRET in "
                       "~/.vision-picar-local-secrets)")
    return bad


def state(rows: list, unpriced_usd: float) -> dict:
    """The file read as frames: each key's last result, the keys whose last
    row is a `sent` with no result, the spend, and the stop rule's verdict."""
    last, results = {}, []
    for r in rows:
        last[r["key"]] = r
        if not r.get("sent"):
            results.append(r)
    answered = {k: r for k, r in last.items() if not r.get("sent") and "error" not in r}
    dangling = [k for k, r in last.items() if r.get("sent")]
    measured = sum(r["usd"] for r in results if isinstance(r.get("usd"), (int, float)))
    # Every failed result and every dangling call may have billed each attempt.
    unknown = sum(1 for r in results if "error" in r and r.get("usd") is None) + len(dangling)
    upper = measured + unknown * unpriced_usd
    stop = None
    for r in results:
        if "error" in r:
            continue
        if r.get("model_id") != SHIPPED_MODEL or r.get("usd") is None:
            stop = f"{r['key']} answered by {r.get('model_id')!r} (usd {r.get('usd')}), not {SHIPPED_MODEL}"
        elif r["usd"] > STOP_CALL_USD:
            stop = f"{r['key']} cost ${r['usd']} > ${STOP_CALL_USD}"
    tail = results[-STOP_ERRORS_IN_A_ROW:]
    if len(tail) == STOP_ERRORS_IN_A_ROW and all("error" in r for r in tail):
        stop = stop or f"the last {STOP_ERRORS_IN_A_ROW} results failed"
    if upper > STOP_TOTAL_USD:
        stop = stop or f"spend up to ${upper:.4f} > ${STOP_TOTAL_USD}"
    return {"answered": answered, "results": results, "dangling": dangling,
            "measured": measured, "upper": upper, "stop": stop}


def summarize(st: dict, n_full: int) -> dict:
    ok = list(st["answered"].values())
    failed = sum("error" in r for r in st["results"]) + len(st["dangling"])
    priced = [r for r in ok if isinstance(r.get("usd"), (int, float))]
    per = statistics.mean(r["usd"] for r in priced) if priced else None
    out = {"answered_frames": len(ok), "results": len(st["results"]), "failed": failed,
           "usd_measured": round(st["measured"], 6), "usd_upper": round(st["upper"], 6),
           "usd_per_frame_mean": round(per, 6) if per is not None else None,
           "input_tokens_median": statistics.median(r["input_tokens"] for r in priced) if priced else None,
           "output_tokens_median": statistics.median(r["output_tokens"] for r in priced) if priced else None,
           "rtt_ms_p50": statistics.median(r["rtt_ms"] for r in ok) if ok else None,
           "agree_with_label": sum(r["target_visible"] == r["visible"] for r in ok),
           "stopped": st["stop"]}
    # C's estimate, the section's formula: frames x mean cost x (1 + the
    # failure share), the failure share as failed / (failed + answered frames).
    if per is not None:
        retry = failed / (failed + len(ok))
        out[f"estimate_C_all_{n_full}_usd"] = round(n_full * per * (1 + retry), 2)
        counts = HERE / "cloud-counts.json"
        if counts.exists():
            c = json.loads(counts.read_text())
            if c.get("set_hash") == load_set()["hash"]:
                out["need_minimum"] = c["need_minimum"]
                out["estimate_C_minimum_usd"] = round(c["need_minimum"] * per * (1 + retry), 2)
            else:
                out["need_minimum"] = "cloud-counts.json is from another set: run count.py again"
    return out


def append(row: dict):
    with ANSWERS.open("a") as f:
        f.write(json.dumps(row) + "\n")
        f.flush()
        os.fsync(f.fileno())


def main():
    sys.path.insert(0, str(HERE.parent.parent))
    from control import admin_prices, walk_replay
    from control.admin_server import NAVIGATE_ATTEMPTS, post_navigate
    from control.brain_config import load_brain_config

    pay = "--pay" in sys.argv
    pinned = load_set()
    pilot = json.loads((HERE / "pilot.json").read_text())
    bad = preconditions(pilot, pinned, pay)
    if bad:
        raise SystemExit("not sent:\n  " + "\n  ".join(bad))
    one = admin_prices.cost_usd(1, SHIPPED_MODEL, {"input": admin_prices.DEFAULT_TOKENS_IN,
                                                   "output": admin_prices.DEFAULT_TOKENS_OUT})
    unpriced = NAVIGATE_ATTEMPTS * one
    if not pay:
        print(f"dry run: {pilot['n']} frames present and pinned; nothing sent. At the default "
              f"tokens one call is ${one}; a failed frame counts ${unpriced} toward the stop.")
        return
    lock = LOCK.open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit("another pilot.py holds the lock; nothing sent")
    url, timeout_s = os.environ["VISION_URL"], load_brain_config()["replay_timeout_s"]
    rows = jsonl(ANSWERS)
    st = state(rows, unpriced)
    for p in pilot["frames"]:
        if st["stop"]:
            break
        if p["key"] in st["answered"]:
            continue
        walk, frame = p["key"].split("/", 1)
        image = (RECORDINGS / p["key"]).read_bytes()
        sent = {"key": p["key"], "sent": datetime.now(timezone.utc).isoformat(timespec="seconds")}
        append(sent)
        rows.append(sent)
        t0 = time.perf_counter()
        out = walk_replay.replay_walk(
            [frame_entry(walk, frame)], lambda e: image, p["target"],
            lambda img, target, model, variant: post_navigate(url, timeout_s, img, target, model, variant),
            max_workers=1)
        row = {"key": p["key"], "object": p["object"], "visible": p["visible"],
               "rtt_ms": round((time.perf_counter() - t0) * 1000), "usd": None}
        if out["entries"]:
            nav = out["entries"][0]["navigate"]
            usage = nav.get("usage") or {}
            ins, outs = usage.get("input_tokens"), usage.get("output_tokens")
            row.update({"model_id": nav.get("model_id"), "input_tokens": ins, "output_tokens": outs,
                        "action": nav.get("action")})
            if isinstance(ins, (int, float)) and isinstance(outs, (int, float)):
                row["usd"] = admin_prices.cost_usd(1, nav.get("model_id"), {"input": ins, "output": outs})
            if isinstance(nav.get("target_visible"), bool):
                row["target_visible"] = nav["target_visible"]
            else:
                # Billed, but no answer to score: a failure that keeps its price.
                row["error"] = f"no boolean target_visible: {nav.get('target_visible')!r}"
        else:
            row["error"] = (out["diff"][0].get("error") if out["diff"] else "no answer")
        append(row)
        rows.append(row)
        print(row, flush=True)
        st = state(rows, unpriced)
    summary = summarize(st, pinned["n"])
    SUMMARY.write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
