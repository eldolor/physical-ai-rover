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

Writes `pilot-answers.jsonl` (one row per frame sent, appended, resumable:
a frame with a row is never sent again) and `pilot-summary.json` (the
measured cost per frame and C's estimate). Stops before the next frame
when (`STOP_*`): the measured spend passes $1.00; one call costs over
$0.10; a call is answered by a model other than the shipped one, or has no
token counts to price it; or 3 frames in a row fail.
"""

import hashlib
import json
import os
import statistics
import sys
import time

from common import HERE, RECORDINGS, SHIPPED_MODEL, jsonl, load_set

ANSWERS, SUMMARY = HERE / "pilot-answers.jsonl", HERE / "pilot-summary.json"
STOP_TOTAL_USD = 1.00
STOP_CALL_USD = 0.10
STOP_ERRORS_IN_A_ROW = 3
FULL_SET = 2261


def frame_entry(walk: str, frame: str) -> dict:
    """The walk's row for the frame, reduced to what replay_walk reads (a
    stream walk's `perceived: false` would make it skip a labelled frame)."""
    path = RECORDINGS / walk / "walk.jsonl"
    for line in (path.read_text().splitlines() if path.exists() else []):
        try:
            e = json.loads(line) if line.strip() else None
        except json.JSONDecodeError:
            continue
        if isinstance(e, dict) and e.get("file") == frame:
            return {"seq": e.get("seq"), "file": frame, "navigate": e.get("navigate")}
    return {"seq": None, "file": frame, "navigate": None}


def preconditions(pilot: dict, pay: bool) -> list:
    """Every reason not to send, or []. Each frame's bytes must hash to the
    pinned set's sha256 for that key."""
    sha = {r["key"]: r["sha256"] for r in load_set()["frames"]}
    bad = []
    for p in pilot["frames"]:
        path = RECORDINGS / p["key"]
        if not path.exists():
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


def summarize(rows: list, stopped) -> dict:
    ok = [r for r in rows if r.get("usd") is not None]
    per = statistics.mean(r["usd"] for r in ok) if ok else None
    errors = sum("error" in r for r in rows)
    out = {"sent": len(rows), "answered": len(ok), "errors": errors,
           "usd_total": round(sum(r["usd"] for r in ok), 6),
           "usd_per_frame_mean": round(per, 6) if per is not None else None,
           "input_tokens_median": statistics.median(r["input_tokens"] for r in ok) if ok else None,
           "output_tokens_median": statistics.median(r["output_tokens"] for r in ok) if ok else None,
           "rtt_ms_p50": statistics.median(r["rtt_ms"] for r in ok) if ok else None,
           "agree_with_label": sum(r["target_visible"] == r["visible"] for r in ok),
           "stopped": stopped}
    # C's estimate, the section's formula: frames x mean cost x (1 + the
    # pilot's failure share), the failure share as errors / sent.
    if per is not None:
        retry = errors / len(rows)
        out["estimate_C_all_2261_usd"] = round(FULL_SET * per * (1 + retry), 2)
        need = HERE / "cloud-counts.json"
        if need.exists():
            n = json.loads(need.read_text())["need_minimum"]
            out["estimate_C_minimum_usd"] = round(n * per * (1 + retry), 2)
            out["need_minimum"] = n
    return out


def main():
    sys.path.insert(0, str(HERE.parent.parent))
    from control import admin_prices, walk_replay
    from control.admin_server import post_navigate
    from control.brain_config import load_brain_config

    pay = "--pay" in sys.argv
    pilot = json.loads((HERE / "pilot.json").read_text())
    bad = preconditions(pilot, pay)
    if bad:
        raise SystemExit("not sent:\n  " + "\n  ".join(bad))
    if not pay:
        print(f"dry run: {pilot['n']} frames present and pinned; nothing sent. "
              f"At the default tokens one call is ${admin_prices.cost_usd(1, SHIPPED_MODEL, {'input': admin_prices.DEFAULT_TOKENS_IN, 'output': admin_prices.DEFAULT_TOKENS_OUT})}")
        return
    url, timeout_s = os.environ["VISION_URL"], load_brain_config()["replay_timeout_s"]
    rows = jsonl(ANSWERS) if ANSWERS.exists() else []
    done = {r["key"] for r in rows}
    stopped, in_a_row = None, 0
    for p in pilot["frames"]:
        if p["key"] in done:
            continue
        spent = sum(r["usd"] for r in rows if r.get("usd") is not None)
        if spent > STOP_TOTAL_USD:
            stopped = f"spent ${spent:.4f} > ${STOP_TOTAL_USD}"
            break
        walk, frame = p["key"].split("/", 1)
        image = (RECORDINGS / p["key"]).read_bytes()
        t0 = time.perf_counter()
        out = walk_replay.replay_walk(
            [frame_entry(walk, frame)], lambda e: image, p["target"],
            lambda img, target, model, variant: post_navigate(url, timeout_s, img, target, model, variant),
            max_workers=1)
        rtt = round((time.perf_counter() - t0) * 1000)
        row = {"key": p["key"], "object": p["object"], "visible": p["visible"], "rtt_ms": rtt,
               "usd": None}
        if out["entries"]:
            nav = out["entries"][0]["navigate"]
            usage = nav.get("usage") or {}
            ins, outs = usage.get("input_tokens"), usage.get("output_tokens")
            row.update({"target_visible": bool(nav.get("target_visible")), "model_id": nav.get("model_id"),
                        "input_tokens": ins, "output_tokens": outs, "action": nav.get("action")})
            if isinstance(ins, (int, float)) and isinstance(outs, (int, float)):
                row["usd"] = admin_prices.cost_usd(1, nav.get("model_id"), {"input": ins, "output": outs})
        else:
            row["error"] = (out["diff"][0].get("error") if out["diff"] else "no answer")
        rows.append(row)
        with ANSWERS.open("a") as f:
            f.write(json.dumps(row) + "\n")
        print(row, flush=True)
        if "error" in row:
            in_a_row += 1
            if in_a_row >= STOP_ERRORS_IN_A_ROW:
                stopped = f"{in_a_row} frames in a row failed"
                break
            continue
        in_a_row = 0
        if row.get("model_id") != SHIPPED_MODEL or row["usd"] is None:
            stopped = f"answered by {row.get('model_id')!r} (usd {row['usd']}), not {SHIPPED_MODEL}"
            break
        if row["usd"] > STOP_CALL_USD:
            stopped = f"one call cost ${row['usd']} > ${STOP_CALL_USD}"
            break
    summary = summarize(rows, stopped)
    SUMMARY.write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
