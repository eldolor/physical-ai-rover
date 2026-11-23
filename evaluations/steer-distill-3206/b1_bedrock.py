# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/steer-distill-3206/b1_bedrock.py

3.206.B1 (amendment 3): Qwen3-VL-235B on Bedrock, asked VE's grounding
question on S1's 60 and S2's 40 steering frames, each also mirrored. Frames
are read from the recordings bucket and sent to Bedrock only; none is
written anywhere. Each reply's text, box and token counts are appended to
`qwen3vl-rows.jsonl` (the pilot's to `qwen3vl-pilot.jsonl`); a re-run skips
the ids already answered, so a crash never pays twice.

    .venv/bin/python evaluations/steer-distill-3206/b1_bedrock.py --pilot   # 2 calls
    .venv/bin/python evaluations/steer-distill-3206/b1_bedrock.py           # 200 calls, stop at $1
    .venv/bin/python evaluations/steer-distill-3206/b1_bedrock.py --score   # b1-scores.json
"""

from __future__ import annotations

import argparse
import importlib.util
import io
import json
import sys
import time
from pathlib import Path
from typing import Optional

HERE = Path(__file__).parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
from service.local_vlm.driver import parse_box                  # noqa: E402
from tools.jetson.bench_identity_llama import PROMPT              # noqa: E402


def _by_path(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


b0 = _by_path("b0_3206", HERE / "b0.py")
mf = b0.s3214.mf
third = b0.s3214.third

MODEL = "qwen.qwen3-vl-235b-a22b"
REGION = "us-east-2"
PRICE_IN, PRICE_OUT = 0.53e-6, 2.66e-6       # $ per token (amendment 3)
STOP_USD = 1.00
FAIL_FAST = 5                                 # consecutive errors stop the run
MAX_TOKENS = 128             # amendment 3, from the reviews: Qwen3-VL repeats the target as a label
# A call that raised after reaching Bedrock may still be billed: counted at
# this many tokens per attempt, so the stop errs on the side of stopping early.
ERROR_IN_TOK, ERROR_OUT_TOK, ATTEMPTS = 1500, MAX_TOKENS, 2
ROWS = HERE / "qwen3vl-rows.jsonl"
PILOT = HERE / "qwen3vl-pilot.jsonl"


def tasks(pilot: bool = False) -> list:
    """`{"id", "key", "set", "target", "mirror"}` for S1 then S2, each
    frame then its mirror. The pilot: the first S1 frame and its mirror."""
    man = mf.build()
    out = []
    for s in ("S1", "S2"):
        for key, v in man["steer"].items():
            if v["set"] == s:
                target = man["frames"][key]["target"]
                out.append({"id": key, "key": key, "set": s, "target": target, "mirror": False})
                out.append({"id": key + "#mirror", "key": key, "set": s, "target": target,
                            "mirror": True})
    return out[:2] if pilot else out


def mirrored(jpeg: bytes) -> bytes:
    """3.80.I's mirroring (JPEG q92), so the frames match every other arm's."""
    from PIL import Image, ImageOps
    buf = io.BytesIO()
    ImageOps.mirror(Image.open(io.BytesIO(jpeg)).convert("RGB")).save(buf, "JPEG", quality=92)
    return buf.getvalue()


def read_box(text: str) -> tuple:
    """`(frac_x, coords_ok)` from a reply: the first box's centre over 1000
    (Qwen3-VL's relative coordinates), and whether every coordinate lies in
    0-1000. No box: `(None, True)`."""
    box = parse_box(text or "")
    if box is None:
        return None, True
    ok = all(0.0 <= v <= 1000.0 for v in box)
    return min(1.0, max(0.0, (box[0] + box[2]) / 2 / 1000.0)), ok


def cost(rows: list) -> float:
    """Measured from the replies' token counts; a row whose call failed after
    reaching Bedrock (`billed_unknown`) counts at the error estimate."""
    total = 0.0
    for r in rows:
        if r.get("billed_unknown"):
            total += ATTEMPTS * (ERROR_IN_TOK * PRICE_IN + ERROR_OUT_TOK * PRICE_OUT)
        else:
            total += r.get("in_tok", 0) * PRICE_IN + r.get("out_tok", 0) * PRICE_OUT
    return total


def _load(path: Path) -> list:
    """Rows of a jsonl record; a torn last line (a crash mid-append) is skipped."""
    if not path.exists():
        return []
    out = []
    lines = [ln for ln in path.read_text().splitlines() if ln.strip()]
    for i, ln in enumerate(lines):
        try:
            out.append(json.loads(ln))
        except json.JSONDecodeError:
            if i != len(lines) - 1:
                raise
    return out


def reply_row(t: dict, got: dict) -> dict:
    """One Converse reply as a row. Every text block is read; a reply cut
    off at the token cap with no box, one with no text, or a box outside
    0-1000 is not an answer (status `truncated`, `no_text`, `out_of_range`)
    and blocks judging until it is resolved."""
    text = "".join(b.get("text", "") for b in got["output"]["message"]["content"])
    frac, ok = read_box(text)
    stop = got.get("stopReason")
    status = ("no_text" if not text else "out_of_range" if not ok
              else "truncated" if stop == "max_tokens" and frac is None else "answered")
    return {**t, "status": status, "text": text, "stop": stop,
            "frac_x": frac if ok else None, "coords_ok": ok,
            "in_tok": got["usage"]["inputTokens"], "out_tok": got["usage"]["outputTokens"]}


def pilot_passed() -> bool:
    """Amendment 3's gate: both pilot replies answered, in range, and the
    pair's thirds mirror-consistent."""
    rows = latest(_load(PILOT))
    a, b = (rows.get(t["id"]) for t in tasks(pilot=True))
    if not (a and b):
        return False
    return b0.SWAP.get(third(a["frac_x"])) == third(b["frac_x"]) and a["frac_x"] is not None


def run(pilot: bool) -> None:  # pragma: no cover -- paid calls
    import boto3
    from botocore.config import Config
    s3 = boto3.client("s3", region_name=mf.REGION)
    br = boto3.client("bedrock-runtime", region_name=REGION,
                      config=Config(read_timeout=60, retries={"mode": "standard",
                                                              "max_attempts": ATTEMPTS}))
    if not pilot and not pilot_passed():
        sys.exit("refused: the pilot has not passed amendment 3's gate (run --pilot first)")
    out = PILOT if pilot else ROWS
    done = _load(out)
    answered = {r["id"] for r in done if r["status"] == "answered"}
    spent, errors_in_row = cost(_load(PILOT)) + (0.0 if pilot else cost(done)), 0
    frames = {}
    for t in tasks(pilot):
        if t["id"] in answered:
            continue
        if spent >= STOP_USD:
            print(f"STOP: ${spent:.4f} spent, at the ${STOP_USD} stop")
            break
        asked = False                          # True once the call may have reached Bedrock
        try:
            if t["key"] not in frames:
                frames = {t["key"]: s3.get_object(Bucket=mf.RECORDINGS_BUCKET,
                                                  Key=f"{mf.RECORDINGS_PREFIX}/{t['key']}")["Body"].read()}
            jpeg = mirrored(frames[t["key"]]) if t["mirror"] else frames[t["key"]]
            t0, asked = time.monotonic(), True
            got = br.converse(
                modelId=MODEL,
                messages=[{"role": "user", "content": [
                    {"image": {"format": "jpeg", "source": {"bytes": jpeg}}},
                    {"text": PROMPT.format(target=t["target"])}]}],
                inferenceConfig={"maxTokens": MAX_TOKENS, "temperature": 0.0})
            row = {**reply_row(t, got), "ms": round((time.monotonic() - t0) * 1000)}
            errors_in_row = 0 if row["status"] == "answered" else errors_in_row + 1
        except Exception as e:                 # recorded, never read as a "no"
            row = {**t, "status": "error", "error": f"{type(e).__name__}: {e}"[:300],
                   "billed_unknown": asked}
            errors_in_row += 1
        with out.open("a") as f:               # one write per row, flushed
            f.write(json.dumps(row) + "\n")
            f.flush()
        spent += cost([row])
        print(f"{row['id'][-46:]:46s} {row['status']:8s} {row.get('frac_x')} "
              f"{(row.get('text') or row.get('error', ''))[:70]!r} ${spent:.4f}")
        if errors_in_row >= FAIL_FAST:
            print(f"STOP: {FAIL_FAST} errors in a row")
            break


def latest(rows: list) -> dict:
    """id -> its last answered row (a re-run may append a second copy)."""
    out = {}
    for r in rows:
        if r["status"] == "answered":
            out[r["id"]] = r
    return out


def scores(rows: list) -> dict:
    """Criterion 2 on S1 (amendment 3), and what it reports beside it."""
    want = {t["id"] for t in tasks()}
    ans = latest(rows)
    blocked = sorted({r["id"] for r in rows if r["status"] != "answered"} - set(ans))
    missing = sorted(want - set(ans))
    man = mf.build()["steer"]
    truth = b0.s1_truth()

    def a(i):
        return third(ans[i]["frac_x"]) if i in ans else None
    direct = {k: a(k) for k in truth}
    mirror = {k: a(k + "#mirror") for k in truth}
    kept = b0.kept(direct, mirror)
    k72 = b0.kept(*b0.qwen_answers(truth))
    kfl = b0.kept(*b0.florence_answers("fl-large-phrase", truth))
    s2 = [k for k, v in man.items() if v["set"] == "S2"]
    flags = {f"{r['walk']}/{r['frame']}": r.get("border", False) for r in
             json.loads((ROOT / "evaluations/tiered-380/I/labels-s1.json").read_text())["rows"]}
    return {
        "coverage": {"expected": len(want), "answered": len(want & set(ans)), "missing": len(missing),
                     "whole": not missing, "first_missing": missing[:3], "unresolved": blocked[:5]},
        "cost_usd": round(cost(rows) + cost(_load(PILOT)), 4),
        "criterion_2": b0.score(kept, truth),
        "s1_right_without_mirror_rule": sum(direct[k] == truth[k] for k in truth),
        "s2_boxes": sum(a(k) is not None for k in s2),
        "border_wrong_kept": sorted(k for k, v in kept.items() if v != truth[k] and flags.get(k)),
        "nonborder_wrong_kept": sorted(k for k, v in kept.items() if v != truth[k] and not flags.get(k)),
        "pairs": {"Qwen3-VL + 72B": b0.score(b0.pair(kept, k72), truth),
                  "Qwen3-VL + Florence-2-large": b0.score(b0.pair(kept, kfl), truth)},
    }


def main(argv: Optional[list] = None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pilot", action="store_true")
    ap.add_argument("--score", action="store_true")
    args = ap.parse_args(argv)
    if args.score:
        out = scores(_load(ROWS))
        if not out["coverage"]["whole"]:
            sys.exit(f"not whole: {out['coverage']} -- re-run to fill the gaps before judging")
        (HERE / "b1-scores.json").write_text(json.dumps(out, indent=1) + "\n")
        print(json.dumps(out, indent=1))
        return
    run(args.pilot)


if __name__ == "__main__":
    main()
