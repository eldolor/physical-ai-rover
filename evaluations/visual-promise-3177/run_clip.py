# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/visual-promise-3177/run_clip.py

3.177.A: one whole-frame CLIP RN50 encode a frame (the shipped `ClipScorer`
and preprocess), cosine against all six objects' words; plus the three
square tiles (recorded, no bar). On the CPU only (P24). Resumable. Free.

    .venv/bin/python evaluations/visual-promise-3177/run_clip.py [--limit N]
"""

import io
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from brain.perceive import Box, ClipScorer          # noqa: E402
from common import HERE, RECORDINGS, done, load_set, texts   # noqa: E402

OUT = HERE / "clip-cpu.jsonl"


def tiles(w: int, h: int):
    """Left, centre and right squares of side h (a landscape frame)."""
    s = min(w, h)
    return [Box(x, 0, x + s, s) if w >= h else Box(0, x, s, x + s)
            for x in (0, (max(w, h) - s) // 2, max(w, h) - s)]


def main():
    from PIL import Image
    limit = int(sys.argv[sys.argv.index("--limit") + 1]) if "--limit" in sys.argv else None
    d = load_set()
    words = texts(d)
    objects = sorted(words)
    order = tuple(words[o] for o in objects)
    scorer = ClipScorer(device="cpu")
    seen = done(OUT)
    n = 0
    with OUT.open("a") as f:
        for r in d["frames"]:
            if r["key"] in seen:
                continue
            if limit is not None and n >= limit:
                break
            jpeg = (RECORDINGS / r["key"]).read_bytes()
            w, h = Image.open(io.BytesIO(jpeg)).size
            t0 = time.perf_counter()
            full = scorer.score(jpeg, Box(0, 0, w, h), order)
            ms = (time.perf_counter() - t0) * 1000
            tile = [scorer.score(jpeg, b, order) for b in tiles(w, h)]
            row = {"key": r["key"], "device": scorer.device, "objects": objects,
                   "full": [round(v, 5) for v in full],
                   "tiles": [[round(v, 5) for v in t] for t in tile],
                   "ms": round(ms, 1), "w": w, "h": h}
            f.write(json.dumps(row) + "\n"); f.flush()
            n += 1
            print(r["key"], row["ms"], flush=True)


if __name__ == "__main__":
    main()
