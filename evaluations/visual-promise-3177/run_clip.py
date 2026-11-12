# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/visual-promise-3177/run_clip.py

3.177.A: one whole-frame CLIP RN50 encode a frame (the shipped `ClipScorer`'s
model, preprocess and text features), cosine against every description in
the set; plus the three square tiles (recorded, no bar). On the CPU only
(P24). Resumable. Free. `ms` times the whole-frame preprocess and encode
only: the frame is decoded once, outside it, as the board already holds it.

    .venv/bin/python evaluations/visual-promise-3177/run_clip.py [--limit N]
"""

import io
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from brain.perceive import ClipScorer                         # noqa: E402
from common import HERE, RECORDINGS, all_texts, load_set, rows   # noqa: E402

OUT = HERE / "clip-cpu.jsonl"


def tiles(w: int, h: int):
    """Left, centre and right squares (x1, y1, x2, y2) of a frame's short side."""
    s, long_ = min(w, h), max(w, h)
    return [(x, 0, x + s, s) if w >= h else (0, x, s, x + s)
            for x in (0, (long_ - s) // 2, long_ - s)]


def encode(scorer, img, feats):
    t = scorer.preprocess(img).unsqueeze(0).to(scorer.device)
    with scorer._torch.no_grad():
        f = scorer.model.encode_image(t)
        f /= f.norm(dim=-1, keepdim=True)
        return [round(float(v), 5) for v in (f @ feats.T)[0]]


def main():
    from PIL import Image
    limit = int(sys.argv[sys.argv.index("--limit") + 1]) if "--limit" in sys.argv else None
    d = load_set()
    order = all_texts(d)
    scorer = ClipScorer(device="cpu")
    feats = scorer._text_features(tuple(order))    # once, before any frame is timed
    seen = set(rows(OUT))
    n = 0
    with OUT.open("a") as f:
        for r in d["frames"]:
            if r["key"] in seen:
                continue
            if limit is not None and n >= limit:
                break
            img = Image.open(io.BytesIO((RECORDINGS / r["key"]).read_bytes())).convert("RGB")
            w, h = img.size
            t0 = time.perf_counter()
            full = encode(scorer, img, feats)
            ms = (time.perf_counter() - t0) * 1000
            row = {"key": r["key"], "device": scorer.device, "texts": order, "full": full,
                   "tiles": [encode(scorer, img.crop(b), feats) for b in tiles(w, h)],
                   "ms": round(ms, 1), "w": w, "h": h}
            f.write(json.dumps(row) + "\n"); f.flush()
            n += 1
            print(r["key"], row["ms"], flush=True)


if __name__ == "__main__":
    main()
