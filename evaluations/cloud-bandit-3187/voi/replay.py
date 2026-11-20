# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/cloud-bandit-3187/voi/replay.py

3.187.G's baseline: today's fixed rule (the tree) replayed over the pinned
set, frame by frame, through the real `TieredVision`, built by
`control/brain_server._tiered_vision_fn` from `config/robot.yaml` (G0).
Free: every model is a stub that returns a recorded answer.

  * perception: `brain.perceive.pipeline_for` replaced by a pipeline that
    returns 3.92.A's perception row (`perception.jsonl`);
  * the local VLM: `brain.local_vlm.LocalVLMClient` replaced by a client
    that returns 3.92.A's VE row (`qwen-VE.jsonl`); `--arm F` only;
  * the cloud: a stub returning `cloud-answers.jsonl`'s answer through
    `brain.navigate.to_scene` (a frame with none raises, as a failed call).

`tier_async_cloud` is off (as 3.91 B6 and A2). Each walk is one mission.
Writes `fixed-<arm>.jsonl`: per frame, whether the tree asked the local
tier, the cloud, and the trigger.

    python evaluations/cloud-bandit-3187/voi/replay.py --arm F    # the tree
    python evaluations/cloud-bandit-3187/voi/replay.py --arm F0   # cloud-only
"""

import argparse
import contextlib
import json
import logging
import math
import sys
from pathlib import Path

VOI = Path(__file__).resolve().parent
sys.path.insert(0, str(VOI.parent))
from common import CAL, ROOT, jsonl, load_set, walk_of  # noqa: E402

import brain.local_vlm as local_vlm  # noqa: E402
import brain.perceive as perceive  # noqa: E402
from brain.navigate import to_scene  # noqa: E402
from brain.perceive import (ABSENT, CLIP_LOGIT_SCALE, DETECTED, Box, Candidate,  # noqa: E402
                            Detection, Perception)
from control.brain_config import load_brain_config  # noqa: E402

ANSWERS = VOI / "cloud-answers.jsonl"
STUB_URL = "http://local-vlm.stub"     # never dialled: the client is replaced
# The rig walks' frames carry no reliable clock in 3.92's rows; the replay's
# clock is the frame index at this spacing unless `cloud-answers.jsonl`
# carries the row's `t` (seconds since the walk's first frame).
DEFAULT_SPACING_S = 1.0


def load_rows(answers: Path = ANSWERS):
    per = {r["key"]: r for r in jsonl(CAL / "perception.jsonl")}
    ve = {r["key"]: r for r in jsonl(CAL / "qwen-VE.jsonl")}
    ans = {r["key"]: r for r in jsonl(answers)}
    return per, ve, ans


def perception_of(row: dict, hfov_deg: float) -> Perception:
    """3.92.A's row as the `Perception` the shipped pipeline returned."""
    size = (row["w"], row["h"])
    if row["status"] != DETECTED or not row.get("box"):
        return Perception(status=ABSENT, image_size=size, reason="recorded (3.92.A)")
    box = Box(*row["box"])
    p = min(max(float(row["p"]), 1e-6), 1 - 1e-6)
    # Scores whose softmax over (target, one distractor) is the recorded P.
    sim = math.log(p / (1 - p)) / CLIP_LOGIT_SCALE
    cand = Candidate(Detection(box, "recorded", 1.0), similarity=sim, best_distractor=0.0,
                     bearing_deg=(box.centre_x / row["w"] - 0.5) * hfov_deg, scores=(sim, 0.0))
    return Perception(status=DETECTED, candidates=[cand], best=cand, image_size=size,
                      reason="recorded (3.92.A)")


class RecordedPipeline(perceive.FrameReportedPipeline):
    """The shipped pipeline's place, answering from 3.92.A's rows."""

    NAME = "recorded perception (3.92.A)"

    def __init__(self, target: str, rows: dict, **_):
        super().__init__(target)
        self.detector = perceive._Named(self.NAME)
        self.rows = rows
        self.hfov_deg = 66.0

    def perceive(self, frame: dict) -> Perception:
        return perception_of(self.rows[frame["key"]], self.hfov_deg)


class Calls:
    """What the tree asked, per frame key."""

    def __init__(self):
        self.local, self.cloud, self.direction = set(), set(), set()


def recorded_local(ve: dict, per: dict, calls: Calls):
    class RecordedLocal:
        def __init__(self, url, target, *, secret=None, deadline_s=3.0, client=None):
            self.target = target

        def ask(self, frame: dict, box: bool = False) -> dict:
            k = frame["key"]
            calls.local.add(k)
            r = ve[k]
            got = {"status": r["status"], "score": r.get("score")}
            if box and r.get("box") is not None:
                w, h = local_vlm.model_size(per[k]["w"], per[k]["h"])
                x1, y1, x2, y2 = r["box"]
                got["box"] = r["box"]
                got["box_frac"] = [min(1.0, max(0.0, v))
                                   for v in (x1 / w, y1 / h, x2 / w, y2 / h)]
            return got

        __call__ = ask

        def direction(self, frame: dict) -> dict:
            calls.direction.add(frame["key"])
            return {"status": "unsupported", "score": None, "reason": "not recorded (3.187.G)"}

        def healthy(self, timeout=None) -> bool:
            return True

        def close(self) -> None:
            pass

    return RecordedLocal


def cloud_stub(ans: dict, target: str, calls: Calls):
    def cloud(frame: dict) -> dict:
        k = frame["key"]
        calls.cloud.add(k)
        a = ans.get(k)
        if a is None:
            raise RuntimeError(f"no recorded cloud answer for {k}")
        return to_scene({"action": a.get("action"), "target_visible": a["target_visible"],
                         "target_reached": a.get("target_reached"),
                         "target_direction": a.get("target_direction") or "not_visible"},
                        target)
    return cloud


@contextlib.contextmanager
def patched(module, name, value):
    old = getattr(module, name)
    setattr(module, name, value)
    try:
        yield
    finally:
        setattr(module, name, old)


def replay_walk(walk: str, frames: list, arm: str, per: dict, ve: dict, ans: dict,
                cfg: dict) -> list:
    """One walk as one mission; per frame what the tree asked."""
    from control.brain_server import _tiered_vision_fn

    target = frames[0]["target"]
    calls = Calls()
    cfg = dict(cfg, tier_async_cloud=False,
               tier_local_vlm_url=STUB_URL if arm == "F" else "")
    pipe = RecordedPipeline(target, per)
    with patched(perceive, "pipeline_for", lambda t, **kw: pipe), \
            patched(local_vlm, "LocalVLMClient", recorded_local(ve, per, calls)):
        tier = _tiered_vision_fn(target, cloud_stub(ans, target, calls), cfg)
    # One clock per walk: the recorded `t` only when every frame has one,
    # else the frame index (a mix would send the tier back in time).
    stamped = all(ans.get(f["key"], {}).get("t") is not None for f in frames)
    out = []
    try:
        for i, f in enumerate(frames):
            k = f["key"]
            t = ans[k]["t"] if stamped else i * DEFAULT_SPACING_S
            frame = {"key": k, "seq": i, "image_base64": "", "captured_at": float(t)}
            try:
                scene, err = tier(frame), None
            except Exception as e:  # noqa: BLE001 -- a raised call: B3.2 counts it, the mission goes on
                scene, err = {}, f"{type(e).__name__}: {e}"
            trig = (scene.get("_tier") or {}).get("trigger")
            out.append({"key": k, "local": k in calls.local, "cloud": k in calls.cloud,
                        "direction": k in calls.direction, "trigger": trig, "error": err})
    finally:
        tier.close()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--arm", choices=("F", "F0"), required=True)
    ap.add_argument("--answers", type=Path, default=ANSWERS)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    logging.basicConfig(level=logging.ERROR)
    d = load_set()
    per, ve, ans = load_rows(args.answers)
    cfg = load_brain_config()
    walks = {}
    for f in d["frames"]:
        walks.setdefault(walk_of(f["key"]), []).append(f)
    rows = []
    for w in sorted(walks):
        fr = sorted(walks[w], key=lambda f: f["key"])
        rows += replay_walk(w, fr, args.arm, per, ve, ans, cfg)
    path = args.out or VOI / f"fixed-{args.arm}.jsonl"
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    print(json.dumps({"arm": args.arm, "frames": len(rows),
                      "local": sum(r["local"] for r in rows),
                      "cloud": sum(r["cloud"] for r in rows),
                      "errors": sum(r["error"] is not None for r in rows),
                      "direction": sum(r["direction"] for r in rows)}))


if __name__ == "__main__":
    main()
