# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-perf-3136/perc_run.py

P1 and P2's perception load: `tools/jetson/perception_loop.py` itself (its
loop, timing, log lines and 4 Hz pacing, as 3.136.A's planted check read
them), run with `brain.perceive.pipeline_for` wrapped:

* every `perceive()` also appends `{"t0", "t1", "status"}` (epoch) to
  $PERC_LOG, for P4's overlap count (the loop's own lines carry whole
  seconds only);
* `--hazards` (P2): YOLOE is prompted with 3.74.C's five hazard classes
  (`cable`, `sock`, `shoe`, `pet bowl`, `toy`) beside the target, in ONE
  predict. Only the target's boxes go on to CLIP (a hazard needs no
  identity, 3.74.C); the hazard boxes are counted, per frame, in $PERC_LOG.
  Nothing else changes: same frames, same gate, same pipeline.

    PERC_LOG=x.jsonl python evaluations/jetson-perf-3136/perc_run.py [--hazards] <seconds>

Run from a checkout with the perception weights and `recordings/` (the
board's bench checkout). Nothing in brain/ or tools/ is edited.
"""

import io
import json
import os
import runpy
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import brain.perceive as bp  # noqa: E402

HAZARDS = ("cable", "sock", "shoe", "pet bowl", "toy")
LOG = os.environ.get("PERC_LOG", "")
hazards = "--hazards" in sys.argv
args = [a for a in sys.argv[1:] if a != "--hazards"]


class HazardYoloE(bp.YoloE):
    """YOLOE with the target plus the five hazard prompts (3.74.C)."""

    last_hazards = None

    def _prompts(self, text):
        return [text, *HAZARDS]

    def _set_text(self, text):  # pragma: no cover - needs the model
        import torch
        from ultralytics.utils.downloads import attempt_download_asset

        from brain.text_encoder import MOBILECLIP, YOLOE_TEXT_MODEL, yoloe_features

        self._classes = None
        if next(self.model.model.parameters()).is_meta:
            from ultralytics import YOLOE as _YE
            self.model = _YE(self.weights)
        net = self.model.model
        if getattr(net, "text_model", YOLOE_TEXT_MODEL) != YOLOE_TEXT_MODEL:
            # The shipped path encodes such a checkpoint in process; P2 would
            # not measure it (refuse rather than prompt with the wrong model).
            raise SystemExit(f"YOLOE text model {net.text_model!r}: not P2's pipeline")
        source = str(Path(attempt_download_asset(MOBILECLIP)).resolve())
        param = next(net.parameters())
        names = self._prompts(text)
        feats = torch.tensor([yoloe_features(source, t) for t in names],
                             dtype=torch.float32).reshape(1, len(names), -1)
        with torch.inference_mode():
            pe = net.model[-1].get_tpe(feats.to(param.device).to(param.dtype))
        self.model.set_classes(names, pe)

    def detect_text(self, image, text):  # pragma: no cover - needs the model
        from PIL import Image

        names = self._prompts(text)
        fresh = self._classes != names
        if fresh:
            self._set_text(text)
            self._classes = names
        img = Image.open(io.BytesIO(image)).convert("RGB")
        results = self.model.predict(img, conf=self.confidence, verbose=False, device=self.device)
        out, n_haz = [], 0
        for r in results:
            for b in getattr(r, "boxes", []):
                if int(b.cls[0]) != 0:
                    n_haz += 1           # a hazard box: counted, never sent to CLIP
                    continue
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
                out.append(bp.Detection(box=bp.Box(x1, y1, x2, y2), label=text,
                                        confidence=float(b.conf[0])))
        HazardYoloE.last_hazards = n_haz
        if fresh:
            self._release()
        return out


_pipeline_for = bp.pipeline_for


def pipeline_for(target, *a, **k):
    p = _pipeline_for(target, *a, **k)
    if hazards:
        backend = getattr(p.detector, "backend", None)
        if not isinstance(backend, bp.YoloE):
            raise SystemExit(f"--hazards needs YOLOE as the crop source, got {type(backend)}")
        backend.__class__ = HazardYoloE
        backend._classes = None
    inner = p.perceive

    def perceive(frame):
        HazardYoloE.last_hazards = None      # a frame the detector skips counts none
        t0 = time.time()
        r = inner(frame)
        if LOG:
            with open(LOG, "a") as f:
                f.write(json.dumps({"t0": round(t0, 6), "t1": round(time.time(), 6),
                                    "status": getattr(r, "status", None),
                                    "hazard_boxes": HazardYoloE.last_hazards if hazards else None})
                        + "\n")
        return r
    p.perceive = perceive
    return p


bp.pipeline_for = pipeline_for
print(f"perc_run: hazards {'on ' + ','.join(HAZARDS) if hazards else 'off'}", flush=True)
sys.argv = [str(ROOT / "tools/jetson/perception_loop.py"), *args]
runpy.run_path(sys.argv[0], run_name="__main__")
