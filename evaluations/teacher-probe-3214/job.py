# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/teacher-probe-3214/job.py

3.214: what runs ON the rented GPU (a Hugging Face Job). It is fetched by
a presigned link and run alone, so it imports nothing from this repo and
judges nothing: it asks each frame's question and stores the reply; the
scoring (`score.py`, with `p_localised` and `parse_box`) happens in the repo.

    python job.py qwen     # Qwen2.5-VL-72B-Instruct-AWQ behind vLLM
    python job.py super    # Alpamayo 2 Super, one camera, one frame
                           # (SUPER_PIXELS=min,max: amendment 4's 800-token arm)

Environment (job secrets; never logged):
  MANIFEST_URL    presigned GET: the staged task list (`manifest.py --stage`)
  RESULT_PUT_URL  presigned PUT: the rows, uploaded every 100 and at the end
  LIMIT           optional: the pilot -- only the tasks the manifest marks
                  `pilot` (in view, out of view and one mirrored), at most N

Each row: `{"id", "key", "target", "mirror", "size": [w, h], ...}` plus,
for the 72B, `"text"` and `"lp"` (per generated token: the token, its
log-probability and the top 20 alternatives -- `p_localised`'s input), and
for Super `"identity"`/`"grounding"` replies with the processed image's
grid. One line of timing per run ends the log: `DONE {...}`.
"""

from __future__ import annotations

import base64
import io
import json
import os
import subprocess
import sys
import time
import urllib.request

# The student's question, word for word (tools/jetson/bench_identity_llama.py
# PROMPT; tests/test_teacher_probe_3214.py pins the two equal).
PROMPT = ('Output the bounding box of the {target} as JSON: '
          '[{{"bbox_2d": [x1, y1, x2, y2]}}]. If it is not visible, '
          'output []. Output JSON only.')
MAX_TOKENS = 64
TOP_LOGPROBS = 20
# VE's image size: 128-512 image tokens of 28 x 28 pixels (3.188's design).
MIN_PIXELS, MAX_PIXELS = 128 * 28 * 28, 512 * 28 * 28
QWEN_MODEL = "Qwen/Qwen2.5-VL-72B-Instruct-AWQ"
# Pinned: a release that serves Qwen2.5-VL and AutoAWQ checkpoints.
VLLM = "vllm==0.8.5.post1"
# vLLM 0.8.5 reads tokenizer attributes transformers 5 dropped (the first
# pilot died on `all_special_tokens_extended`): hold transformers to 4.x.
VLLM_TRANSFORMERS = "transformers==4.51.3"
SUPER_MODEL = "nvidia/Alpamayo2-Super"
SUPER_COMMIT = "59776920bbd981f2c989213bba5ee8594e71ed1d"
SUPER_TARBALL = f"https://github.com/NVlabs/alpamayo2/archive/{SUPER_COMMIT}.tar.gz"
# Amendment 1: Super's two questions.
IDENTITY_Q = "Is there {target} visible in this image? Answer yes or no."
GROUNDING_Q = "Find the {target} and return its bounding box in JSON."
SUPER_CAMERA_ID = 1          # front wide
PORT = 8000
TOP_FOR = 16                 # generated tokens whose alternatives are kept
FAIL_FAST = 5                # a run whose first five tasks all fail stops there


def log(*a) -> None:
    print(*a, flush=True)


def fetch(url: str, timeout: float = 60.0, tries: int = 4) -> bytes:
    """A presigned link's bytes. The link is never printed: an error names
    only the HTTP status."""
    for i in range(tries):
        try:
            with urllib.request.urlopen(url, timeout=timeout) as r:
                return r.read()
        except Exception as e:  # noqa: BLE001 -- the status only, never the link
            code = getattr(e, "code", type(e).__name__)
            if i == tries - 1:
                raise RuntimeError(f"fetch failed: {code}") from None
            time.sleep(2 * (i + 1))
    raise AssertionError("unreachable")


def upload(url: str, rows: list) -> bool:
    """The rows so far, as JSONL, to the presigned PUT link. False (logged
    without the link) when it fails: the run goes on and tries again."""
    body = "".join(json.dumps(r) + "\n" for r in rows).encode()
    try:
        req = urllib.request.Request(url, data=body, method="PUT")
        with urllib.request.urlopen(req, timeout=120) as r:
            return 200 <= r.status < 300
    except Exception as e:  # noqa: BLE001
        log(f"upload failed: {getattr(e, 'code', type(e).__name__)}")
        return False


def preflight() -> None:
    """Before anything is installed or loaded: the task list, one frame
    and the result upload must all work, or the job stops in its first
    seconds instead of after a model load."""
    tasks = json.loads(fetch(os.environ["MANIFEST_URL"]))
    fetch(tasks[0]["url"])
    put = os.environ.get("RESULT_PUT_URL")
    if put and not upload(put, []):
        raise RuntimeError("preflight: the result upload link does not work")
    log(f"preflight ok: {len(tasks)} tasks listed")


def load_tasks() -> list:
    tasks = json.loads(fetch(os.environ["MANIFEST_URL"]))
    limit = int(os.environ.get("LIMIT") or 0)
    if not limit:
        return tasks
    return [t for t in tasks if t.get("pilot")][:limit]


def image_of(task: dict):
    """(jpeg bytes, (w, h)) for a task; mirrored left-right when asked, at
    3.80.I's JPEG quality (common.mirrored)."""
    from PIL import Image, ImageOps

    raw = fetch(task["url"])
    im = Image.open(io.BytesIO(raw)).convert("RGB")
    if not task["mirror"]:
        return raw, im.size
    out = io.BytesIO()
    ImageOps.mirror(im).save(out, "JPEG", quality=92)
    return out.getvalue(), im.size


def run(tasks: list, ask, workers: int = 1) -> dict:
    """Ask every task, keep the rows in task order, upload as it goes.
    A task that raises is a row with `"error"`: counted, never dropped."""
    from concurrent.futures import ThreadPoolExecutor

    put = os.environ.get("RESULT_PUT_URL")
    rows, t0, ok = [], time.time(), True

    def one(task):
        try:
            jpeg, size = image_of(task)
            got = ask(task, jpeg)
        except Exception as e:  # noqa: BLE001
            got, size = {"error": f"{type(e).__name__}: {str(e)[:200]}"}, None
        return {"id": task["id"], "key": task["key"], "target": task["target"],
                "mirror": task["mirror"], "size": size, **got}

    with ThreadPoolExecutor(max_workers=workers) as pool:
        for i, row in enumerate(pool.map(one, tasks), 1):
            rows.append(row)
            if i == FAIL_FAST and all("error" in r for r in rows):
                # Dead links or a dead server: stop paying for nothing.
                log("ROW " + json.dumps(rows[0]))
                raise RuntimeError(f"the first {FAIL_FAST} tasks all failed: stopping")
            if i == FAIL_FAST or i % 100 == 0 or i == len(tasks):
                log(f"{i}/{len(tasks)} rows, {time.time() - t0:.0f} s, "
                    f"{sum('error' in r for r in rows)} errors")
                if put:
                    ok = upload(put, rows)
    asked_s = time.time() - t0
    if not put or not ok:
        # The fallback: the rows in the log itself, one JSON line each.
        for r in rows:
            log("ROW " + json.dumps(r))
    return {"rows": len(rows), "errors": sum("error" in r for r in rows),
            "asked_s": round(asked_s, 1), "uploaded": bool(put and ok and rows),
            "s_per_row": round(asked_s / max(1, len(rows)), 3)}


# ------------------------------------------------------------- the 72B ----
def qwen_body(jpeg: bytes, target: str) -> dict:
    """bench_identity_llama.ask's request: the image, then the question."""
    return {
        "model": QWEN_MODEL,
        "messages": [{"role": "user", "content": [
            {"type": "image_url", "image_url": {
                "url": "data:image/jpeg;base64," + base64.b64encode(jpeg).decode()}},
            {"type": "text", "text": PROMPT.format(target=target)}]}],
        "max_tokens": MAX_TOKENS, "temperature": 0,
        "logprobs": True, "top_logprobs": TOP_LOGPROBS,
    }


def qwen_row(reply: dict) -> dict:
    choice = reply["choices"][0]
    content = (choice.get("logprobs") or {}).get("content") or []
    # The alternatives only for the first TOP_FOR tokens: `p_localised`
    # decides at the token that opens the list (and the few after a bare
    # `[`), and the rows are re-uploaded as the run goes.
    return {"text": choice["message"]["content"],
            "lp": [{"token": c["token"], "logprob": c["logprob"],
                    "top_logprobs": [{"token": a["token"], "logprob": a["logprob"]}
                                     for a in (c.get("top_logprobs") or [])] if i < TOP_FOR else []}
                   for i, c in enumerate(content)]}


def main_qwen() -> dict:  # pragma: no cover -- needs the GPU
    t0 = time.time()
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-q", VLLM, VLLM_TRANSFORMERS, "pillow"])
    installed = time.time() - t0
    server = subprocess.Popen(
        [sys.executable, "-m", "vllm.entrypoints.openai.api_server",
         "--model", QWEN_MODEL, "--port", str(PORT), "--max-model-len", "4096",
         "--limit-mm-per-prompt", "image=1", "--max-logprobs", str(TOP_LOGPROBS),
         "--mm-processor-kwargs",
         json.dumps({"min_pixels": MIN_PIXELS, "max_pixels": MAX_PIXELS}),
         # The plain AWQ kernel: the second pilot loaded the model and then
         # died in vLLM's faster `awq_marlin` kernel (a CUDA illegal memory
         # access on its first forward pass).
         "--quantization", "awq",
         "--gpu-memory-utilization", "0.92"])
    base = f"http://127.0.0.1:{PORT}"
    for _ in range(360):                       # up to an hour: 43 GB to fetch and load
        if server.poll() is not None:
            raise RuntimeError(f"vLLM exited with {server.returncode} before it was ready")
        try:
            urllib.request.urlopen(base + "/health", timeout=5)
            break
        except Exception:  # noqa: BLE001
            time.sleep(10)
    else:
        raise RuntimeError("vLLM was not ready in an hour")
    ready = time.time() - t0

    def ask(task, jpeg):
        req = urllib.request.Request(
            base + "/v1/chat/completions", data=json.dumps(qwen_body(jpeg, task["target"])).encode(),
            headers={"content-type": "application/json"})
        with urllib.request.urlopen(req, timeout=300) as r:
            return qwen_row(json.loads(r.read()))

    try:
        stats = run(load_tasks(), ask, workers=16)
    finally:
        server.terminate()
    return {**stats, "model": QWEN_MODEL, "vllm": VLLM,
            "install_s": round(installed), "ready_s": round(ready)}


# ----------------------------------------------------------- Super --------
def super_frames(jpeg: bytes):
    """One frame as Alpamayo's `image_frames`: uint8 [camera 1, frame 1, 3, H, W]."""
    import numpy as np
    import torch
    from PIL import Image

    im = np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB")).copy()
    return torch.from_numpy(im).permute(2, 0, 1)[None, None].contiguous()


def main_super() -> dict:  # pragma: no cover -- needs the GPU
    t0 = time.time()
    import tarfile

    src = f"/tmp/alpamayo2-{SUPER_COMMIT}"          # the tarball's own top directory
    if not os.path.isdir(src):
        with tarfile.open(fileobj=io.BytesIO(fetch(SUPER_TARBALL, timeout=120)), mode="r:gz") as tar:
            tar.extractall("/tmp", filter="data")
    sys.path.insert(0, src + "/src")
    import torch
    from transformers import LogitsProcessorList

    from alpamayo2_super import helper
    from alpamayo2_super.models.alpamayo2_super import (
        Alpamayo2Super, MaskDiscreteTrajectoryLogitsProcessor)
    from alpamayo2_super.models.token_utils import extract_text_tokens
    from alpamayo2_super.text_tasks import prepare_vqa_inputs

    model = Alpamayo2Super.from_pretrained(SUPER_MODEL, dtype=torch.bfloat16, device_map="cuda:0")
    model.eval()
    pixels = os.environ.get("SUPER_PIXELS")
    if pixels:                       # amendment 4: the pixel budget, nothing else
        lo, hi = (int(v) for v in pixels.split(","))
        model.config.min_pixels, model.config.max_pixels = lo, hi
    log(f"pixels per image: {model.config.min_pixels}-{model.config.max_pixels}")
    ready = time.time() - t0
    log(f"loaded in {ready:.0f} s; GPU {torch.cuda.max_memory_allocated() / 2**30:.1f} GiB")
    mask = LogitsProcessorList([MaskDiscreteTrajectoryLogitsProcessor(
        traj_token_offset=min(model.config.traj_ids["history_id0"],
                              model.config.traj_ids["future_id0"]),
        traj_vocab_size=model.config.traj_vocab_size)])
    tok = model.tokenizer

    def generate(frames, question: str, max_new: int) -> dict:
        data = {"image_frames": frames, "camera_indices": torch.tensor([SUPER_CAMERA_ID])}
        inputs = prepare_vqa_inputs(data, model.config, tok, question)
        td = helper.to_device(dict(inputs["tokenized_data"]), "cuda")
        n = td["input_ids"].shape[1]
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
            # Greedy (amendment 1): `generate_text` samples at temperature 0.6.
            out = model.vlm.generate(
                **td, do_sample=False, max_new_tokens=max_new, logits_processor=mask,
                output_scores=True, return_dict_in_generate=True,
                pad_token_id=tok.pad_token_id)
        text = extract_text_tokens(tok, out.sequences[:, n:], task="vqa")
        first = torch.log_softmax(out.scores[0][0].float(), dim=-1).topk(10)
        grid = td.get("image_grid_thw")
        return {"text": text["raw_outputs"][0],
                "first": [[tok.decode([int(i)]), round(float(v), 4)]
                          for v, i in zip(first.values, first.indices)],
                "grid": grid[0].tolist() if grid is not None else None,
                "prompt_tokens": int(n)}

    def ask(task, jpeg):
        frames = super_frames(jpeg)
        return {"identity": generate(frames, IDENTITY_Q.format(target=task["target"]), 8),
                "grounding": generate(frames, GROUNDING_Q.format(target=task["target"]), 96)}

    stats = run(load_tasks(), ask, workers=1)
    return {**stats, "model": SUPER_MODEL, "repo": SUPER_COMMIT, "ready_s": round(ready),
            "pixels": [model.config.min_pixels, model.config.max_pixels],
            "gpu_gib": round(torch.cuda.max_memory_allocated() / 2**30, 1)}


if __name__ == "__main__":  # pragma: no cover
    started = time.time()
    preflight()
    stats = {"qwen": main_qwen, "super": main_super}[sys.argv[1]]()
    log("DONE " + json.dumps({**stats, "wall_s": round(time.time() - started)}))
