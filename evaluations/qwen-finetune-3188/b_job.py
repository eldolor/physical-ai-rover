# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/qwen-finetune-3188/b_job.py

3.188.B (amendment 2): what runs ON the rented GPU (a Hugging Face Job).
It is fetched by a presigned link and run alone, so it imports nothing
from this repo and judges nothing: it trains the folds it is told to,
asks each model its tasks and stores the replies; `b_score.py` judges in
the repo.

    python b_job.py stock            # the stock model, every fold's tasks
    python b_job.py train            # FOLDS=1 (the pilot) or FOLDS=2,3,4,5,6,all

Environment (job secrets; never logged):
  MANIFEST_URL    presigned GET: `b_manifest.py --stage`'s staged manifest
                  (frame links, every fold's training rows and tasks,
                  adapter upload links)
  RESULT_PUT_URL  presigned PUT: the rows, uploaded after each model and at the end
  FOLDS           with `train`: the folds, in order

Each row: `{"id", "key", "target", "mirror", "model", "size", "text", "lp"}`
-- `lp` as 3.214's 72B rows (per generated token: the token's raw piece,
its log-probability and the top 20, the alternatives for the first 16),
read from the raw logits, so `p_localised` scores it as it scores VE.
One line ends the log: `DONE {...}` (timings per fold).
"""

from __future__ import annotations

import io
import json
import math
import os
import random
import subprocess
import sys
import tarfile
import time
import urllib.request

# The student's question, word for word (tools/jetson/bench_identity_llama.py
# PROMPT; tests/test_qwen_finetune_b.py pins the two equal).
PROMPT = ('Output the bounding box of the {target} as JSON: '
          '[{{"bbox_2d": [x1, y1, x2, y2]}}]. If it is not visible, '
          'output []. Output JSON only.')
MODEL = "Qwen/Qwen2.5-VL-3B-Instruct"
REVISION = "66285546d2b821cf421d4f5eb2576359d3770cd3"
# Pinned: a transformers that has Qwen2.5-VL with the language model at
# `model.layers` (the LoRA pattern below), and a peft of the same season.
PIP = ["transformers==4.51.3", "peft==0.15.2", "accelerate==1.6.0", "pillow"]
# VE's image size: 128-512 image tokens of 28 x 28 pixels (3.188's design).
MIN_PIXELS, MAX_PIXELS = 128 * 28 * 28, 512 * 28 * 28
MAX_TOKENS = 64
TOP_LOGPROBS, TOP_FOR = 20, 16
EVAL_BATCH = 16

# Amendment 2's hyperparameters: fixed before the first fold, identical
# across folds, never tuned on a held-out object.
SEED = 3188
LORA_R, LORA_ALPHA, LORA_DROPOUT = 16, 32, 0.05
# The language model's attention and MLP projections only: the vision
# encoder and its merger get no adapter and stay frozen.
LORA_TARGETS = (r"^(?:base_model\.model\.)?model\.(?:language_model\.)?layers\.\d+\."
                r"(?:self_attn\.(?:q|k|v|o)_proj|mlp\.(?:gate|up|down)_proj)$")
LR, WARMUP_SHARE, EPOCHS = 2e-4, 0.05, 1
MICRO_BATCH, ACCUMULATE = 8, 2          # 16 samples a step
MAX_GRAD_NORM = 1.0
FAIL_FAST = 2                            # eval batches that all fail before the run stops
DEVICE = os.environ.get("B_DEVICE", "cuda")   # "cpu" only for the local smoke test


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


def put(url: str, body: bytes, tries: int = 3) -> bool:
    """A PUT to a presigned link; False (logged without the link) when it fails."""
    for i in range(tries):
        try:
            req = urllib.request.Request(url, data=body, method="PUT")
            with urllib.request.urlopen(req, timeout=300) as r:
                if 200 <= r.status < 300:
                    return True
        except Exception as e:  # noqa: BLE001
            log(f"upload failed: {getattr(e, 'code', type(e).__name__)}")
        time.sleep(3 * (i + 1))
    return False


def rows_body(rows: list) -> bytes:
    return "".join(json.dumps(r) + "\n" for r in rows).encode()


# ------------------------------------------------- pure helpers (tested) ---
def chat_prompt(target: str) -> list:
    """The one user turn VE sends: the image, then the question."""
    return [{"role": "user", "content": [{"type": "image"},
                                         {"type": "text", "text": PROMPT.format(target=target)}]}]


def reply_labels(ids: list, reply_ids: list, ignore: int = -100) -> list:
    """Labels for one training row: the reply's tokens (at the end of `ids`)
    are learned, everything before is not. Refuses a row whose tail is not
    the reply as tokenized alone (a merge across the boundary would teach
    the wrong tokens)."""
    n = len(reply_ids)
    if n == 0 or list(ids[-n:]) != list(reply_ids):
        raise ValueError("the reply is not the tail of the tokenized row")
    return [ignore] * (len(ids) - n) + list(reply_ids)


def lp_entry(i: int, token: str, logprob: float, top: list) -> dict:
    """One generated token as 3.214's rows hold it; alternatives kept for
    the first TOP_FOR tokens only (`p_localised` decides there)."""
    return {"token": token, "logprob": logprob,
            "top_logprobs": [{"token": t, "logprob": lp} for t, lp in top] if i < TOP_FOR else []}


def plan_steps(n_samples: int) -> tuple:
    """(optimizer steps, warmup steps) for one fold's training."""
    steps = math.ceil(n_samples * EPOCHS / (MICRO_BATCH * ACCUMULATE))
    return steps, max(1, math.ceil(WARMUP_SHARE * steps))


# --------------------------------------------------------- the GPU side ---
def install() -> None:  # pragma: no cover -- needs the job's image
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-q", *PIP])


def images(manifest: dict, keys: set) -> dict:  # pragma: no cover
    """Every frame the run needs, fetched once, as JPEG bytes."""
    from concurrent.futures import ThreadPoolExecutor

    keys = sorted(keys)
    with ThreadPoolExecutor(max_workers=32) as pool:
        got = list(pool.map(lambda k: fetch(manifest["urls"][k]), keys))
    return dict(zip(keys, got))


def pil(jpeg: bytes, mirror: bool):  # pragma: no cover
    from PIL import Image, ImageOps

    im = Image.open(io.BytesIO(jpeg)).convert("RGB")
    return ImageOps.mirror(im) if mirror else im


class Runner:  # pragma: no cover -- needs the GPU
    def __init__(self):
        import torch
        from transformers import AutoProcessor, Qwen2_5_VLForConditionalGeneration

        self.torch = torch
        self.proc = AutoProcessor.from_pretrained(MODEL, revision=REVISION,
                                                  min_pixels=MIN_PIXELS, max_pixels=MAX_PIXELS)
        self.proc.tokenizer.padding_side = "left"
        self.tok = self.proc.tokenizer
        self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
            MODEL, revision=REVISION, torch_dtype=torch.bfloat16,
            attn_implementation="sdpa").to(DEVICE)
        self.model.eval()
        self.end = self.tok.convert_tokens_to_ids("<|im_end|>")
        self.stops = {self.end, self.tok.convert_tokens_to_ids("<|endoftext|>")}

    def prompt_text(self, target: str) -> str:
        return self.proc.apply_chat_template(chat_prompt(target), tokenize=False,
                                             add_generation_prompt=True)

    # ---------------------------------------------------------- asking ---
    def ask(self, tasks: list, frames: dict, name: str) -> list:
        torch = self.torch
        rows, failed = [], 0
        for s in range(0, len(tasks), EVAL_BATCH):
            batch = tasks[s:s + EVAL_BATCH]
            try:
                ims = [pil(frames[t["key"]], t["mirror"]) for t in batch]
                enc = self.proc(text=[self.prompt_text(t["target"]) for t in batch], images=ims,
                                padding=True, return_tensors="pt").to(DEVICE)
                with torch.inference_mode():
                    out = self.model.generate(
                        **enc, do_sample=False, max_new_tokens=MAX_TOKENS,
                        repetition_penalty=1.0, temperature=None, top_p=None, top_k=None,
                        output_logits=True, return_dict_in_generate=True)
                n = enc["input_ids"].shape[1]
                gen = out.sequences[:, n:]
                logps = [torch.log_softmax(lg.float(), dim=-1) for lg in out.logits]
                for b, t in enumerate(batch):
                    lp = []
                    for i in range(gen.shape[1]):
                        tid = int(gen[b, i])
                        top = []
                        if i < TOP_FOR:
                            v, ix = logps[i][b].topk(TOP_LOGPROBS)
                            top = [(self.tok.convert_ids_to_tokens(int(j)), float(x)) for x, j in zip(v, ix)]
                        lp.append(lp_entry(i, self.tok.convert_ids_to_tokens(tid),
                                           float(logps[i][b, tid]), top))
                        if tid in self.stops:      # the reply's own end; padding follows
                            break
                    rows.append({"id": t["id"], "key": t["key"], "target": t["target"],
                                 "mirror": t["mirror"], "model": name, "size": list(ims[b].size),
                                 "text": self.tok.decode(gen[b], skip_special_tokens=True), "lp": lp})
            except Exception as e:  # noqa: BLE001 -- a row per task, never dropped
                failed += 1
                log(f"batch {s // EVAL_BATCH} failed: {type(e).__name__}: {str(e)[:200]}")
                rows += [{"id": t["id"], "key": t["key"], "target": t["target"], "mirror": t["mirror"],
                          "model": name, "error": f"{type(e).__name__}: {str(e)[:200]}"} for t in batch]
                if failed >= FAIL_FAST and all("error" in r for r in rows):
                    raise RuntimeError(f"{name}: the first {failed} batches all failed: stopping")
            if (s // EVAL_BATCH) % 20 == 0:
                log(f"{name}: {len(rows)}/{len(tasks)} asked")
        return rows

    # -------------------------------------------------------- training ---
    def train(self, fold: str, train: list, frames: dict) -> dict:
        torch = self.torch
        from peft import LoraConfig, get_peft_model
        from transformers import get_cosine_schedule_with_warmup

        random.seed(SEED)
        torch.manual_seed(SEED)
        cfg = LoraConfig(r=LORA_R, lora_alpha=LORA_ALPHA, lora_dropout=LORA_DROPOUT,
                         target_modules=LORA_TARGETS, bias="none", task_type="CAUSAL_LM")
        model = get_peft_model(self.model, cfg)
        names = [n for n, _ in model.named_modules() if n.endswith("lora_A")]
        if not names or any("visual" in n for n in names):
            raise RuntimeError(f"LoRA landed wrong: {len(names)} modules, visual ones: "
                               f"{[n for n in names if 'visual' in n][:2]}")
        trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
        log(f"fold {fold}: LoRA on {len(names)} modules, {trainable} trainable parameters")

        order = list(train)
        random.Random(SEED).shuffle(order)
        steps, warmup = plan_steps(len(order))
        opt = torch.optim.AdamW([p for p in model.parameters() if p.requires_grad], lr=LR,
                                weight_decay=0.0)
        sched = get_cosine_schedule_with_warmup(opt, warmup, steps)
        model.train()
        t0, losses, step = time.time(), [], 0
        micro = [order[i:i + MICRO_BATCH] for i in range(0, len(order), MICRO_BATCH)]
        for m, batch in enumerate(micro):
            enc, labels = self.encode_train(batch, frames)
            with torch.autocast(DEVICE, dtype=torch.bfloat16):
                loss = model(**enc, labels=labels).loss
            (loss / ACCUMULATE).backward()
            losses.append(float(loss))
            if (m + 1) % ACCUMULATE == 0 or m + 1 == len(micro):
                torch.nn.utils.clip_grad_norm_(
                    [p for p in model.parameters() if p.requires_grad], MAX_GRAD_NORM)
                opt.step()
                sched.step()
                opt.zero_grad(set_to_none=True)
                step += 1
                if step % 20 == 0 or step == steps:
                    log(f"fold {fold}: step {step}/{steps}, loss {sum(losses[-40:]) / len(losses[-40:]):.4f}, "
                        f"{time.time() - t0:.0f} s")
        model.eval()
        self.peft = model
        return {"samples": len(order), "steps": step, "planned_steps": steps,
                "train_s": round(time.time() - t0, 1), "trainable": trainable,
                "loss_first": round(sum(losses[:40]) / max(1, len(losses[:40])), 4),
                "loss_last": round(sum(losses[-40:]) / max(1, len(losses[-40:])), 4)}

    def encode_train(self, batch: list, frames: dict):
        torch = self.torch
        texts = [self.prompt_text(t["target"]) + t["answer"] + "<|im_end|>" for t in batch]
        enc = self.proc(text=texts, images=[pil(frames[t["key"]], t["mirror"]) for t in batch],
                        padding=True, return_tensors="pt")
        labels = torch.full_like(enc["input_ids"], -100)
        for b, t in enumerate(batch):
            reply = self.tok(t["answer"] + "<|im_end|>", add_special_tokens=False)["input_ids"]
            ids = enc["input_ids"][b].tolist()       # left padded: the reply ends the row
            labels[b] = torch.tensor(reply_labels(ids, reply))
        return enc.to(DEVICE), labels.to(DEVICE)

    def save_adapter(self, url: str) -> int:
        """The fold's adapter as a tar to the deploy bucket's private prefix;
        its size in bytes, or 0 when the upload failed."""
        path = "/tmp/adapter"
        self.peft.save_pretrained(path)
        buf = io.BytesIO()
        with tarfile.open(fileobj=buf, mode="w") as tar:
            tar.add(path, arcname="adapter")
        body = buf.getvalue()
        return len(body) if put(url, body) else 0

    def drop_adapter(self) -> None:
        self.model = self.peft.unload()      # the base weights, untouched (never merged)
        self.peft = None
        if DEVICE == "cuda":
            self.torch.cuda.empty_cache()


def preflight(manifest: dict) -> None:  # pragma: no cover
    """Before anything is installed or loaded: one frame and the result
    upload must work, or the job stops in its first seconds."""
    fetch(next(iter(manifest["urls"].values())))
    if not put(os.environ["RESULT_PUT_URL"], b""):
        raise RuntimeError("preflight: the result upload link does not work")
    log(f"preflight ok: {len(manifest['urls'])} frames listed")


def main(mode: str) -> dict:  # pragma: no cover -- needs the GPU
    t0 = time.time()
    manifest = json.loads(fetch(os.environ["MANIFEST_URL"]))
    preflight(manifest)
    install()
    result = os.environ["RESULT_PUT_URL"]
    folds = [f for f in (os.environ.get("FOLDS") or "").split(",") if f] if mode == "train" else []
    if mode == "train" and not folds:
        raise RuntimeError("train needs FOLDS")
    want = {t["key"] for t in manifest["stock"]} if mode == "stock" else set()
    for f in folds:
        want |= {t["key"] for t in manifest["folds"][f]["train"] + manifest["folds"][f]["asked"]}
    frames = images(manifest, want)
    fetched = time.time() - t0
    log(f"{len(frames)} frames fetched in {fetched:.0f} s")
    runner = Runner()
    ready = time.time() - t0
    rows, stats = [], {"mode": mode, "fetch_s": round(fetched), "ready_s": round(ready), "folds": {}}
    if mode == "stock":
        e0 = time.time()
        rows = runner.ask(manifest["stock"], frames, "stock")
        stats["stock"] = {"asked": len(rows), "errors": sum("error" in r for r in rows),
                          "ask_s": round(time.time() - e0, 1)}
        stats["uploaded"] = put(result, rows_body(rows))
    for f in folds:
        fold = manifest["folds"][f]
        st = runner.train(f, fold["train"], frames)
        st["adapter_bytes"] = runner.save_adapter(manifest["adapter_put"][f])
        e0 = time.time()
        got = runner.ask(fold["asked"], frames, f"fold-{f}")
        st.update(asked=len(got), errors=sum("error" in r for r in got), ask_s=round(time.time() - e0, 1))
        rows += got
        stats["folds"][f] = st
        stats["uploaded"] = put(result, rows_body(rows))
        log(f"fold {f} done: " + json.dumps(st))
        runner.drop_adapter()
    if not stats.get("uploaded"):
        for r in rows:            # the fallback: the rows in the log itself
            log("ROW " + json.dumps(r))
    return stats


if __name__ == "__main__":  # pragma: no cover
    started = time.time()
    out = main(sys.argv[1])
    log("DONE " + json.dumps({**out, "wall_s": round(time.time() - started)}))
