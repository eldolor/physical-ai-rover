# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/qwen-finetune-3188/resolve.py

3.188.A2: resolve split.json's folds to real frame files, check the 299
arrival frames (3.48) against the 2,261, and list the lookalike frames
(3.48's teal bin, 3.91 B4's taupe bag).

Reads walks from the recordings bucket (S3WalkStore's layout:
`recordings/<walk>/{labels.json,walk.jsonl,frame-NNNN.jpg}`) with the
environment's AWS credentials. It LISTS every frame object (key, size, ETag)
and fetches only `labels.json` and `walk.jsonl` per walk -- never a frame.
Commits keys, sizes, ETags and counts only (recordings are home images).

    PICAR_RECORDINGS_BUCKET=<bucket> .venv/bin/python \
        evaluations/qwen-finetune-3188/resolve.py [--cache DIR]

Writes `frames.json` (per walk: label + file records), `overlap-299.json`,
`lookalikes.json`.
"""

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).parent
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HERE))
import split as split_mod  # noqa: E402  (obj() and the pinned source rows)

BUCKET = os.environ.get("PICAR_RECORDINGS_BUCKET",
                        "vision-picar-recordings-303351622021-us-east-2")
PREFIX = "recordings"

# 3.48's pinned arrival set (docs/plans/ros-alignment/3.48-local-identity.md)
ARRIVAL_WALKS = ["blue-bottle-20260907-142454", "blue-bottle-20260907-185007",
                 "blue-shoes-20260907-152528", "red-backpack-20260907-144856"]
ARRIVAL_COUNT = 299
TEAL_WALK = "blue-bottle-20260907-185007"
TEAL_COUNT = 34                      # 3.48's recorded size
TAUPE_WALK = "red-backpack-gpt-6-astra-20261021-205737"
TAUPE_PROBES = ROOT / "evaluations/tier-fusion-391/B6-replay-205737.json"
TAUPE_COUNT = 7                      # 3.91 B4's detected frames
OPUS_MODEL = "us.anthropic.claude-opus-4-5-20251101-v1:0"


def client():
    import boto3
    return boto3.client("s3")


def list_frames(s3, walk: str) -> dict:
    """{file: {size, etag}} for every frame-*.jpg under the walk."""
    out, token = {}, None
    while True:
        kw = dict(Bucket=BUCKET, Prefix=f"{PREFIX}/{walk}/frame-")
        if token:
            kw["ContinuationToken"] = token
        page = s3.list_objects_v2(**kw)
        for o in page.get("Contents", []):
            name = o["Key"].rsplit("/", 1)[1]
            out[name] = {"size": o["Size"], "etag": o["ETag"].strip('"')}
        if not page.get("IsTruncated"):
            return out
        token = page["NextContinuationToken"]


def fetch(s3, walk: str, name: str, cache: Path) -> bytes:
    p = cache / walk / name
    if not p.exists():
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(s3.get_object(Bucket=BUCKET,
                                    Key=f"{PREFIX}/{walk}/{name}")["Body"].read())
    return p.read_bytes()


def jsonl(b: bytes):
    return [json.loads(l) for l in b.decode().splitlines() if l.strip()]


def main(cache: Path) -> dict:
    s3 = client()
    rs = split_mod.rows()                     # 3.92's 2,261 rows (key, target, visible)
    by_key = {r["key"]: r for r in rs}
    walks = sorted({r["key"].split("/")[0] for r in rs})

    # --- frames.json: every walk, its labels today and its files on S3
    frames, mismatch, missing = {}, [], []
    for w in walks:
        labels = json.loads(fetch(s3, w, "labels.json", cache))
        files = list_frames(s3, w)
        tv = labels["target_visible_labels"]
        for f in sorted(tv):
            k = f"{w}/{f}"
            if f not in files:
                missing.append(k)
            if k not in by_key:
                mismatch.append({"key": k, "why": "labelled today, not in 3.92's set"})
            elif by_key[k]["visible"] != tv[f]:
                mismatch.append({"key": k, "why": "label changed since 3.92",
                                 "3.92": by_key[k]["visible"], "today": tv[f]})
        for k in (k for k in by_key if k.startswith(w + "/")):
            if k.split("/")[1] not in tv:
                mismatch.append({"key": k, "why": "in 3.92's set, no label today"})
        frames[w] = {"object": split_mod.obj(labels["description"]),
                     "description": labels["description"],
                     "labelled": len(tv), "in_view": sum(1 for v in tv.values() if v),
                     "frames": {f: {"visible": tv[f], **files.get(f, {"size": None, "etag": None})}
                                for f in sorted(tv)}}

    # --- the folds, resolved to files
    folds = {}
    for o, f in json.loads((HERE / "split.json").read_text())["folds"].items():
        held = sorted(f["held_out_walks"])
        train = [w for w in walks if frames[w]["object"] != o]
        n = lambda ws: sum(frames[w]["labelled"] for w in ws)
        folds[o] = {"held_out_walks": held, "train_walks": train,
                    "held_frames": n(held), "train_frames": n(train),
                    "split_json_train_frames": f["train_frames"],
                    "agrees_with_split_json": n(train) == f["train_frames"]
                    and n(held) == f["stock"]["frames"]}
    all_keys = sorted(f"{w}/{x}" for w in walks for x in frames[w]["frames"])
    out = {"bucket": BUCKET, "prefix": PREFIX,
           "read_from": "S3 (list + labels.json only; no frame bytes read)",
           "walks": len(walks), "frames": len(all_keys),
           "keys_sha256_12": hashlib.sha256("\n".join(all_keys).encode()).hexdigest()[:12],
           "etags_sha256_12": hashlib.sha256("\n".join(
               f"{w}/{x}:{frames[w]['frames'][x]['etag']}" for w in walks
               for x in frames[w]["frames"]).encode()).hexdigest()[:12],
           "missing_on_s3": missing, "label_vs_3.92": mismatch, "folds": folds}
    (HERE / "frames.json").write_text(json.dumps({"summary": out, "walks": frames}, indent=1) + "\n")
    (HERE / "folds.json").write_text(json.dumps(out, indent=1) + "\n")

    # --- the 299 vs the 2,261
    arrival = [f"{w}/{x}" for w in ARRIVAL_WALKS for x in sorted(frames[w]["frames"])] \
        if all(w in frames for w in ARRIVAL_WALKS) else []
    in_set = [k for k in arrival if k in by_key]
    ov = {"arrival_walks": ARRIVAL_WALKS, "arrival_frames": len(arrival),
          "arrival_frames_expected": ARRIVAL_COUNT,
          "in_2261": len(in_set), "not_in_2261": len(arrival) - len(in_set),
          "per_walk": {w: {"frames": frames[w]["labelled"],
                           "in_view": frames[w]["in_view"], "object": frames[w]["object"],
                           "fold": frames[w]["object"]} for w in ARRIVAL_WALKS},
          "in_view": sum(frames[w]["in_view"] for w in ARRIVAL_WALKS),
          "rule": "judge each frame only by the fold model that held out its "
                  "object (folds.json: the fold named by per_walk[...].fold)"}
    (HERE / "overlap-299.json").write_text(json.dumps(ov, indent=1) + "\n")

    # --- the lookalikes
    wj = jsonl(fetch(s3, TEAL_WALK, "walk.jsonl", cache))
    tv = json.loads(fetch(s3, TEAL_WALK, "labels.json", cache))["target_visible_labels"]
    teal, models = [], set()
    for r in wj:
        nav = r.get("navigate") or {}
        models.add(nav.get("model_id"))
        if r["file"] in tv and not tv[r["file"]] and nav.get("target_visible") is True \
                and nav.get("model_id") in (OPUS_MODEL, None):
            teal.append(f"{TEAL_WALK}/{r['file']}")
    tw = jsonl(fetch(s3, TAUPE_WALK, "walk.jsonl", cache))
    seq_file = {r["teleop_seq"]: r["file"] for r in tw if "teleop_seq" in r}
    probes = [p["seq"] for p in json.loads(TAUPE_PROBES.read_text())["probes"]]
    taupe = [f"{TAUPE_WALK}/{seq_file[s]}" for s in probes if s in seq_file]
    tfiles = list_frames(s3, TAUPE_WALK)
    look = {"teal_bin": {"walk": TEAL_WALK, "expected": TEAL_COUNT, "frames": len(teal),
                         "in_2261": all(k in by_key for k in teal), "keys": teal,
                         "model_ids_seen": sorted(m for m in models if m),
                         "rule": "label false and Opus' recorded navigate.target_visible true"},
            "taupe_bag": {"walk": TAUPE_WALK, "expected": TAUPE_COUNT, "frames": len(taupe),
                          "in_2261": [k in by_key for k in taupe].count(True),
                          "on_s3": sum(k.rsplit('/', 1)[1] in tfiles for k in taupe),
                          "probe_teleop_seqs": probes, "keys": taupe,
                          "rule": "3.91 B4's 7 DETECTED asked frames, "
                                  "B6-replay-205737.json probes, teleop_seq -> file"},
            "fold_note": "teal bin (blue-bottle target): judge by the blue bottle "
                         "fold; taupe (red-backpack target): by the red backpack fold"}
    (HERE / "lookalikes.json").write_text(json.dumps(look, indent=1) + "\n")
    return {"folds": out, "overlap": ov, "look": look}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", type=Path, required=True,
                    help="scratch dir for labels.json / walk.jsonl (never the repo)")
    a = ap.parse_args()
    if ROOT in a.cache.resolve().parents or a.cache.resolve() == ROOT:
        sys.exit("cache must be outside the repo")
    r = main(a.cache)
    o = r["folds"]
    print(o["walks"], "walks,", o["frames"], "frames; missing on S3:", len(o["missing_on_s3"]),
          "; label diffs:", len(o["label_vs_3.92"]))
    for k, f in o["folds"].items():
        print(f"  {k:30s} held {f['held_frames']:4d} train {f['train_frames']:4d} "
              f"agrees {f['agrees_with_split_json']}")
    v = r["overlap"]
    print("299:", v["arrival_frames"], "in 2261:", v["in_2261"], "in view", v["in_view"])
    l = r["look"]
    print("teal", l["teal_bin"]["frames"], "taupe", l["taupe_bag"]["frames"],
          "taupe on s3", l["taupe_bag"]["on_s3"])
