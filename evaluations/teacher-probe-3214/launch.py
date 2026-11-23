# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/teacher-probe-3214/launch.py

3.214: start, read and stop the Hugging Face Jobs that run `job.py`
(amendment 1). Run by hand; every start is a paid call, so it prints the
flavour, its price and the timeout's worst case before it posts.

    python manifest.py --stage 6 --links /path/outside/the/repo/links.json
    python launch.py start qwen  --links .../links.json --limit 5 --timeout-min 40   # the pilot
    python launch.py start super --links .../links.json --limit 5 --timeout-min 45
    python launch.py status <job id>
    python launch.py logs <job id> [--seconds 20]
    python launch.py cancel <job id>

The token is never read here: in a Claude Code cloud session the proxy
puts it on requests to huggingface.co, and elsewhere `HF_TOKEN` is used.
The presigned links travel as job SECRETS (never the environment, which
the job page shows) and are never printed.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

API = "https://huggingface.co/api"
IMAGE = "pytorch/pytorch:2.6.0-cuda12.4-cudnn9-devel"
FETCH = ("python -c \"import os,urllib.request;"
         "open('/tmp/job.py','wb').write(urllib.request.urlopen(os.environ['JOB_URL']).read())\"")
PY312 = "/tmp/v/bin/python"
BOOT = {
    # vLLM 0.8.5 pins torch 2.6.0, the image's own: nothing heavy to install.
    "qwen": f"set -e; {FETCH}; python /tmp/job.py qwen",
    # Alpamayo needs Python 3.12, torch 2.8.0 and transformers 4.57.1
    # (its pyproject); flash-attn is not needed for the SDPA path.
    "super": ("set -e; pip install -q uv; uv venv -q -p 3.12 /tmp/v; "
              f"uv pip install -q -p {PY312} --index-url https://download.pytorch.org/whl/cu126 "
              "torch==2.8.0 torchvision==0.23.0; "
              f"uv pip install -q -p {PY312} transformers==4.57.1 accelerate einops hydra-core "
              "hydra-colorlog numpy pillow scipy pandas matplotlib; "
              f"{FETCH}; {PY312} /tmp/job.py super"),
}
STOP_USD = 12.0        # criterion 10: the section's whole spend


def call(method: str, path: str, body=None, timeout: float = 60.0):
    req = urllib.request.Request(
        API + path, method=method,
        data=None if body is None else json.dumps(body).encode(),
        headers={"content-type": "application/json",
                 "authorization": "Bearer " + os.environ.get("HF_TOKEN", "hf_proxy_injected")})
    return urllib.request.urlopen(req, timeout=timeout)


def get(path: str):
    with call("GET", path) as r:
        return json.loads(r.read())


def namespace() -> str:
    return get("/whoami-v2")["name"]


def price_per_min(flavor: str) -> float:
    for h in get("/jobs/hardware"):
        if h.get("name") == flavor:
            if h.get("unitLabel") != "minute":   # read 2026-11-23: priced per minute
                sys.exit(f"{flavor} is priced per {h.get('unitLabel')!r}, not per minute: check")
            return float(h["unitCostUSD"])
    sys.exit(f"no such flavour: {flavor}")


def job_spec(model: str, links: dict, flavor: str, timeout_min: int, limit: int) -> dict:
    """The POST body (huggingface_hub's `_create_job_spec` shape)."""
    secrets = {"JOB_URL": links["JOB_URL"], "MANIFEST_URL": links[model]["MANIFEST_URL"],
               "RESULT_PUT_URL": links[model]["PILOT_PUT_URL" if limit else "RESULT_PUT_URL"]}
    return {"command": ["bash", "-c", BOOT[model]], "arguments": [],
            "environment": {"LIMIT": str(limit)} if limit else {},
            "secrets": secrets, "flavor": flavor, "timeoutSeconds": int(timeout_min * 60),
            "dockerImage": IMAGE}


def start(args) -> None:  # pragma: no cover -- a paid call
    links = json.loads(Path(args.links).read_text())
    per_min = price_per_min(args.flavor)
    worst = per_min * args.timeout_min
    print(f"{args.model} on {args.flavor}: ${per_min * 60:.2f}/h, timeout {args.timeout_min} min "
          f"=> at most ${worst:.2f}; spent so far (said by --spent): ${args.spent:.2f}")
    if args.spent + worst > STOP_USD:
        sys.exit(f"refused: ${args.spent:.2f} + ${worst:.2f} is over the section's ${STOP_USD:.0f} stop")
    spec = job_spec(args.model, links, args.flavor, args.timeout_min, args.limit)
    with call("POST", f"/jobs/{namespace()}", spec) as r:
        info = json.loads(r.read())
    print("job", info.get("id"), (info.get("status") or {}).get("stage"))


def status(args) -> None:  # pragma: no cover
    info = get(f"/jobs/{namespace()}/{args.job}")
    print(json.dumps({k: info.get(k) for k in ("id", "createdAt", "flavor", "status")}, indent=1))


def logs(args) -> None:  # pragma: no cover
    """The job's log lines so far. The stream never ends while the job
    runs, so reading stops after `--seconds`."""
    end = time.time() + args.seconds
    try:
        with call("GET", f"/jobs/{namespace()}/{args.job}/logs", timeout=args.seconds + 5) as r:
            for raw in r:
                line = raw.decode(errors="replace").strip()
                if line.startswith("data:"):
                    try:
                        print(json.loads(line[5:]).get("data", ""))
                    except json.JSONDecodeError:
                        print(line[5:])
                if time.time() > end:
                    break
    except (TimeoutError, urllib.error.URLError, OSError):
        pass


def cancel(args) -> None:  # pragma: no cover
    call("POST", f"/jobs/{namespace()}/{args.job}/cancel").close()
    print("cancel sent for", args.job)


def main(argv=None) -> None:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("start")
    s.add_argument("model", choices=sorted(BOOT))
    s.add_argument("--links", required=True)
    s.add_argument("--flavor", default="a100-large")
    s.add_argument("--timeout-min", type=int, required=True)
    s.add_argument("--limit", type=int, default=0)
    s.add_argument("--spent", type=float, required=True, help="dollars this section has spent so far")
    s.set_defaults(fn=start)
    for name, fn in (("status", status), ("logs", logs), ("cancel", cancel)):
        p = sub.add_parser(name)
        p.add_argument("job")
        if name == "logs":
            p.add_argument("--seconds", type=float, default=20)
        p.set_defaults(fn=fn)
    args = ap.parse_args(argv)
    args.fn(args)


if __name__ == "__main__":
    main()
