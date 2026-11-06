# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/truth_pressure.py

3.151 criteria 3-4 on the board, with no motion: the truth body alone (sim,
its recorder running, the fake board idle) at `oom_score_adj` -900 as F2 runs
it, and `tools/jetson/memory_hog.py` to the first OOM kill (the twin's units
stopped and the local VLM held off, as every bench does, so the kernel's pick
is the hog), locked and unlocked alternately.

    python evaluations/jetson-fmea-3135/truth_pressure.py board [--runs 3] [--sha HEAD]
        (the desktop: the board lock, 3.136's runner and lease, as found after)
    python evaluations/jetson-fmea-3135/truth_pressure.py one <out> <lock 0|1> <port>
        (on the board, one run; what `board` runs there)
    python evaluations/jetson-fmea-3135/truth_pressure.py score <dir>

Per run (`<out>/run.json`): the lock's answer, the body's `VmRSS` / `VmLck`
idle, after the lock and after 60 s of recording (criterion 3), the hog's
end, `MemAvailable`'s minimum (sampled every 0.2 s), and the largest gaps
between truth samples from the hog's start to 10 s after its end
(criterion 4: none over 0.2 s locked).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PY = sys.executable
GAP_BAR_S = 0.2
AFTER_S = 10.0
RECORD_S = 60.0
OOM_ADJ = -900
HOG_LIMIT_S = 180


def _status(pid: int) -> dict:
    out = {}
    with open(f"/proc/{pid}/status") as f:
        for line in f:
            k = line.split(":")[0]
            if k in ("VmRSS", "VmLck", "VmSize"):
                out[k] = int(line.split()[1])
    return out


def _mem_available_kb() -> int:
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1])
    raise RuntimeError("no MemAvailable")


def _post(url: str, timeout: float = 30.0) -> dict:
    req = urllib.request.Request(url, data=b"", method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def gaps(rows: list, t0: float, t1: float) -> dict:
    """The largest gaps between consecutive truth samples whose later sample
    falls in [t0, t1] (monotonic seconds), and how many exceed the bar."""
    ts = [r["t"] for r in rows]
    g = sorted(((b - a, a) for a, b in zip(ts, ts[1:]) if t0 <= b <= t1), reverse=True)
    return {"n_samples": sum(t0 <= t <= t1 for t in ts), "max_s": round(g[0][0], 4) if g else None,
            "over_bar": sum(d > GAP_BAR_S for d, _ in g), "top": [[round(d, 4), round(a, 3)] for d, a in g[:5]]}


def one(out: Path, lock: bool, port: int) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, FMEA_DIR=str(out), SIM_BODY_SHM=f"picar_3151_{os.getpid()}",
               PYTHONUNBUFFERED="1")
    log = open(out / "body.log", "w")
    body = subprocess.Popen([PY, str(HERE / "truth_body.py"), str(port)], cwd=ROOT, env=env,
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rec = {"lock": lock, "port": port, "body_pid": body.pid}
    try:
        end = time.monotonic() + 60
        while True:
            try:
                urllib.request.urlopen(f"http://127.0.0.1:{port}/health", timeout=1).read()
                break
            except OSError:
                if time.monotonic() > end or body.poll() is not None:
                    raise RuntimeError("the truth body did not come up")
                time.sleep(0.2)
        subprocess.run(["sudo", "-n", "tee", f"/proc/{body.pid}/oom_score_adj"], input=f"{OOM_ADJ}\n",
                       text=True, capture_output=True, check=True)
        rec["mem_idle"] = _status(body.pid)
        if lock:
            raised = subprocess.run(["sudo", "-n", "prlimit", "--pid", str(body.pid),
                                     "--memlock=unlimited:unlimited"], capture_output=True).returncode == 0
            rec["lock_answer"] = {**_post(f"http://127.0.0.1:{port}/fmea/mlock"), "limit_raised": raised}
        rec["mem_after_lock"] = _status(body.pid)
        time.sleep(RECORD_S)
        rec["mem_after_60s"] = _status(body.pid)
        avail, stop = [], threading.Event()

        def sampler():
            while not stop.is_set():
                avail.append((time.monotonic(), _mem_available_kb()))
                time.sleep(0.2)
        th = threading.Thread(target=sampler, daemon=True)
        th.start()
        rec["hog_start"] = time.monotonic()
        hog = subprocess.run([PY, str(ROOT / "tools/jetson/memory_hog.py"), str(HOG_LIMIT_S)],
                             capture_output=True, text=True, cwd=ROOT)
        rec["hog_end"] = time.monotonic()
        rec["hog_rc"] = hog.returncode
        (out / "hog.log").write_text(hog.stdout + hog.stderr)
        time.sleep(AFTER_S)
        stop.set()
        th.join(2)
        rec["mem_available_min_mb"] = round(min(a for _, a in avail) / 1024, 1) if avail else None
        rec["body_alive"] = body.poll() is None
        rec["mem_end"] = _status(body.pid) if rec["body_alive"] else None
    finally:
        if body.poll() is None:
            os.killpg(body.pid, 15)
            try:
                body.wait(15)
            except subprocess.TimeoutExpired:
                os.killpg(body.pid, 9)
        log.close()
    rows = [json.loads(x) for x in (out / "truth.jsonl").read_text().splitlines() if x.strip()]
    rec["gaps"] = gaps(rows, rec["hog_start"], rec["hog_end"] + AFTER_S)
    (out / "run.json").write_text(json.dumps(rec, indent=1))
    return rec


def board_cmd(runs: int, port: int) -> str:
    """The board side: the VLM held off (its flag; put back as found), then
    the runs, locked first and alternating."""
    order = " ".join(("1" if i % 2 == 0 else "0") for i in range(2 * runs))
    return (f"source tools/jetson/vlm_hold.sh && trap vlm_restore EXIT && "
            f"{{ vlm_hold $OUT/vlm_hold.log || exit 2; }} && "
            f"i=0; for L in {order}; do i=$((i+1)); "
            f".venv/bin/python evaluations/jetson-fmea-3135/truth_pressure.py one "
            f"$OUT/run$i-lock$L $L {port} || exit 3; sleep 20; done")


def score(d: Path) -> dict:
    runs = []
    for f in sorted(d.rglob("run.json")):
        r = json.loads(f.read_text())
        runs.append({"run": f.parent.name, "lock": r["lock"],
                     "locked": (r.get("lock_answer") or {}).get("locked"),
                     "gap_max_s": r["gaps"]["max_s"], "over_bar": r["gaps"]["over_bar"],
                     "mem_available_min_mb": r.get("mem_available_min_mb"),
                     "rss_idle_kb": r["mem_idle"]["VmRSS"], "rss_60s_kb": r["mem_after_60s"]["VmRSS"],
                     "lck_60s_kb": r["mem_after_60s"].get("VmLck"), "body_alive": r.get("body_alive")})
    on = [r for r in runs if r["lock"]]
    off = [r for r in runs if not r["lock"]]
    reached = all((r["mem_available_min_mb"] or 1e9) <= 100 for r in runs)
    rss = {k: [r["rss_60s_kb"] for r in g] for k, g in (("locked", on), ("unlocked", off))}
    return {"runs": runs, "all_reached_100mb": reached,
            "c3_rss_added_mb": (round((max(rss["locked"]) - min(rss["unlocked"])) / 1024, 1)
                                if on and off else None),
            "c4_locked_no_gap": bool(on) and all(r["locked"] and r["over_bar"] == 0 for r in on),
            "c4_plant_shown": any(r["over_bar"] > 0 for r in off)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    o = sub.add_parser("one")
    o.add_argument("out")
    o.add_argument("lock", choices=["0", "1"])
    o.add_argument("port", type=int)
    b = sub.add_parser("board")
    b.add_argument("--runs", type=int, default=3)
    b.add_argument("--sha", default="HEAD")
    b.add_argument("--port", type=int, default=int(os.environ.get("PICAR_BODY_PORT", 10352)))
    i = sub.add_parser("_board")
    i.add_argument("--runs", type=int, required=True)
    i.add_argument("--sha", required=True)
    i.add_argument("--port", type=int, required=True)
    i.add_argument("--out", required=True)
    s = sub.add_parser("score")
    s.add_argument("dir")
    a = ap.parse_args(argv)
    if a.cmd == "one":
        print(json.dumps(one(Path(a.out), a.lock == "1", a.port), indent=1))
        return 0
    if a.cmd == "score":
        print(json.dumps(score(Path(a.dir)), indent=1))
        return 0
    sys.path.insert(0, str(ROOT / "evaluations/jetson-perf-3136"))
    import boardrun as br
    if a.cmd == "board":
        sha = subprocess.run(["git", "rev-parse", a.sha], cwd=ROOT, capture_output=True, text=True,
                             check=True).stdout.strip()
        out = HERE / "board" / "truth-pressure-3151" / time.strftime("%Y%m%d-%H%M")
        return br.locked([PY, str(Path(__file__).resolve()), "_board", "--runs", str(a.runs),
                          "--sha", sha, "--port", str(a.port), "--out", str(out)],
                         "3.151 truth under memory pressure (no motion)", desktop_mode="shared",
                         desktop_wait=True)
    import run as perf
    out = Path(a.out)
    rec = perf.board_run("3151", board_cmd(a.runs, a.port), out, sha=a.sha, allow_sudo=True,
                         stop_twin=True, bench_dir="bench-3151")
    perf_out = out / "board"
    if perf_out.exists():
        (out / "summary.json").write_text(json.dumps(score(perf_out), indent=1))
    (out / "record.json").write_text(json.dumps({k: v for k, v in rec.items()
                                                 if k not in ("as_found", "after")}, indent=1, default=str))
    return 0 if rec.get("cmd_exit") == 0 and rec.get("left_as_found") else 1


if __name__ == "__main__":
    sys.exit(main())
