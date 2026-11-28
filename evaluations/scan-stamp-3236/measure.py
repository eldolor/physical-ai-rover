# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/scan-stamp-3236/measure.py

PLAN-ros-alignment.md 3.236.A: `/scan`'s `stamp_unix` (3.221's capture
stamp) against GROUND TRUTH under the fake lidar's real timing.

    python evaluations/scan-stamp-3236/measure.py [--seconds 60] [--load] [--out FILE]

The path is the car's, minus the hardware:

* `sim/fake_lidar.py`'s `Emitter` casts point k at `t0 + k / 4500` s on the
  monotonic clock and writes its packets, in real time, to a real pty;
* the real driver (`robot.lidar_ld19.Ld19Lidar`, its reader thread) reads
  them, as it reads `/dev/picar-lidar` on the Rover;
* the robot server's own `/scan` route serves the driver's scan (the
  simulated body's `get_scan` is replaced by the driver's, nothing else);
* a poller asks for `/scan` at the bridge's 10 Hz with a random phase.

Revolution r is the emitter's points k in [450 r, 450 r + 450) (the driver's
assembler ends a revolution where the angle wraps), so its true midpoint is
`t0 + (450 r + 224.5) / 4500` -- on the monotonic clock, carried to the wall
clock with the offset read beside each poll.

`--jitter` polls each time at a fresh random phase (supplementary).
`--load` keeps every core but one busy with a CPU burner (criterion 3).
Standard library plus the repo; `run()` is what tests/test_scan_stamp_truth.py
calls for its short version.
"""

import argparse
import json
import multiprocessing
import os
import random
import statistics
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot.lidar_ld19 import Ld19Lidar  # noqa: E402
from sim.fake_lidar import POINTS_PER_S, RATE_HZ, Emitter, open_pty  # noqa: E402

POINTS_PER_REV = int(round(POINTS_PER_S / RATE_HZ))     # 450
POLL_HZ = 10.0                                          # picar_bridge's SCAN_HZ


def _burn(stop):
    x = 0
    while not stop.is_set():
        x = (x * 1103515245 + 12345) & 0xFFFFFFFF


def _server(lidar, tmp_dir):
    """The robot server's real app, on the simulated config, with the body's
    scan replaced by the driver's -- so `/scan`'s stamping code is the code
    under test."""
    import yaml
    from fastapi.testclient import TestClient
    import robot.server as server
    from sim.mock_robot import MockRobot

    cfg = yaml.safe_load((ROOT / "config/robot.yaml").read_text())
    cfg["mode"] = "sim"
    path = Path(tmp_dir) / "robot.yaml"
    path.write_text(yaml.safe_dump(cfg))
    saved = {k: os.environ.pop(k, None)
             for k in ("ROBOT_MODE", "APP_SHARED_SECRET", "ROBOT_LIDAR_SELF_BOX_CM")}
    real = MockRobot.get_scan
    MockRobot.get_scan = lambda self, max_range_m=None: lidar.get_scan(max_range_m)

    def restore():
        MockRobot.get_scan = real
        os.environ.update({k: v for k, v in saved.items() if v is not None})

    try:
        return TestClient(server.create_app(str(path))), restore
    except BaseException:
        restore()
        raise


def run(seconds: float = 60.0, load: bool = False, seed: int = 0, tmp_dir=None,
        jitter: bool = False) -> dict:
    """One measured run; returns the per-reply rows and their summary."""
    import tempfile
    rng = random.Random(seed)
    tmp_dir = tmp_dir or tempfile.mkdtemp(prefix="scan-stamp-3236-")
    master, slave, path = open_pty(None)
    stop = threading.Event()
    burners = []
    if load:
        mstop = multiprocessing.Event()
        for _ in range(max(1, (os.cpu_count() or 2) - 1)):
            p = multiprocessing.Process(target=_burn, args=(mstop,), daemon=True)
            p.start()
            burners.append(p)

    t0 = time.monotonic() + 0.05
    emitter = Emitter(lambda angle, t: 1500, t0=t0)

    def emit():
        while not stop.is_set():
            data = b"".join(emitter.due(time.monotonic()))
            if data:
                try:
                    os.write(master, data)
                except BlockingIOError:
                    pass                    # nobody reading: lost, as on a UART
            time.sleep(0.002)               # the fake's own loop period

    lidar = Ld19Lidar(path)
    th = threading.Thread(target=emit, name="emitter", daemon=True)
    th.start()
    client, restore = _server(lidar, tmp_dir)
    rows = []
    try:
        with client:
            time.sleep(0.35)                 # the first, partial revolution is discarded
            end = time.monotonic() + seconds
            next_at = time.monotonic() + rng.uniform(0, 1.0 / POLL_HZ)
            while time.monotonic() < end:
                time.sleep(max(0.0, next_at - time.monotonic()))
                # `jitter`: each poll at a fresh random phase (supplementary: a
                # fixed 10 Hz poll beside a 10 Hz lidar sees one phase a run)
                next_at += 1.0 / POLL_HZ + (rng.uniform(-0.05, 0.05) if jitter else 0.0)
                off = time.time() - time.monotonic()
                before = time.time()
                s = client.get("/scan").json()
                after = time.time()
                if not s.get("usable"):
                    rows.append({"usable": False, "before": before})
                    continue
                stamp = s["stamp_unix"]
                # the revolution served: the nearest true midpoint to the stamp
                mono = stamp - off
                r = round(((mono - t0) * POINTS_PER_S - (POINTS_PER_REV - 1) / 2.0) / POINTS_PER_REV)
                truth = off + t0 + (POINTS_PER_REV * r + (POINTS_PER_REV - 1) / 2.0) / POINTS_PER_S
                rows.append({"usable": True, "source": s["stamp_source"], "rev": r,
                             "stamp": stamp, "truth": truth, "before": before, "after": after,
                             "err_ms": 1000.0 * (stamp - truth),
                             "req_err_ms": 1000.0 * (before - truth),
                             "age_lo_ms": 1000.0 * (before - stamp),
                             "age_hi_ms": 1000.0 * (after - stamp)})
    finally:
        restore()
        stop.set()
        th.join(timeout=1.0)
        lidar.close()
        for fd in (master, slave):
            try:
                os.close(fd)
            except OSError:
                pass
        if burners:
            mstop.set()
            for p in burners:
                p.join(timeout=2.0)
    summary = summarise(rows, seconds, load)
    summary["jitter"] = jitter
    return {"rows": rows, "summary": summary}


def _pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round(q * (len(xs) - 1))))]


def summarise(rows, seconds=None, load=False) -> dict:
    ok = [r for r in rows if r["usable"]]
    distinct = {}
    for r in ok:
        distinct.setdefault(r["stamp"], r)
    d = sorted(distinct.values(), key=lambda r: r["stamp"])
    errs = [r["err_ms"] for r in d]
    aerr = [abs(e) for e in errs]
    gaps = [1000.0 * (b["stamp"] - a["stamp"]) for a, b in zip(d, d[1:])]
    revs = [r["rev"] for r in d]
    return {
        "seconds": seconds, "load": load, "cores": os.cpu_count(),
        "replies": len(rows), "unusable": len(rows) - len(ok),
        "sources": sorted({r["source"] for r in ok}),
        "not_capture": sum(1 for r in ok if r["source"] != "capture"),
        "distinct_revolutions": len(d),
        "one_stamp_per_revolution": len(set(revs)) == len(revs),
        "err_ms": {"median": statistics.median(errs) if errs else None,
                   "p95_abs": _pct(aerr, 0.95) if aerr else None,
                   "max_abs": max(aerr) if aerr else None,
                   "min": min(errs) if errs else None, "max": max(errs) if errs else None},
        "request_stamp_err_ms": {   # what the stamp was before 3.221: the request time
            "median": statistics.median(r["req_err_ms"] for r in ok) if ok else None,
            "min": min(r["req_err_ms"] for r in ok) if ok else None,
            "max": max(r["req_err_ms"] for r in ok) if ok else None},
        "age_ms": {"min_lo": min(r["age_lo_ms"] for r in ok) if ok else None,
                   "max_hi": max(r["age_hi_ms"] for r in ok) if ok else None,
                   "outside_0_250": sum(1 for r in ok
                                        if r["age_lo_ms"] < 0 or r["age_hi_ms"] > 250.0)},
        "gap_ms": {"median": statistics.median(gaps) if gaps else None,
                   "min": min(gaps) if gaps else None, "max": max(gaps) if gaps else None},
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--load", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--jitter", action="store_true")
    ap.add_argument("--out")
    args = ap.parse_args(argv)
    res = run(args.seconds, args.load, args.seed, jitter=args.jitter)
    print(json.dumps(res["summary"], indent=2))
    if args.out:
        Path(args.out).write_text(json.dumps({"summary": res["summary"]}, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
