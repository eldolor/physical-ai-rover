# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/local-vlm-3152/hog.py

3.152 criterion 10's memory hog, in its own process (amendments 2 and 4). It
raises its own `oom_score_adj` to 100 (no privilege is needed to raise one's
own): above the stack's -900 and the ROS container's 0, and far enough
below VE's 1000 that VE is the kernel's pick whenever VE runs, even while it
loads (badness is rss + adj x RAM / 1000). It
then grows 32 MB at a time, each chunk resident, until the kernel kills VE:
`oom_kill` in /proc/vmstat rises, and the newest `oom-kill:` line in dmesg
names `task_memcg=/system.slice/picar-local-vlm.service`. It adds
`margin_mb` more, so VE's next load is killed again, and holds that until
SIGTERM, its parent's death or `hold_max_s`. A kill of anything else while
it grows ends it at once, releasing everything (recorded: a failure of the
bar, amendment 3). It grows without a floor: with no swap, only an
allocation brings the kernel to choose, so its own are what do. With no VE
kill in `give_up_s` it gives up.

It dies with its parent: it sets `PR_SET_PDEATHSIG` on itself first thing
(no `preexec_fn` in the threaded harness) and checks `getppid()` throughout,
so a harness killed hard never leaves memory held.

    python evaluations/local-vlm-3152/hog.py <out.json> <margin_mb> <give_up_s> <hold_max_s>
"""

import ctypes
import json
import os
import signal
import subprocess
import sys
import time

CHUNK = 32 * 1024 * 1024
OOM_SCORE_ADJ = 100
VE_MEMCG = "picar-local-vlm.service"


def oom_kills() -> int:
    with open("/proc/vmstat") as f:
        for line in f:
            if line.startswith("oom_kill "):
                return int(line.split()[1])
    return -1


def mem_available_mb() -> int:
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) // 1024
    return 0


def last_oom_line() -> str:
    out = subprocess.run(["sudo", "-n", "dmesg"], capture_output=True, text=True).stdout
    lines = [x for x in out.splitlines() if "oom-kill:" in x]
    return lines[-1] if lines else ""


def chunk() -> bytearray:
    b = bytearray(CHUNK)
    b[::4096] = b"\x01" * len(b[::4096])     # resident
    return b


def main(out: str, margin_mb: int, give_up_s: float, hold_max_s: float) -> int:
    stop = {"now": False}

    def on_term(*_):
        stop["now"] = True
    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)
    parent = os.getppid()
    ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGKILL)   # PR_SET_PDEATHSIG
    if os.getppid() != parent:          # the parent died before the line above
        return 6
    with open("/proc/self/oom_score_adj", "w") as f:
        f.write(str(OOM_SCORE_ADJ))

    def orphaned() -> bool:
        return os.getppid() != parent
    rec = {"t0": round(time.time(), 3), "kills_at_start": oom_kills(), "oom_score_adj": OOM_SCORE_ADJ}
    held = []

    def write():
        with open(out, "w") as f:
            json.dump(rec, f, indent=1)

    write()
    try:
        kills = rec["kills_at_start"]
        while not stop["now"] and not orphaned():
            now = oom_kills()
            if now > kills:
                line = last_oom_line()
                rec.setdefault("kills_seen", []).append(line[-300:])
                if VE_MEMCG not in line:
                    rec["wrong_victim"] = line[-300:]
                    return 3
                rec["first_kill_t"] = round(time.time(), 3)
                rec["first_kill_mb"] = len(held) * CHUNK // 2**20
                break
            if time.time() - rec["t0"] > give_up_s:
                rec["gave_up"] = "no VE kill while growing"
                return 4
            held.append(chunk())
            rec["mem_mb_last"] = mem_available_mb()
            time.sleep(0.05)
        if stop["now"] or orphaned():
            rec["stopped_while_growing"] = "orphaned" if orphaned() else "SIGTERM"
            return 5
        for _ in range(margin_mb * 2**20 // CHUNK):
            held.append(chunk())
        rec["held_mb"] = len(held) * CHUNK // 2**20
        rec["holding_t"] = round(time.time(), 3)
        write()
        while not stop["now"] and not orphaned() and time.time() - rec["holding_t"] < hold_max_s:
            time.sleep(0.2)
        rec["ended_by"] = ("SIGTERM" if stop["now"] else "orphaned" if orphaned() else "hold_max_s")
        return 0
    finally:
        held.clear()
        rec["released_t"] = round(time.time(), 3)
        write()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])))
