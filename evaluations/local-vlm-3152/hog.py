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

While it holds (amendment 6), it watches VE's /health: if VE stays ready
for `TOPUP_AFTER_S` (a reload that fit, as the missions' own memory moved),
it grows again until VE's next kill and adds `margin_mb` more. Each top-up
is recorded.

It dies with its parent: it sets `PR_SET_PDEATHSIG` on itself first thing
(no `preexec_fn` in the threaded harness) and checks `getppid()` throughout,
so a harness killed hard never leaves memory held.

    python evaluations/local-vlm-3152/hog.py <out.json> <margin_mb> <give_up_s> <hold_max_s> <parent_pid>
"""

import ctypes
import json
import os
import signal
import subprocess
import sys
import time

CHUNK = 32 * 1024 * 1024
TOPUP_AFTER_S = 20.0
VE_HEALTH = "http://127.0.0.1:8005/health"
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


def ve_ready() -> bool:
    import urllib.request
    try:
        with urllib.request.urlopen(VE_HEALTH, timeout=2) as r:
            return json.loads(r.read()).get("ready") is True
    except Exception:  # noqa: BLE001 -- down or loading: not ready
        return False


def chunk() -> bytearray:
    b = bytearray(CHUNK)
    b[::4096] = b"\x01" * len(b[::4096])     # resident
    return b


def main(out: str, margin_mb: int, give_up_s: float, hold_max_s: float, parent: int) -> int:
    stop = {"now": False}

    def on_term(*_):
        stop["now"] = True
    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)
    # The harness's pid comes in argv (review 5): read here, a parent that had
    # already died would read as init and never as gone.
    if ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGKILL) != 0:   # PR_SET_PDEATHSIG
        with open(out, "w") as f:
            json.dump({"prctl_failed": ctypes.get_errno()}, f)
        return 6
    if os.getppid() != parent:          # the harness died before PDEATHSIG was set
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
        ready_since, kills = None, oom_kills()
        rec["topups"] = []
        while not stop["now"] and not orphaned() and time.time() - rec["holding_t"] < hold_max_s:
            now_kills = oom_kills()
            if now_kills > kills:
                kills, ready_since = now_kills, None
                line = last_oom_line()
                if VE_MEMCG not in line:
                    rec["wrong_victim_holding"] = line[-300:]   # scored by the harness
            if ve_ready():
                ready_since = ready_since or time.time()
            else:
                ready_since = None
            if ready_since and time.time() - ready_since >= TOPUP_AFTER_S:
                # VE's reload fit: grow until its next kill, then the margin.
                t, before = time.time(), len(held)
                while (not stop["now"] and not orphaned() and oom_kills() == kills
                       and time.time() - t < give_up_s):
                    held.append(chunk())
                    time.sleep(0.05)
                got = oom_kills() > kills
                if got:
                    kills = oom_kills()
                    line = last_oom_line()
                    if VE_MEMCG not in line:
                        rec["wrong_victim_holding"] = line[-300:]
                    for _ in range(margin_mb * 2**20 // CHUNK):
                        held.append(chunk())
                rec["topups"].append({"t": round(t, 3), "added_mb": (len(held) - before) * CHUNK // 2**20,
                                      "killed": got, "held_mb": len(held) * CHUNK // 2**20})
                write()
                ready_since = None
            time.sleep(0.5)
        rec["ended_by"] = ("SIGTERM" if stop["now"] else "orphaned" if orphaned() else "hold_max_s")
        return 0
    finally:
        held.clear()
        rec["released_t"] = round(time.time(), 3)
        write()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]),
                          int(sys.argv[5])))
