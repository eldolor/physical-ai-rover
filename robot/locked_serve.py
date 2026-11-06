# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
robot/locked_serve.py

3.155: the robot server, memory-locked, then served.

    python -m robot.locked_serve --port 8000 --host 127.0.0.1 --log-level warning

takes uvicorn's arguments for `robot.server:app`. Under an OOM squeeze the
robot server's pages were reclaimed and faulted back, and its wheel loop ran
late (3.135.B F2's new record: 204-439 ms ticks at 10-11 MB `MemAvailable`),
though `oom_score_adj` -900 kept it alive. So, before serving:
`mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT)` -- every page it has
touched or touches stays resident, and nothing it only maps is faulted in
(3.151 measured why `MCL_ONFAULT`: thread stacks and reservations).

Only under an unlimited memlock limit (`LimitMEMLOCK=infinity` in
`picar-stack.service`, or a harness's `prlimit` after the spawn, waited for
up to `PICAR_MEMLOCK_WAIT_S`): under a finite one `MCL_FUTURE` would make a
later thread stack or heap mapping fail mid-run, so it serves unlocked. One
stderr line says what happened either way. `robot/server.py` is unchanged.
"""

import ctypes
import errno as errno_mod
import json
import os
import resource
import sys
import time

MCL_CURRENT, MCL_FUTURE, MCL_ONFAULT = 1, 2, 4          # <sys/mman.h> on Linux
APP = "robot.server:app"


def _libc():
    return ctypes.CDLL(None, use_errno=True)


def _kb(v):
    return "unlimited" if v == resource.RLIM_INFINITY else v // 1024


def _status_kb(field: str):
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith(field + ":"):
                    return int(line.split()[1])
    except OSError:
        pass
    return None


def lock(wait_s: float = 0.0, sleep=time.sleep, now=time.monotonic) -> dict:
    """Lock this process's touched pages, if the memlock limit is (or within
    `wait_s` becomes) unlimited; what happened, a refusal included."""
    end = now() + wait_s
    while True:
        soft, hard = resource.getrlimit(resource.RLIMIT_MEMLOCK)
        if soft == resource.RLIM_INFINITY or now() >= end:
            break
        sleep(0.1)
    out = {"locked": False, "errno": None, "error": None,
           "limit_soft_kb": _kb(soft), "limit_hard_kb": _kb(hard)}
    if soft != resource.RLIM_INFINITY:
        out["error"] = "limit_not_raised"
    else:
        rc = _libc().mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT)
        e = ctypes.get_errno() if rc != 0 else 0
        out.update(locked=rc == 0, errno=e or None,
                   error=errno_mod.errorcode.get(e, str(e)) if e else None)
    out.update(vm_lck_kb=_status_kb("VmLck"), vm_rss_kb=_status_kb("VmRSS"))
    return out


def _serve(argv: list) -> None:
    from uvicorn.main import main as uvicorn_main
    sys.argv = ["uvicorn", *argv]
    uvicorn_main()


def main(argv=None) -> None:
    argv = sys.argv[1:] if argv is None else argv
    got = lock(float(os.environ.get("PICAR_MEMLOCK_WAIT_S") or 0))
    print("robot server memory lock: " + json.dumps(got), file=sys.stderr, flush=True)
    _serve([APP, *argv])


if __name__ == "__main__":
    main()
