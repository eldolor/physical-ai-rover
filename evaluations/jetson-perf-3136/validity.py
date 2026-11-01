"""
evaluations/jetson-perf-3136/validity.py

3.136's validity rule as code (the section's "Every timed run is valid only
if", 3.131's rule). A timed run counts only if ALL hold:

1. the bench ran inside the full-stack window (start >= window start, end
   <= window end);
2. 0 late wheel ticks in the window, and the window had moving ticks (a
   wheel loop that never ran has 0 late ticks too);
3. 0 OOM kills in the window (`/proc/vmstat` `oom_kill`, and the bench's
   own count when it has one);
4. nav2's goals all succeeded, and there was at least one;
5. the board at 15 W (`nvpmodel -q`);
6. the desktop's load recorded;
7. temperature recorded, and the sysfs clocks recorded.

An invalid run is flagged with every reason, never scored: `score()` returns
None for it. A quiet-board run (no window) is reported with no bar: it is
never "valid" for a bar, and its numbers stand beside the full-stack ones.
"""

from __future__ import annotations

from typing import Optional


def judge(run: dict) -> dict:
    """`run` keys: kind ("full" | "quiet" | "smoke"), window [t0, t1] or None,
    bench [b0, b1], wheel {"late": int, "moving": int} (deltas over the
    window), oom_kills (int), nav {"ok": int, "n": int}, power_mode (str),
    desktop_load (list), temps (int: samples with a temperature), clocks
    (int: samples with the sysfs clocks). Missing keys are reasons."""
    reasons = []
    kind = run.get("kind")
    if kind != "full":
        reasons.append(f"not a full-stack run ({kind}): reported with no bar")
    win, bench = run.get("window"), run.get("bench")
    if not win or win[0] is None or win[1] is None:
        reasons.append("no full-stack window")
    elif not bench or bench[0] is None or bench[1] is None:
        reasons.append("bench start/end not recorded")
    else:
        if bench[0] < win[0]:
            reasons.append(f"bench started {win[0] - bench[0]:.1f} s before the window")
        if bench[1] > win[1]:
            reasons.append(f"bench ended {bench[1] - win[1]:.1f} s after the window")
    w = run.get("wheel")
    if not w or w.get("late") is None:
        reasons.append("wheel loop not read")
    else:
        if w["late"] != 0:
            reasons.append(f"{w['late']} late wheel ticks")
        if not w.get("moving"):
            reasons.append("no moving wheel ticks in the window")
    oom = run.get("oom_kills")
    if oom is None:
        reasons.append("OOM kills not counted")
    elif oom != 0:
        reasons.append(f"{oom} OOM kills")
    nav = run.get("nav")
    if not nav or nav.get("n") is None:
        reasons.append("nav2 goals not counted")
    elif nav["n"] == 0:
        reasons.append("no nav2 goals")
    elif nav["ok"] != nav["n"]:
        reasons.append(f"nav2 goals {nav['ok']}/{nav['n']} succeeded")
    pm = run.get("power_mode") or ""
    if "15W" not in pm.replace(" ", ""):
        reasons.append(f"power mode {pm!r}, not 15W")
    if not run.get("desktop_load"):
        reasons.append("desktop load not recorded")
    if not run.get("temps"):
        reasons.append("temperature not recorded")
    if not run.get("clocks"):
        reasons.append("sysfs clocks not recorded")
    return {"valid": not reasons, "reasons": reasons}


def score(run: dict, value) -> Optional[object]:
    """The row's number if the run is valid, else None (flagged, not scored)."""
    return value if judge(run)["valid"] else None


def telemetry_counts(sysfs_rows: list, t0: Optional[float] = None, t1: Optional[float] = None) -> dict:
    """From the sampler's rows inside [t0, t1]: samples with a temperature,
    with clocks, the OOM-kill delta, max temperature (C), min MemAvailable,
    and how often a CPU or the GPU sat below its maximum clock."""
    rows = [r for r in sysfs_rows
            if (t0 is None or r.get("t", 0) >= t0) and (t1 is None or r.get("t", 0) <= t1)]
    temps = [max(r["temps"].values()) / 1000 for r in rows if r.get("temps")]
    clocks = [r for r in rows if r.get("cpu_khz") and r.get("gpu_hz")]
    ooms = [r["oom_kill"] for r in rows if isinstance(r.get("oom_kill"), int)]
    below_cpu = sum(any(c < m for c, m in r["cpu_khz"]) for r in clocks)
    return {"samples": len(rows), "temps": len(temps), "clocks": len(clocks),
            "max_temp_c": max(temps) if temps else None,
            "oom_kills": (ooms[-1] - ooms[0]) if ooms else None,
            "mem_available_min_mb": min((r["mem_mb"] for r in rows if "mem_mb" in r), default=None),
            "cpu_below_max_samples": below_cpu}
