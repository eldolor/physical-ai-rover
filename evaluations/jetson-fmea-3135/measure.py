# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/measure.py

The section's measures, from the ground-truth log `truth_body.py` writes
(one row every 20 ms: monotonic `t`, the true pose `x`/`y` in metres, `th`,
the fake board's wheel setpoints `sp` in m/s, the chassis' true `gap_cm`).
Pure functions over rows, so the tests feed them made-up logs.

* *Stopped:* `wheels_zero_s` -- from the fault to the first sample whose
  setpoints are both zero (the fake board's own targets, what the firmware
  drives); `travel_cm` -- the true path length (translation) over a window.
* *Contacts:* samples with `gap_cm` under 1.0 cm (footprint_sweep's G bar).
* *Moving:* whether the robot was driving FORWARD at an instant (both
  setpoints above 0.05 m/s), which is when every "mid-move" fault fires.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Optional

CONTACT_CM = 1.0            # tests/footprint_sweep.py G_BAR_CM
FORWARD_M_S = 0.05


def read_truth(path: Path) -> list:
    rows = []
    for line in Path(path).read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except ValueError:
            continue                    # a line cut by the kill
    rows.sort(key=lambda r: r["t"])
    return rows


def is_zero(sp) -> bool:
    return sp is not None and all(abs(v) < 1e-6 for v in sp)


def is_forward(sp) -> bool:
    return sp is not None and all(v > FORWARD_M_S for v in sp)


HOLD_ZERO_S = 0.5
MAX_STEP_S = 0.1            # five recorder periods


def wheels_zero_s(rows: list, t0: float, hold_s: float = HOLD_ZERO_S) -> Optional[float]:
    """Seconds from `t0` to the first sample at or after it from which both
    setpoints stay zero for `hold_s` (a zero that the next command undoes
    is not a stop); the log must reach that far. None if they never did."""
    after = [r for r in rows if r["t"] >= t0]
    start, prev = None, None            # the first sample of the current run of zeros
    for r in after:
        if not is_zero(r["sp"]) or (prev is not None and r["t"] - prev > MAX_STEP_S):
            start = None                # a hole cannot vouch for a hold
        prev = r["t"]
        if not is_zero(r["sp"]):
            continue
        if start is None:
            start = r["t"]
        if r["t"] >= start + hold_s:
            return round(start - t0, 3)
    return None


def travel_cm(rows: list, t0: float, t1: Optional[float] = None) -> float:
    """True path length (cm) of the chassis' centre between t0 and t1."""
    pts = [r for r in rows if r["t"] >= t0 and (t1 is None or r["t"] <= t1)]
    return round(100 * sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"])
                           for a, b in zip(pts, pts[1:])), 2)


def contacts(rows: list, t0: Optional[float] = None, t1: Optional[float] = None) -> int:
    return sum(1 for r in rows
               if (t0 is None or r["t"] >= t0) and (t1 is None or r["t"] <= t1)
               and r.get("gap_cm") is not None and r["gap_cm"] < CONTACT_CM)


def nonzero_after(rows: list, t0: float, t1: float) -> int:
    """Samples in [t0, t1] with any setpoint non-zero: something drove."""
    return sum(1 for r in rows if t0 <= r["t"] <= t1 and not is_zero(r["sp"]))


def span(rows: list) -> Optional[list]:
    return [rows[0]["t"], rows[-1]["t"]] if rows else None


def gaps_s(rows: list, t0: float, t1: float) -> float:
    """The longest hole in the log over [t0, t1], its edges included: a
    recorder that stalled or died cannot vouch for what happened in it."""
    ts = [t0] + [r["t"] for r in rows if t0 < r["t"] < t1] + [t1]
    inside = [r for r in rows if t0 <= r["t"] <= t1]
    if not inside:
        return math.inf
    return round(max(b - a for a, b in zip(ts, ts[1:])), 3)


def forward_at(rows: list, t: float) -> Optional[bool]:
    """Whether the board drove both wheels forward at the last sample at or
    before `t` (None with no sample there)."""
    before = [r for r in rows if r["t"] <= t]
    return is_forward(before[-1]["sp"]) if before else None
